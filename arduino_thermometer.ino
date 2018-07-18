/* The output of the LM 35 is 10 mV (0.01 volts) per degree Celsius */
#include <avr/pgmspace.h>
#include <EEPROM.h>

#ifdef TXLED0 // Macro only in Pro Micro/Fio board definition, used to detect board type.
  #define PRO_MICRO // Sparkfun Pro Micro, else Genuino/Arduino Micro
#endif

#ifdef PRO_MICRO
  #define LED_BUILTIN 17
  #define SENSE_PIN A1
#else
  #define SENSE_PIN A0
#endif

#define MODE_PIN 2 // Pullup, only used during startup and then turned off.

#ifdef PRO_MICRO
  #define DISPLAY_PIN A2 // Used to toggle display of hi/lo after startup and to reset eeprom
#endif

#ifndef PRO_MICRO
  // I broke the USB port on my pro micro, so no serial :(
  #define ENABLE_SERIAL
#endif

// How often to poll temp sensor, in milliseconds
#define NORMAL_SENSOR_POLL_TIME 5000
#define FAST_SENSOR_POLL_TIME 500 // read faster when temp average changes between reading
uint16_t sensorPollTime = NORMAL_SENSOR_POLL_TIME; // max 65,535ms between readings

#define CELCIUS HIGH
#define FAHRENHEIT LOW
byte scale = CELCIUS; // CELCIUS or FAHRENHEIT

const byte numDigits = 4;
const byte digitPins[] = {3, 4, 5, 6}; // disp 6 8 9 12
#ifdef PRO_MICRO
// const byte segmentPins[] = {8, 15, A0, 10, 16, 7, 9, 14}; // disp: 11 5 7 1 2 3 4 10
const byte segmentPins[] = {8, 9, 15, 16, 10, 7, A0, 14}; // disp: 11 5 7 1 2 3 4 10
                         // A  C  B   D   E   F  G   H

                         // 7   10  A0  16  9    14   15  8
                         // F   E   G   D   C    H    B   A
                         //  AAAA          0000
                         // F    B        5    1
                         // F    B        5    1
                         //  GGGG          6666
                         // E    C        4    2
                         // E    C        4    2        (Segment H is often called
                         //  DDDD  H       3333  7      DP, for Decimal Point)

#else // Arduino/Genuino Micro
  const byte segmentPins[] = {8, A5, 10, A1, A2, A3, A4, 7};
#endif

// #define TEST_PATTERN // show test patterns on display, not temperature
#ifdef TEST_PATTERN
  // Bit-segment mapping:  0bHGFEDCBA
  byte testData[4] = { 0b11111111, 0b11001001, 0b00110110, 0b00011011 };
#endif

#define LOOP_DELAY 2

#define LED_COUNTER_VALUE 10 // How long the LED_BUILTIN stays lit after reading temp sensor
uint8_t ledCounter = LED_COUNTER_VALUE;
boolean ledState = LOW;
#ifdef PRO_MICRO
  /* Alternate the blinking of the LEDs between TX and RX.
     LOW == RX, HIGH == TX */
  boolean currentLed = LOW;
#endif

unsigned long timer = 0; // will be reset every loop()
unsigned long current = millis();

#define AVERAGE_COUNT 20
float temperatures[AVERAGE_COUNT];
uint8_t averageTempPosition = 0;
float previousAverageTemperature;

// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
float readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // return result; // Vcc in millivolts
  return float(result)/1000; // Vcc in volts
}

float vcc;
#define VCC_READ_COUNT 10
uint8_t vccReadCounter = VCC_READ_COUNT;

#include "SevSeg.h"
SevSeg sevseg; //Initiate a seven segment controller object

void waitAndRefreshDisplay(uint16_t ms) {
  current = millis();
  while (millis() < (current + ms)) {
    sevseg.refreshDisplay(); // Must run repeatedly
    delay(1);
  }
}

void setCharsAndWait(char str[], uint16_t ms) {
  sevseg.setChars(str);
  waitAndRefreshDisplay(ms);
}

void setFloatAndWait(float numToShow, char decPlaces, uint16_t ms) {
  sevseg.setNumber(numToShow, decPlaces);
  waitAndRefreshDisplay(ms);
}

void blankAndWait(uint16_t ms) {
  sevseg.blank();
  waitAndRefreshDisplay(ms);
}

// #define SHOW_VERSION_ON_STARTUP
const uint8_t eepromVersion = 8;
#define DEFAULT_LOWEST_TEMP 155.0
#define DEFAULT_HIGHEST_TEMP -55.0
struct Settings {
  uint8_t version;
  float lowestTemp;
  float highestTemp;

} settings;

boolean eepromWasReset = false;

void setup() {
  delay(500);

  #ifdef ENABLE_SERIAL
    Serial.begin(9600);
  #endif

  pinMode(SENSE_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DISPLAY_PIN, INPUT);

  ledOff();

  pinMode(MODE_PIN, INPUT_PULLUP);
  scale = digitalRead(MODE_PIN);
  pinMode(MODE_PIN, INPUT); // don't need it any more, turn off pullup to save power

  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins);

  EEPROM.get(0, settings);
  #ifdef SHOW_VERSION_ON_STARTUP
    setCharsAndWait("VERS", 500);
    sevseg.setNumber(settings.version);
    waitAndRefreshDisplay(1000);
  #endif

  if ((settings.version != eepromVersion) || (digitalRead(DISPLAY_PIN) == HIGH)) {
    eepromWasReset = true;
    setCharsAndWait("RSET", 250);
    settings.version = eepromVersion;
    settings.lowestTemp = DEFAULT_LOWEST_TEMP;
    settings.highestTemp = DEFAULT_HIGHEST_TEMP;
    EEPROM.put(0, settings);
  }

  /* Show scale on display for 1 second or SENSOR_POLL_TIME, whichever is greater
     <blank> <degree sign> <letter/blank> <blank> */
  uint8_t scaleDisplay[numDigits] = { B00000000, B01100011, B00000000, B00000000 };
  if (scale == LOW) {
    // "F"
    scaleDisplay[2] = B01110001;
  } else {
    // "C"
    scaleDisplay[2] = B00111001;
  }
  sevseg.setSegments(scaleDisplay);
  waitAndRefreshDisplay(1000);

  if (!eepromWasReset) {
    showHiAndLo();
  }

  // set current vcc
  vcc = readVcc();

  // Prefill the average array
  float initialTemperature = currentTemperatureReading();
  #ifdef ENABLE_SERIAL
    Serial.print(F("Initial: "));
    Serial.println(initialTemperature);
  #endif
  for (uint8_t i = 0; i < AVERAGE_COUNT;i++) {
    temperatures[i] = initialTemperature;
  }
  previousAverageTemperature = initialTemperature;

}

void showHiAndLo() {
  setCharsAndWait("HI", 500);
  setFloatAndWait(convertedTemperature(settings.highestTemp), 1, 1000);
  blankAndWait(500);

  setCharsAndWait("LO", 500);
  setFloatAndWait(convertedTemperature(settings.lowestTemp), 1, 1000);
  blankAndWait(500);
}

void ledOn() {
  #ifdef PRO_MICRO
    if (currentLed == HIGH) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      TXLED1;
    }
  #else
    digitalWrite(LED_BUILTIN, HIGH);
  #endif
  ledState = HIGH;
  ledCounter = LED_COUNTER_VALUE;
}

void ledOff() {
  #ifdef PRO_MICRO
    TXLED0;
    digitalWrite(LED_BUILTIN, HIGH); // RX LED is active low
    currentLed = !currentLed;
  #else
    digitalWrite(LED_BUILTIN, LOW);
  #endif
  ledState = LOW;
  ledCounter = 0;
}

float currentTemperatureReading() {
  ledOn();

  float voltage = analogRead(SENSE_PIN) * (vcc / 1023.0);
  /* The output of the LM 35 is 10 mV (0.01 volts) per degree Celsius.
     Multiplying the analogRead voltage by 100 turns 0.251 to 25.1. */
  return voltage*100;
}

#define EEPROM_CHECK_TIME 60000
// #define SAVE_HI_LO_ON_STARTUP
#ifdef SAVE_HI_LO_ON_STARTUP
  unsigned long lastEepromSave = millis() - EEPROM_CHECK_TIME;
#else
  unsigned long lastEepromSave = millis();
#endif
// #define SHOW_EEPROM_STATUS // show eeprom save/yep/nope messages

void loop() {
  current = millis();
  if ((current > (timer + sensorPollTime)) || timer == 0) {
    float temperature = currentTemperatureReading();

    // Add current reading to temperature array
    averageTempPosition++;
    if (averageTempPosition >= AVERAGE_COUNT) {
      averageTempPosition = 0;
    }
    temperatures[averageTempPosition] = temperature;

    // Average all
    float averageTemperatureTotal = 0;
    for (uint8_t i = 0; i < AVERAGE_COUNT;i++) {
      averageTemperatureTotal += temperatures[i];
    }
    float averageTemperature = averageTemperatureTotal / AVERAGE_COUNT;

    #ifdef ENABLE_SERIAL
      Serial.print(F("VCC: "));
      Serial.print(vcc);
      Serial.print(F(" Instantaneous Temp: "));
      Serial.print(temperature);
      Serial.print(F(", Average Temp: "));
      Serial.print(averageTemperature);
      Serial.print(F("C"));
    #endif

    if (previousAverageTemperature == averageTemperature) {
      sensorPollTime = NORMAL_SENSOR_POLL_TIME;
    } else {
      sensorPollTime = FAST_SENSOR_POLL_TIME;
    }

    previousAverageTemperature = averageTemperature;

    if (averageTemperature > settings.highestTemp)
      settings.highestTemp = averageTemperature;

    if (averageTemperature < settings.lowestTemp)
      settings.lowestTemp = averageTemperature;

    if (current > (lastEepromSave + EEPROM_CHECK_TIME)) {
      #ifdef SHOW_EEPROM_STATUS
        setCharsAndWait("SAVE", 100);
      #endif
      Settings existingSettings;
      EEPROM.get(0, existingSettings);
      if ((settings.highestTemp != existingSettings.highestTemp) ||
          (settings.lowestTemp != existingSettings.lowestTemp)) {
        #ifdef SHOW_EEPROM_STATUS
          setCharsAndWait("YEP", 250);
        #endif
        EEPROM.put(0, settings);
      } else {
        #ifdef SHOW_EEPROM_STATUS
          setCharsAndWait("NOPE", 250);
        #endif
      }
      lastEepromSave = current;
    }

    #ifdef TEST_PATTERN
      sevseg.setSegments(testData);
    #else
      sevseg.setNumber(convertedTemperature(averageTemperature), 1);
    #endif

    timer = current;

    vccReadCounter--;
    if (vccReadCounter <= 0) {
      #ifdef ENABLE_SERIAL
        Serial.println("Updating VCC");
      #endif
      vcc = readVcc();
      vccReadCounter = VCC_READ_COUNT;
    }

  } else {
    // place delay this way so we don't get display flicker when above runs
    delay(LOOP_DELAY);
  }

  if (ledCounter > 0) {
    ledCounter--;
  }

  if ((ledCounter <= 0) && (ledState == HIGH)) {
    ledOff();
  }

  if (digitalRead(DISPLAY_PIN) == HIGH) {
    showHiAndLo();
  }

  sevseg.refreshDisplay(); // Must run repeatedly
}

float convertedTemperature(float celcius) {
  if (scale == FAHRENHEIT) {
    float fahrenheit = (celcius * 1.8) + 32;
    #ifdef ENABLE_SERIAL
      Serial.print(F(" "));
      Serial.print(fahrenheit);
      Serial.print(F("F"));
      Serial.println("");
    #endif
    return fahrenheit;
  } else {
    #ifdef ENABLE_SERIAL
      Serial.println("");
    #endif
    return celcius;
  }

}
