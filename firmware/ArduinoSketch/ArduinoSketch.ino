/*
 * FanControl Arduino Uno / Nano ATmega328P (16 MHz)
 * Five independent PWM controls, two TACH inputs.
 * PWM fan header pin 4 must be driven via one BC337 open collector PER FAN.
 * BC337 base via 1k; emitter to common ground; collector to fan PWM.
 * 100k base-emitter resistor optional.
 * FAN 1..5: PWM D5 D6 D9 D10 D11 (~980,980,490,490,490 Hz)
 * TACH FAN 1 -> D2; FAN 2 -> D3; individual 4.7k pull-ups to 5V.
 * Fan power is EXTERNAL 12V. Connect grounds together.
 * USB serial protocol: PING, NUM_FANS, FAN_NAMES,
 * SET_FAN:<1..5>:<0..255>, GET_RPM:<1..5>.
 * Only FAN 1 and 2 have tach sensors. Other GET_RPM returns -1.
 * DS18B20 TEMP 1: A1, TEMP 2: A2; individual 4.7k pull-ups to 5V.
 * GET_TEMP:1/2 -> TEMP:1/2:<Celsius> or TEMP:1/2:ERR.
 */
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const uint8_t FAN_COUNT = 5;
const uint8_t fanPins[FAN_COUNT] = {5, 6, 9, 10, 11};
const char* const fanNames[FAN_COUNT] = {
  "Ardui1-Output5", "Ardui2-Output6", "Ardui3-Output9",
  "Ardui4-Output10", "Ardui5-Output11"
};
const uint8_t tachPins[2] = {2, 3};
const uint8_t DEFAULT_PWM = 128; // 50 percent (failover)
const unsigned long COMMAND_TIMEOUT_MS = 5000UL;
const unsigned long TACH_WINDOW_MS = 1000UL;
const unsigned long MIN_TACH_US = 8000UL;
const uint8_t PULSES_PER_REV = 2;

// Two independent powered (three-wire) DS18B20 buses.
const uint8_t TEMP_COUNT = 2;
const uint8_t tempPins[TEMP_COUNT] = {A1, A2};
OneWire oneWireTemp1(A1);
OneWire oneWireTemp2(A2);
DallasTemperature tempSensor1(&oneWireTemp1);
DallasTemperature tempSensor2(&oneWireTemp2);
DallasTemperature* const tempSensors[TEMP_COUNT] = {&tempSensor1, &tempSensor2};
const unsigned long TEMP_CONVERSION_MS = 750UL;  // 12-bit DS18B20 max conversion time
const unsigned long TEMP_SAMPLE_INTERVAL_MS = 1000UL;
float tempC[TEMP_COUNT] = {0.0f, 0.0f};
bool tempValid[TEMP_COUNT] = {false, false};
bool tempConversionPending = false;
unsigned long tempConversionStartedMs = 0;
unsigned long lastTempSampleMs = 0;

uint8_t pwmValues[FAN_COUNT];
unsigned long lastCommandMs[FAN_COUNT];
volatile unsigned long tachAccepted[2] = {0, 0};
volatile unsigned long tachRejected[2] = {0, 0};
volatile unsigned long lastTachUs[2] = {0, 0};
volatile bool firstTach[2] = {true, true};
float rpm[2] = {0.0f, 0.0f};
unsigned long lastRpmMs = 0;
char commandBuffer[48];
uint8_t commandLength = 0;

void tachEdge(uint8_t channel) {
  unsigned long now = micros();
  if (firstTach[channel] ||
      (unsigned long)(now - lastTachUs[channel]) >= MIN_TACH_US) {
    ++tachAccepted[channel];
    lastTachUs[channel] = now;
    firstTach[channel] = false;
  } else {
    ++tachRejected[channel];
  }
}
void tach1ISR() { tachEdge(0); }
void tach2ISR() { tachEdge(1); }

void applyPWM(uint8_t channel, uint8_t value) {
  pwmValues[channel] = value;
  // BC337 INVERTS the Arduino pin: 255 fan -> 0 Arduino, 0 fan -> 255 Arduino.
  analogWrite(fanPins[channel], (uint8_t)(255 - value));
}

void updateRpm() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastRpmMs;
  if (elapsed < TACH_WINDOW_MS) return;
  unsigned long counts[2];
  noInterrupts();
  counts[0] = tachAccepted[0];
  counts[1] = tachAccepted[1];
  tachAccepted[0] = 0;
  tachAccepted[1] = 0;
  // Keep timestamps and first-pulse state across windows to avoid boundary artifacts.
  interrupts();
  for (uint8_t i = 0; i < 2; ++i)
    rpm[i] = (float)((double)counts[i] * 60000.0 / ((double)elapsed * PULSES_PER_REV));
  lastRpmMs = now;
}

void startTemperatureConversion(unsigned long now) {
  // Request both conversions without holding up serial, RPM or PWM safety code.
  for (uint8_t i = 0; i < TEMP_COUNT; ++i)
    tempSensors[i]->requestTemperatures();
  tempConversionStartedMs = now;
  tempConversionPending = true;
}

void updateTemperatures() {
  unsigned long now = millis();
  if (tempConversionPending) {
    if ((unsigned long)(now - tempConversionStartedMs) < TEMP_CONVERSION_MS)
      return;
    for (uint8_t i = 0; i < TEMP_COUNT; ++i) {
      float value = tempSensors[i]->getTempCByIndex(0);
      // DallasTemperature reports -127 C on disconnect and 85 C may be
      // a power-on default rather than a completed conversion.
      tempValid[i] = value != DEVICE_DISCONNECTED_C && value != 85.0f &&
                     value >= -55.0f && value <= 125.0f;
      if (tempValid[i]) tempC[i] = value;
    }
    tempConversionPending = false;
    lastTempSampleMs = now;
  }
  if (!tempConversionPending &&
      (unsigned long)(now - lastTempSampleMs) >= TEMP_SAMPLE_INTERVAL_MS)
    startTemperatureConversion(now);
}

void processCommand(char* line) {
  if (strcmp(line, "PING") == 0) {
    Serial.println(F("PONG"));
  } else if (strcmp(line, "NUM_FANS") == 0) {
    Serial.println(FAN_COUNT);
  } else if (strcmp(line, "FAN_NAMES") == 0) {
    for (uint8_t i = 0; i < FAN_COUNT; ++i) Serial.println(fanNames[i]);
  } else if (strncmp(line, "SET_FAN:", 8) == 0) {
    char* endIndex;
    long index = strtol(line + 8, &endIndex, 10);
    if (*endIndex != ':') return;
    char* endValue;
    long val = strtol(endIndex + 1, &endValue, 10);
    if (*endValue != '\0' || index < 1 || index > FAN_COUNT || val < 0 || val > 255) return;
    applyPWM((uint8_t)(index - 1), (uint8_t)val);
    lastCommandMs[index - 1] = millis();
  } else if (strncmp(line, "GET_RPM:", 8) == 0) {
    char* endIndex;
    long index = strtol(line + 8, &endIndex, 10);
    if (*endIndex != '\0' || index < 1 || index > FAN_COUNT) return;
    updateRpm();
    Serial.print(F("RPM:"));
    Serial.print(index);
    Serial.print(':');
    if (index <= 2) Serial.println(rpm[index - 1], 1);
    else Serial.println(-1);
  } else if (strncmp(line, "GET_TEMP:", 9) == 0) {
    char* endIndex;
    long index = strtol(line + 9, &endIndex, 10);
    if (*endIndex != '\0' || index < 1 || index > TEMP_COUNT) return;
    updateTemperatures();
    Serial.print(F("TEMP:"));
    Serial.print(index);
    Serial.print(':');
    if (tempValid[index - 1]) Serial.println(tempC[index - 1], 2);
    else Serial.println(F("ERR"));
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < FAN_COUNT; ++i) {
    pinMode(fanPins[i], OUTPUT);
    // Full speed during setup, before software starts controlling.
    digitalWrite(fanPins[i], LOW);
    applyPWM(i, DEFAULT_PWM);
    lastCommandMs[i] = millis();
  }
  for (uint8_t i = 0; i < 2; ++i) pinMode(tachPins[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tachPins[0]), tach1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(tachPins[1]), tach2ISR, FALLING);
  lastRpmMs = millis();
  for (uint8_t i = 0; i < TEMP_COUNT; ++i) {
    tempSensors[i]->begin();
    tempSensors[i]->setResolution(12);
    tempSensors[i]->setWaitForConversion(false);
  }
  startTemperatureConversion(millis());
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (commandLength != 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;  // Discard oversized command.
    }
  }
  updateRpm();
  updateTemperatures();
  unsigned long now = millis();
  for (uint8_t i = 0; i < FAN_COUNT; ++i) {
    if ((unsigned long)(now - lastCommandMs[i]) > COMMAND_TIMEOUT_MS) {
      if (pwmValues[i] != DEFAULT_PWM) applyPWM(i, DEFAULT_PWM);
      lastCommandMs[i] = now;
    }
  }
}
