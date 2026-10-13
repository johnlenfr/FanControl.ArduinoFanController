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
 * OLED SSD1306 128x64 I2C A4 SDA / A5 SCL; navigation switch D4 to GND.
 * External +12V voltage monitor: 27k (+12V to sense) / 10k (sense to GND),
 * sense via 47k series resistor to A0, 100nF from A0 to GND.
 */
// 1 = OLED SSD1306 + bouton D4 + menu ; 0 = version sans ecran.
// Mettre cette valeur a 0 pour ne pas compiler Wire/U8x8 et l'interface OLED.
#define ENABLE_OLED 1

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#if ENABLE_OLED
#include <Wire.h>
#include <U8x8lib.h>
#endif
#include <EEPROM.h>
#include <avr/pgmspace.h>

const uint8_t FAN_COUNT = 5;
const uint8_t fanPins[FAN_COUNT] = {5, 6, 9, 10, 11};
const uint8_t tachPins[2] = {2, 3};
// Factory fail-safe: 50%, customizable in OLED menu and saved to EEPROM.
const uint8_t FACTORY_FAILSAFE_PERCENT = 50;
uint8_t failsafePercent = FACTORY_FAILSAFE_PERCENT;
uint8_t failsafePwm = 128;
const unsigned long COMMAND_TIMEOUT_MS = 5000UL;
const unsigned long TACH_WINDOW_MS = 1000UL;
const unsigned long MIN_TACH_US = 8000UL;
const uint8_t PULSES_PER_REV = 2;
const uint8_t SUPPLY_SENSE_PIN = A0;
// Calibrate ADC reference if measured Arduino 5V differs from 5.000V.
// Resistor divider: 27k top / 10k bottom; ADC input = supply * 10/37.
// A0 conversion reference, adjustable in the OLED menu (not physical VCC).
const uint16_t VREF_DEFAULT_MV = 5000;
const uint16_t VREF_MIN_MV = 4500;
const uint16_t VREF_MAX_MV = 5200;
const uint8_t VREF_STEP_MV = 10;
uint16_t adcReferenceMv = VREF_DEFAULT_MV;
const uint32_t DIVIDER_TOP_OHMS = 27000UL;
const uint32_t DIVIDER_BOTTOM_OHMS = 10000UL;
const unsigned long VOLTAGE_SAMPLE_MS = 500UL;
uint16_t supplyMillivolts = 0;
bool supplyReadingReady = false;
unsigned long lastVoltageSampleMs = 0;


#if ENABLE_OLED
// SSD1306 128x64 OLED via I2C: A4=SDA, A5=SCL.
// U8x8 text mode uses no full-screen framebuffer (important on ATmega328P).
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);
const uint8_t OLED_BUTTON_PIN = 4; // momentary switch between D4 and GND
const unsigned long BUTTON_DEBOUNCE_MS = 35UL;
const unsigned long BUTTON_LONG_MS = 900UL;
const unsigned long OLED_REFRESH_MS = 600UL;
const uint8_t OLED_PAGE_COUNT = 3;
const char FW_VERSION[] PROGMEM = "0.3.0-dev";
const unsigned long MENU_TIMEOUT_MS = 30000UL;

#endif

// Persistent settings. Byte-wise EEPROM.update() writes only on changes.
struct StoredSettings {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t version;
  uint8_t failsafe;     // percent
  uint8_t brightness;   // index into contrastTable
  uint8_t timeout;      // index into timeoutSeconds; 0xFFFFFFFF = always on
  uint8_t check;
};
// Version 2 appends the ADC reference; old v1 settings remain readable.
struct StoredSettingsV2 {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t version;
  uint8_t failsafe;
  uint8_t brightness;
  uint8_t timeout;
  uint8_t check;
  uint16_t vrefMv;
};
const uint8_t EEPROM_MAGIC1 = 0xF3;
const uint8_t EEPROM_MAGIC2 = 0x62;
const uint8_t EEPROM_FORMAT = 2;
const uint8_t failsafeChoices[] = {20, 30, 40, 50, 60, 70, 80, 90, 100};
const uint8_t brightnessChoices[] = {10, 25, 50, 75, 100};
const uint8_t contrastTable[] = {12, 40, 96, 170, 255};
const unsigned long timeoutSeconds[] PROGMEM = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 30, 90, 120,
  300, 900, 1800, 0xFFFFFFFFUL
};
const uint8_t TIMEOUT_COUNT = sizeof(timeoutSeconds) / sizeof(timeoutSeconds[0]);
uint8_t brightnessIndex = 3;
uint8_t timeoutIndex = TIMEOUT_COUNT - 1;

#if ENABLE_OLED
uint8_t oledPage = 0;
unsigned long lastOledRefreshMs = 0;
uint8_t oledRenderedPage = 255;
uint32_t oledRowHash[8] = {0};
bool oledDirty = true;
bool lastButtonRaw = HIGH;
bool buttonStable = HIGH;
bool buttonLongConsumed = false;
bool ignoreButtonUntilRelease = false;
unsigned long lastButtonChangeMs = 0;
unsigned long buttonPressedMs = 0;
unsigned long lastUserInteractionMs = 0;
bool oledAwake = true;
unsigned long lastSerialCommandMs = 0;
bool serialCommandSeen = false;

enum UiMode : uint8_t { UI_PAGES, UI_MENU, UI_OLED_MENU, UI_CHOICES, UI_FW_INFO };
enum ChoiceKind : uint8_t { CHOICE_FAILSAFE, CHOICE_BRIGHTNESS, CHOICE_TIMEOUT, CHOICE_VREF };
UiMode uiMode = UI_PAGES;
ChoiceKind choiceKind = CHOICE_FAILSAFE;
uint8_t menuIndex = 0;
uint8_t oledMenuIndex = 0;
uint8_t choiceIndex = 0;
uint16_t vrefChoiceMv = VREF_DEFAULT_MV;

#endif

uint8_t settingsCheck(const StoredSettings& s) {
  return (uint8_t)(0xA7 ^ s.magic1 ^ s.magic2 ^ s.version ^
                   s.failsafe ^ s.brightness ^ s.timeout);
}
uint8_t settingsCheckV2(const StoredSettingsV2& s) {
  return (uint8_t)(0xA7 ^ s.magic1 ^ s.magic2 ^ s.version ^
                   s.failsafe ^ s.brightness ^ s.timeout ^
                   (uint8_t)s.vrefMv ^ (uint8_t)(s.vrefMv >> 8));
}
bool settingsValuesValid(uint8_t failsafe, uint8_t brightness, uint8_t timeout) {
  return failsafe >= 20 && failsafe <= 100 &&
         brightness < sizeof(brightnessChoices) && timeout < TIMEOUT_COUNT;
}
void setFailsafePercent(uint8_t percent) {
  failsafePercent = percent;
  failsafePwm = (uint8_t)((percent * 255UL + 50UL) / 100UL);
}
void loadSettings() {
  StoredSettings s;
  EEPROM.get(0, s);
  if (s.magic1 == EEPROM_MAGIC1 && s.magic2 == EEPROM_MAGIC2 &&
      settingsValuesValid(s.failsafe, s.brightness, s.timeout)) {
    if (s.version == EEPROM_FORMAT) {
      StoredSettingsV2 v2;
      EEPROM.get(0, v2);
      if (v2.check == settingsCheckV2(v2) &&
          v2.vrefMv >= VREF_MIN_MV && v2.vrefMv <= VREF_MAX_MV &&
          (v2.vrefMv % VREF_STEP_MV) == 0) {
        setFailsafePercent(v2.failsafe);
        brightnessIndex = v2.brightness;
        timeoutIndex = v2.timeout;
        adcReferenceMv = v2.vrefMv;
        return;
      }
    } else if (s.version == 1 && s.check == settingsCheck(s)) {
      // One-time migration: preserve previous failsafe/OLED settings.
      setFailsafePercent(s.failsafe);
      brightnessIndex = s.brightness;
      timeoutIndex = s.timeout;
      adcReferenceMv = VREF_DEFAULT_MV;
      return;
    }
  }
  setFailsafePercent(FACTORY_FAILSAFE_PERCENT);
  brightnessIndex = 3;
  timeoutIndex = TIMEOUT_COUNT - 1;
  adcReferenceMv = VREF_DEFAULT_MV;
}
void saveSettings() {
  StoredSettingsV2 s = {EEPROM_MAGIC1, EEPROM_MAGIC2, EEPROM_FORMAT,
                        failsafePercent, brightnessIndex, timeoutIndex, 0,
                        adcReferenceMv};
  s.check = settingsCheckV2(s);
  const uint8_t* p = (const uint8_t*)&s;
  // Write version LAST: interrupted writes cannot validate as v2.
  for (uint8_t i = 0; i < sizeof(s); ++i) {
    if (i != 2) EEPROM.update(i, p[i]);
  }
  EEPROM.update(2, s.version);
}
#if ENABLE_OLED
void forceOledRedraw() {
  oledDirty = true;
  lastOledRefreshMs = millis() - OLED_REFRESH_MS;
}
void applyBrightness() {
  oled.setContrast(contrastTable[brightnessIndex]);
}
void wakeOled() {
  if (!oledAwake) {
    oled.setPowerSave(0);
    oledAwake = true;
    forceOledRedraw();
  }
  lastUserInteractionMs = millis();
}
void updateOledPower() {
  if (!oledAwake || pgm_read_dword(&timeoutSeconds[timeoutIndex]) == 0xFFFFFFFFUL) return;
  unsigned long timeoutMs = pgm_read_dword(&timeoutSeconds[timeoutIndex]) * 1000UL;
  if ((unsigned long)(millis() - lastUserInteractionMs) >= timeoutMs) {
    oled.setPowerSave(1);
    oledAwake = false;
  }
}

#endif

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
bool rpmWindowReady = false;
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

// One short ADC conversion every 500 ms; no delay() or averaging loop.
// The 100nF capacitor on A0 filters the divider/47k series resistance.
void updateSupplyVoltage() {
  unsigned long now = millis();
  if (supplyReadingReady &&
      (unsigned long)(now - lastVoltageSampleMs) < VOLTAGE_SAMPLE_MS) return;
  lastVoltageSampleMs = now;
  uint16_t adc = analogRead(SUPPLY_SENSE_PIN);
  // 32-bit integer calculation avoids float code and SRAM overhead.
  // 5000 mV reference gives 18500 mV full scale; adjustable via EEPROM.
  uint32_t fullScaleMv = (uint32_t)adcReferenceMv *
      (DIVIDER_TOP_OHMS + DIVIDER_BOTTOM_OHMS) / DIVIDER_BOTTOM_OHMS;
  supplyMillivolts = (uint16_t)(((uint32_t)adc * fullScaleMv + 511UL) / 1023UL);
  supplyReadingReady = true;
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
  rpmWindowReady = true;
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

#if ENABLE_OLED
// Nonblocking button handling. Short press is dispatched ON RELEASE.
// Long press fires after 900 ms, once per physical press.
uint8_t choiceCount() {
  if (choiceKind == CHOICE_FAILSAFE) return sizeof(failsafeChoices);
  if (choiceKind == CHOICE_BRIGHTNESS) return sizeof(brightnessChoices);
  if (choiceKind == CHOICE_VREF) return (VREF_MAX_MV - VREF_MIN_MV) / VREF_STEP_MV + 1;
  return TIMEOUT_COUNT;
}
void shortPress() {
  lastUserInteractionMs = millis();
  switch (uiMode) {
    case UI_PAGES: oledPage = (uint8_t)((oledPage + 1) % OLED_PAGE_COUNT); break;
    case UI_MENU: menuIndex = (uint8_t)((menuIndex + 1) % 5); break;
    case UI_OLED_MENU: oledMenuIndex = (uint8_t)((oledMenuIndex + 1) % 3); break;
    case UI_CHOICES:
      if (choiceKind == CHOICE_VREF) {
        vrefChoiceMv += VREF_STEP_MV;
        if (vrefChoiceMv > VREF_MAX_MV) vrefChoiceMv = VREF_MIN_MV;
      } else choiceIndex = (uint8_t)((choiceIndex + 1) % choiceCount());
      break;
    case UI_FW_INFO: uiMode = UI_MENU; break;
  }
  // Only redraw rows whose text changes; no clearing on selection.
}
void beginChoices(ChoiceKind kind) {
  choiceKind = kind;
  uiMode = UI_CHOICES;
  choiceIndex = 0;
  if (kind == CHOICE_VREF) {
    vrefChoiceMv = adcReferenceMv;
  } else if (kind == CHOICE_FAILSAFE) {
    for (uint8_t i = 0; i < choiceCount(); ++i)
      if (failsafeChoices[i] == failsafePercent) choiceIndex = i;
  } else if (kind == CHOICE_BRIGHTNESS) choiceIndex = brightnessIndex;
  else choiceIndex = timeoutIndex;
}
void applyPWM(uint8_t channel, uint8_t value);
void longPress() {
  lastUserInteractionMs = millis();
  if (uiMode == UI_PAGES) {
    uiMode = UI_MENU;
    menuIndex = 0;
  } else if (uiMode == UI_MENU) {
    switch (menuIndex) {
      case 0: beginChoices(CHOICE_FAILSAFE); break;
      case 1: uiMode = UI_OLED_MENU; oledMenuIndex = 0; break;
      case 2: beginChoices(CHOICE_VREF); break;
      case 3: uiMode = UI_FW_INFO; break;
      default: uiMode = UI_PAGES; break;
    }
  } else if (uiMode == UI_OLED_MENU) {
    if (oledMenuIndex == 0) beginChoices(CHOICE_BRIGHTNESS);
    else if (oledMenuIndex == 1) beginChoices(CHOICE_TIMEOUT);
    else uiMode = UI_MENU;
  } else if (uiMode == UI_CHOICES) {
    if (choiceKind == CHOICE_FAILSAFE) {
      uint8_t previous = failsafePwm;
      setFailsafePercent(failsafeChoices[choiceIndex]);
      // If a channel is already in fallback, apply the new value immediately.
      for (uint8_t i = 0; i < FAN_COUNT; ++i) {
        if (pwmValues[i] == previous &&
            (unsigned long)(millis() - lastCommandMs[i]) >= COMMAND_TIMEOUT_MS)
          applyPWM(i, failsafePwm);
      }
      uiMode = UI_MENU;
    } else if (choiceKind == CHOICE_BRIGHTNESS) {
      brightnessIndex = choiceIndex;
      applyBrightness();
      uiMode = UI_OLED_MENU;
    } else if (choiceKind == CHOICE_TIMEOUT) {
      timeoutIndex = choiceIndex;
      uiMode = UI_OLED_MENU;
    } else {
      adcReferenceMv = vrefChoiceMv;
      supplyReadingReady = false;  // Recompute supply with the new calibration.
      uiMode = UI_MENU;
    }
    saveSettings();
  } else if (uiMode == UI_FW_INFO) uiMode = UI_MENU;
  // Changed text is detected by the row hash cache.
}
void updateOledButton() {
  bool raw = digitalRead(OLED_BUTTON_PIN);
  unsigned long now = millis();
  if (raw != lastButtonRaw) {
    lastButtonRaw = raw;
    lastButtonChangeMs = now;
  }
  if (raw != buttonStable &&
      (unsigned long)(now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS) {
    buttonStable = raw;
    if (raw == LOW) {
      buttonPressedMs = now;
      buttonLongConsumed = false;
      if (!oledAwake) {
        wakeOled();
        ignoreButtonUntilRelease = true; // Wake only: don't also navigate.
      } else lastUserInteractionMs = now;
    } else {
      if (!buttonLongConsumed && !ignoreButtonUntilRelease) shortPress();
      ignoreButtonUntilRelease = false;
    }
  }
  if (buttonStable == LOW && !buttonLongConsumed && !ignoreButtonUntilRelease &&
      (unsigned long)(now - buttonPressedMs) >= BUTTON_LONG_MS) {
    buttonLongConsumed = true;
    longPress();
  }
}

// Approximate free SRAM at this point: distance between heap and stack.
// AVR ATmega328P only; no heap allocation or large screen buffer needed.
extern unsigned int __heap_start;
extern void *__brkval;
int freeRam() {
  uint8_t stackMarker;
  return (int)(&stackMarker) -
         (int)(__brkval ? __brkval : (void *)&__heap_start);
}

// U8x8: 16x8 characters. 32 bytes of hashes replace a 136-byte row cache.
// Do not clear the whole OLED on page or menu changes.
void drawOledRow(uint8_t row, const char* content) {
  char padded[17];
  uint32_t hash = 2166136261UL; // FNV-1a 32-bit
  uint8_t n = 0;
  while (n < 16 && content[n] != '\0') { padded[n] = content[n]; ++n; }
  while (n < 16) padded[n++] = ' ';
  padded[16] = '\0';
  for (uint8_t i = 0; i < 16; ++i) {
    hash ^= (uint8_t)padded[i];
    hash *= 16777619UL;
  }
  if (oledDirty || oledRowHash[row] != hash) {
    oled.drawString(0, row, padded);
    oledRowHash[row] = hash;
  }
}
void drawOledFlash(uint8_t row, const __FlashStringHelper* text) {
  char buf[17];
  strncpy_P(buf, (PGM_P)text, 16);
  buf[16] = '\0';
  drawOledRow(row, buf);
}
#define OLED_TEXT(row, literal) drawOledFlash(row, F(literal))

void drawOledTempRow(uint8_t row, uint8_t index) {
  char line[17];
  if (tempValid[index]) {
    char celsius[9];
    dtostrf(tempC[index], 1, 1, celsius);
    snprintf(line, sizeof(line), "TEMP %u: %s C", (unsigned)(index + 1), celsius);
  } else snprintf(line, sizeof(line), "TEMP %u: --", (unsigned)(index + 1));
  drawOledRow(row, line);
}
void drawTimeoutChoice(uint8_t row, uint8_t index) {
  char text[17];
  unsigned long secs = pgm_read_dword(&timeoutSeconds[index]);
  if (secs == 0xFFFFFFFFUL) strcpy(text, "Always ON");
  else if (secs >= 60) snprintf(text, sizeof(text), "%lu min", secs / 60UL);
  else snprintf(text, sizeof(text), "%lu s", secs);
  drawOledRow(row, text);
}

void updateMenuTimeout() {
  if (uiMode != UI_PAGES &&
      (unsigned long)(millis() - lastUserInteractionMs) >= MENU_TIMEOUT_MS) {
    uiMode = UI_PAGES;
    // Unconfirmed choices are discarded. EEPROM is not modified.
  }
}

void updateOled() {
  if (!oledAwake) return;
  unsigned long now = millis();
  if ((unsigned long)(now - lastOledRefreshMs) < OLED_REFRESH_MS) return;
  lastOledRefreshMs = now;
  char line[17];
  // Redraw all 8 rows logically, including blanks. Only changed rows
  // are actually sent over I2C. This prevents stale text between views.
  if (uiMode == UI_MENU) {
    OLED_TEXT(0, "MENU CONFIG");
    for (uint8_t i = 0; i < 5; ++i) {
      const __FlashStringHelper* label;
      switch (i) {
        case 0: label = F("PWM FAILSAFE"); break;
        case 1: label = F("OLED SETTINGS"); break;
        case 2: label = F("ADC VREF"); break;
        case 3: label = F("FW INFOS"); break;
        default: label = F("EXIT"); break;
      }
      char text[17];
      strncpy_P(text + 2, (PGM_P)label, 14);
      text[16] = '\0';
      text[0] = menuIndex == i ? '>' : ' ';
      text[1] = ' ';
      drawOledRow(i + 1, text);
    }
    // Write each row only once; previously row 1 was blanked then redrawn,
    // which caused visible flicker even with the row-hash cache.
    OLED_TEXT(6, "SHORT: SELECT");
    OLED_TEXT(7, "LONG: ENTER");
  } else if (uiMode == UI_OLED_MENU) {
    OLED_TEXT(0, " OLED SETTINGS ");
    OLED_TEXT(1, "");
    for (uint8_t i = 0; i < 3; ++i) {
      const __FlashStringHelper* label = i == 0 ? F("BRIGHTNESS") : (i == 1 ? F("AUTO OFF") : F("BACK"));
      char text[17];
      strncpy_P(text + 2, (PGM_P)label, 14);
      text[16] = '\0'; text[0] = oledMenuIndex == i ? '>' : ' '; text[1] = ' ';
      drawOledRow(i + 2, text);
    }
    OLED_TEXT(5, "");
    OLED_TEXT(6, "SHORT: SELECT");
    OLED_TEXT(7, "LONG: ENTER");
  } else if (uiMode == UI_FW_INFO) {
    OLED_TEXT(0, " FIRMWARE INFO  ");
    OLED_TEXT(1, "");
    char version[14];
    strcpy_P(version, FW_VERSION);
    snprintf(line, sizeof(line), "FW: %s", version);
    drawOledRow(2, line);
    OLED_TEXT(3, "UNO/NANO 16 MHz");
    OLED_TEXT(4, "5 PWM / 2 RPM");
    OLED_TEXT(5, "2 DS18B20");
    OLED_TEXT(6, "");
    OLED_TEXT(7, "LONG: BACK");
  } else if (uiMode == UI_CHOICES) {
    if (choiceKind == CHOICE_FAILSAFE) {
      OLED_TEXT(0, "  PWM FAILSAFE  ");
      snprintf(line, sizeof(line), "> %u %%", failsafeChoices[choiceIndex]);
      drawOledRow(3, line);
    } else if (choiceKind == CHOICE_BRIGHTNESS) {
      OLED_TEXT(0, "OLED BRIGHTNESS");
      snprintf(line, sizeof(line), "> %u %%", brightnessChoices[choiceIndex]);
      drawOledRow(3, line);
    } else if (choiceKind == CHOICE_TIMEOUT) {
      OLED_TEXT(0, " OLED AUTO OFF  ");
      drawTimeoutChoice(3, choiceIndex);
    } else {
      OLED_TEXT(0, "    ADC VREF    ");
      snprintf(line, sizeof(line), "> %u.%03u V",
               (unsigned)(vrefChoiceMv / 1000U),
               (unsigned)(vrefChoiceMv % 1000U));
      drawOledRow(3, line);
    }
    OLED_TEXT(1, ""); OLED_TEXT(2, "");
    OLED_TEXT(4, ""); OLED_TEXT(5, "");
    OLED_TEXT(6, "SHORT: SELECT");
    OLED_TEXT(7, "LONG: CONFIRM");
  } else if (oledPage == 0) {
    OLED_TEXT(0, "FANS STATUS  1/3");
    OLED_TEXT(1, "");
    for (uint8_t i = 0; i < FAN_COUNT; ++i) {
      // Round PWM duty cycle to the nearest whole percent (0..100).
      // Integer-only calculation keeps AVR code small.
      const unsigned percent = (unsigned)((pwmValues[i] * 100UL + 127UL) / 255UL);
      if (i < 2) {
        if (rpmWindowReady) {
          const unsigned long measuredRpm = (unsigned long)(rpm[i] + 0.5f);
          // Maximum of 16 characters on the 128x64 U8x8 text display.
          if (measuredRpm <= 9999UL) {
            snprintf(line, sizeof(line), "%u: %3u%% %4luRPM", (unsigned)(i + 1), percent, measuredRpm);
          } else {
            snprintf(line, sizeof(line), "%u: %3u%% %5lu", (unsigned)(i + 1), percent, measuredRpm);
          }
        } else {
          snprintf(line, sizeof(line), "%u: %3u%% %4sRPM", (unsigned)(i + 1), percent, "--");
        }
      } else {
        snprintf(line, sizeof(line), "%u: %3u%%", (unsigned)(i + 1), percent);
      }
      drawOledRow(i + 2, line);
    }
    OLED_TEXT(7, "MENU: LONG PRESS");
  } else if (oledPage == 1) {
    OLED_TEXT(0, "TEMPERATURES 2/3"); OLED_TEXT(1, "");
    drawOledTempRow(2, 0); OLED_TEXT(3, ""); drawOledTempRow(4, 1);
    OLED_TEXT(5, ""); OLED_TEXT(6, "");
    OLED_TEXT(7, "MENU: LONG PRESS");
  } else {
    OLED_TEXT(0, "SETTINGS     3/3"); OLED_TEXT(1, "");
    bool recentRx = serialCommandSeen &&
      (unsigned long)(now - lastSerialCommandMs) <= COMMAND_TIMEOUT_MS;
    OLED_TEXT(2, "");
    drawOledFlash(2, recentRx ? F("SERIAL: ACTIVE") : F("SERIAL: NO RX"));
    if (serialCommandSeen) {
      unsigned long seconds = (now - lastSerialCommandMs) / 1000UL;
      if (seconds > 999UL) seconds = 999UL;
      snprintf(line, sizeof(line), "LAST RX: %lus", seconds);
      drawOledRow(3, line);
    } else OLED_TEXT(3, "LAST RX: --");
    snprintf(line, sizeof(line), "PWM SAFE: %u%%", failsafePercent);
    drawOledRow(4, line);
    if (supplyReadingReady) {
      // One decimal digit; -- before the first conversion.
      uint16_t tenths = (uint16_t)((supplyMillivolts + 50U) / 100U);
      snprintf(line, sizeof(line), "12V INPUT: %u.%uV",
               (unsigned)(tenths / 10U), (unsigned)(tenths % 10U));
      drawOledRow(5, line);
    } else {
      OLED_TEXT(5, "12V INPUT: --");
    }
    // Diagnostic: estimated free SRAM between heap and stack.
    // Updated only with the usual OLED refresh, without delays.
    const int availableRam = freeRam();
    snprintf(line, sizeof(line), "FREE RAM: %dB", availableRam);
    drawOledRow(6, line);
    OLED_TEXT(7, "MENU: LONG PRESS");
  }
  oledDirty = false;
}

#endif

void processCommand(char* line) {
  // Tracks received commands, not whether a USB cable is electrically present.
#if ENABLE_OLED
  lastSerialCommandMs = millis();
  serialCommandSeen = true;
#endif
  if (strcmp(line, "PING") == 0) {
    Serial.println(F("PONG"));
  } else if (strcmp(line, "NUM_FANS") == 0) {
    Serial.println(FAN_COUNT);
  } else if (strcmp(line, "FAN_NAMES") == 0) {
    Serial.println(F("Arduino FAN 1 PWM"));
    Serial.println(F("Arduino FAN 2 PWM"));
    Serial.println(F("Arduino FAN 3 PWM"));
    Serial.println(F("Arduino FAN 4 PWM"));
    Serial.println(F("Arduino FAN 5 PWM"));
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
  loadSettings();
#if ENABLE_OLED
  pinMode(OLED_BUTTON_PIN, INPUT_PULLUP);
  Wire.begin();
  oled.begin();
  oled.setPowerSave(0);
  oled.setFont(u8x8_font_chroma48medium8_r);
  applyBrightness();
  lastUserInteractionMs = millis();
  oled.drawString(0, 0, "FanControl Nano");
#endif
  for (uint8_t i = 0; i < FAN_COUNT; ++i) {
    pinMode(fanPins[i], OUTPUT);
    // Full speed during setup, before software starts controlling.
    digitalWrite(fanPins[i], LOW);
    applyPWM(i, failsafePwm);
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
#if ENABLE_OLED
  updateOledButton();
#endif
  unsigned long now = millis();
  for (uint8_t i = 0; i < FAN_COUNT; ++i) {
    if ((unsigned long)(now - lastCommandMs[i]) > COMMAND_TIMEOUT_MS) {
      if (pwmValues[i] != failsafePwm) applyPWM(i, failsafePwm);
      lastCommandMs[i] = now;
    }
  }
  updateSupplyVoltage();
  // OLED last: priority to serial commands, RPM and fail-safe.
#if ENABLE_OLED
  updateMenuTimeout();
  updateOledPower();
  updateOled();
#endif
}
