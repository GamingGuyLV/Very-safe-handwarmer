#include <SPI.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// =====================================================
// PIN ASSIGNMENTS
// =====================================================
const uint8_t FCU_PIN = 3;
const uint8_t MOSFET_PIN = 2;

const uint8_t TC1_CS_PIN = 17;
const uint8_t TC2_CS_PIN = 20;

// MAX31855 SPI:
// SCK  = D18
// MISO = D16
// CS1  = D17
// CS2  = D20

// =====================================================
// PWM SETTINGS
// =====================================================
const unsigned long PWM_TIMEOUT_US = 25000;
const unsigned int PWM_VALID_MIN = 950;
const unsigned int PWM_VALID_MAX = 2050;
const unsigned int PWM_OFF_MAX = 1100;
const unsigned int PWM_INPUT_MIN = 1000;
const unsigned int PWM_INPUT_MAX = 2000;

// =====================================================
// TEMPERATURE SETTINGS
// =====================================================
const float MIN_TARGET_C = 50.0;
const float MAX_TARGET_C = 200.0;

const float PWM_TARGET_MIN_C = 60.0;
const float PWM_TARGET_MAX_C = 200.0;

const float HARD_CUTOFF_C = 210.0;
const float HYSTERESIS_C = 3.0;

// =====================================================
// OPERATING MODES
// =====================================================
enum OperatingMode {
  MODE_DEBUG,
  MODE_TEST,
  MODE_DEPLOY
};

OperatingMode currentMode = MODE_DEPLOY;

// =====================================================
// SERIAL OUTPUT
// =====================================================
enum SerialOutputMode {
  OUTPUT_MONITOR,
  OUTPUT_PLOTTER
};

SerialOutputMode serialOutputMode = OUTPUT_MONITOR;

// =====================================================
// GLOBAL STATE
// =====================================================
bool heaterOn = false;
bool requestedHeaterState = false;

float manualTargetC = 200.0;
bool pwmFailActive = false;

float tc1TempC = 0.0;
float tc2TempC = 0.0;

bool tc1Valid = false;
bool tc2Valid = false;

unsigned long lastStatusPrint = 0;

char consoleBuffer[40];
uint8_t consoleIndex = 0;

// =====================================================
// HEATER OFF
// =====================================================
void heaterOff() {
  heaterOn = false;
  digitalWrite(MOSFET_PIN, LOW);
}

// =====================================================
// SAFE HEATER OUTPUT
// =====================================================
void setHeater(bool state) {

  if (currentMode == MODE_DEBUG) {
    heaterOff();
    return;
  }

  if (!tc1Valid || !tc2Valid) {
    heaterOff();
    return;
  }

  if (tc1TempC >= HARD_CUTOFF_C ||
      tc2TempC >= HARD_CUTOFF_C) {
    heaterOff();
    return;
  }

  heaterOn = state;
  digitalWrite(MOSFET_PIN, state ? HIGH : LOW);
}

// =====================================================
// READ MAX31855
// =====================================================
bool readThermocoupleC(uint8_t csPin, float &tempC) {

  uint32_t frame = 0;

  SPI.beginTransaction(
    SPISettings(1000000, MSBFIRST, SPI_MODE0)
  );

  digitalWrite(csPin, LOW);

  for (uint8_t i = 0; i < 4; i++) {
    frame = (frame << 8) | SPI.transfer(0x00);
  }

  digitalWrite(csPin, HIGH);

  SPI.endTransaction();

  if (frame == 0x00000000UL ||
      frame == 0xFFFFFFFFUL) {
    return false;
  }

  if (frame & 0x00010000UL) {
    return false;
  }

  int16_t rawTemp = (int16_t)((frame >> 18) & 0x3FFF);

  if (rawTemp & 0x2000) {
    rawTemp |= (int16_t)0xC000;
  }

  tempC = (float)rawTemp * 0.25f;

  if (isnan(tempC) ||
      tempC < -50.0f ||
      tempC >= HARD_CUTOFF_C) {
    return false;
  }

  return true;
}

// =====================================================
// TEMPERATURE CONTROL
// =====================================================
bool temperatureControl(
  float currentTemp,
  float targetTemp,
  bool previousState
) {

  if (currentTemp >= HARD_CUTOFF_C) {
    return false;
  }

  if (currentTemp <= targetTemp - HYSTERESIS_C) {
    return true;
  }

  if (currentTemp >= targetTemp + HYSTERESIS_C) {
    return false;
  }

  return previousState;
}

// =====================================================
// PWM TO TEMPERATURE
// =====================================================
float pwmToTemperature(unsigned long pulse) {

  if (pulse < PWM_OFF_MAX) {
    return -1.0f;
  }

  if (pulse > PWM_INPUT_MAX) {
    pulse = PWM_INPUT_MAX;
  }

  float percentage =
    (float)(pulse - PWM_INPUT_MIN) /
    (float)(PWM_INPUT_MAX - PWM_INPUT_MIN);

  float target =
    PWM_TARGET_MIN_C +
    ((percentage - 0.10f) / 0.90f) *
    (PWM_TARGET_MAX_C - PWM_TARGET_MIN_C);

  if (target < PWM_TARGET_MIN_C) {
    target = PWM_TARGET_MIN_C;
  }

  if (target > PWM_TARGET_MAX_C) {
    target = PWM_TARGET_MAX_C;
  }

  return target;
}

// =====================================================
// MODE SWITCHING
// =====================================================
void setMode(OperatingMode newMode) {

  heaterOff();
  requestedHeaterState = false;
  currentMode = newMode;

  if (serialOutputMode == OUTPUT_MONITOR) {
    Serial.print(F("Operating mode: "));

    switch (currentMode) {
      case MODE_DEBUG:
        Serial.println(F("DEBUG"));
        break;

      case MODE_TEST:
        Serial.println(F("TEST"));
        break;

      case MODE_DEPLOY:
        Serial.println(F("DEPLOY"));
        break;
    }
  }
}

// =====================================================
// SERIAL COMMANDS
// =====================================================
void processCommand(char *command) {

  while (*command == ' ') {
    command++;
  }

  if (strcmp(command, "debug") == 0) {
    setMode(MODE_DEBUG);
    return;
  }

  if (strcmp(command, "test") == 0) {
    setMode(MODE_TEST);
    return;
  }

  if (strcmp(command, "deploy") == 0) {
    setMode(MODE_DEPLOY);
    return;
  }

  if (strcmp(command, "monitor") == 0) {
    serialOutputMode = OUTPUT_MONITOR;
    Serial.println(F("Serial Monitor mode"));
    return;
  }

  if (strcmp(command, "plot") == 0) {
    serialOutputMode = OUTPUT_PLOTTER;
    return;
  }

  if (strncmp(command, "target ", 7) == 0) {

    char *endPtr;
    float value = strtod(command + 7, &endPtr);

    while (*endPtr == ' ') {
      endPtr++;
    }

    if (*endPtr != '\0' ||
        isnan(value) ||
        value < MIN_TARGET_C ||
        value > MAX_TARGET_C) {

      if (serialOutputMode == OUTPUT_MONITOR) {
        Serial.println(F("Invalid target: range 50-200 C"));
      }

      return;
    }

    manualTargetC = value;

    if (serialOutputMode == OUTPUT_MONITOR) {
      Serial.print(F("Manual target: "));
      Serial.print(manualTargetC, 1);
      Serial.println(F(" C"));
    }

    return;
  }

  if (strcmp(command, "pwmfail on") == 0) {
    pwmFailActive = true;

    if (serialOutputMode == OUTPUT_MONITOR) {
      Serial.println(F("PWM fallback ON"));
    }

    return;
  }

  if (strcmp(command, "pwmfail off") == 0) {
    pwmFailActive = false;

    if (serialOutputMode == OUTPUT_MONITOR) {
      Serial.println(F("PWM fallback OFF"));
    }

    return;
  }

  if (strcmp(command, "help") == 0) {

    if (serialOutputMode == OUTPUT_MONITOR) {

      Serial.println(F("---- COMMANDS ----"));
      Serial.println(F("debug       - Simulation mode"));
      Serial.println(F("test        - Physical heater test"));
      Serial.println(F("deploy      - PWM mode"));
      Serial.println(F("target X    - Set target temperature"));
      Serial.println(F("pwmfail on  - Manual target on PWM loss"));
      Serial.println(F("pwmfail off - Heater OFF on PWM loss"));
      Serial.println(F("monitor     - Serial Monitor output"));
      Serial.println(F("plot        - Serial Plotter output"));
      Serial.println(F("help        - Show commands"));
    }

    return;
  }

  if (serialOutputMode == OUTPUT_MONITOR) {
    Serial.println(F("Unknown command. Type help."));
  }
}

// =====================================================
// READ SERIAL INPUT
// =====================================================
void readConsole() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      if (consoleIndex > 0) {

        consoleBuffer[consoleIndex] = '\0';
        processCommand(consoleBuffer);
        consoleIndex = 0;
      }
    }
    else if (consoleIndex < sizeof(consoleBuffer) - 1) {

      consoleBuffer[consoleIndex++] = c;
    }
    else {

      consoleIndex = 0;

      if (serialOutputMode == OUTPUT_MONITOR) {
        Serial.println(F("Command too long"));
      }
    }
  }
}

// =====================================================
// STATUS OUTPUT
// =====================================================
void printStatus(float targetC, unsigned long pulse) {

  if (millis() - lastStatusPrint < 500) {
    return;
  }

  lastStatusPrint = millis();

  // SERIAL PLOTTER
  if (serialOutputMode == OUTPUT_PLOTTER) {

    Serial.print(F("TC1:"));
    Serial.print(tc1Valid ? tc1TempC : 0.0f, 2);

    Serial.print(F("\tTC2:"));
    Serial.print(tc2Valid ? tc2TempC : 0.0f, 2);

    Serial.print(F("\tTarget:"));
    Serial.print(targetC >= 0 ? targetC : 0.0f, 2);

    Serial.print(F("\tMOSFET_REQUEST:"));
    Serial.print(requestedHeaterState ? 1 : 0);

    Serial.print(F("\tMOSFET_PHYSICAL:"));
    Serial.println(heaterOn ? 1 : 0);

    return;
  }

  // SERIAL MONITOR
  Serial.println();
  Serial.println(F("=============================="));

  Serial.print(F("Mode: "));
  if (currentMode == MODE_DEBUG) Serial.println(F("DEBUG"));
  if (currentMode == MODE_TEST) Serial.println(F("TEST"));
  if (currentMode == MODE_DEPLOY) Serial.println(F("DEPLOY"));

  Serial.print(F("TC1: "));
  if (tc1Valid) {
    Serial.print(tc1TempC, 2);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("SENSOR ERROR"));
  }

  Serial.print(F("TC2: "));
  if (tc2Valid) {
    Serial.print(tc2TempC, 2);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("SENSOR ERROR"));
  }

  Serial.print(F("Target: "));
  if (targetC >= 0) {
    Serial.print(targetC, 2);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("OFF"));
  }

  Serial.print(F("MOSFET REQUEST: "));
  Serial.println(requestedHeaterState ? F("ON") : F("OFF"));

  Serial.print(F("PHYSICAL MOSFET: "));
  Serial.println(heaterOn ? F("ON") : F("OFF"));

  if (currentMode == MODE_DEPLOY) {
    Serial.print(F("PWM: "));
    Serial.print(pulse);
    Serial.println(F(" us"));
  }

  Serial.print(F("PWM FAILSAFE: "));
  Serial.println(pwmFailActive ? F("ON") : F("OFF"));

  Serial.println(F("=============================="));
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  pinMode(FCU_PIN, INPUT);

  pinMode(TC1_CS_PIN, OUTPUT);
  pinMode(TC2_CS_PIN, OUTPUT);

  digitalWrite(TC1_CS_PIN, HIGH);
  digitalWrite(TC2_CS_PIN, HIGH);

  // Hardware SPI uses board-defined SCK/MISO pins.
  SPI.begin();

  Serial.begin(115200);

  currentMode = MODE_DEPLOY;
  serialOutputMode = OUTPUT_MONITOR;

  heaterOff();

  Serial.println();
  Serial.println(F("HEATER CONTROLLER STARTED"));
  Serial.println(F("TC1 CS: D17"));
  Serial.println(F("TC2 CS: D20"));
  Serial.println(F("Default mode: DEPLOY"));
  Serial.println(F("Default target: 200 C"));
  Serial.println(F("PWM fallback: OFF"));
  Serial.println(F("Type help for commands"));
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {

  readConsole();

  // Read both thermocouples independently.
  tc1Valid = readThermocoupleC(TC1_CS_PIN, tc1TempC);
  tc2Valid = readThermocoupleC(TC2_CS_PIN, tc2TempC);

  // Both sensors must be valid for physical heating.
  if (!tc1Valid || !tc2Valid) {

    heaterOff();

    printStatus(-1.0f, 0);

    delay(50);
    return;
  }

  // ===================================================
  // DEBUG MODE
  // ===================================================
  if (currentMode == MODE_DEBUG) {

    float controlTemp = max(tc1TempC, tc2TempC);

    requestedHeaterState = temperatureControl(
      controlTemp,
      manualTargetC,
      requestedHeaterState
    );

    heaterOff();

    printStatus(manualTargetC, 0);

    delay(50);
    return;
  }

  // ===================================================
  // TEST MODE
  // ===================================================
  if (currentMode == MODE_TEST) {

    // Use hotter thermocouple for temperature control.
    float controlTemp = max(tc1TempC, tc2TempC);

    bool requestedState = temperatureControl(
      controlTemp,
      manualTargetC,
      heaterOn
    );

    setHeater(requestedState);

    printStatus(manualTargetC, 0);

    delay(50);
    return;
  }

  // ===================================================
  // DEPLOY MODE
  // ===================================================
  if (currentMode == MODE_DEPLOY) {

    unsigned long pulse =
      pulseIn(FCU_PIN, HIGH, PWM_TIMEOUT_US);

    float targetC = -1.0f;

    if (pulse < PWM_VALID_MIN ||
        pulse > PWM_VALID_MAX) {

      if (pwmFailActive) {
        targetC = manualTargetC;
      }
      else {
        heaterOff();
      }
    }
    else {
      targetC = pwmToTemperature(pulse);
    }

    if (targetC < 0.0f) {

      heaterOff();
    }
    else {

      float controlTemp = max(tc1TempC, tc2TempC);

      bool requestedState = temperatureControl(
        controlTemp,
        targetC,
        heaterOn
      );

      setHeater(requestedState);
    }

    printStatus(targetC, pulse);

    delay(50);
    return;
  }

  // Unknown mode: fail safe.
  heaterOff();

  delay(50);
}
