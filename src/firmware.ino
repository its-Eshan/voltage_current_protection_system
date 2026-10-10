#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ---------- Hardware ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte VOLTAGE_POT = A0;
const byte CURRENT_POT = A1;
const byte RELAY_PIN = 8;
const byte LED_PIN = 9;   
const bool RELAY_ACTIVE_LOW = true;
const byte ROW_PINS[4] = {2, 3, 4, 5};
const byte COL_PINS[4] = {6, 7, 10, 11};
const char KEYS[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
const unsigned long KEY_DEBOUNCE = 30;

//knob ranges 
const int RAW_AT_KNOB_MIN = 895;  
const int RAW_AT_KNOB_MAX = 128;  

const float MAX_SIM_VOLTAGE = 400.0;
const float MAX_SIM_CURRENT = 14.0;

//Default
const float DEFAULT_VMAX = 220.0;      
const float DEFAULT_IMAX = 10.0;       
float maxVoltage = DEFAULT_VMAX;
float maxCurrent = DEFAULT_IMAX;

const unsigned long UPDATE_INTERVAL = 250;   
const unsigned long REPORT_INTERVAL = 1000;  
const unsigned long MESSAGE_TIME = 1500;     
const unsigned long EDIT_TIMEOUT = 20000;   
unsigned long lastUpdate = 0;
unsigned long lastReport = 0;

char commandBuffer[40];
byte commandLength = 0;

enum Status { ST_NORMAL, ST_OVER_V, ST_OVER_I };

const char* const STATUS_TEXT[] = {
  "NORMAL", "OVER VOLTAGE", "OVER CURRENT"
};

Status lastStatus = ST_NORMAL;
bool firstReading = true;

int rawVoltage = 0;
int rawCurrent = 0;
float voltage = 0;
float current = 0;

enum UiMode { UI_RUN, UI_EDIT_V, UI_EDIT_I, UI_MESSAGE };

UiMode uiMode = UI_RUN;
UiMode afterMessage = UI_RUN;
unsigned long messageUntil = 0;
unsigned long lastKeyTime = 0;

char inputBuffer[6];       
byte inputLength = 0;
float pendingVoltage = 0;   

float knobPosition(int raw);
Status evaluate(float v, float i);
void setOutputs(bool normal);
void displayReadings(Status status);
void printReport(Status status);
void printSettings();
void processCommand(char* cmd);

char readKeypad();
void handleKey(char key);
void startEdit(UiMode mode);
void drawEditScreen();
void showMessage(const char* line1, const char* line2, UiMode next);
void lcdLine(byte row, const char* text);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  setOutputs(false);
  pinMode(RELAY_PIN, OUTPUT);
  setOutputs(false);

  for (byte r = 0; r < 4; r++) pinMode(ROW_PINS[r], INPUT);
  for (byte c = 0; c < 4; c++) pinMode(COL_PINS[c], INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("VOLT PROTECTOR");
  lcd.setCursor(0, 1);
  lcd.print("PRESS A TO SET");
  delay(1500);
  lcd.clear();

  Serial.println(F("Voltage Protector - SIMULATION"));
  Serial.println(F("Voltage knob: 45 deg = 0 V, 180 deg = 200 V, 315 deg = 400 V"));
  Serial.println(F("Current knob: 45 deg = 0 A, 180 deg = 7 A, 315 deg = 14 A"));
  printSettings();
  Serial.println(F("Keypad: A = set limits, # = OK, * = decimal point,"));
  Serial.println(F("        D = delete, C = cancel"));
  Serial.println(F("Type HELP for commands. Serial baud: 9600"));
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println(F("ERROR: command too long"));
    }
  }

  char key = readKeypad();
  if (key) {
    handleKey(key);
  }
  unsigned long now = millis();
  if (uiMode == UI_MESSAGE && (long)(now - messageUntil) >= 0) {
    uiMode = afterMessage;
    if (uiMode == UI_EDIT_V || uiMode == UI_EDIT_I) {
      drawEditScreen();
    }
  }
  if ((uiMode == UI_EDIT_V || uiMode == UI_EDIT_I) &&
      now - lastKeyTime >= EDIT_TIMEOUT) {
    Serial.println(F("KEYPAD: timeout, limits not changed"));
    showMessage("TIMEOUT", "NOT CHANGED", UI_RUN);
  }

  if (now - lastUpdate < UPDATE_INTERVAL) {
    return;
  }
  lastUpdate = now;
  rawVoltage = analogRead(VOLTAGE_POT);
  rawCurrent = analogRead(CURRENT_POT);
  voltage = knobPosition(rawVoltage) * MAX_SIM_VOLTAGE;
  current = knobPosition(rawCurrent) * MAX_SIM_CURRENT;

  Status status = evaluate(voltage, current);

  setOutputs(status == ST_NORMAL);

  if (firstReading || status != lastStatus) {
    Serial.print(F("STATE: "));
    Serial.print(STATUS_TEXT[status]);
    Serial.print(F(" (V="));
    Serial.print(voltage, 1);
    Serial.print(F(" V, I="));
    Serial.print(current, 2);
    Serial.print(F(" A) -> "));
    Serial.println(status == ST_NORMAL ? F("LED ON, RELAY ON") : F("LED OFF, RELAY OFF"));
    lastStatus = status;
    firstReading = false;
  }
  if (uiMode == UI_RUN) {
    displayReadings(status);
  }

  if (now - lastReport >= REPORT_INTERVAL) {
    lastReport = now;
    printReport(status);
  }
}

float knobPosition(int raw) {
  float pos = (float)(RAW_AT_KNOB_MIN - raw) /
              (RAW_AT_KNOB_MIN - RAW_AT_KNOB_MAX);
  if (pos < 0.0) pos = 0.0;
  if (pos > 1.0) pos = 1.0;
  return pos;
}

Status evaluate(float v, float i) {
  if (v > maxVoltage) return ST_OVER_V;
  if (i > maxCurrent) return ST_OVER_I;
  return ST_NORMAL;
}

void setOutputs(bool normal) {
  bool level = RELAY_ACTIVE_LOW ? !normal : normal;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
  digitalWrite(LED_PIN, normal ? HIGH : LOW);
}


char scanKeypad() {
  char found = 0;
  for (byte r = 0; r < 4; r++) {
    pinMode(ROW_PINS[r], OUTPUT);
    digitalWrite(ROW_PINS[r], LOW);
    delayMicroseconds(10);
    for (byte c = 0; c < 4; c++) {
      if (digitalRead(COL_PINS[c]) == LOW) {
        found = KEYS[r][c];
      }
    }
    pinMode(ROW_PINS[r], INPUT);
  }
  return found;
}

char readKeypad() {
  static char lastRaw = 0;
  static char stableKey = 0;
  static unsigned long changedAt = 0;

  char raw = scanKeypad();
  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = millis();
  }

  if (millis() - changedAt >= KEY_DEBOUNCE && raw != stableKey) {
    stableKey = raw;
    if (stableKey) {
      return stableKey;
    }
  }
  return 0;
}

void handleKey(char key) {
  lastKeyTime = millis();

  if (uiMode == UI_MESSAGE) {
    return;   
  }

  if (uiMode == UI_RUN) {
    if (key == 'A') {
      Serial.println(F("KEYPAD: setting limits"));
      startEdit(UI_EDIT_V);
    }
    return;
  }

  if (key >= '0' && key <= '9') {
    char* dot = strchr(inputBuffer, '.');
    bool decimalFull = dot != NULL && strlen(dot) >= 2;   // one decimal only
    if (inputLength < sizeof(inputBuffer) - 1 && !decimalFull) {
      inputBuffer[inputLength++] = key;
      inputBuffer[inputLength] = '\0';
    }
  } 
  else if (key == '*') {
    if (strchr(inputBuffer, '.') == NULL &&
        inputLength < sizeof(inputBuffer) - 2) {
      if (inputLength == 0) {
        inputBuffer[inputLength++] = '0';
      }
      inputBuffer[inputLength++] = '.';
      inputBuffer[inputLength] = '\0';
    }
  } 
  else if (key == 'D') {
    if (inputLength > 0) {
      inputBuffer[--inputLength] = '\0';
    }
  } 
  else if (key == 'C') {
    Serial.println(F("KEYPAD: cancelled, limits not changed"));
    showMessage("CANCELLED", "NOT CHANGED", UI_RUN);
    return;
  } 
  else if (key == '#') {
    bool isVoltage = (uiMode == UI_EDIT_V);
    float oldValue = isVoltage ? maxVoltage : maxCurrent;
    float limit = isVoltage ? MAX_SIM_VOLTAGE : MAX_SIM_CURRENT;
    float value = (inputLength == 0) ? oldValue : atof(inputBuffer);

    if (value <= 0 || value > limit) {
      Serial.print(F("KEYPAD: invalid value "));
      Serial.println(inputBuffer);
      showMessage("INVALID VALUE",
                  isVoltage ? "MAX 400 V" : "MAX 14 A",
                  uiMode);
      inputLength = 0;
      inputBuffer[0] = '\0';
      return;
    }

    if (isVoltage) {
      pendingVoltage = value;
      startEdit(UI_EDIT_I);
    } else {
      maxVoltage = pendingVoltage;
      maxCurrent = value;
      Serial.print(F("KEYPAD UPDATED: "));
      printSettings();

      char line2[17];
      char vText[8];
      char iText[8];
      dtostrf(maxVoltage, 1, 0, vText);
      dtostrf(maxCurrent, 1, 1, iText);
      snprintf(line2, sizeof(line2), "%sV  %sA", vText, iText);
      showMessage("SAVED", line2, UI_RUN);
    }
    return;
  }
  drawEditScreen();
}

void startEdit(UiMode mode) {
  uiMode = mode;
  inputLength = 0;
  inputBuffer[0] = '\0';
  lastKeyTime = millis();
  drawEditScreen();
}

void drawEditScreen() {
  char line1[17];
  char line2[17];
  char valueText[8];
  char typed[8];

  if (uiMode == UI_EDIT_V) {
    dtostrf(maxVoltage, 1, 1, valueText);
    snprintf(line1, sizeof(line1), "VMAX now %sV", valueText);
  } else {
    dtostrf(maxCurrent, 1, 1, valueText);
    snprintf(line1, sizeof(line1), "IMAX now %sA", valueText);
  }

  snprintf(typed, sizeof(typed), "%s_", inputBuffer);
  snprintf(line2, sizeof(line2), "New:%-6s#=OK", typed);

  lcdLine(0, line1);
  lcdLine(1, line2);
}

void showMessage(const char* line1, const char* line2, UiMode next) {
  lcdLine(0, line1);
  lcdLine(1, line2);
  uiMode = UI_MESSAGE;
  afterMessage = next;
  messageUntil = millis() + MESSAGE_TIME;
}

void lcdLine(byte row, const char* text) {
  lcd.setCursor(0, row);
  lcd.print(text);
  for (int i = strlen(text); i < 16; i++) lcd.print(' ');
}

void displayReadings(Status status) {
  char line1[17];
  char line2[17];

  int v = (int)(voltage + 0.5);
  int i10 = (int)(current * 10.0 + 0.5);

  snprintf(line1, sizeof(line1), "V=%3dV I=%2d.%1dA",
           v, i10 / 10, i10 % 10);
  snprintf(line2, sizeof(line2), "%s", STATUS_TEXT[status]);

  lcdLine(0, line1);
  lcdLine(1, line2);
}

void printReport(Status status) {
  Serial.print(F("V_SIM="));
  Serial.print(voltage, 1);
  Serial.print(F(" V, I_SIM="));
  Serial.print(current, 2);
  Serial.print(F(" A, STATUS="));
  Serial.print(STATUS_TEXT[status]);
  Serial.print(F(", LED="));
  Serial.print(status == ST_NORMAL ? F("ON") : F("OFF"));
  Serial.print(F("  [raw A0="));
  Serial.print(rawVoltage);
  Serial.print(F(" A1="));
  Serial.print(rawCurrent);
  Serial.println(F("]"));
}

void printSettings() {
  Serial.print(F("Limits: VMAX="));
  Serial.print(maxVoltage, 1);
  Serial.print(F(" V, IMAX="));
  Serial.print(maxCurrent, 1);
  Serial.println(F(" A"));
}

void processCommand(char* cmd) {
  // Upper-case and strip spaces so "vmax = 230" also works.
  char* out = cmd;
  for (char* p = cmd; *p; p++) {
    if (*p != ' ' && *p != '\t') *out++ = toupper(*p);
  }
  *out = '\0';

  if (strcmp(cmd, "HELP") == 0) {
    Serial.println(F("Commands:"));
    Serial.println(F("  VMAX=220   over-voltage limit (V)"));
    Serial.println(F("  IMAX=10    over-current limit (A)"));
    Serial.println(F("  STATUS     show readings and limits"));
    Serial.println(F("  DEFAULTS   restore 220 V, 10 A"));
    Serial.println(F("Keypad: A = set limits, # = OK, * = decimal point,"));
    Serial.println(F("        D = delete, C = cancel"));
    return;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    printSettings();
    printReport(evaluate(voltage, current));
    return;
  }
  if (strcmp(cmd, "DEFAULTS") == 0) {
    maxVoltage = DEFAULT_VMAX;
    maxCurrent = DEFAULT_IMAX;
    Serial.print(F("UPDATED: "));
    printSettings();
    return;
  }

  char* equals = strchr(cmd, '=');
  if (equals == NULL) {
    Serial.println(F("ERROR: use NAME=VALUE, or type HELP"));
    return;
  }

  *equals = '\0';
  char* name = cmd;
  char* valueText = equals + 1;

  char* endPtr;
  float value = strtod(valueText, &endPtr);
  if (valueText == endPtr || *endPtr != '\0') {
    Serial.println(F("ERROR: invalid number"));
    return;
  }

  if (strcmp(name, "VMAX") == 0) {
    if (value <= 0 || value > MAX_SIM_VOLTAGE) {
      Serial.println(F("ERROR: VMAX must be above 0 and at most 400 V"));
      return;
    }
    maxVoltage = value;
  } else if (strcmp(name, "IMAX") == 0) {
    if (value <= 0 || value > MAX_SIM_CURRENT) {
      Serial.println(F("ERROR: IMAX must be above 0 and at most 14 A"));
      return;
    }
    maxCurrent = value;
  } else {
    Serial.println(F("ERROR: unknown setting, type HELP"));
    return;
  }

  Serial.print(F("UPDATED: "));
  printSettings();
}
