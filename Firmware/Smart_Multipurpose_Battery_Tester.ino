/*
====================================================================================================================
  PROJECT      : DIY Smart Multipurpose Battery Tester
  VERSION      : Beta Version
  UPDATED ON   : 25-Oct-2024
  AUTHOR       : Open Green Energy

  LICENSE
  ------------------------------------------------------------------------------------------------------------------
  Copyright (c) 2026 Open Green Energy

  This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.

  You are free to:
    - Share  : Copy and redistribute the material in any medium or format
    - Adapt  : Remix, transform, and build upon the material

  Under the following terms:
    - Attribution   : You must give appropriate credit to Open Green Energy
    - NonCommercial : You may not use the material for commercial purposes
    - ShareAlike    : If you remix, transform, or build upon the material, you must distribute your
                      contributions under the same license

  License URL:
    https://creativecommons.org/licenses/by-nc-sa/4.0/

  HARDWARE OVERVIEW
  ------------------------------------------------------------------------------------------------------------------
  MCU            : Seeed XIAO ESP32-C3
  DISPLAY        : 0.96 inch OLED, 128 x 64, SSD1306, I2C
  BUTTONS        : Mode, Up, Down
  LOAD CONTROL   : PWM controlled electronic load
  CHARGE CONTROL : MOSFET based switching
  SENSING        : Battery voltage divider + external Vref input
  BUZZER         : Status beeper

  WHAT THIS DEVICE DOES
  ------------------------------------------------------------------------------------------------------------------
  1. Charges rechargeable cells up to the defined full voltage.
  2. Discharges cells using selectable current steps.
  3. Calculates discharged capacity in mAh.
  4. Runs an Analyze mode with charge, rest, and discharge sequence.
  5. Measures approximate internal resistance using loaded and unloaded voltage.
  6. Displays voltage, time, capacity, and test status on the OLED.

  PIN MAPPING
  ------------------------------------------------------------------------------------------------------------------
  MODE BUTTON    : D3
  UP BUTTON      : D6
  DOWN BUTTON    : D9
  PWM OUTPUT     : D8
  BUZZER         : D7
  BAT ADC        : A0
  VREF ADC       : A1
  CHARGE MOSFET  : D2

  NOTES
  ------------------------------------------------------------------------------------------------------------------
  - This sketch is kept functionally identical to the working reference version.
  - Comments and section formatting are revised for GitHub readability.
  - Logic, variable flow, and operating behaviour are intentionally unchanged.
====================================================================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <JC_Button.h>

// ============================================================================
// OLED display configuration
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Create SSD1306 display object.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// Button pin mapping
// ============================================================================
#define MODE_PIN D3
#define UP_PIN D6
#define DOWN_PIN D9

// Instantiate push button objects.
Button Mode_Button(MODE_PIN, 25, false, true);  // GPIO 3 on XIAO ESP32-C3 (D3)
Button UP_Button(UP_PIN, 25, false, true);      // GPIO 6 on XIAO ESP32-C3 (D6)
Button Down_Button(DOWN_PIN, 25, false, true);  // GPIO 9 on XIAO ESP32-C3 (D9)

// ============================================================================
// Mode selection state
// ============================================================================
int selectedMode = 0;
bool modeSelected = false;
bool inAnalyzeMode = false;  // Tracks whether Analyze mode is currently running.

// ============================================================================
// Battery thresholds and limits
// ============================================================================
float cutoffVoltage = 3.0;           // Default cutoff voltage selected by user.
const float Min_BAT_level = 2.8;     // Minimum selectable cutoff voltage.
const float Max_BAT_level = 3.2;     // Maximum selectable cutoff voltage.
const float FULL_BAT_level = 4.18;   // Charge termination voltage.
const float DAMAGE_BAT_level = 2.5;  // Battery below this is treated as damaged.
const float NO_BAT_level = 0.3;      // Slot considered empty below this voltage.

// ============================================================================
// Discharge current table and PWM table
// ============================================================================
int Current[] = {0, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
int PWM[] = {0, 4, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
int Array_Size = sizeof(Current) / sizeof(Current[0]);
int Current_Value = 0;
int currentOffset = 25;  // Default offset current.
int PWM_Value = 0;
int PWM_Index = 0;

// ============================================================================
// Measurement and runtime variables
// ============================================================================
unsigned long Capacity = 0;
float Capacity_f = 0;
float Vref_Voltage = 1.227;  // LM385-1.2V reference voltage. Adjust for calibration.
float Vcc = 3.3;
float BAT_Voltage = 0;
float Resistance = 0;
float sample = 0;
bool calc = false, Done = false, Report_Info = true;

// ============================================================================
// Timing variables
// ============================================================================
unsigned long previousMillis = 0;
const long interval = 50;        // Battery icon animation update interval.
unsigned long startTime = 0;     // Start time for active process.
unsigned long elapsedTime = 0;   // Elapsed process time.

// Time fields shown on the display.
int Hour = 0;
int Minute = 0;
int Second = 0;

// ============================================================================
// Control and sensing pins
// ============================================================================
const byte PWM_Pin = D8;      // PWM output for load control.
const byte Buzzer = D7;       // Buzzer output.
const int BAT_Pin = A0;       // Battery voltage sense input.
const int Vref_Pin = A1;      // ADC reference input.
const byte Mosfet_Pin = D2;   // Charge MOSFET control.

// Battery icon fill level.
int batteryLevel = 0;

// ============================================================================
// Voltage divider resistor values
// ============================================================================
const float R1 = 200000.0;  // 200 kOhm
const float R2 = 100000.0;  // 100 kOhm

// ============================================================================
// Function: setup
// Runs once at startup.
// ============================================================================
void setup() {
    pinMode(PWM_Pin, OUTPUT);
    pinMode(Buzzer, OUTPUT);
    pinMode(Mosfet_Pin, OUTPUT);
    analogWrite(PWM_Pin, PWM_Value);
    UP_Button.begin();
    Down_Button.begin();
    Mode_Button.begin();

    // Initialize the OLED display at I2C address 0x3C.
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for (;;);  // Halt if display initialization fails.
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Startup splash text.
    display.setTextSize(1);
    display.setCursor(10, 25);
    display.print("Open Green Energy");
    display.display();
    delay(2000);

    // Enter mode selection screen.
    selectMode();
}

// ============================================================================
// Function: loop
// Main state dispatcher.
// ============================================================================
void loop() {
    if (modeSelected) {
        if (selectedMode == 0) {
            chargeMode();
        } else if (selectedMode == 1) {
            dischargeMode();
        } else if (selectedMode == 2) {
            analyzeMode();
        } else if (selectedMode == 3) {
            internalResistanceMode();
        }
    }
}

// ============================================================================
// Function: selectMode
// Shows mode selection menu and waits for user confirmation.
// ============================================================================
void selectMode() {
    modeSelected = false;
    selectedMode = 0;

    while (!modeSelected) {
        Mode_Button.read();
        UP_Button.read();
        Down_Button.read();

        // Move upward through the menu.
        if (UP_Button.isPressed()) {
            selectedMode = (selectedMode == 0) ? 3 : selectedMode - 1;
            beep(100);
            delay(300);
        }

        // Move downward through the menu.
        if (Down_Button.isPressed()) {
            selectedMode = (selectedMode == 3) ? 0 : selectedMode + 1;
            beep(100);
            delay(300);
        }

        // Confirm current selection.
        if (Mode_Button.isPressed()) {
            beep(300);
            modeSelected = true;
            delay(300);
        }

        // Draw mode selection menu.
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(25, 0);
        display.print("Select Mode:");
        display.setCursor(25, 12);
        display.print((selectedMode == 0) ? "> Charge" : "  Charge");
        display.setCursor(25, 26);
        display.print((selectedMode == 1) ? "> Discharge" : "  Discharge");
        display.setCursor(25, 40);
        display.print((selectedMode == 2) ? "> Analyze" : "  Analyze");
        display.setCursor(25, 54);
        display.print((selectedMode == 3) ? "> IR Test" : "  IR Test");
        display.display();
    }

    // Briefly show selected mode.
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(10, 20);
    if (selectedMode == 0) {
        display.print("Charge..");
    } else if (selectedMode == 1) {
        display.print("Dischrg..");
    } else if (selectedMode == 2) {
        display.print("Analyze..");
    } else if (selectedMode == 3) {
        display.print("IR Test..");
    }
    display.display();
    delay(500);
}

// ============================================================================
// Function: measureVcc
// Measures supply voltage using the external reference input.
// ============================================================================
float measureVcc() {
    float vrefSum = 0;
    for (int i = 0; i < 100; i++) {
        vrefSum += analogRead(Vref_Pin);
        delay(2);
    }
    float averageVrefReading = vrefSum / 100.0;
    float vcc = (Vref_Voltage * 4096.0) / averageVrefReading;
    return vcc;
}

// ============================================================================
// Function: measureBatteryVoltage
// Reads battery voltage through the resistor divider.
// ============================================================================
float measureBatteryVoltage() {
    Vcc = measureVcc();

    float batterySum = 0;
    for (int i = 0; i < 100; i++) {
        batterySum += analogRead(BAT_Pin);
        delay(2);
    }
    float averageBatteryReading = batterySum / 100.0;
    float voltageDividerRatio = (R1 + R2) / R2;
    float batteryVoltage = (averageBatteryReading * Vcc / 4096.0) * voltageDividerRatio;
    return batteryVoltage;
}

// ============================================================================
// Function: updateTiming
// Updates elapsed time and converts it to hh:mm:ss.
// ============================================================================
void updateTiming() {
    unsigned long currentMillis = millis();
    elapsedTime = currentMillis - startTime;

    Second = (elapsedTime / 1000) % 60;
    Minute = (elapsedTime / (1000 * 60)) % 60;
    Hour = (elapsedTime / (1000 * 60 * 60));
}

// ============================================================================
// Function: chargeMode
// Charges the battery until the full voltage threshold is reached.
// ============================================================================
void chargeMode() {
    calc = true;
    Done = false;
    Capacity = 0;
    unsigned long lastUpdateTime = millis();
    batteryLevel = 0;

    BAT_Voltage = measureBatteryVoltage();

    if (BAT_Voltage < NO_BAT_level) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(15, 25);
        display.print("EMPTY BAT SLOT");
        display.display();
        delay(3000);
        selectMode();
        return;
    } else if (BAT_Voltage < DAMAGE_BAT_level) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(25, 25);
        display.print("BAT DAMAGED");
        display.display();
        delay(3000);
        selectMode();
        return;
    }

    digitalWrite(Mosfet_Pin, HIGH);

    while (!Done) {
        updateTiming();
        BAT_Voltage = measureBatteryVoltage();
        display.clearDisplay();
        updateBatteryDisplay(true);
        display.setTextSize(1);
        display.setCursor(25, 5);
        display.print("Charging..");
        display.setCursor(40, 25);
        display.print(Hour);
        display.print(":");
        display.print(Minute);
        display.print(":");
        display.print(Second);
        display.setTextSize(2);
        display.setCursor(15, 40);
        display.print("V:");
        display.print(BAT_Voltage, 2);
        display.print("V");
        display.display();

        if (BAT_Voltage >= FULL_BAT_level) {
            Done = true;
            digitalWrite(Mosfet_Pin, LOW);
            beep(300);
            displayFinalCapacity(Capacity_f, true);
        }
    }
    selectMode();
}

// ============================================================================
// Function: dischargeMode
// Discharges the battery at user-selected current down to cutoff voltage.
// ============================================================================
void dischargeMode() {
    bool cutoffSelected = selectCutoffVoltage();
    bool currentSelected = selectDischargeCurrent();

    if (cutoffSelected && currentSelected) {
        calc = true;
        Done = false;
        Capacity = 0;
        unsigned long lastUpdateTime = millis();

        digitalWrite(Mosfet_Pin, LOW);
        analogWrite(PWM_Pin, PWM_Value);

        while (!Done) {
            updateTiming();
            BAT_Voltage = measureBatteryVoltage();

            unsigned long currentTime = millis();
            float elapsedTimeInHours = (currentTime - lastUpdateTime) / 3600000.0;

            if (calc) {
                Capacity_f += (Current[PWM_Index] + currentOffset) * elapsedTimeInHours;
                lastUpdateTime = currentTime;
            }

            display.clearDisplay();
            updateBatteryDisplay(false);
            display.setTextSize(1);
            display.setCursor(15, 5);
            display.print("Discharging..");
            display.setCursor(15, 20);
            display.print("Time: ");
            display.print(Hour);
            display.print(":");
            display.print(Minute);
            display.print(":");
            display.print(Second);
            display.setCursor(15, 35);
            display.print("Cap:");
            display.print(Capacity_f, 1);
            display.print("mAh");
            display.setCursor(15, 50);
            display.print("V: ");
            display.print(BAT_Voltage, 2);
            display.print("V");
            display.display();

            if (BAT_Voltage <= cutoffVoltage) {
                Done = true;
                analogWrite(PWM_Pin, 0);
                beep(300);
                displayFinalCapacity(Capacity_f, false);
            }
        }
    }
    selectMode();
}

// ============================================================================
// Function: analyzeMode
// Charges the battery, rests it, then discharges it to estimate true capacity.
// ============================================================================
void analyzeMode() {
    inAnalyzeMode = true;
    calc = true;
    Done = false;
    Capacity = 0;
    unsigned long lastUpdateTime = millis();

    BAT_Voltage = measureBatteryVoltage();

    if (BAT_Voltage < NO_BAT_level) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(15, 25);
        display.print("EMPTY BAT SLOT");
        display.display();
        delay(3000);
        selectMode();
        return;
    } else if (BAT_Voltage < DAMAGE_BAT_level) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(25, 25);
        display.print("BAT DAMAGED");
        display.display();
        delay(3000);
        selectMode();
        return;
    }

    digitalWrite(Mosfet_Pin, HIGH);

    while (!Done) {
        updateTiming();
        BAT_Voltage = measureBatteryVoltage();
        display.clearDisplay();
        updateBatteryDisplay(true);
        display.setTextSize(1);
        display.setCursor(10, 5);
        display.print("Analyzing - C");
        display.setCursor(40, 25);
        display.print(Hour);
        display.print(":");
        display.print(Minute);
        display.print(":");
        display.print(Second);
        display.setTextSize(2);
        display.setCursor(15, 40);
        display.print("V:");
        display.print(BAT_Voltage, 2);
        display.print("V");
        display.display();

        if (BAT_Voltage >= FULL_BAT_level) {
            Done = true;
            digitalWrite(Mosfet_Pin, LOW);
        }
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print("Resting..");
    display.display();
    delay(180000);  // Wait for 3 minutes (kept unchanged from working sketch).

    cutoffVoltage = 3.0;
    PWM_Index = 6;
    PWM_Value = PWM[PWM_Index];

    Done = false;
    Capacity = 0;
    lastUpdateTime = millis();
    digitalWrite(Mosfet_Pin, LOW);
    analogWrite(PWM_Pin, PWM_Value);

    while (!Done) {
        updateTiming();
        BAT_Voltage = measureBatteryVoltage();

        unsigned long currentTime = millis();
        float elapsedTimeInHours = (currentTime - lastUpdateTime) / 3600000.0;

        if (calc) {
            Capacity_f += (Current[PWM_Index] + currentOffset) * elapsedTimeInHours;
            lastUpdateTime = currentTime;
        }

        display.clearDisplay();
        updateBatteryDisplay(false);
        display.setTextSize(1);
        display.setCursor(10, 5);
        display.print("Analyzing - D");
        display.setCursor(15, 20);
        display.print("Time: ");
        display.print(Hour);
        display.print(":");
        display.print(Minute);
        display.print(":");
        display.print(Second);
        display.setCursor(15, 35);
        display.print("Cap:");
        display.print(Capacity_f, 1);
        display.print("mAh");
        display.setCursor(15, 50);
        display.print("V: ");
        display.print(BAT_Voltage, 2);
        display.print("V");
        display.display();

        if (BAT_Voltage <= cutoffVoltage) {
            Done = true;
            analogWrite(PWM_Pin, 0);
            beep(300);
            displayFinalCapacity(Capacity_f, false);
        }
    }

    inAnalyzeMode = false;
    selectMode();
}

// ============================================================================
// Function: internalResistanceMode
// Measures approximate internal resistance using no-load and loaded voltage.
// ============================================================================
void internalResistanceMode() {
    float voltageNoLoad = 0;
    float voltageLoad = 0;
    float internalResistance = 0;
    bool resistanceMeasured = false;

    digitalWrite(Mosfet_Pin, LOW);

    analogWrite(PWM_Pin, 0);
    delay(500);
    voltageNoLoad = measureBatteryVoltage();

    PWM_Index = 6;
    PWM_Value = PWM[PWM_Index];
    analogWrite(PWM_Pin, PWM_Value);

    delay(500);
    voltageLoad = measureBatteryVoltage();

    float currentDrawn = Current[PWM_Index] / 1000.0;

    if (currentDrawn > 0) {
        internalResistance = (voltageNoLoad - voltageLoad) / currentDrawn;
    } else {
        internalResistance = 0;
    }

    displayIRTestIcon(voltageNoLoad, voltageLoad, internalResistance);
    beep(300);

    analogWrite(PWM_Pin, 0);
    delay(5000);

    resistanceMeasured = true;

    if (resistanceMeasured) {
        selectMode();
    }
}

// ============================================================================
// Function: displayFinalCapacity
// Shows final screen after charge or discharge completes.
// ============================================================================
void displayFinalCapacity(float capacity, bool chargingComplete) {
    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(15, 5);
    display.print("Complete");
    display.setCursor(15, 20);
    display.print("Time: ");
    display.print(Hour);
    display.print(":");
    display.print(Minute);
    display.print(":");
    display.print(Second);
    display.setCursor(15, 35);
    display.print("Cap:");
    display.print(Capacity_f, 1);
    display.print("mAh");
    display.setCursor(15, 50);
    display.print("V: ");
    display.print(BAT_Voltage, 2);
    display.print("V");

    if (chargingComplete) {
        drawBatteryOutline();
        drawBatteryFill(100);
    } else {
        drawBatteryOutline();
        drawBatteryFill(0);
    }

    display.display();

    bool buttonPressed = false;
    while (!buttonPressed) {
        Mode_Button.read();
        UP_Button.read();
        Down_Button.read();

        if (Mode_Button.wasPressed() || UP_Button.wasPressed() || Down_Button.wasPressed()) {
            buttonPressed = true;
        }

        delay(100);
    }

    selectMode();
}

// ============================================================================
// Function: selectCutoffVoltage
// Lets the user choose the cutoff voltage for discharge mode.
// ============================================================================
bool selectCutoffVoltage() {
    bool cutoffSelected = false;
    while (!cutoffSelected) {
        UP_Button.read();
        Down_Button.read();
        Mode_Button.read();

        if (UP_Button.isPressed() && cutoffVoltage < Max_BAT_level) {
            cutoffVoltage += 0.1;
            beep(100);
            delay(300);
        }

        if (Down_Button.isPressed() && cutoffVoltage > Min_BAT_level) {
            cutoffVoltage -= 0.1;
            beep(100);
            delay(300);
        }

        if (Mode_Button.isPressed()) {
            cutoffSelected = true;
            beep(300);
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(2, 10);
        display.print("Select Cutoff Volt:");
        display.setTextSize(2);
        display.setCursor(20, 30);
        display.print("V:");
        display.print(cutoffVoltage, 1);
        display.print("V");
        display.display();
    }
    return cutoffSelected;
}

// ============================================================================
// Function: selectDischargeCurrent
// Lets the user choose the discharge current.
// ============================================================================
bool selectDischargeCurrent() {
    bool currentSelected = false;
    PWM_Index = 0;
    PWM_Value = PWM[PWM_Index];

    while (!currentSelected) {
        UP_Button.read();
        Down_Button.read();
        Mode_Button.read();

        if (UP_Button.isPressed() && PWM_Index < (Array_Size - 1)) {
            PWM_Value = PWM[++PWM_Index];
            beep(100);
            delay(300);
        }

        if (Down_Button.isPressed() && PWM_Index > 0) {
            PWM_Value = PWM[--PWM_Index];
            beep(100);
            delay(300);
        }

        if (Mode_Button.isPressed()) {
            currentSelected = true;
            beep(300);
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(2, 10);
        display.print("Select Dischrg Curr:");
        display.setTextSize(2);
        display.setCursor(15, 30);
        display.print("I:");
        display.print(Current[PWM_Index]);
        display.print("mA");
        display.display();
    }
    return currentSelected;
}

// ============================================================================
// Function: drawBatteryOutline
// Draws the static battery icon outline.
// ============================================================================
void drawBatteryOutline() {
    display.drawRect(100, 15, 12, 20, SSD1306_WHITE);
    display.drawRect(102, 12, 8, 3, SSD1306_WHITE);
}

// ============================================================================
// Function: drawBatteryFill
// Fills the battery icon according to percentage.
// ============================================================================
void drawBatteryFill(int level) {
    int fillHeight = map(level, 0, 100, 0, 18);
    display.fillRect(102, 33 - fillHeight, 8, fillHeight, SSD1306_WHITE);
}

// ============================================================================
// Function: updateBatteryDisplay
// Animates battery icon during charging or discharging.
// ============================================================================
void updateBatteryDisplay(bool charging) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        if (charging) {
            batteryLevel += 4;
            if (batteryLevel > 100) batteryLevel = 0;
        } else {
            batteryLevel -= 4;
            if (batteryLevel < 0) batteryLevel = 100;
        }

        drawBatteryOutline();
        drawBatteryFill(batteryLevel);
    }
}

// ============================================================================
// Function: displayIRTestIcon
// Shows internal resistance result on OLED.
// ============================================================================
void displayIRTestIcon(float voltageNoLoad, float voltageLoad, float internalResistance) {
    display.clearDisplay();

    display.drawLine(34, 15, 54, 15, SSD1306_WHITE);
    display.drawLine(54, 15, 59, 20, SSD1306_WHITE);
    display.drawLine(59, 20, 64, 10, SSD1306_WHITE);
    display.drawLine(64, 10, 69, 20, SSD1306_WHITE);
    display.drawLine(69, 20, 74, 15, SSD1306_WHITE);
    display.drawLine(74, 15, 94, 15, SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(2, 35);
    display.print("IR:");
    display.print(internalResistance * 1000, 0);
    display.print("mOhm");
    display.display();
}

// ============================================================================
// Function: beep
// Simple buzzer feedback helper.
// ============================================================================
void beep(int duration) {
    digitalWrite(Buzzer, HIGH);
    delay(duration);
    digitalWrite(Buzzer, LOW);
}
