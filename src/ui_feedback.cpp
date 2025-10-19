#include "ui_feedback.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// CRITICAL FIX: The I2S module uses GPIO25 and GPIO33.
// We must move the conflicting Green LED pin (was 25).
// GPIO15 is a safe, free pin for this purpose.

// OLED setup (I2C)
static const uint8_t SCREEN_WIDTH = 128;
static const uint8_t SCREEN_HEIGHT = 64;
// NOTE: Use -1 for the reset pin if it's tied to power or unused (common on modules)
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
static bool oled_initialized = false;

// LED / Button pins
static const uint8_t GREEN_LED  = 15; // *** CRITICAL FIX: Moved from 25 to 15 ***
static const uint8_t RED_LED    = 26;
static const uint8_t YELLOW_LED = 27;
static const uint8_t BUTTON_PIN_DEFAULT = 13; // safer default for ESP32

// Debounce
static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 50;

// Long press
static unsigned long buttonPressStart = 0;
static const unsigned long longPressDuration = 2000; // ms

// State
static bool buttonPreviouslyPressed = false;
static uint8_t buttonPin = BUTTON_PIN_DEFAULT;

void UIFeedback::init(uint8_t userButtonPin) {
    // allow caller to override button pin (0 means use default)
    if (userButtonPin != 0) buttonPin = userButtonPin;

    Serial.println("[UI] Initializing UI...");

    // Initialize LED pins
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);

    // initialize I2C bus explicitly (SDA=21, SCL=22 by default on many ESP32 boards)
    Wire.begin();

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("[UI] SSD1306 allocation failed");
        // oled_initialized remains false, functions will gracefully skip display
    } else {
        oled_initialized = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.display();
    }

    // Seed random for demo enroll/auth simulation (if used)
    randomSeed(analogRead(0));

    showMessage("System Ready");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);

    Serial.println("{\"state\":\"READY\"}");
}

void UIFeedback::loopPoll(void (*onShortPress)(), void (*onLongPress)()) {
    // Call this regularly from main loop to poll the button
    int reading = digitalRead(buttonPin);
    unsigned long now = millis();

    // Debounce logic
    if (reading == LOW) { // Button is pressed (PULLUP)
        if (!buttonPreviouslyPressed) {
            // possible press (just started)
            if (now - lastDebounceTime > debounceDelay) {
                buttonPreviouslyPressed = true;
                buttonPressStart = now;
                lastDebounceTime = now;
            }
        }
        // If button is held, it stays in the 'buttonPreviouslyPressed' state
    } else { // reading == HIGH (Button released)
        if (buttonPreviouslyPressed) {
            // possible release
            if (now - lastDebounceTime > debounceDelay) {
                unsigned long pressDuration = now - buttonPressStart;
                buttonPreviouslyPressed = false;
                lastDebounceTime = now;

                if (pressDuration >= longPressDuration) {
                    // long press detected on release
                    if (onLongPress) onLongPress();
                } else {
                    // short press detected on release
                    if (onShortPress) onShortPress();
                }
            }
        }
    }
}

// Visual / state functions
void UIFeedback::showMessage(const char *msg) {
    if (oled_initialized) {
        display.clearDisplay();
        display.setCursor(0, 20);
        display.println(msg);
        display.display();
    }
}

void UIFeedback::indicateListening() {
    showMessage("Listening...");
    Serial.println("{\"state\":\"LISTENING\"}");
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
}

void UIFeedback::indicateAnalyzing() {
    showMessage("Analyzing Voice...");
    Serial.println("{\"state\":\"ANALYZING\"}");
    digitalWrite(YELLOW_LED, LOW);
    // Keep status LEDs off or flicker for processing state if desired
}

void UIFeedback::indicateAccessGranted() {
    showMessage("Access Granted");
    Serial.println("{\"state\":\"AUTH_OK\"}");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
}

void UIFeedback::indicateAccessDenied() {
    showMessage("Access Denied");
    Serial.println("{\"state\":\"AUTH_FAIL\"}");
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
}

void UIFeedback::indicateEnrollStart() {
    showMessage("Voice Enrollment...");
    Serial.println("{\"state\":\"ENROLL\"}");
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
}

void UIFeedback::indicateEnrollDone() {
    showMessage("Voice Saved");
    Serial.println("{\"state\":\"ENROLL_DONE\"}");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
}

void UIFeedback::resetSystem() {
    showMessage("System Ready");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    Serial.println("{\"state\":\"READY\"}");
}