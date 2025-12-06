/**
 * ---------------------------------------------------------------------------
 * Project: ESP32 BLE Air Mouse using MPU6050
 * Description:
 *   This project turns an ESP32 into a BLE (Bluetooth Low Energy) Mouse.
 *   Motion tracking is done using the MPU6050 gyroscope sensor.
 *   Left button controls drag (press & hold), right button performs right-click.
 *
 * Hardware Connections:
 *   MPU6050 -> ESP32
 *      SDA -> GPIO 21
 *      SCL -> GPIO 22
 *   Left Button  -> GPIO 19 (Pull-up enabled)
 *   Right Button -> GPIO 5  (Pull-up enabled)
 *
 * Libraries Required:
 *   - BleMouse
 *   - MPU6050_light
 *   - Wire
 * ---------------------------------------------------------------------------
 */

#include <Wire.h>
#include <BleMouse.h>
#include <MPU6050_light.h>

// ------------------------- Pin Definitions -------------------------------
#define SDA_PIN 21
#define SCL_PIN 22
const int leftPin = 19;
const int rightPin = 5;

// ------------------------- Motion Settings --------------------------------
const float gyroDeadzone = 2.0;   // Minimum gyro threshold (deg/s)
const float filterAlpha  = 0.2;   // Smoothing factor for low-pass filter (0-1)
const float sensitivity  = 0.25;  // Cursor sensitivity

// ------------------------- Button Debounce --------------------------------
const int debounceDelay = 40; // milliseconds
int lastLeftState, stableLeftState;
int lastRightState, stableRightState;
unsigned long lastLeftTime, lastRightTime;

// ------------------------- BLE & Sensor Objects ---------------------------
MPU6050 mpu(Wire);
BleMouse bleMouse;

// ------------------------- State Variables --------------------------------
float smoothX = 0, smoothY = 0;
bool leftDragging = false;

// ===========================================================================
//                                SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);

  // Initialize I2C for MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  mpu.calcGyroOffsets();  // Gyro auto-calibration

  // Start BLE mouse
  bleMouse.begin();

  // Configure buttons (input with pull-up)
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);

  // Initialize button states
  lastLeftState = stableLeftState = digitalRead(leftPin);
  lastRightState = stableRightState = digitalRead(rightPin);
  lastLeftTime = lastRightTime = millis();
}

// ===========================================================================
//                                LOOP
// ===========================================================================
void loop() {
  // Only run when device is connected via BLE
  if (bleMouse.isConnected()) {
    
    // ---------- Read Gyroscope Data ----------
    mpu.update();
    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();

    // Apply a deadzone to ignore small noise values
    if (fabs(gyroX) < gyroDeadzone) gyroX = 0;
    if (fabs(gyroY) < gyroDeadzone) gyroY = 0;

    // Apply low-pass filtering to smooth motion
    smoothX = smoothX * (1 - filterAlpha) + gyroX * filterAlpha;
    smoothY = smoothY * (1 - filterAlpha) + gyroY * filterAlpha;

    // Convert gyro movement to cursor movement
    int moveX = (int)(sensitivity * smoothY);
    int moveY = (int)(-sensitivity * smoothX);

    bleMouse.move(moveX, moveY);  // Move cursor

    // ---------- LEFT BUTTON (Drag / Hold) ----------
    int readingLeft = digitalRead(leftPin);
    if (readingLeft == LOW && lastLeftState == HIGH) {
        bleMouse.press(MOUSE_LEFT);   // Start drag
        leftDragging = true;
    }
    if (readingLeft == HIGH && lastLeftState == LOW) {
        bleMouse.release(MOUSE_LEFT); // End drag
        leftDragging = false;
    }
    lastLeftState = readingLeft;

    // ---------- RIGHT BUTTON (Debounced Click) ----------
    int readingRight = digitalRead(rightPin);
    if (readingRight != lastRightState) {
      lastRightTime = millis();
    }
    if ((millis() - lastRightTime) > debounceDelay) {
      if (readingRight != stableRightState) {
        stableRightState = readingRight;
        if (stableRightState == LOW) {
          Serial.println("Right Click");
          bleMouse.click(MOUSE_RIGHT);
        }
      }
    }
    lastRightState = readingRight;
  }

  delay(10);  // Loop stability delay
}
