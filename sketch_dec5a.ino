#include <Wire.h>
#include <BleMouse.h>
#include <MPU6050_light.h>

#define SDA_PIN 21
#define SCL_PIN 22

MPU6050 mpu(Wire);
BleMouse bleMouse;

const int leftPin = 19;
const int rightPin = 5;
const int debounceDelay = 40;

const float gyroDeadzone = 2.0; // deg/s threshold for deadzone
const float filterAlpha = 0.2;  // smoothing factor (0<α<1)
const float sensitivity = 0.25; // scale factor for cursor speed

int lastLeftState, stableLeftState;
int lastRightState, stableRightState;
unsigned long lastLeftTime, lastRightTime;

float smoothX = 0, smoothY = 0;
bool leftDragging = false;  // tracks if left button is held down for dragging

void setup() {
  Serial.begin(115200);
  // Initialize I2C and MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  mpu.calcGyroOffsets();  // Calibrate gyroscope to remove offset
  // Initialize BLE mouse
  bleMouse.begin();

  // Setup buttons with pull-ups
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  lastLeftState = stableLeftState = digitalRead(leftPin);
  lastRightState = stableRightState = digitalRead(rightPin);
  lastLeftTime = lastRightTime = millis();
}

void loop() {
  if (bleMouse.isConnected()) {
    // Read and filter gyro data
    mpu.update();
    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    // Deadzone: ignore small values
    if (fabs(gyroX) < gyroDeadzone) gyroX = 0;
    if (fabs(gyroY) < gyroDeadzone) gyroY = 0;
    // Low-pass filter for smoothing
    smoothX = smoothX * (1 - filterAlpha) + gyroX * filterAlpha;
    smoothY = smoothY * (1 - filterAlpha) + gyroY * filterAlpha;

    // Map gyro to cursor movement
    int moveX = (int)(sensitivity * smoothY);
    int moveY = (int)(-sensitivity * smoothX);
    bleMouse.move(moveX, moveY);

    int readingLeft = digitalRead(leftPin);
    // Detect button press (edge detection)
    if (readingLeft == LOW && lastLeftState == HIGH) {
        bleMouse.press(MOUSE_LEFT);  // start drag (press and hold)
        leftDragging = true;
    }

    // Detect button release
    if (readingLeft == HIGH && lastLeftState == LOW) {
        bleMouse.release(MOUSE_LEFT); // stop drag (release)
        leftDragging = false;
    }

    // Update last state
    lastLeftState = readingLeft;


    // Debounce right button
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
  delay(10);  // short delay to control loop timing
}