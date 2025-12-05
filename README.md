# ESP32-Air-Mouse
ESP32 Air Mouse using MPU6050 & BLE

This project turns an ESP32 into a wireless BLE Air Mouse, using an MPU6050 gyroscope to control cursor movement and physical buttons for left/right click. The ESP32 connects to a computer or smartphone via Bluetooth BLE and acts like a mouse.

Features

✔️ MPU6050 Motion-based cursor control

✔️ BLE Mouse support using BleMouse.h

✔️ Left button supports drag & hold functionality

✔️ Right button with debouncing for accurate clicking

✔️ Smooth cursor movement using low-pass filtering

✔️ Adjustable sensitivity & deadzone

Hardware Requirements
Component	Quantity
ESP32 Dev Board	1
MPU6050 Sensor	1
Push Buttons	2
Jumper wires	As required
Pin Connections
MPU6050	ESP32
VCC	3.3V
GND	GND
SCL	GPIO 22
SDA	GPIO 21
Buttons	ESP32
Left Click	GPIO 19
Right Click	GPIO 5
Libraries Required

Install via Arduino Library Manager:

BleMouse

MPU6050_light

Wire

How it Works

The ESP32 continuously reads gyroscope values from the MPU6050.

After filtering and deadzone elimination, the values are converted to cursor movements.

Button presses send BLE mouse events:

Left button: press & hold for drag

Right button: single click with debouncing

Adjustable Parameters
Variable	Function
gyroDeadzone	Ignore small unintentional movements
filterAlpha	Smoothing factor for gyro filtering
sensitivity	Cursor speed scale
