# Arduino PID Line Follower

This is a simple line follower robot that uses PID (Proportional, Integral, Derivative) control to adjust motor speeds based on sensor readings. The robot follows a line by adjusting the speed of the motors according to the difference in sensor values, which helps the robot stay on the line.

## Hardware Components

- Arduino Uno (or any compatible microcontroller)
- 2 Motors (Left and Right)
- 2 Line sensors (Left and Right)
- Motor driver (L298N or similar)
- Chassis for the robot
- Power supply for motors and Arduino

## Pin Connections

- **Left Motor Pins:**
  - `leftMotorPin1`: Pin 3
  - `leftMotorPin2`: Pin 4
- **Right Motor Pins:**
  - `rightMotorPin1`: Pin 5
  - `rightMotorPin2`: Pin 6
- **Sensor Pins:**
  - `leftSensorPin`: Pin A0
  - `rightSensorPin`: Pin A1

## PID Control

This implementation uses a simple PID (Proportional, Integral, Derivative) control algorithm to adjust the motor speeds based on the error between the left and right sensor readings.

- **Kp (Proportional Gain):** 1.0
- **Ki (Integral Gain):** 0.0
- **Kd (Derivative Gain):** 0.0

These constants may need to be adjusted depending on your robot's hardware and line-following environment.

## How it Works

1. **Sensors:** The left and right line sensors detect the position of the line.
2. **Error Calculation:** The error is calculated as the difference between the left and right sensor readings.
3. **PID Computation:** The error is used to calculate the proportional, integral, and derivative components of the PID control loop.
4. **Motor Speed Adjustment:** The motors' speeds are adjusted based on the PID output to ensure the robot follows the line smoothly.
5. **Direction Control:** The robot adjusts its speed on either side to correct its course when it deviates from the line.

## Code Breakdown

### Pin Definitions

```cpp
// Define motor pins
const int leftMotorPin1 = 3;
const int leftMotorPin2 = 4;
const int rightMotorPin1 = 5;
const int rightMotorPin2 = 6;

// Define sensor pins
const int leftSensorPin = A0;
const int rightSensorPin = A1;
```

## PID Constants:

```cpp
// PID constants
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;
```

- These constants control how the robot responds to error. Kp affects the proportional control, Ki affects the integral control, and Kd affects the derivative control.

## Setup Function:

```cpp
void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
}

```

- Here’s a detailed README.md for your Arduino PID Line Follower project:

markdown

# Arduino PID Line Follower

This is a simple line follower robot that uses PID (Proportional, Integral, Derivative) control to adjust motor speeds based on sensor readings. The robot follows a line by adjusting the speed of the motors according to the difference in sensor values, which helps the robot stay on the line.

## Hardware Components

- Arduino Uno (or any compatible microcontroller)
- 2 Motors (Left and Right)
- 2 Line sensors (Left and Right)
- Motor driver (L298N or similar)
- Chassis for the robot
- Power supply for motors and Arduino

## Pin Connections

- **Left Motor Pins:**
  - `leftMotorPin1`: Pin 3
  - `leftMotorPin2`: Pin 4
- **Right Motor Pins:**
  - `rightMotorPin1`: Pin 5
  - `rightMotorPin2`: Pin 6
- **Sensor Pins:**
  - `leftSensorPin`: Pin A0
  - `rightSensorPin`: Pin A1

## PID Control

This implementation uses a simple PID (Proportional, Integral, Derivative) control algorithm to adjust the motor speeds based on the error between the left and right sensor readings.

- **Kp (Proportional Gain):** 1.0
- **Ki (Integral Gain):** 0.0
- **Kd (Derivative Gain):** 0.0

These constants may need to be adjusted depending on your robot's hardware and line-following environment.

## How it Works

1. **Sensors:** The left and right line sensors detect the position of the line.
2. **Error Calculation:** The error is calculated as the difference between the left and right sensor readings.
3. **PID Computation:** The error is used to calculate the proportional, integral, and derivative components of the PID control loop.
4. **Motor Speed Adjustment:** The motors' speeds are adjusted based on the PID output to ensure the robot follows the line smoothly.
5. **Direction Control:** The robot adjusts its speed on either side to correct its course when it deviates from the line.

## Code Breakdown

### Pin Definitions

```cpp
// Define motor pins
const int leftMotorPin1 = 3;
const int leftMotorPin2 = 4;
const int rightMotorPin1 = 5;
const int rightMotorPin2 = 6;

// Define sensor pins
const int leftSensorPin = A0;
const int rightSensorPin = A1;
```

### PID Constants

```cpp

// PID constants
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;
```

- These constants control how the robot responds to error. Kp affects the proportional control, Ki affects the integral control, and Kd affects the derivative control.

### Setup Function

```cpp
void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
}
```

The setup function initializes the motor pins as outputs.

### Loop Function:

```cpp
void loop() {
  // Read sensor values
  int leftSensorValue = analogRead(leftSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  // Calculate error
  int error = leftSensorValue - rightSensorValue;

  // PID calculations
  double proportional = Kp * error;
  integral += error;
  double derivative = error - previousError;
  double pidOutput = proportional + (Ki * integral) + (Kd * derivative);

  // Update previous error
  previousError = error;

  // Set motor speeds
  int leftSpeed = 255 - pidOutput;
  int rightSpeed = 255 + pidOutput;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive motors
  if (leftSpeed > 0) {
      analogWrite(leftMotorPin1, leftSpeed);
      analogWrite(leftMotorPin2, 0);
  } else {
      analogWrite(leftMotorPin1, 0);
      analogWrite(leftMotorPin2, -leftSpeed);
  }

  if (rightSpeed > 0) {
      analogWrite(rightMotorPin1, rightSpeed);
      analogWrite(rightMotorPin2, 0);
  } else {
      analogWrite(rightMotorPin1, 0);
      analogWrite(rightMotorPin2, -rightSpeed);
  }

  // Delay for a short period to ensure smooth operation
  delay(10);
}

```

## Explanation

- Sensor Reading: The analogRead() function reads the values from the left and right line sensors.
- Error Calculation: The difference between the left and right sensor values is used as the error term.
- PID Calculation: Proportional, integral, and derivative terms are calculated using the error.
- Motor Control: The motor speeds are adjusted according to the PID output, and the analogWrite() function is used to drive the motors.

## Installation:

1. Connect the motors and sensors to the Arduino as per the pin definitions.
2. Upload the code to your Arduino board using the Arduino IDE.
3. Power up the robot and place it on a track with a clear line to follow.

## Tuning:

- You can experiment with the Kp, Ki, and Kd values to optimize the line-following performance for your robot. Start by adjusting Kp to correct proportional errors, then fine-tune with Ki and Kd.
