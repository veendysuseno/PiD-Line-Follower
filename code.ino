// Define motor pins
const int leftMotorPin1 = 3;
const int leftMotorPin2 = 4;
const int rightMotorPin1 = 5;
const int rightMotorPin2 = 6;

// Define sensor pins
const int leftSensorPin = A0;
const int rightSensorPin = A1;

// PID constants
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

// PID variables
double setPoint = 0;
double input, output;
double previousError = 0;
double integral = 0;

void setup() {
  // Initialize motor pins
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
}

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

    // Constrain motor speeds to valid PWM range
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
