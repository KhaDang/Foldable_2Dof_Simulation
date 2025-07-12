#include <Arduino.h>
#include "platform_kinematics.h"

// === Motor Definitions ===
StepperMotor motor1 = {18, 19, 800, 0.0};  // STEP, DIR, steps/rev, initial angle
StepperMotor motor2 = {21, 22, 800, 0.0};  // Adjust pins as needed

PlatformParams params1 = {50, 20, 30, 25, 35, 15, 60, 70};
PlatformParams params2 = {48, 19, 29, 26, 34, 16, 60, 70};

// === Motor Control ===
void setupMotor(const StepperMotor& m) {
    pinMode(m.stepPin, OUTPUT);
    pinMode(m.dirPin, OUTPUT);
}

void moveMotorToAngle(StepperMotor& m, float targetAngle) {
    float deltaAngle = targetAngle - m.currentAngle;

    // Normalize to [-180, 180] for shortest path
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

    int steps = abs(deltaAngle) * m.stepsPerRev / 360.0;
    bool direction = deltaAngle > 0;

    digitalWrite(m.dirPin, direction ? HIGH : LOW);

    for (int i = 0; i < steps; ++i) {
        digitalWrite(m.stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(m.stepPin, LOW);
        delayMicroseconds(1000);
    }

    m.currentAngle = targetAngle;  // Update motor state
}

// === Setup & Loop ===
void setup() {
    Serial.begin(115200);

    setupMotor(motor1);
    setupMotor(motor2);

    delay(1000); // optional pause

    // Simulate tilt input
    float roll_deg = 10.0;
    float pitch_deg = 15.0;

    float roll = roll_deg * DEG_TO_RAD;
    float pitch = pitch_deg * DEG_TO_RAD;

    float alpha1, alpha2;
    computeMotorAngles(roll, pitch, params1, params2, alpha1, alpha2);

    float deg1 = alpha1 * RAD_TO_DEG;
    float deg2 = alpha2 * RAD_TO_DEG;

    Serial.print("Target α1: "); Serial.print(deg1); Serial.println("°");
    Serial.print("Target α2: "); Serial.print(deg2); Serial.println("°");

    moveMotorToAngle(motor1, deg1);
    moveMotorToAngle(motor2, deg2);

    Serial.println("Motors moved.");
}

void loop() {
    // Add feedback loop or new command input here
}
