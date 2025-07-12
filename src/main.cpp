#include <Arduino.h>
#include "platform_kinematics.h"

PlatformParams motor1 = {
    .pa = 50,
    .pb = 20,
    .ha = 30,
    .hb = 25,
    .ta = 35,
    .tb = 15,
    .r  = 60,
    .l  = 70
};

PlatformParams motor2 = {
    .pa = 48,
    .pb = 19,
    .ha = 29,
    .hb = 26,
    .ta = 34,
    .tb = 16,
    .r  = 60,
    .l  = 70
};

void setup() {
    Serial.begin(115200);

    float roll_deg = 10.0;
    float pitch_deg = 15.0;
    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;

    float alpha1 = 0, alpha2 = 0;

    computeMotorAngles(roll_rad, pitch_rad, motor1, motor2, alpha1, alpha2);

    Serial.print("Motor 1 angle: ");
    Serial.print(alpha1 * RAD_TO_DEG, 2);
    Serial.println(" deg");

    Serial.print("Motor 2 angle: ");
    Serial.print(alpha2 * RAD_TO_DEG, 2);
    Serial.println(" deg");
}

void loop() {
    // Nothing for now
}
