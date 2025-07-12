#ifndef PLATFORM_KINEMATICS_H
#define PLATFORM_KINEMATICS_H

struct PlatformParams {
    float pa;
    float pb;
    float ha;
    float hb;
    float ta;
    float tb;
    float r;
    float l;
};

struct StepperMotor {
    int stepPin;
    int dirPin;
    int stepsPerRev;     // e.g. 800 for 1.8° motor with 1/4 microstepping
    float currentAngle;  // tracked in degrees
};


// Compute motor angle α given platform angle θ (radians)
float computeMotorAngle(float theta, const PlatformParams &params);

// Compute 2 motor angles (e.g., roll and pitch actuators)
void computeMotorAngles(float roll, float pitch, const PlatformParams &params1, const PlatformParams &params2, float &alpha1, float &alpha2);

#endif
