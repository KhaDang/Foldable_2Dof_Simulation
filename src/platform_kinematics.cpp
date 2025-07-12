#include "platform_kinematics.h"
#include <cmath>

float computeMotorAngle(float theta, const PlatformParams &params) {
    float mh = params.pa * cos(theta) - params.pb + (params.ha - params.ta) * sin(theta);
    float mv = params.pa * sin(theta) + params.tb - params.hb + (params.ta - params.ha) * cos(theta);
    float m = sqrt(mh * mh + mv * mv);

    float cos_beta = (params.r * params.r + m * m - params.l * params.l) / (2.0f * params.r * m);
    cos_beta = fminf(1.0f, fmaxf(-1.0f, cos_beta));
    float beta = acos(cos_beta);

    float cos_alpha_plus_beta = (mh * mh + m * m - params.r * params.r) / (2.0f * mh * m);
    cos_alpha_plus_beta = fminf(1.0f, fmaxf(-1.0f, cos_alpha_plus_beta));
    float alpha_plus_beta = acos(cos_alpha_plus_beta);

    float alpha = alpha_plus_beta - beta;
    return alpha;
}

void computeMotorAngles(float roll, float pitch, const PlatformParams &params1, const PlatformParams &params2, float &alpha1, float &alpha2) {
    
    float theta1 = roll;  // could be any mapping depending on geometry
    float theta2 = pitch;

    alpha1 = computeMotorAngle(theta1, params1);
    alpha2 = computeMotorAngle(theta2, params2);
}
