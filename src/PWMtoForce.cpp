#include "PWMtoForce.hpp"

using namespace Math;
using namespace VehicleInfo;

void PWMtoForce::populateThrusterForces(float pwm[]) {
    for (int i = 0; i < 8; i++) {                       // from T1 to T8
        thrusterForces[i] = pwmToForce(pwm[i]);
        // std::cout<<thrusterForces[i]<<",";
    }
    // std::cout<<'\n';
}

float PWMtoForce::pwmToForce(float pwm) {
    double force = 0.0;
    if (pwm <= 1470) {
        force = -(1.76039251e-08)*(std::pow(pwm, 3)) + (5.61838863e-05)*(std::pow(pwm, 2)) - (4.68876355e-02)*(pwm) + 3.43221934e+00;  // in kgf
    } else if (pwm >= 1530) {
        pwm = 3000 - pwm;
        // force = -(1.80258467e-08)*(std::pow(pwm, 3)) + (1.09184800e-04)*(std::pow(pwm, 2)) - (2.02323186e-01)*(pwm) + 1.18530182e+02;  // in kgf
        force = (1.76039251e-08)*(std::pow(pwm, 3)) - (5.61838863e-05)*(std::pow(pwm, 2)) + (4.68876355e-02)*(pwm) - 3.43221934e+00;
    } else {
        force = 0;
    }

    return force * 9.80665;     // in Newtons
}

glm::vec3 PWMtoForce::calculateNetForce() {             // in NED Frame
    double x = 0.0, y = 0.0, z = 0.0;
    for (int i = 0; i < 4; i++) {                       // Horizontal plane thrusters
        x += HOR_THRUSTERS_QUADRANT[i][0] * thrusterForces[i] * INV_SQRT2;
        y += HOR_THRUSTERS_QUADRANT[i][1] * thrusterForces[i] * INV_SQRT2;
    }
    for (int i = 4; i < 8; i++) {                       // Vertical thrusters
        z -= thrusterForces[i];
    }
    return glm::vec3(x, y, z);
}

glm::vec3 PWMtoForce::calculateNetTorque() {            // in NED Frame
    double x = 0.0, y = 0.0, z = 0.0;
    for (int i = 0; i < 4; i++) {                       // Horizontal plane thrusters
        z += (HOR_THRUSTERS_QUADRANT[i][0] * HOR_THRUSTERS_QUADRANT[i][1]) * thrusterForces[i] * HOR_THRUSTERS_DIST_FROM_Z_AXIS;
    }
    for (int i = 4; i < 8; i++) {                       // Vertical thrusters
        x -= thrusterForces[i] * VER_THRUSTERS_POS[i-4][1];
        y += thrusterForces[i] * VER_THRUSTERS_POS[i-4][0];
    }

    return glm::vec3(x, y, z);
}