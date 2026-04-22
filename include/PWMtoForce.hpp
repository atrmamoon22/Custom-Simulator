#ifndef PWM_TO_FORCE_HPP
#define PWM_TO_FORCE_HPP

#include <glm/vec3.hpp>
#include <cmath>
#include <iostream>
#include "MathConstants.hpp"
#include "VehicleInfo.hpp"

class PWMtoForce {
    public:
        glm::vec3 calculateNetForce();

        glm::vec3 calculateNetTorque();

        float pwmToForce(float pwm);

        void populateThrusterForces(float pwms[]);

    private:
        double thrusterForces[8] = {};
};
#endif