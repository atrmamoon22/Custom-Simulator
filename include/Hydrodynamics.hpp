#ifndef HYDRODYNAMICS_HPP
#define HYDRODYNAMICS_HPP

#include <glm/vec3.hpp>
#include <iostream>
#include <glm/geometric.hpp>
#include <cmath>
#include "VehicleInfo.hpp"

class Hydrodynamics {
    public:
        static glm::vec3 calculateLinearDrag(glm::vec3 vel);

        static glm::vec3 calculateQuadraticDrag(glm::vec3 vel);

        static glm::vec3 calculateCorriolisA(glm::vec3 vel, glm::vec3 ang_vel, bool torque);

        static glm::vec3 calculateCorriolisRB(glm::vec3 vel, glm::vec3 ang_vel, bool torque);
};
#endif