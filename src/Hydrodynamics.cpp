#include "Hydrodynamics.hpp"

using namespace VehicleInfo;

glm::vec3 Hydrodynamics::calculateQuadraticDrag(glm::vec3 vel) {
    // std::cout<<vel.x<<","<<vel.y<<","<<vel.z<<std::endl;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    x = xUabsU * std::abs(vel.x) * vel.x;
    y = yVabsV * std::abs(vel.y) * vel.y;
    z = zWabsW * std::abs(vel.z) * vel.z;
    // std::cout<<x<<","<<y<<","<<z<<std::endl;
    return glm::vec3(x, y, z);
}

glm::vec3 Hydrodynamics::calculateLinearDrag(glm::vec3 vel) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    x = xU * vel.x;
    y = yV * vel.y;
    z = zW * vel.z;
    // std::cout<<x<<","<<y<<","<<z<<std::endl;
    return glm::vec3(x, y, z);
}

glm::vec3 Hydrodynamics::calculateCorriolisA(glm::vec3 vel, glm::vec3 ang_vel, bool torque) {
    glm::vec3 M_A_linear = glm::vec3(xDotU, yDotV, zDotW);
    glm::vec3 p = M_A_linear * vel;
   
    if (!torque) {
        glm::vec3 test = glm::cross(ang_vel, p);
        // std::cout<<test.x<<","<<test.y<<","<<test.z<<std::endl;
        return test;
    } else {
        glm::vec3 M_A_angular = glm::vec3(kDotP, mDotQ, nDotR);
        glm::vec3 h = M_A_angular * ang_vel;
        return (glm::cross(ang_vel, h) + glm::cross(vel, p));
    }
}

glm::vec3 Hydrodynamics::calculateCorriolisRB(glm::vec3 vel, glm::vec3 ang_vel, bool torque) {
    glm::vec3 p = VEHICLE_MASS * vel;        // if center of gravity is not at center, add glm::cross(vel, rB), where rB is distance between center of gravity and body center
   
    if (!torque) {
        glm::vec3 test = glm::cross(ang_vel, p);
        // std::cout<<test.x<<","<<test.y<<","<<test.z<<std::endl;
        return test;
    } else {
        glm::vec3 MomentOfInertia = glm::vec3(VEHICLE_Ixx, VEHICLE_Iyy, VEHICLE_Izz);
        glm::vec3 h = MomentOfInertia * ang_vel;
        return glm::cross(ang_vel, h);        // if center of gravity is not at center, add glm::cross(vel, p), where p is new p
    }
}