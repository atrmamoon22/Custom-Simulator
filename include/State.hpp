#ifndef STATE_HPP
#define STATE_HPP

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

struct State {
    glm::vec3 position;
    glm::vec3 velocity;
    glm::quat orientation;
    glm::vec3 angular_velocity;

    void operator*(float h);
    void operator+(const State& s1);
    State& operator+=(const State& s2);
    State& operator*=(float h);
};
#endif