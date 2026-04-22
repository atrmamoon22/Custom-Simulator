#include "State.hpp"

void State::operator*(float h) {
    this->position *= h;
    this->velocity *= h;
    this->orientation *= h;
    this->angular_velocity *= h;
}

void State::operator+(const State& s1) {
    this->position += s1.position;
    this->velocity += s1.velocity;
    this->orientation += s1.orientation;
    this->angular_velocity += s1.angular_velocity;
}

State& State::operator+=(const State& s2) {
    position += s2.position;
    velocity += s2.velocity;
    orientation += s2.orientation;
    angular_velocity += s2.angular_velocity;

    return *this;
}

State& State::operator*=(float h) {
    position *= h;
    velocity *= h;
    orientation *= h;
    angular_velocity *= h;

    return *this;
}