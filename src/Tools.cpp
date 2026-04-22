#include "Tools.hpp"

using namespace Math;

void Tools::Runge_Kutta4(State* s, void (*f)(State*, float*), float* pwm) {
    State s0 = *s;        // s0 is original state
    // 1. k1 = h * f(s0)
    f(s, pwm);
    *s * TIMESTEP;
    State ds = *s;        // delta_s = k1
    // 2. k2 = h * f(s0 + 0.5*k1)
    *s * 0.5f;
    *s + s0;
    f(s, pwm);
    *s * (TIMESTEP * 2.0f);
    ds += *s;        // delta_s = k1 + 2*k2
    // 3. k3 = h * f(s0 + 0.5*k2)
    *s * 0.25f;
    *s + s0;
    f(s, pwm);
    *s * (TIMESTEP * 2.0f);
    ds += *s;        // delta_s = k1 + 2*k2 + 2*k3
    // 4. k4 = h * f(s0 + k3)
    *s * 0.5f;
    *s + s0;
    f(s, pwm);
    *s * TIMESTEP;
    ds += *s;        // delta_s = k1 + 2*k2 + 2*k3 + k4
    ds *= 0.16666666f;        // delta_s = 1/6 * (k1 + 2*k2 + 2*k3 + k4)
    s0 += ds;
    s0.orientation = glm::normalize(s0.orientation);
    *s = s0;
}