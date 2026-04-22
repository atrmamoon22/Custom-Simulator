#ifndef TOOLS_HPP
#define TOOLS_HPP

#include "State.hpp"
#include <iostream>
#include "MathConstants.hpp"

class Tools {
    public:
        static void Runge_Kutta4(State* s, void (*f)(State*, float*), float* pwm);
};
#endif