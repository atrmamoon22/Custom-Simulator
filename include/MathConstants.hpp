#ifndef MATH_COMSTANTS_HPP
#define MATH_COMSTANTS_HPP

namespace Math {

inline constexpr float PI = 3.14159265358979323846f;
inline constexpr float TWO_PI = 2 * PI;
inline constexpr float HALF_PI = PI / 2.0f;
inline constexpr float SQRT2 = 1.41421356237f;
inline constexpr float INV_SQRT2 = 0.70710678118f;

inline constexpr float WATER_DENSITY = 1000;        // in (kg per meters cube)
inline constexpr float TIMESTEP = 0.01;
};
#endif