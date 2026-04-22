    #ifndef VEHICLE_INFO_HPP
    #define VEHICLE_INFO_HPP

    #include <glm/mat2x4.hpp>

    namespace VehicleInfo {
    
    inline constexpr int HOR_THRUSTERS_QUADRANT[4][2] = {
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1}
    };        // Quadrant of direction of positive force
    inline constexpr float VEHICLE_MASS = 17.3;        // in kg
    inline constexpr float VEHICLE_Ixx = 0.8043058;
    inline constexpr float VEHICLE_Iyy = 1.0919833;
    inline constexpr float VEHICLE_Izz = 1.6520058;
    inline constexpr float VOLUME_DISPLACED_WATER = 0.0173;        // in meters cube. 0.02227
    inline constexpr float HOR_THRUSTERS_DIST_FROM_Z_AXIS = 0.328;        // in meters
    inline constexpr float VER_THRUSTERS_POS[4][2] = {
        {0.1200, -0.1580},
        {0.1200, 0.1580},
        {-0.1200, -0.1580},
        {-0.1200, 0.1580}
    };        // in meters

    // Hydrodynamic constants

    inline constexpr float xDotU = 0;        // added mass in surge
    inline constexpr float yDotV = 0;        // added mass in sway
    inline constexpr float zDotW = 0;        // added mass in heave
    inline constexpr float kDotP = 0;        // added mass in roll
    inline constexpr float mDotQ = 0;        // added mass in pitch
    inline constexpr float nDotR = 0;        // added mass in yaw
    inline constexpr float xUabsU = -67.43;       // quadratic drag in surge
    inline constexpr float xU = -2.023;           // linear drag in surge
    inline constexpr float yVabsV = -56.43;       // quadratic drag in sway
    inline constexpr float yV = -1.69;            // linear drag in sway
    inline constexpr float zWabsW = -116.4;       // quadratic drag in heave
    inline constexpr float zW = -3.492;           // linear drag in heave
    inline constexpr float kPabsP = -5.18;        // quadratic drag in roll
    inline constexpr float kP = -0.21;            // lineat drag in roll
    inline constexpr float mQabsQ = -5.228;       // quadratic drag in pitch
    inline constexpr float mQ = -0.226;           // linear drag in pitch
    inline constexpr float nRabsR = -9.00;        // quadratic drag in yaw
    inline constexpr float nR = -0.45;            // linear drag in yaw
    }
    #endif