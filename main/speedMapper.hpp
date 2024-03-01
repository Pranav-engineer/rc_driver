
// rc_driver - micro ros2 node for locomotion 
// Copyright (C) 2023  akshay <akshayb@gmx.com>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#ifndef SPEED_MAPPER_HPP
#define SPEED_MAPPER_HPP

#include <inttypes.h>

/**
 * @brief joystick inputs
 * 
 */
struct joyStick
{
    float x = 0, y = 0, sin = 0, cos = 0, mag = 0;
};


struct circularJoyStick
{
    float xc = 0, yc = 0;
};


/**
 * @brief wheelSpeed to represent normalized wheel speed
 */
struct wheelSpeed {

    float rawSpeed[4] = {0.0f};
    float &v1 = rawSpeed[0];
    float &v2 = rawSpeed[1];
    float &v3 = rawSpeed[2];
    float &v4 = rawSpeed[3];
};


/**
 * @brief speed in terms of circular path 
 * mag - normalized magnitude of the speed
 * u - ratio of minor wheel to the major wheel ( exterior ) while rotation 
 * left - vector belongs to left of y axis or not.
 * 
 */
struct pathSpeed
{
    float mag = 0.0f, u = 0.0f;
    bool left = true;
};


/**
 * @brief speed in terms of circular path 
 * mag - normalized magnitude of the speed
 * r - radius of rotation (if radius < 0 will imply radius = infinity (straight path))
 * left - path belongs to left of y axis or not.
 * 
 */
struct radialSpeed
{
    float mag = 0.0f, r = 0.0f;
    bool left = true;
};


/**
 * @brief speed in terms of physical params
 * v - normalized forward velocity
 * w - normalized angular speed
 */
struct phySpeed {
    float param[2] = {0.0f};
    float& v = param[0];
    float& w = param[1];
};


/**
 * @brief speedMapper maps various representation of motion to motor speeds
 * extend the class to redefine speed mapping according to the vehicle.
 * By default the class defines mapping for a two wheeler diffrential drive bot.
 * 
 */
class speedMapper {

public:


    uint8_t factor = 0.0f;
    float wheelRatio = 0.0f,  maxRatio = 1.0f, minRatio = 0.2f;
    virtual wheelSpeed map(pathSpeed& vec);
    virtual wheelSpeed map(phySpeed& phy);
    virtual wheelSpeed map(joyStick& joy);
    // virtual wheelSpeed map(circularJoyStick& joy);
};


#endif //  SPEED_MAPPER_HPP
