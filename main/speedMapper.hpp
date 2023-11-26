
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
 * @brief body velocity in terms of vector representation
 * mag - normalized magnitude of the speed
 * cos - cosine of the vector, with respect to +y axis (forward direction)
 * left - vector belongs to left of y axis or not.
 * 
 */
struct vectorSpeed
{
    float mag = 0.0f, cos = 0.0f;
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

    virtual wheelSpeed map(vectorSpeed& vec);
    virtual wheelSpeed map(phySpeed& phy);
};


#endif //  SPEED_MAPPER_HPP
