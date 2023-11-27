
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

#include "speedMapper.hpp"

/** @todo define following */
wheelSpeed speedMapper::map(pathSpeed& vec){
    // dummy test code
    wheelSpeed speed;
    speed.rawSpeed[0] = vec.mag;
    speed.rawSpeed[1] = vec.mag * vec.u;

    if(vec.left){
        speed.rawSpeed[0] = speed.rawSpeed[1];
        speed.rawSpeed[1] = vec.mag;
    }

    return speed;
};


/** @todo define following */
wheelSpeed speedMapper::map(phySpeed& phy){
    return wheelSpeed();
};