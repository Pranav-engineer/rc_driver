
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

#ifndef SPEED_SRC_HPP
#define SPEED_SRC_HPP

#include "speedMapper.hpp"
class Env;


/**
 * @brief base class to define interface of various speed sources
 * 
 */
class speedSrc {
public:
    Env* env = nullptr;


    speedSrc(Env* en) : env(en)  { };
    
    /**
     * @brief get the calculated speed 
     * 
     * @return wheelSpeed resultant speed
     */
    virtual wheelSpeed getSpeed() { return wheelSpeed(); };

};




#endif //  SPEED_SRC_HPP