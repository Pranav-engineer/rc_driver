
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

#ifndef PS4_HANDLER_HPP
#define PS4_HANDLER_HPP


#include <PS4Controller.h>

#include "speedSrc.hpp"


/**
 * @brief ps4Handler handles ps4 interface and UI and complete I/O
 * 
 */
class ps4Handler : public speedSrc {

private:

    static PS4Controller ps;
    static ps4Handler* handler;


    static void connectCallback();
    static void disConnectCallback();
    static void eventCallback();

public:

    float maxSpeed = 0.5f;
    int8_t& joyX = ps.data.analog.stick.rx , &joyY = ps.data.analog.stick.ry;

    ps4Handler(Env* env, const char* macId);
    vectorSpeed velocity;
    wheelSpeed getSpeed();
};


#endif //  PS4_HANDLER_HPP