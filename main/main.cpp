
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



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>
#include <qmd.hpp>

#include "env.hpp"
#include "ps4Handler.hpp"
#include "qmdHandler.hpp"


Env env;
speedMapper map;
int pwmPins[] = {12, 13, 14, 27};
int dirPins[] = {26, 25, 33, 32};

qmdHandler upd;

extern "C" void app_main(void){
    printf("RC driver is currently under development\n");

    env =  Env{
        .mapper = &map,
        .src = new ps4Handler(&env, "b8:d6:1a:44:98:7e"),
        .motorHandler = new qmd(4, pwmPins, dirPins)
    };

    upd = qmdHandler(&env);
    upd.run();
};      