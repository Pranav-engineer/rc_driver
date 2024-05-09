
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
#include <esp_log.h>
#include <driver/gpio.h>

#include "env.hpp"
#include "ps4Handler.hpp"
#include "qmdHandler.hpp"

#define LOG "MAIN"

Env env;
speedMapper map;
int pwmPins[] = {
    GPIO_NUM_12,             //  front right
    GPIO_NUM_13,            //  front left
    GPIO_NUM_14,             // centre right
    GPIO_NUM_27,             // center left
    GPIO_NUM_26,             // back right
    GPIO_NUM_25,             // back left
    GPIO_NUM_33,
    GPIO_NUM_32,
    GPIO_NUM_22,
    GPIO_NUM_23,
    GPIO_NUM_15,
    GPIO_NUM_19,
    };

int dirPins[] = {26, 25, 33, 32};
// int pwmPins[] = {12, 13, 14, 27};
// int dirPins[] = {26, 25, 33, 32};

qmdHandler upd;


/**
 * @brief testSource to simulate speedSrc for debugging hardware errors 
 * 
 */
class testSource : public speedSrc{

float val = 0.0f, diff = 0.01f;
public:

    testSource(Env* env) : speedSrc(env) {};
     
    wheelSpeed getSpeed() {
        if(val >= 0.9f || val <= 0.1f) diff = - diff;
        val += diff;

        wheelSpeed spd;
        spd.rawSpeed[0] = val;
        spd.rawSpeed[1] = val;
        spd.rawSpeed[2] = val;
        spd.rawSpeed[3] = 0;
        spd.rawSpeed[4] = 0;
        spd.rawSpeed[5] = 0;

        return spd;
    };
};




extern "C" void app_main(void){
    ESP_LOGI(LOG, "RC driver is currently under development\n");

    env =  Env{
        .mapper = &map,
        .motorHandler = new qmd(6, pwmPins, dirPins)
    };
    
    // env.src = new testSource(&env);
    env.src = new ps4Handler(&env, "B0:DC:EF:DD:39:56");
    // env.src = new urosHandler(&env);


    // upd = qmdHandler(&env);
    // upd.run();
};      