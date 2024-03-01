
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

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "ps4Handler.hpp"
#include "env.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/idf_additions.h"

PS4Controller ps4Handler::ps;
ps4Handler* ps4Handler::handler;

ps4Handler::ps4Handler(Env* env, const char* macId) : speedSrc(env) {

    ps.attach(eventCallback);
    ps.attachOnDisconnect(disConnectCallback);
    ps.attachOnConnect(connectCallback);


    if(macId){
        ps.begin(macId);
    }
    else ps.begin();
    handler = this;
    run();
};


wheelSpeed ps4Handler::getSpeed(){
    
    // float x = joyX, y = joyY, mag, cos, theta;
    float x = joyX, y = joyY, mag, cos, sin;

    mag = sqrt(x * x + y * y);
    cos = x / mag;
    sin = y / mag;


    joyStick joy {
        .x =  x,
        .y = y,
        .sin = sin,
        .cos = cos,
        .mag = mag
    };

    return env->mapper->map(joy);
};


/** @todo switch to ros handler here */
void ps4Handler::disConnectCallback(){
    
    ps.data = ps4_t();
    handler->env->src = handler;

};


void ps4Handler::connectCallback(){
    handler->env->src = handler;
    ESP_LOGI("PS4", " Connected");
    ps.setLed(0xff, 0x00, 0x00);
    ps.sendToController();

};


void ps4Handler::eventCallback(){
    // ESP_LOGI("PS4", "event recieved");
    
}

void ps4Handler::run(){
    if(!uXtask) xTaskCreatePinnedToCore(ps4Handler::srun, "ps4UxTask", 4096, this, 1, &uXtask, 1);
}


void ps4Handler::srun(void *ptr){

    ps4Handler* hand = (ps4Handler*) ptr;
    
    uint8_t srumble = 0, brumble = 0; 
    float x, y, mag, cos, sin;
    while (1)
    {

        if(ps.L2()) hand->env->mapper->factor = ps.L2Value();
        else hand->env->mapper->factor = 0;


        x = hand->joyX;
        y = hand->joyY;

        mag = sqrt(x * x + y * y);
        cos = x / mag;
        sin = y / mag;



        joyStick joy {
            .x =  x,
            .y = y,
            .sin = sin,
            .cos = cos,
            .mag = mag
        };


        memcpy(hand->env->motorHandler->speeds, hand->env->mapper->map(joy).rawSpeed, 4 * sizeof(float));
        hand->env->motorHandler->update();

        vTaskDelay(pdMS_TO_TICKS(1000.0f / hand->uxUpdateFreq));
    }

};
