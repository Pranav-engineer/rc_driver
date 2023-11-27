
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

#include "esp_log.h"
#include "ps4Handler.hpp"
#include "env.hpp"


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

};


wheelSpeed ps4Handler::getSpeed(){
    
    float x = joyX, y = joyY, mag, cos, theta;

    mag = sqrt(x * x + y * y);
    velocity.mag = mag / 127.0f;
    if(velocity.mag > 1.0f) velocity.mag = 1.0f;

    if(y < 0){
        velocity.mag *= -1.0f;
    }

    velocity.left = x < 0;

    cos = (float) y / mag;
    theta = (y < 0) ? acos(-cos) : acos(cos);


    if(theta > 1.5f) theta = 1.5f;
    velocity.u = (0.0f - 1.0f) * theta / 1.5f + 1.0f;


    ESP_LOGI("PS4", " vals %f %f %f %f \n", velocity.mag, cos, theta, velocity.u);

    return env->mapper->map(velocity);
    // return wheelSpeed();
};


/** @todo switch to ros handler here */
void ps4Handler::disConnectCallback(){
    
    ps.data = ps4_t();
    handler->env->src = handler;

};


void ps4Handler::connectCallback(){
    handler->env->src = handler;
    ESP_LOGI("PS4", " Connected");

};


void ps4Handler::eventCallback(){
    // ESP_LOGI("PS4", "event recieved");
    
};
