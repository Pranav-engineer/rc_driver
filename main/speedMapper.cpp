
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
#include <esp_log.h>
#include <math.h>

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
}


wheelSpeed speedMapper::map(joyStick &joy)
{
    // ESP_LOGI("mapper", "mag %f ", joy.mag);
    if(joy.mag < 80.0f) return wheelSpeed();
    wheelSpeed whl;

    float max_r = 0.0f;
    if( abs(joy.x) > abs(joy.y)) max_r = abs(joy.mag / joy.x);
    else max_r = abs(joy.mag / joy.y);

    float magnitude = 1.0f;
    // float magnitude = joy.mag / max_r;
    
    float turnDamp = 1.5f;
    
    // flip y axis
    joy.sin = - joy.sin;
    joy.cos = - joy.cos;
    whl.rawSpeed[2]  = magnitude * (joy.sin + joy.cos / turnDamp);
    whl.rawSpeed[3]  = magnitude * (joy.sin - joy.cos / turnDamp);

    wheelRatio = factor * (maxRatio - minRatio) + minRatio;

    whl.rawSpeed[0] = whl.rawSpeed[2] * wheelRatio;
    whl.rawSpeed[1] = whl.rawSpeed[3] * wheelRatio;

    for(int i = 0; i < 4; i++) {
        if(whl.rawSpeed[i] > 1.0f) whl.rawSpeed[i] = 1.0f;
        if(whl.rawSpeed[i] < -1.0f) whl.rawSpeed[i] = -1.0f;
    };
    ESP_LOGI("mapper", "left %f right %f ", whl.rawSpeed[0], whl.rawSpeed[1]);
    return whl;
};


// wheelSpeed speedMapper::map(joyStick &joy)
// {
//     ESP_LOGI("mapper", "mag %f ", joy.mag);
//     if(joy.mag < 80.0f) return wheelSpeed();
//     wheelSpeed whl;
//     whl.rawSpeed[0] = whl.rawSpeed[2] = joy.sin + joy.cos;
//     whl.rawSpeed[1] = whl.rawSpeed[3] = joy.sin - joy.cos;

//     for(int i = 0; i < 4; i++) {
//         if(whl.rawSpeed[i] > 1.0f) whl.rawSpeed[i] = 1.0f;
//         if(whl.rawSpeed[i] < -1.0f) whl.rawSpeed[i] = -1.0f;
//     };
//     ESP_LOGI("mapper", "left %f right %f ", whl.rawSpeed[0], whl.rawSpeed[1]);
//     return whl;
// };


// wheelSpeed speedMapper::map(circularJoyStick &joy)
// {
//     // ESP_LOGI("mapper", "circular joy %f %f", joy.xc, joy.yc);


//     float p = sqrt(joy.xc * joy.xc + joy.yc * joy.yc );
//     float theta = atan2(joy.yc, joy.xc);

//     float max = 1.0f, min = 0.0f;
//     float s = (max - min) * (cos(2.0f * theta) + 1) / 2.0f + min;
//     float alpha = 0.5f;
//     wheelSpeed whl;
//     whl.rawSpeed[0] = alpha * p * p * ( 1.0f - s) * sin(2.0f * theta);
//     whl.rawSpeed[1] = alpha *  s * joy.xc;

//     // if(joy.mag < 80.0f) return wheelSpeed();
//     // whl.rawSpeed[0] = whl.rawSpeed[2] = joy.sin + joy.cos;
//     // whl.rawSpeed[1] = whl.rawSpeed[3] = joy.sin - joy.cos;

//     ESP_LOGI("mapper", "left %f right %f ", whl.rawSpeed[0], whl.rawSpeed[1]);
//     return whl;
// };