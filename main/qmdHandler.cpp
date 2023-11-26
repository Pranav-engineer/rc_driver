
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


#include <string.h>
#include <esp_log.h>

#include "qmdHandler.hpp"


void qmdHandler::run(){
    xTaskCreate(srun, "qmdHandler",  16000, this, 1, &task);
};

void qmdHandler::srun(void* hand){

    qmdHandler* handler = (qmdHandler*) hand;
    Env* en = ((qmdHandler*)hand)->env;
    while (true)
    {
        memcpy(en->motorHandler->speeds, en->src->getSpeed().rawSpeed, 4 * sizeof(float));
        en->motorHandler->update();

        vTaskDelay(pdMS_TO_TICKS(1000 /  handler->updateFrequency));
    }
    
};