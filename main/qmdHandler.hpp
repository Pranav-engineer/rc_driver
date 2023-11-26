
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

#ifndef QMD_HANDLER_HPP
#define QMD_HANDLER_HPP


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "env.hpp"

class qmdHandler {
    
public:
    Env* env;
    int updateFrequency = 50;

    qmdHandler(){};
    qmdHandler(Env* en) : env(en) {};

    void run();
    static void srun(void* update);

    private:
    TaskHandle_t task;
};
#endif //  QMD_HANDLER_HPP