
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


#ifndef  UROS_HPP
#define  UROS_HPP

#include <string>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

// update pinouts here
#define UROS_UART_TXD  (GPIO_NUM_12)
#define UROS_UART_RXD  (GPIO_NUM_13)
#define UROS_UART_RTS  (GPIO_NUM_5)
#define UROS_UART_CTS  (GPIO_NUM_17)

#define UROS_DEF_UART_PORT UART_NUM_1
#define UROS_UART_BAUD_RATE 115200

#define UROS_DEFAULT_NODE_NAME "rc_driver"


class uros{

public:

    class pub{
    public:

        pub(std::string topic, const rosidl_message_type_support_t* msgType);

        virtual void pubCallback() = 0;

        static void s_pubCallback(rcl_timer_t * tmr, int64_t count);

    protected:
    rcl_publisher_t pubPtr;
    rcl_timer_t timer;
    private:
        std::string topic;
    };


    static void init();

    static std::string nodeName;
    static bool isUrosInit;
    static rcl_allocator_t allocator;
    static rclc_support_t support;
	static rclc_executor_t executor;    
    static TaskHandle_t uros_task_handle;
	static rcl_node_t node;


    static void urosTask(void*);

};
#endif // UROS_HPP
