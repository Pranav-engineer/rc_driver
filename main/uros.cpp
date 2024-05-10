
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
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h> // Include the appropriate message type for cmd_vel
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t cmd_vel_subscriber; // Declare a subscriber variable for cmd_vel
geometry_msgs__msg__Twist received_cmd_vel; // Declare a message variable to store received cmd_vel data

void cmd_vel_subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    // Process received cmd_vel data here
}

void micro_ros_task(void * arg)
{

    // Create subscriber for cmd_vel topic
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), // Use appropriate message type for cmd_vel
        "cmd_vel"));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &received_cmd_vel, &cmd_vel_subscription_callback, ON_NEW_DATA));

    // Spin forever
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Clean-up code...
}

void app_main(void)
{
    // Initialize Micro-ROS network interface if required
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    // Create Micro-ROS task
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
