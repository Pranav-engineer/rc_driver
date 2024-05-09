
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
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "uros.hpp"
#include <custom_transport.h>

#define UART_BUFFER_SIZE (512)

std::string uros::nodeName = UROS_DEFAULT_NODE_NAME;
bool uros::isUrosInit = false;

rcl_allocator_t uros::allocator;
rclc_support_t uros::support;
rclc_executor_t uros::executor;
rcl_node_t uros::node;
TaskHandle_t uros::uros_task_handle;


#define LOG "UROS"


// uros callbacks for uart communication
bool uros_uart_open_cb(struct uxrCustomTransport * transport);
size_t uros_uart_write_cb(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t uros_uart_read_cb(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
bool uros_uart_close_cb(struct uxrCustomTransport * transport);



void uros::urosTask(void*){

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		// usleep(10000);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}



void uros::init(){
    if(isUrosInit) return;

    uart_port_t uart_port = UROS_DEF_UART_PORT;
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        uros_uart_open_cb,
        uros_uart_close_cb,
        uros_uart_write_cb,
        uros_uart_read_cb
	);


    // create init_options
    allocator = rcl_get_default_allocator();
	rclc_support_init(&support, 0, NULL, &allocator);
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // create node
	rclc_node_init_default(&node, nodeName.c_str(), "esp32", &support);

    xTaskCreate(urosTask, "urosTask", 4096, nullptr, 1, &uros_task_handle);

    isUrosInit = true;
};







bool uros_uart_open_cb(struct uxrCustomTransport * transport){
    uart_port_t * uart_port = (uart_port_t*) transport->args;

    uart_config_t uart_config = {
        .baud_rate = UROS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(*uart_port, &uart_config) == ESP_FAIL) {
        return false;
    }
    if (uart_set_pin(*uart_port, UROS_UART_TXD, UROS_UART_RXD, UROS_UART_RTS, UROS_UART_CTS) == ESP_FAIL) {
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL) {
        return false;
    }

    return true;
}


bool uros_uart_close_cb(struct uxrCustomTransport * transport){
    uart_port_t * uart_port = (uart_port_t*) transport->args;

    return uart_driver_delete(*uart_port) == ESP_OK;
}


size_t uros_uart_write_cb(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    uart_port_t * uart_port = (uart_port_t*) transport->args;
    const int txBytes = uart_write_bytes(*uart_port, (const char*) buf, len);
    return txBytes;
}


size_t uros_uart_read_cb(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    uart_port_t * uart_port = (uart_port_t*) transport->args;
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return rxBytes;
}


rcl_publisher_t publisherr;
uros::pub::pub(std::string topic, const rosidl_message_type_support_t* msgType) : topic(topic){
    // impl = 0;
    // init uros
    init();

    // add publisher
	rclc_publisher_init_default(
		&pubPtr,
		&uros::node,
		msgType,
		topic.c_str());
    
    ESP_LOGI(LOG, "created publisher %s", topic.c_str());

    publisherr = pubPtr;
    // create timer,
	const unsigned int timer_timeout = 1000;

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        uros::pub::s_pubCallback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // xTaskCreate(s_pubCallback, "urosTask", 4096, this, 1, &uros_task_handle);
}

std_msgs__msg__Int32 msg;

void uros::pub::s_pubCallback(rcl_timer_t * tmr, int64_t diff){
    
    rcl_publish(&publisherr, &msg, NULL);
    msg.data++;
    // uros::pub* pubPtr = (uros::pub*) ptr;

    // while (1)
    // {
    //     pubPtr->pubCallback();
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
    
}
