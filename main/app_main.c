/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <byteswap.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_master.h"
#include "esp_spi_flash.h"
#include "rom/ets_sys.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "log_sys.h"
#include "io_define.h"
#include "task_manager.h"
#include "isr_manager.h"
#include "nrf24_interface.h"

//Main application
void app_main(void)
{
    //fflush(stdout);
    welkin_log_system_init();
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    /* test for NRF24+ commnuitation */
    rf24_init(&radio);
    // Let these addresses be used for the pair
    uint8_t address[][6] = {"1Node", "2Node"};
    float payload = 0.0;
    // to use different addresses on a pair of radios, we need a variable to
    // uniquely identify which address this radio will use to transmit
    bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
    // Used to control whether this node is sending or receiving
    bool role = false;  // true = TX role, false = RX role

    //radio.printDetails(&radio);

    CHK_EXIT(radio.begin(&radio));
    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    CHK_EXIT(radio.setPALevel(&radio, RF24_PA_LOW, true));  // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    CHK_EXIT(radio.setPayloadSize(&radio, sizeof(payload))); // float datatype occupies 4 bytes

    // set the TX address of the RX node into the TX pipe
    CHK_EXIT(radio.openWritingPipeAddr(&radio, address[radioNumber]));     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    CHK_EXIT(radio.openReadingPipeAddr(&radio, 1, address[!radioNumber])); // using pipe 1

    // additional setup specific to the node's role
    if (role) {
        CHK_EXIT(radio.stopListening(&radio));  // put radio in TX mode
    } else {
        CHK_EXIT(radio.startListening(&radio)); // put radio in RX mode
    }

    xTaskCreate(mpu_get_sensor_data, "mpu_get_sensor_data", 2048, NULL, 2 | portPRIVILEGE_BIT, &mpu_isr_handle);
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //vTaskStartScheduler();

    while (true) {
    }
}
