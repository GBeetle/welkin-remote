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
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "class/cdc/cdc_device.h"

#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 64

static const char *TAG = "example";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
// Used to control whether this node is sending or receiving
bool  role    = true;  // true = TX role, false = RX role
float payload = 0.0;

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        ESP_LOGI(TAG, "Got data (%d bytes): %s, itf: %d", rx_size, buf, itf);
    } else {
        ESP_LOGE(TAG, "Read error");
    }

    /* write back */
    tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    tinyusb_cdcacm_write_flush(itf, 0);
}

void tinyusb_cdc_rx_wanted_char_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        ESP_LOGI(TAG, "Got wanted char => data (%d bytes): %s", rx_size, buf);
    } else {
        ESP_LOGE(TAG, "Read error");
    }
}

// connected / disconnected
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! dtr:%d, rst:%d", dtr, rst);
}

void loop(void* arg)
{
    while (true) {
        if (role) {
            ESP_LOGI(TAG, "Transmission begin! ");  // payload was delivered
            // This device is a TX node
            WK_RESULT report = radio.write(&radio, &payload, sizeof(float));  // transmit & save the report

            if (report >= 0) {
                ESP_LOGI(TAG, "Transmission successful! ");  // payload was delivered
                payload += 0.01;          // increment float payload
            } else {
                ESP_LOGI(TAG, "Transmission failed or timed out");  // payload was not delivered
            }
            // to make this example readable in the serial monitor
            // slow transmissions down by 1 second
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            // This device is a RX node
            uint8_t pipe;
            if (radio.available(&radio, &pipe)) {              // is there a payload? get the pipe number that recieved it
                uint8_t bytes = radio.getPayloadSize(&radio);  // get the size of the payload
                radio.read(&radio, &payload, bytes);             // fetch payload from FIFO
                ESP_LOGI(TAG, "Received data");
            }
        }
    }
}

//Main application
void app_main(void)
{
    //fflush(stdout);
    welkin_log_system_init();

    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = {}; // the configuration using default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = NULL,
        .callback_rx_wanted_char = &tinyusb_cdc_rx_wanted_char_callback,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    // SET wanted char => ' '
    tud_cdc_n_set_wanted_char(amc_cfg.cdc_port, ' ');
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb

#if 0
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif
    ESP_LOGI(TAG, "USB initialization DONE");

    /* test for NRF24+ commnuitation */
    rf24_init(&radio);
    // Let these addresses be used for the pair
    uint8_t address[][6] = {"1Node", "2Node"};
    // to use different addresses on a pair of radios, we need a variable to
    // uniquely identify which address this radio will use to transmit
    bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

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

    ESP_LOGI(TAG, "NRF24 initialization DONE");

    xTaskCreate(loop, "nrf24_loop", 2048, NULL, 2 | portPRIVILEGE_BIT, NULL);
}
