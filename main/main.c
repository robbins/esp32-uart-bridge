/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define TX_PIN 4              // GPIO4 as UART1 TX
#define RX_PIN 5              // GPIO5 as UART1 RX
#define BUF_SIZE 4096

const uart_port_t uart_num = UART_NUM_1;
static QueueHandle_t uart1_queue;
//char test_str = '\x00';

static void uart_read_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    // Read data from UART.
    while (1) {
      //uart_write_bytes(uart_num, &test_str, 1);
      //test_str++;

      int length = 0;
      ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
      length = uart_read_bytes(uart_num, data, length, 0);
      if (length != 0) {
        data[length] = '\0';
        printf("%s", (char*)data);
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: disabled, CTS: disabled)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install driver
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE, BUF_SIZE, 10, &uart1_queue, 0));

    xTaskCreate(uart_read_task, "uart_read_task", 2048, NULL, 10, NULL);
}
