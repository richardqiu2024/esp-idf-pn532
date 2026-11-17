#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "ble_oob_config.h"

typedef enum {
    BLE_REMOTE_LED_IDLE,
    BLE_REMOTE_LED_FAILURE,
    BLE_REMOTE_LED_CONNECTED,
    BLE_REMOTE_LED_PROVISIONING,
} ble_remote_led_state_t;

esp_err_t ble_remote_init(void);
void ble_remote_set_led_state(ble_remote_led_state_t state);
bool ble_remote_is_ready(void);
bool ble_remote_is_provision_mode(void);
void ble_remote_request_provision(void);
void ble_remote_cancel_provision(void);
bool ble_remote_take_provision_data(ble_oob_info_t *info, TickType_t timeout);
void ble_remote_request_connect(const ble_oob_info_t *info);
