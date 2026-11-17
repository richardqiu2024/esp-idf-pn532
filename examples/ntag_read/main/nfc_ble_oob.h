#ifndef NFC_BLE_OOB_H
#define NFC_BLE_OOB_H

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "pn532_driver.h"
#include "ble_oob_config.h"

esp_err_t nfc_ble_oob_read(pn532_io_handle_t io, ble_oob_info_t *info,
                           bool *blank_tag);
esp_err_t nfc_ble_oob_write(pn532_io_handle_t io,
                            const ble_oob_info_t *info);

#endif
