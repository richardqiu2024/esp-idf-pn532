#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nfc_ble_oob.h"
#include "pn532.h"
#include "pn532_driver_i2c.h"
#include "remote_ble.h"

#define PN532_SCL_GPIO   5
#define PN532_SDA_GPIO   4
#define PN532_RESET_GPIO 7
#define PN532_IRQ_GPIO   6
#define NO_TAG_CONFIRMATIONS 3

static const char *TAG = "nfc_remote";

static bool handle_new_tag(pn532_io_handle_t io)
{
    bool write_mode = ble_remote_is_provision_mode();
    ESP_LOGI(TAG, "NFC tag selected; reading BLE OOB data%s",
             write_mode ? " (tag write mode)" : "");
    ble_oob_info_t info = {0};
    bool blank = false;
    esp_err_t err = nfc_ble_oob_read(io, &info, &blank);
    if (write_mode) {
        if (err == ESP_ERR_NOT_FOUND && blank) {
            ESP_LOGI(TAG, "Blank NTAG accepted in explicit tag write mode");
            return true;
        }
        ble_remote_set_led_state(BLE_REMOTE_LED_FAILURE);
        ESP_LOGW(TAG, "Tag write mode only accepts a blank NTAG; preserving tag contents (%s)",
                 esp_err_to_name(err));
        return false;
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BLE OOB data found; connecting to %s", info.name);
        ble_remote_request_connect(&info);
    } else {
        if (!ble_remote_is_ready()) {
            ble_remote_set_led_state(BLE_REMOTE_LED_FAILURE);
        } else {
            ESP_LOGW(TAG, "NFC reread failed; keeping LED green while BLE control is ready");
        }
        ESP_LOGW(TAG, "Tag has no supported BLE OOB record (%s)",
                 esp_err_to_name(err));
    }
    return false;
}

void app_main(void)
{
    pn532_io_t pn532_io;
    ESP_ERROR_CHECK(ble_remote_init());
    esp_err_t err;
    do {
        err = pn532_new_driver_i2c(PN532_SDA_GPIO, PN532_SCL_GPIO,
                                   PN532_RESET_GPIO, PN532_IRQ_GPIO,
                                   0, &pn532_io);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "PN532 driver creation failed: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        err = pn532_init(&pn532_io);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "PN532 init failed: %s", esp_err_to_name(err));
            pn532_delete_driver(&pn532_io);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (err != ESP_OK);

    uint32_t firmware = 0;
    do {
        err = pn532_get_firmware_version(&pn532_io, &firmware);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "PN532 firmware query failed; retrying");
            pn532_reset(&pn532_io);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (err != ESP_OK);

    ESP_LOGI(TAG, "PN532 firmware %u.%u", (unsigned)((firmware >> 16) & 0xff),
             (unsigned)((firmware >> 8) & 0xff));
    ESP_ERROR_CHECK(pn532_set_passive_activation_retries(&pn532_io, 0));
    ESP_LOGI(TAG, "Ready; tap a BLE OOB tag to connect, or hold GPIO9 for 5 seconds to write a blank tag");

    bool tag_present = false;
    bool provisioning = false;
    bool provision_data_ready = false;
    ble_oob_info_t provision_info = {0};
    uint8_t no_tag_count = 0;
    while (true) {
        uint8_t uid[10] = {0};
        uint8_t uid_length = 0;
        err = pn532_read_passive_target_id(&pn532_io,
                                           PN532_BRTY_ISO14443A_106KBPS,
                                           uid, &uid_length, 500);
        if (err == ESP_OK) {
            no_tag_count = 0;
            if (!tag_present) {
                tag_present = true;
                provisioning = handle_new_tag(&pn532_io);
            }

            if (!provision_data_ready &&
                ble_remote_take_provision_data(&provision_info, 0)) {
                provision_data_ready = true;
            }
            if (provisioning && provision_data_ready) {
                err = nfc_ble_oob_write(&pn532_io, &provision_info);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "BLE OOB data written and verified; restarting");
                    vTaskDelay(pdMS_TO_TICKS(500));
                    esp_restart();
                } else {
                    ble_remote_set_led_state(BLE_REMOTE_LED_FAILURE);
                    ESP_LOGE(TAG, "Could not provision NFC tag: %s",
                             esp_err_to_name(err));
                }
                provisioning = false;
            }
        } else if (err == ESP_ERR_NOT_FOUND) {
            if (no_tag_count < NO_TAG_CONFIRMATIONS) {
                no_tag_count++;
            }

            if (tag_present && no_tag_count >= NO_TAG_CONFIRMATIONS) {
                ESP_LOGI(TAG, "NFC tag removed; ready for the next tag");
                if (provisioning) {
                    ESP_LOGI(TAG, "Blank tag removed before provisioning completed");
                }
                tag_present = false;
                provisioning = false;
                if (ble_remote_is_provision_mode()) {
                    ble_remote_set_led_state(BLE_REMOTE_LED_PROVISIONING);
                }
                no_tag_count = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(40));
        } else {
            no_tag_count = 0;
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
}
