#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "ble_oob_config.h"

void ble_store_config_init(void);

#ifndef CONFIG_LAMP_LED_GPIO
#define CONFIG_LAMP_LED_GPIO 48
#endif

#define LAMP_RMT_RESOLUTION_HZ 10000000

static const char *TAG = "ble_lamp";
static const ble_uuid16_t s_service_uuid = BLE_UUID16_INIT(NFC_BLE_SERVICE_UUID);
static const uint8_t s_oob_tk[16] = NFC_BLE_OOB_TK_INIT;
static uint8_t s_lamp_state;
static uint8_t s_addr_type;
static uint8_t s_led_pixel[3];
static rmt_channel_handle_t s_led_channel;
static rmt_encoder_handle_t s_led_encoder;

static const rmt_symbol_word_t s_ws2812_zero = {
    .level0 = 1,
    .duration0 = 3,
    .level1 = 0,
    .duration1 = 9,
};

static const rmt_symbol_word_t s_ws2812_one = {
    .level0 = 1,
    .duration0 = 9,
    .level1 = 0,
    .duration1 = 3,
};

static const rmt_symbol_word_t s_ws2812_reset = {
    .level0 = 0,
    .duration0 = 250,
    .level1 = 0,
    .duration1 = 250,
};

static int gap_event(struct ble_gap_event *event, void *arg);

static size_t encode_ws2812(const void *data, size_t data_size,
                            size_t symbols_written, size_t symbols_free,
                            rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    (void)arg;
    if (symbols_free < 8) {
        return 0;
    }

    size_t byte_index = symbols_written / 8;
    if (byte_index < data_size) {
        const uint8_t *bytes = data;
        uint8_t value = bytes[byte_index];
        for (int bit = 7; bit >= 0; --bit) {
            symbols[7 - bit] = (value & (1U << bit)) ? s_ws2812_one : s_ws2812_zero;
        }
        return 8;
    }

    symbols[0] = s_ws2812_reset;
    *done = true;
    return 1;
}

static void init_lamp_led(void)
{
    const rmt_tx_channel_config_t channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = CONFIG_LAMP_LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = LAMP_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&channel_config, &s_led_channel));

    const rmt_simple_encoder_config_t encoder_config = {
        .callback = encode_ws2812,
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&encoder_config, &s_led_encoder));
    ESP_ERROR_CHECK(rmt_enable(s_led_channel));
}

static void set_lamp(bool on)
{
    s_lamp_state = on ? 1 : 0;
    s_led_pixel[0] = on ? 24 : 0; // Green
    s_led_pixel[1] = on ? 24 : 0; // Red
    s_led_pixel[2] = on ? 24 : 0; // Blue
    const rmt_transmit_config_t tx_config = {.loop_count = 0};
    ESP_ERROR_CHECK(rmt_transmit(s_led_channel, s_led_encoder, s_led_pixel,
                                 sizeof(s_led_pixel), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_led_channel, portMAX_DELAY));
    ESP_LOGI(TAG, "Lamp %s", on ? "ON" : "OFF");
}

static int command_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0 ||
        !desc.sec_state.encrypted || !desc.sec_state.authenticated ||
        !desc.sec_state.bonded) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR ||
        OS_MBUF_PKTLEN(ctxt->om) != 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t command;
    if (os_mbuf_copydata(ctxt->om, 0, sizeof(command), &command) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (command == 0) {
        set_lamp(false);
    } else if (command == 1) {
        set_lamp(true);
    } else if (command == 2) {
        set_lamp(!s_lamp_state);
    } else {
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    return 0;
}

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(NFC_BLE_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(NFC_BLE_COMMAND_UUID),
                .access_cb = command_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {0},
        },
    },
    {0},
};

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)NFC_BLE_LAMP_NAME;
    fields.name_len = sizeof(NFC_BLE_LAMP_NAME) - 1;
    fields.name_is_complete = 1;
    fields.uuids16 = (ble_uuid16_t *)&s_service_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE advertisement setup failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &params,
                           gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE advertising start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as %s", NFC_BLE_LAMP_NAME);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Remote connected");
        } else {
            ESP_LOGW(TAG, "Connection attempt failed: %d", event->connect.status);
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Remote disconnected");
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
            struct ble_sm_io io = {.action = BLE_SM_IOACT_OOB};
            memcpy(io.oob, s_oob_tk, sizeof(io.oob));
            int rc = ble_sm_inject_io(event->passkey.conn_handle, &io);
            if (rc != 0) {
                ESP_LOGE(TAG, "Supplying NFC OOB key failed: %d", rc);
                ble_gap_terminate(event->passkey.conn_handle,
                                  BLE_ERR_REM_USER_CONN_TERM);
            } else {
                ESP_LOGI(TAG, "NFC OOB key supplied to BLE security manager");
            }
        } else {
            ESP_LOGE(TAG, "Unexpected pairing method: %u",
                     event->passkey.params.action);
            ble_gap_terminate(event->passkey.conn_handle,
                              BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status != 0) {
            ESP_LOGE(TAG, "OOB pairing failed: %d", event->enc_change.status);
            ble_gap_terminate(event->enc_change.conn_handle,
                              BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        {
            struct ble_gap_conn_desc desc = {0};
            int rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
            ESP_LOGI(TAG,
                     "Security state: rc=%d encrypted=%u authenticated=%u bonded=%u",
                     rc, desc.sec_state.encrypted, desc.sec_state.authenticated,
                     desc.sec_state.bonded);
            if (rc != 0 || !desc.sec_state.encrypted ||
                !desc.sec_state.authenticated || !desc.sec_state.bonded) {
                ESP_LOGE(TAG, "Connection did not reach authenticated bonded state");
                ble_gap_terminate(event->enc_change.conn_handle,
                                  BLE_ERR_REM_USER_CONN_TERM);
            } else {
                ESP_LOGI(TAG, "NFC OOB pairing authenticated and bond stored");
            }
        }
        return 0;
    default:
        return 0;
    }
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE address setup failed: %d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset: %d", reason);
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    init_lamp_led();
    set_lamp(false);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_oob_data_flag = 1;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_sec_lvl = 3;
    int rc = ble_svc_gap_device_name_set(NFC_BLE_LAMP_NAME);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(s_services);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);
    rc = ble_gatts_add_svcs(s_services);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);
    ble_store_config_init();

    ESP_LOGI(TAG, "WS2812 RGB LED output GPIO%d", CONFIG_LAMP_LED_GPIO);
    nimble_port_freertos_init(ble_host_task);
}
