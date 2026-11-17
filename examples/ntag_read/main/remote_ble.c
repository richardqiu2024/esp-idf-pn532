#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
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
#include "store/config/ble_store_config.h"

#include "ble_oob_config.h"
#include "remote_ble.h"

void ble_store_config_init(void);

#ifndef CONFIG_REMOTE_BUTTON_GPIO
#define CONFIG_REMOTE_BUTTON_GPIO 9
#endif

#ifndef CONFIG_REMOTE_LED_GPIO
#define CONFIG_REMOTE_LED_GPIO 48
#endif

#define REMOTE_LED_RMT_RESOLUTION_HZ 10000000
#define REMOTE_BUTTON_DEBOUNCE_MS 40
#define REMOTE_BUTTON_PROVISION_HOLD_MS 5000

static const char *TAG = "ble_remote";
static const ble_uuid16_t s_lamp_service_uuid = BLE_UUID16_INIT(NFC_BLE_SERVICE_UUID);
static const ble_uuid16_t s_lamp_command_uuid = BLE_UUID16_INIT(NFC_BLE_COMMAND_UUID);
static const uint8_t s_local_oob_tk[16] = NFC_BLE_OOB_TK_INIT;

static volatile bool s_host_synced;
static volatile bool s_provision_requested;
static volatile bool s_provision_mode;
static volatile bool s_connect_pending;
static volatile bool s_connected;
static volatile bool s_command_ready;
static volatile bool s_connecting;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_command_handle;
static uint8_t s_own_addr_type;
static uint16_t s_service_start;
static uint16_t s_service_end;
static ble_addr_t s_target_addr;
static uint8_t s_pairing_tk[16];
static QueueHandle_t s_provision_queue;
static QueueHandle_t s_led_queue;
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

static void start_provision_scan(void);
static void start_connect(void);

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

static esp_err_t init_status_led(void)
{
    const rmt_tx_channel_config_t channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = CONFIG_REMOTE_LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = REMOTE_LED_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&channel_config, &s_led_channel),
                        TAG, "status LED channel setup failed");

    const rmt_simple_encoder_config_t encoder_config = {
        .callback = encode_ws2812,
    };
    ESP_RETURN_ON_ERROR(rmt_new_simple_encoder(&encoder_config, &s_led_encoder),
                        TAG, "status LED encoder setup failed");
    return rmt_enable(s_led_channel);
}

static void status_led_task(void *arg)
{
    (void)arg;
    ble_remote_led_state_t current = BLE_REMOTE_LED_IDLE;
    ble_remote_led_state_t requested;
    bool have_current = false;
    while (xQueueReceive(s_led_queue, &requested, portMAX_DELAY) == pdTRUE) {
        if (have_current && requested == current) {
            continue;
        }

        // WS2812 pixels on these ESP32-S3 boards use GRB byte order.
        s_led_pixel[0] = requested == BLE_REMOTE_LED_CONNECTED ? 24 : 0;
        s_led_pixel[1] = requested == BLE_REMOTE_LED_FAILURE ? 24 : 0;
        s_led_pixel[2] = requested == BLE_REMOTE_LED_PROVISIONING ? 24 : 0;
        const rmt_transmit_config_t tx_config = {.loop_count = 0};
        esp_err_t err = rmt_transmit(s_led_channel, s_led_encoder, s_led_pixel,
                                     sizeof(s_led_pixel), &tx_config);
        if (err == ESP_OK) {
            err = rmt_tx_wait_all_done(s_led_channel, pdMS_TO_TICKS(100));
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Status LED update failed: %s", esp_err_to_name(err));
        } else {
            current = requested;
            have_current = true;
        }
    }
}

void ble_remote_set_led_state(ble_remote_led_state_t state)
{
    if (s_led_queue != NULL) {
        xQueueOverwrite(s_led_queue, &state);
    }
}

bool ble_remote_is_ready(void)
{
    return s_connected && s_command_ready;
}

bool ble_remote_is_provision_mode(void)
{
    return s_provision_mode;
}

static void set_led_failure(void)
{
    ble_remote_set_led_state(BLE_REMOTE_LED_FAILURE);
}

static void set_led_connected(void)
{
    ble_remote_set_led_state(BLE_REMOTE_LED_CONNECTED);
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE address setup failed: %d", rc);
        return;
    }
    s_host_synced = true;
    ESP_LOGI(TAG, "BLE ready; waiting for a Bluetooth OOB NFC tag");
    if (s_provision_requested) {
        start_provision_scan();
    } else if (s_connect_pending) {
        start_connect();
    }
}

static void on_reset(int reason)
{
    s_host_synced = false;
    ESP_LOGE(TAG, "BLE host reset: %d", reason);
}

static bool is_lamp_advertisement(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0 ||
        fields.name == NULL || fields.name_len != sizeof(NFC_BLE_LAMP_NAME) - 1 ||
        memcmp(fields.name, NFC_BLE_LAMP_NAME, sizeof(NFC_BLE_LAMP_NAME) - 1) != 0) {
        return false;
    }

    for (unsigned i = 0; i < fields.num_uuids16; ++i) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == NFC_BLE_SERVICE_UUID) {
            return true;
        }
    }
    return false;
}

static int on_command_discovered(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (error->status == 0) {
        s_command_handle = chr->val_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "Command characteristic discovery failed: %d", error->status);
        set_led_failure();
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    if (s_command_handle == 0) {
        ESP_LOGE(TAG, "Lamp has no command characteristic");
        set_led_failure();
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    s_conn_handle = conn_handle;
    s_command_ready = true;
    s_connected = true;
    s_connecting = false;
    set_led_connected();
    ESP_LOGI(TAG, "Authenticated and bonded to NFC-BLE-LAMP; button toggles it");
    return 0;
}

static int on_service_discovered(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;
    if (error->status == 0) {
        s_service_start = service->start_handle;
        s_service_end = service->end_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "Lamp service discovery failed: %d", error->status);
        set_led_failure();
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    if (s_service_start == 0) {
        ESP_LOGE(TAG, "Lamp control service was not found");
        set_led_failure();
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    int rc = ble_gattc_disc_chrs_by_uuid(conn_handle, s_service_start,
                                          s_service_end,
                                          &s_lamp_command_uuid.u,
                                          on_command_discovered, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Starting command characteristic discovery failed: %d", rc);
        set_led_failure();
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

static void inject_oob_tk(uint16_t conn_handle)
{
    struct ble_sm_io io = {.action = BLE_SM_IOACT_OOB};
    memcpy(io.oob, s_pairing_tk, sizeof(io.oob));
    int rc = ble_sm_inject_io(conn_handle, &io);
    if (rc != 0) {
        ESP_LOGE(TAG, "Supplying NFC OOB key failed: %d", rc);
        set_led_failure();
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        ESP_LOGI(TAG, "NFC OOB key supplied to BLE security manager");
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (!s_provision_requested || !is_lamp_advertisement(&event->disc)) {
            return 0;
        }
        {
            ble_oob_info_t info = {0};
            memcpy(info.address, event->disc.addr.val, sizeof(info.address));
            info.address_type = event->disc.addr.type & 1;
            memcpy(info.tk, s_local_oob_tk, sizeof(info.tk));
            memcpy(info.name, NFC_BLE_LAMP_NAME, sizeof(NFC_BLE_LAMP_NAME));
            s_provision_requested = false;
            xQueueOverwrite(s_provision_queue, &info);
            ESP_LOGI(TAG, "Lamp found; NFC tag provisioning data is ready");
            (void)ble_gap_disc_cancel();
        }
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_provision_requested) {
            start_provision_scan();
        } else if (s_connect_pending) {
            start_connect();
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "BLE connection failed: %d", event->connect.status);
            set_led_failure();
            s_connecting = false;
            s_connected = false;
            s_command_ready = false;
            return 0;
        }
        s_conn_handle = event->connect.conn_handle;
        s_command_handle = 0;
        s_service_start = 0;
        s_service_end = 0;
        ESP_LOGI(TAG, "Connected to address from NFC OOB record; securing link");
        {
            int rc = ble_gap_security_initiate(s_conn_handle);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                ESP_LOGE(TAG, "Starting authenticated pairing failed: %d", rc);
                set_led_failure();
                ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
        }
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
            inject_oob_tk(event->passkey.conn_handle);
        } else {
            ESP_LOGE(TAG, "Unexpected BLE pairing action: %u",
                     event->passkey.params.action);
            set_led_failure();
            ble_gap_terminate(event->passkey.conn_handle,
                              BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status != 0) {
            ESP_LOGE(TAG, "BLE authenticated pairing failed: %d",
                     event->enc_change.status);
            set_led_failure();
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
                ESP_LOGE(TAG, "BLE link did not reach authenticated bonded state");
                set_led_failure();
                ble_gap_terminate(event->enc_change.conn_handle,
                                  BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
        }
        ESP_LOGI(TAG, "BLE OOB pairing authenticated and bond stored");
        if (ble_gattc_disc_svc_by_uuid(event->enc_change.conn_handle,
                                        &s_lamp_service_uuid.u,
                                        on_service_discovered, NULL) != 0) {
            ESP_LOGE(TAG, "Starting lamp service discovery failed");
            set_led_failure();
            ble_gap_terminate(event->enc_change.conn_handle,
                              BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected; status=%d", event->disconnect.reason);
        set_led_failure();
        s_connected = false;
        s_command_ready = false;
        s_connecting = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_command_handle = 0;
        return 0;

    default:
        return 0;
    }
}

static void start_provision_scan(void)
{
    if (!s_host_synced || !s_provision_requested || ble_gap_disc_active()) {
        return;
    }
    struct ble_gap_disc_params params = {0};
    params.filter_duplicates = 1;
    params.passive = 1;
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params,
                          gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "Lamp discovery scan failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Looking up lamp address for NFC tag provisioning");
    }
}

static void start_connect(void)
{
    if (!s_host_synced || !s_connect_pending || s_connected || s_connecting ||
        ble_gap_disc_active()) {
        return;
    }
    s_connect_pending = false;
    s_connecting = true;
    int rc = ble_gap_connect(s_own_addr_type, &s_target_addr, 30000, NULL,
                             gap_event, NULL);
    if (rc != 0) {
        s_connecting = false;
        ESP_LOGE(TAG, "Direct BLE connection from NFC record failed: %d", rc);
        set_led_failure();
    }
}

void ble_remote_request_provision(void)
{
    if (s_provision_queue == NULL) {
        set_led_failure();
        return;
    }
    s_provision_mode = true;
    ble_remote_set_led_state(BLE_REMOTE_LED_PROVISIONING);
    s_provision_requested = true;
    ESP_LOGI(TAG, "Tag write mode enabled; searching for NFC-BLE-LAMP provisioning data");
    start_provision_scan();
}

void ble_remote_cancel_provision(void)
{
    s_provision_mode = false;
    s_provision_requested = false;
    if (s_provision_queue != NULL) {
        xQueueReset(s_provision_queue);
    }
    if (s_host_synced && ble_gap_disc_active()) {
        (void)ble_gap_disc_cancel();
    }
}

bool ble_remote_take_provision_data(ble_oob_info_t *info, TickType_t timeout)
{
    return info != NULL && s_provision_queue != NULL &&
           xQueueReceive(s_provision_queue, info, timeout) == pdTRUE;
}

void ble_remote_request_connect(const ble_oob_info_t *info)
{
    if (info == NULL || s_connected || s_connecting) {
        return;
    }
    memcpy(s_target_addr.val, info->address, sizeof(s_target_addr.val));
    s_target_addr.type = info->address_type;
    memcpy(s_pairing_tk, info->tk, sizeof(s_pairing_tk));
    ble_remote_set_led_state(BLE_REMOTE_LED_IDLE);
    s_connect_pending = true;
    ESP_LOGI(TAG, "Using Bluetooth LE OOB data from NFC; connecting directly");
    start_connect();
}

static void send_toggle(void)
{
    if (!s_connected || !s_command_ready) {
        ESP_LOGW(TAG, "Button pressed before the lamp connected");
        return;
    }

    const uint8_t toggle = 2;
    int rc = ble_gattc_write_no_rsp_flat(s_conn_handle, s_command_handle,
                                          &toggle, sizeof(toggle));
    if (rc != 0) {
        ESP_LOGW(TAG, "Lamp command failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Toggle command sent");
    }
}

static void button_task(void *arg)
{
    (void)arg;
    bool stable_released = gpio_get_level(CONFIG_REMOTE_BUTTON_GPIO) != 0;
    bool sampled_released = stable_released;
    bool provision_mode_entered = false;
    TickType_t last_sample_change = xTaskGetTickCount();
    TickType_t pressed_at = stable_released ? 0 : last_sample_change;
    while (true) {
        bool released = gpio_get_level(CONFIG_REMOTE_BUTTON_GPIO) != 0;
        TickType_t now = xTaskGetTickCount();
        if (released != sampled_released) {
            sampled_released = released;
            last_sample_change = now;
        } else if (released != stable_released &&
                   now - last_sample_change >=
                       pdMS_TO_TICKS(REMOTE_BUTTON_DEBOUNCE_MS)) {
            stable_released = released;
            if (!stable_released) {
                pressed_at = now;
                provision_mode_entered = false;
            } else if (!provision_mode_entered &&
                       !ble_remote_is_provision_mode()) {
                send_toggle();
            }
        }

        if (!stable_released && !provision_mode_entered &&
            now - pressed_at >= pdMS_TO_TICKS(REMOTE_BUTTON_PROVISION_HOLD_MS)) {
            provision_mode_entered = true;
            ble_remote_request_provision();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t ble_remote_init(void)
{
    gpio_config_t button_cfg = {
        .pin_bit_mask = 1ULL << CONFIG_REMOTE_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&button_cfg), TAG, "button GPIO setup failed");
    ESP_RETURN_ON_ERROR(init_status_led(), TAG, "status LED init failed");

    s_led_queue = xQueueCreate(1, sizeof(ble_remote_led_state_t));
    if (s_led_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(status_led_task, "remote_status_led", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ble_remote_set_led_state(BLE_REMOTE_LED_IDLE);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS init failed");

    s_provision_queue = xQueueCreate(1, sizeof(ble_oob_info_t));
    if (s_provision_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "WS2812 status LED output GPIO%d", CONFIG_REMOTE_LED_GPIO);
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "NimBLE init failed");
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
    int rc = ble_svc_gap_device_name_set("NFC-BLE-REMOTE");
    if (rc != 0) {
        return ESP_FAIL;
    }
    ble_store_config_init();
    if (xTaskCreate(button_task, "remote_button", 8192, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "Button input GPIO%d (active low)", CONFIG_REMOTE_BUTTON_GPIO);
    return ESP_OK;
}
