#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "pn532.h"
#include "nfc_ble_oob.h"

#define NTAG213_USER_BYTES 144
#define NTAG215_USER_BYTES 504
#define NTAG216_USER_BYTES 888
#define MAX_NTAG_USER_BYTES NTAG216_USER_BYTES
#define NDEF_MIME_TYPE "application/vnd.bluetooth.le.oob"
#define NDEF_MIME_TYPE_LEN (sizeof(NDEF_MIME_TYPE) - 1)

static esp_err_t read_user_memory(pn532_io_handle_t io, uint8_t *memory,
                                  size_t capacity)
{
    for (size_t offset = 0; offset < capacity; offset += 16) {
        uint8_t block[16];
        size_t length = capacity - offset;
        if (length > sizeof(block)) {
            length = sizeof(block);
        }
        ESP_RETURN_ON_ERROR(ntag2xx_read_page(io, 4 + offset / 4, block, length),
                            "nfc_oob", "tag read failed");
        memcpy(memory + offset, block, length);
    }
    return ESP_OK;
}

static esp_err_t parse_oob_payload(const uint8_t *payload, size_t length,
                                   ble_oob_info_t *info)
{
    bool have_address = false;
    bool have_role = false;
    bool have_service = false;
    bool have_tk = false;
    size_t offset = 0;

    while (offset < length) {
        uint8_t field_length = payload[offset++];
        if (field_length == 0) {
            break;
        }
        if (field_length > length - offset) {
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t field_type = payload[offset++];
        const uint8_t *field_data = payload + offset;
        size_t field_data_length = field_length - 1;
        switch (field_type) {
        case 0x03: // Complete list of 16-bit service UUIDs
            if (field_data_length % 2 != 0) {
                return ESP_ERR_INVALID_SIZE;
            }
            for (size_t i = 0; i < field_data_length; i += 2) {
                if (field_data[i] == (NFC_BLE_SERVICE_UUID & 0xff) &&
                    field_data[i + 1] == (NFC_BLE_SERVICE_UUID >> 8)) {
                    have_service = true;
                }
            }
            break;
        case 0x1b: // LE Bluetooth Device Address
            if (field_data_length != 7 || field_data[6] > 1) {
                return ESP_ERR_INVALID_SIZE;
            }
            memcpy(info->address, field_data, 6);
            info->address_type = field_data[6];
            have_address = true;
            break;
        case 0x1c: // LE Role
            if (field_data_length != 1) {
                return ESP_ERR_INVALID_SIZE;
            }
            have_role = field_data[0] == 0x00 || field_data[0] == 0x02;
            break;
        case 0x10: // LE Security Manager TK Value
            if (field_data_length != sizeof(info->tk)) {
                return ESP_ERR_INVALID_SIZE;
            }
            memcpy(info->tk, field_data, sizeof(info->tk));
            have_tk = true;
            break;
        case 0x09: // Complete Local Name
        case 0x08: // Shortened Local Name
            if (field_data_length < sizeof(info->name)) {
                memcpy(info->name, field_data, field_data_length);
                info->name[field_data_length] = '\0';
            }
            break;
        default:
            break;
        }
        offset += field_data_length;
    }

    return have_address && have_role && have_service && have_tk &&
                   strcmp(info->name, NFC_BLE_LAMP_NAME) == 0
               ? ESP_OK
               : ESP_ERR_NOT_FOUND;
}

static esp_err_t parse_ndef_message(const uint8_t *ndef, size_t length,
                                    ble_oob_info_t *info)
{
    if (length < 3) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t header = ndef[0];
    if ((header & 0xd7) != 0xd2 || (header & 0x08) != 0) {
        ESP_LOGW("nfc_oob", "Unsupported NDEF record header 0x%02X, length %u",
                 header, (unsigned)length);
        return ESP_ERR_NOT_SUPPORTED;
    }
    size_t type_length = ndef[1];
    size_t payload_length = ndef[2];
    if (type_length != NDEF_MIME_TYPE_LEN ||
        type_length + payload_length + 3 != length ||
        memcmp(ndef + 3, NDEF_MIME_TYPE, NDEF_MIME_TYPE_LEN) != 0) {
        return ESP_ERR_NOT_FOUND;
    }

    return parse_oob_payload(ndef + 3 + type_length, payload_length, info);
}

esp_err_t nfc_ble_oob_read(pn532_io_handle_t io, ble_oob_info_t *info,
                           bool *blank_tag)
{
    if (io == NULL || info == NULL || blank_tag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *blank_tag = false;
    memset(info, 0, sizeof(*info));

    NTAG2XX_MODEL model = NTAG2XX_UNKNOWN;
    ESP_RETURN_ON_ERROR(ntag2xx_get_model(io, &model), "nfc_oob",
                        "unsupported tag");
    size_t capacity = 0;
    switch (model) {
    case NTAG2XX_NTAG213:
        capacity = NTAG213_USER_BYTES;
        break;
    case NTAG2XX_NTAG215:
        capacity = NTAG215_USER_BYTES;
        break;
    case NTAG2XX_NTAG216:
        capacity = NTAG216_USER_BYTES;
        break;
    default:
        ESP_LOGW("nfc_oob", "Unsupported NTAG model value %d", (int)model);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t memory[MAX_NTAG_USER_BYTES];
    ESP_RETURN_ON_ERROR(read_user_memory(io, memory, capacity), "nfc_oob",
                        "tag memory read failed");
    ESP_LOGI("nfc_oob", "Type 2 user data prefix: %02X %02X %02X %02X",
             memory[0], memory[1], memory[2], memory[3]);
    if (memory[0] == 0x00 || memory[0] == 0xfe) {
        *blank_tag = true;
        return ESP_ERR_NOT_FOUND;
    }

    size_t offset = 0;
    while (offset < capacity && offset < sizeof(memory)) {
        uint8_t tlv = memory[offset++];
        if (tlv == 0x00) {
            continue;
        }
        if (tlv == 0xfe) {
            return ESP_ERR_NOT_FOUND;
        }
        if (tlv != 0x03) {
            if (tlv == 0x01) {
                if (offset >= capacity || memory[offset] >= capacity - offset) {
                    return ESP_ERR_INVALID_SIZE;
                }
                offset += memory[offset] + 1;
                continue;
            }
            return ESP_ERR_NOT_FOUND;
        }
        if (offset >= capacity) {
            return ESP_ERR_INVALID_SIZE;
        }

        size_t ndef_length = memory[offset++];
        if (ndef_length == 0xff) {
            if (offset + 2 > capacity) {
                return ESP_ERR_INVALID_SIZE;
            }
            ndef_length = ((size_t)memory[offset] << 8) | memory[offset + 1];
            offset += 2;
        }
        if (ndef_length == 0) {
            *blank_tag = true;
            return ESP_ERR_NOT_FOUND;
        }
        if (ndef_length > capacity - offset || ndef_length > sizeof(memory) - offset) {
            return ESP_ERR_INVALID_SIZE;
        }
        ESP_LOGI("nfc_oob", "NDEF TLV length %u, record header/type-length/payload-length: %02X %02X %02X",
                 (unsigned)ndef_length, memory[offset], memory[offset + 1],
                 memory[offset + 2]);
        return parse_ndef_message(memory + offset, ndef_length, info);
    }
    return ESP_ERR_NOT_FOUND;
}

static size_t append_ad(uint8_t *payload, size_t offset, uint8_t type,
                        const uint8_t *data, size_t data_length)
{
    payload[offset++] = data_length + 1;
    payload[offset++] = type;
    memcpy(payload + offset, data, data_length);
    return offset + data_length;
}

esp_err_t nfc_ble_oob_write(pn532_io_handle_t io, const ble_oob_info_t *info)
{
    if (io == NULL || info == NULL || info->address_type > 1 ||
        info->name[0] == '\0' || memchr(info->name, '\0', sizeof(info->name)) == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ble_oob_info_t existing = {0};
    bool blank = false;
    esp_err_t err = nfc_ble_oob_read(io, &existing, &blank);
    if (err != ESP_ERR_NOT_FOUND || !blank) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    uint8_t payload[96];
    size_t payload_length = 0;
    const uint8_t flags[] = {0x06};
    const uint8_t service[] = {0xf0, 0xff};
    const uint8_t role[] = {0x00};
    uint8_t address[7];
    memcpy(address, info->address, 6);
    address[6] = info->address_type & 1;

    payload_length = append_ad(payload, payload_length, 0x01, flags,
                               sizeof(flags));
    payload_length = append_ad(payload, payload_length, 0x03, service,
                               sizeof(service));
    payload_length = append_ad(payload, payload_length, 0x09,
                               (const uint8_t *)info->name, strlen(info->name));
    payload_length = append_ad(payload, payload_length, 0x1b, address,
                               sizeof(address));
    payload_length = append_ad(payload, payload_length, 0x1c, role,
                               sizeof(role));
    payload_length = append_ad(payload, payload_length, 0x10, info->tk,
                               sizeof(info->tk));

    size_t ndef_length = 3 + NDEF_MIME_TYPE_LEN + payload_length;
    if (ndef_length > 0xfe) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t record[128];
    size_t record_length = 0;
    record[record_length++] = 0xd2;
    record[record_length++] = NDEF_MIME_TYPE_LEN;
    record[record_length++] = payload_length;
    memcpy(record + record_length, NDEF_MIME_TYPE, NDEF_MIME_TYPE_LEN);
    record_length += NDEF_MIME_TYPE_LEN;
    memcpy(record + record_length, payload, payload_length);
    record_length += payload_length;

    uint8_t memory[NTAG213_USER_BYTES] = {0};
    memory[0] = 0x03;
    memory[1] = record_length;
    memcpy(memory + 2, record, record_length);
    memory[2 + record_length] = 0xfe;

    size_t write_length = 2 + record_length + 1;
    size_t page_count = (write_length + 3) / 4;
    if (page_count > NTAG213_USER_BYTES / 4) {
        return ESP_ERR_INVALID_SIZE;
    }
    for (size_t i = page_count; i > 1; --i) {
        ESP_RETURN_ON_ERROR(ntag2xx_write_page(io, 4 + i - 1,
                                               memory + (i - 1) * 4),
                            "nfc_oob", "tag write failed");
    }
    ESP_RETURN_ON_ERROR(ntag2xx_write_page(io, 4, memory), "nfc_oob",
                        "tag header write failed");

    ble_oob_info_t check = {0};
    ESP_RETURN_ON_ERROR(nfc_ble_oob_read(io, &check, &blank), "nfc_oob",
                        "tag verification failed");
    if (memcmp(check.address, info->address, sizeof(check.address)) != 0 ||
        check.address_type != info->address_type ||
        memcmp(check.tk, info->tk, sizeof(check.tk)) != 0 ||
        strcmp(check.name, info->name) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
