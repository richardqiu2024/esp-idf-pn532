#ifndef BLE_OOB_CONFIG_H
#define BLE_OOB_CONFIG_H

#include <stdint.h>

#define NFC_BLE_LAMP_NAME "NFC-BLE-LAMP"
#define NFC_BLE_SERVICE_UUID 0xFFF0
#define NFC_BLE_COMMAND_UUID 0xFFF1

/* Development key shared by these two firmware images. Replace per device. */
#define NFC_BLE_OOB_TK_INIT \
    { 0xef, 0x8c, 0x0a, 0xbf, 0xe4, 0xad, 0xc8, 0xd9, \
      0xe9, 0xef, 0xa5, 0x75, 0xc9, 0x98, 0xca, 0x7e }

typedef struct {
    uint8_t address[6];
    uint8_t address_type;
    uint8_t tk[16];
    char name[32];
} ble_oob_info_t;

#endif
