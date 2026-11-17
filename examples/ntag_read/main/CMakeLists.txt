idf_component_register(SRCS "ntag_read.c" "remote_ble.c" "nfc_ble_oob.c"
                    PRIV_REQUIRES bt nvs_flash driver esp_driver_rmt
                    INCLUDE_DIRS "." "../../common")
