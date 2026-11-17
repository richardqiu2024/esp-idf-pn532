idf_component_register(SRCS "ble_lamp.c"
                    PRIV_REQUIRES bt nvs_flash driver esp_driver_rmt
                    INCLUDE_DIRS "." "../../common")
