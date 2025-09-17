idf_component_register(SRCS "main.c" "lv_fs_port.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES display_driver touch_driver nvs_flash esp_bsp lvgl_ui lvgl esp_event esp_netif esp_wifi driver libEspnow)
                    