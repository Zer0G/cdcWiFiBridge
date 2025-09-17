idf_component_register(SRCS "app_main_server.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_gpio libEspnow   
                    INCLUDE_DIRS ".")
