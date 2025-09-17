
idf_component_register(
    SRCS "libEspnow.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif esp_wifi esp_timer nvs_flash
)
