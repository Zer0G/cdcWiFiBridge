idf_component_register(
    SRCS "st7789v.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_lcd esp_driver_spi esp_driver_gpio log esp_timer"
)
