idf_component_register(
    SRCS "gt911.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_driver_i2c esp_driver_gpio i2c_bus"
)
