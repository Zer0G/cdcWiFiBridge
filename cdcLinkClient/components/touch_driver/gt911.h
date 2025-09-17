#pragma once

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GT911 touch point structure
 */
typedef struct {
    uint8_t id;         /*!< Touch point ID */
    uint16_t x;         /*!< X coordinate */
    uint16_t y;         /*!< Y coordinate */
    uint16_t size;      /*!< Touch point size */
} gt911_touch_point_t;

/**
 * @brief GT911 touch data structure
 */
typedef struct {
    uint8_t point_count;                    /*!< Number of touch points */
    gt911_touch_point_t points[5];          /*!< Touch points array (max 5 points) */
    uint8_t gesture_id;                     /*!< Gesture ID */
} gt911_touch_data_t;

/**
 * @brief GT911 configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;        /*!< I2C port number */
    uint8_t i2c_addr;           /*!< I2C device address */
    gpio_num_t int_pin;         /*!< Interrupt pin */
    gpio_num_t rst_pin;         /*!< Reset pin */
    uint16_t width;             /*!< Display width */
    uint16_t height;            /*!< Display height */
} gt911_config_t;

/**
 * @brief GT911 handle structure
 */
typedef struct gt911_handle* gt911_handle_t;

/**
 * @brief Initialize GT911 touch controller
 *
 * @param config GT911 configuration
 * @param handle Returned GT911 handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_ERR_NOT_FOUND: Device not found
 */
esp_err_t gt911_init(const gt911_config_t *config, gt911_handle_t *handle);

/**
 * @brief Deinitialize GT911 touch controller
 *
 * @param handle GT911 handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t gt911_deinit(gt911_handle_t handle);

/**
 * @brief Read touch data from GT911
 *
 * @param handle GT911 handle
 * @param touch_data Pointer to store touch data
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_TIMEOUT: Timeout
 */
esp_err_t gt911_read_touch(gt911_handle_t handle, gt911_touch_data_t *touch_data);

/**
 * @brief Check if touch interrupt is active
 *
 * @param handle GT911 handle
 * @return true if interrupt is active, false otherwise
 */
bool gt911_is_interrupt_active(gt911_handle_t handle);

/**
 * @brief Reset GT911 touch controller
 *
 * @param handle GT911 handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t gt911_reset(gt911_handle_t handle);

#ifdef __cplusplus
}
#endif
