#pragma once

#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ST7789V LCD panel configuration structure
 */
typedef struct {
    int reset_gpio_num;          /*!< GPIO used for reset line */
    int cs_gpio_num;             /*!< GPIO used for CS line */
    int dc_gpio_num;             /*!< GPIO used for DC line */
    int sclk_gpio_num;           /*!< GPIO used for SCLK line */
    int sda_gpio_num;            /*!< GPIO used for SDA line */
    int bl_gpio_num;             /*!< GPIO used for backlight control */
    int pclk_hz;                 /*!< Frequency of pixel clock */
    int width;                   /*!< Panel width */
    int height;                  /*!< Panel height */
    int offset_x;                /*!< Panel X offset */
    int offset_y;                /*!< Panel Y offset */
    bool invert_colors;          /*!< Invert colors */
    bool mirror_x;               /*!< Mirror X axis */
    bool mirror_y;               /*!< Mirror Y axis */
    bool swap_xy;                /*!< Swap X and Y axis */
} st7789v_panel_config_t;

/**
 * @brief Create ST7789V LCD panel
 *
 * @param io LCD panel IO handle
 * @param panel_config LCD panel configuration
 * @param ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 */
esp_err_t st7789v_new_panel(const esp_lcd_panel_io_handle_t io, const st7789v_panel_config_t *panel_config, esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
