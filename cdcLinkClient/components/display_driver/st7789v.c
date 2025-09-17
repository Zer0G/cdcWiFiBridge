#include <stdint.h>
#include "esp_lcd_panel_interface.h"
#include "st7789v.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "st7789v";

// ST7789V commands
#define ST7789V_CMD_SWRESET     0x01
#define ST7789V_CMD_SLPOUT      0x11
#define ST7789V_CMD_NORON       0x13
#define ST7789V_CMD_INVOFF      0x20
#define ST7789V_CMD_INVON       0x21
#define ST7789V_CMD_DISPOFF     0x28
#define ST7789V_CMD_DISPON      0x29
#define ST7789V_CMD_CASET       0x2A
#define ST7789V_CMD_RASET       0x2B
#define ST7789V_CMD_RAMWR       0x2C
#define ST7789V_CMD_MADCTL      0x36
#define ST7789V_CMD_COLMOD      0x3A
#define ST7789V_CMD_RAMCTRL     0xB0
#define ST7789V_CMD_RGBCTRL     0xB1
#define ST7789V_CMD_PORCTRL     0xB2
#define ST7789V_CMD_FRCTRL1     0xB3
#define ST7789V_CMD_PARCTRL     0xB5
#define ST7789V_CMD_GCTRL       0xB7
#define ST7789V_CMD_GTADJ       0xB8
#define ST7789V_CMD_DGMEN       0xBA
#define ST7789V_CMD_VCOMS       0xBB
#define ST7789V_CMD_LCMCTRL     0xC0
#define ST7789V_CMD_IDSET       0xC1
#define ST7789V_CMD_VDVVRHEN    0xC2
#define ST7789V_CMD_VRHS        0xC3
#define ST7789V_CMD_VDVSET      0xC4
#define ST7789V_CMD_VCMOFSET    0xC5
#define ST7789V_CMD_FRCTR2      0xC6
#define ST7789V_CMD_CABCCTRL    0xC7
#define ST7789V_CMD_REGSEL1     0xC8
#define ST7789V_CMD_REGSEL2     0xCA
#define ST7789V_CMD_PWMFRSEL    0xCC
#define ST7789V_CMD_PWCTRL1     0xD0
#define ST7789V_CMD_VAPVANEN    0xD2
#define ST7789V_CMD_CMD2EN      0xDF
#define ST7789V_CMD_PVGAMCTRL   0xE0
#define ST7789V_CMD_NVGAMCTRL   0xE1
#define ST7789V_CMD_DGMLUTR     0xE2
#define ST7789V_CMD_DGMLUTB     0xE3
#define ST7789V_CMD_GATECTRL    0xE4
#define ST7789V_CMD_PWCTRL2     0xE8
#define ST7789V_CMD_EQCTRL      0xE9
#define ST7789V_CMD_PROMCTRL    0xEC
#define ST7789V_CMD_PROMEN      0xFA
#define ST7789V_CMD_NVMSET     0xFC
#define ST7789V_CMD_PROMACT     0xFE

// MADCTL bits
#define ST7789V_MADCTL_MY       0x80
#define ST7789V_MADCTL_MX       0x40
#define ST7789V_MADCTL_MV       0x20
#define ST7789V_MADCTL_ML       0x10
#define ST7789V_MADCTL_BGR      0x08
#define ST7789V_MADCTL_MH       0x04

// Color modes
#define ST7789V_COLMOD_16BIT    0x55

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int cs_gpio_num;
    int dc_gpio_num;
    int bl_gpio_num;
    int width;
    int height;
    int offset_x;
    int offset_y;
    bool invert_colors;
    bool mirror_x;
    bool mirror_y;
    bool swap_xy;
} st7789v_panel_t;

static esp_err_t st7789v_del(esp_lcd_panel_t *panel);
static esp_err_t st7789v_reset(esp_lcd_panel_t *panel);
static esp_err_t st7789v_init(esp_lcd_panel_t *panel);
static esp_err_t st7789v_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t st7789v_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t st7789v_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t st7789v_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t st7789v_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t st7789v_disp_on_off(esp_lcd_panel_t *panel, bool off);

static void st7789v_send_cmd(st7789v_panel_t *st7789v, uint8_t cmd);
static void st7789v_send_data(st7789v_panel_t *st7789v, uint8_t data);
static void st7789v_send_color(st7789v_panel_t *st7789v, uint8_t data);

esp_err_t st7789v_new_panel(const esp_lcd_panel_io_handle_t io, const st7789v_panel_config_t *panel_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7789v_panel_t *st7789v = NULL;

    ESP_GOTO_ON_FALSE(io && panel_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7789v = calloc(1, sizeof(st7789v_panel_t));
    ESP_GOTO_ON_FALSE(st7789v, ESP_ERR_NO_MEM, err, TAG, "no mem for st7789v panel");

    if (panel_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST failed");
    }

    if (panel_config->bl_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_config->bl_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for BL failed");
        gpio_set_level(panel_config->bl_gpio_num, 1);
    }

    st7789v->io = io;
    st7789v->reset_gpio_num = panel_config->reset_gpio_num;
    st7789v->cs_gpio_num = panel_config->cs_gpio_num;
    st7789v->dc_gpio_num = panel_config->dc_gpio_num;
    st7789v->bl_gpio_num = panel_config->bl_gpio_num;
    st7789v->width = panel_config->width;
    st7789v->height = panel_config->height;
    st7789v->offset_x = panel_config->offset_x;
    st7789v->offset_y = panel_config->offset_y;
    st7789v->invert_colors = panel_config->invert_colors;
    st7789v->mirror_x = panel_config->mirror_x;
    st7789v->mirror_y = panel_config->mirror_y;
    st7789v->swap_xy = panel_config->swap_xy;

    st7789v->base.del = st7789v_del;
    st7789v->base.reset = st7789v_reset;
    st7789v->base.init = st7789v_init;
    st7789v->base.draw_bitmap = st7789v_draw_bitmap;
    st7789v->base.invert_color = st7789v_invert_color;
    st7789v->base.mirror = st7789v_mirror;
    st7789v->base.swap_xy = st7789v_swap_xy;
    st7789v->base.set_gap = st7789v_set_gap;
    st7789v->base.disp_on_off = st7789v_disp_on_off;

    *ret_panel = &(st7789v->base);
    ESP_LOGI(TAG, "new st7789v panel @%p", st7789v);

    return ESP_OK;

err:
    if (st7789v) {
        if (panel_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_config->reset_gpio_num);
        }
        if (panel_config->bl_gpio_num >= 0) {
            gpio_reset_pin(panel_config->bl_gpio_num);
        }
        free(st7789v);
    }
    return ret;
}

static esp_err_t st7789v_del(esp_lcd_panel_t *panel)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);

    if (st7789v->reset_gpio_num >= 0) {
        gpio_reset_pin(st7789v->reset_gpio_num);
    }
    if (st7789v->bl_gpio_num >= 0) {
        gpio_reset_pin(st7789v->bl_gpio_num);
    }
    ESP_LOGI(TAG, "del st7789v panel @%p", st7789v);
    free(st7789v);
    return ESP_OK;
}

static esp_err_t st7789v_reset(esp_lcd_panel_t *panel)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;

    // perform hardware reset
    if (st7789v->reset_gpio_num >= 0) {
        gpio_set_level(st7789v->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7789v->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        // perform software reset
        esp_lcd_panel_io_tx_param(io, ST7789V_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static void st7789v_send_cmd(st7789v_panel_t *st7789v, uint8_t cmd)
{
    esp_lcd_panel_io_tx_param(st7789v->io, cmd, NULL, 0);
}

static void st7789v_send_data(st7789v_panel_t *st7789v, uint8_t data)
{
    esp_lcd_panel_io_tx_color(st7789v->io, -1, &data, 1);
}

static void st7789v_send_color(st7789v_panel_t *st7789v, uint8_t data)
{
    esp_lcd_panel_io_tx_color(st7789v->io, -1, &data, 1);
}

static esp_err_t st7789v_init(esp_lcd_panel_t *panel)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_NORON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set color mode to 16bit
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_COLMOD, (uint8_t[]) {
        ST7789V_COLMOD_16BIT
    }, 1);

    // Set memory data access control
    uint8_t madctl_val = 0;
    if (st7789v->mirror_x) {
        madctl_val |= ST7789V_MADCTL_MX;
    }
    if (st7789v->mirror_y) {
        madctl_val |= ST7789V_MADCTL_MY;
    }
    if (st7789v->swap_xy) {
        madctl_val |= ST7789V_MADCTL_MV;
    }
    if (st7789v->invert_colors) {
        madctl_val |= ST7789V_MADCTL_BGR;
    }
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_MADCTL, (uint8_t[]) {
        madctl_val
    }, 1);

    // Turn on display
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_DISPON, NULL, 0);

    ESP_LOGI(TAG, "st7789v panel initialized");
    return ESP_OK;
}

static esp_err_t st7789v_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = st7789v->io;

    x_start += st7789v->offset_x;
    y_start += st7789v->offset_y;
    x_end += st7789v->offset_x;
    y_end += st7789v->offset_y;

    // define an area of frame memory where MCU can access
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4);
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * 2; // 2 bytes per pixel
    esp_lcd_panel_io_tx_color(io, ST7789V_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t st7789v_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    int command = 0;
    if (invert_color_data) {
        command = ST7789V_CMD_INVON;
    } else {
        command = ST7789V_CMD_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t st7789v_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    if (mirror_x) {
        st7789v->mirror_x = true;
    }
    if (mirror_y) {
        st7789v->mirror_y = true;
    }
    // MADCTL register bits
    // 7 6 5 4 3 2 1 0
    // MY MX MV ML BGR MH X X
    uint8_t madctl_val = 0;
    if (st7789v->mirror_x) {
        madctl_val |= ST7789V_MADCTL_MX;
    }
    if (st7789v->mirror_y) {
        madctl_val |= ST7789V_MADCTL_MY;
    }
    if (st7789v->swap_xy) {
        madctl_val |= ST7789V_MADCTL_MV;
    }
    if (st7789v->invert_colors) {
        madctl_val |= ST7789V_MADCTL_BGR;
    }
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_MADCTL, (uint8_t[]) {
        madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t st7789v_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    if (swap_axes) {
        st7789v->swap_xy = true;
    }
    // MADCTL register bits
    // 7 6 5 4 3 2 1 0
    // MY MX MV ML BGR MH X X
    uint8_t madctl_val = 0;
    if (st7789v->mirror_x) {
        madctl_val |= ST7789V_MADCTL_MX;
    }
    if (st7789v->mirror_y) {
        madctl_val |= ST7789V_MADCTL_MY;
    }
    if (st7789v->swap_xy) {
        madctl_val |= ST7789V_MADCTL_MV;
    }
    if (st7789v->invert_colors) {
        madctl_val |= ST7789V_MADCTL_BGR;
    }
    esp_lcd_panel_io_tx_param(io, ST7789V_CMD_MADCTL, (uint8_t[]) {
        madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t st7789v_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    st7789v->offset_x = x_gap;
    st7789v->offset_y = y_gap;
    return ESP_OK;
}

static esp_err_t st7789v_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    st7789v_panel_t *st7789v = __containerof(panel, st7789v_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789v->io;
    int command = 0;

    if (on_off) {
        command = ST7789V_CMD_DISPON;
    } else {
        command = ST7789V_CMD_DISPOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}
