#include "gt911.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static const char *TAG = "gt911";

// GT911 register addresses
#define GT911_REG_PRODUCT_ID        0x8140
#define GT911_REG_FIRMWARE_VERSION  0x8144
#define GT911_REG_X_COORD           0x814E
#define GT911_REG_Y_COORD           0x8150
#define GT911_REG_TOUCH_STATUS      0x814E
#define GT911_REG_TOUCH_POINTS      0x814C
#define GT911_REG_CONFIG_VERSION    0x8047
#define GT911_REG_CONFIG_CHECKSUM   0x80FF

// GT911 commands
#define GT911_CMD_READ_COORD        0x814E
#define GT911_CMD_CLEAR_BUFFER      0x814E

// GT911 status bits
#define GT911_STATUS_TOUCH_VALID    0x80
#define GT911_STATUS_LARGE_TOUCH    0x40
#define GT911_STATUS_PROXIMITY      0x20

// GT911 configuration
#define GT911_MAX_TOUCH_POINTS      5
#define GT911_I2C_TIMEOUT_MS        100

struct gt911_handle {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t int_pin;
    gpio_num_t rst_pin;
    uint16_t width;
    uint16_t height;
    bool initialized;
};

static esp_err_t gt911_i2c_read_reg(gt911_handle_t handle, uint16_t reg_addr, uint8_t *data, size_t len);
static esp_err_t gt911_i2c_write_reg(gt911_handle_t handle, uint16_t reg_addr, const uint8_t *data, size_t len);
static esp_err_t gt911_hardware_reset(gt911_handle_t handle);
static esp_err_t gt911_software_reset(gt911_handle_t handle);

esp_err_t gt911_init(const gt911_config_t *config, gt911_handle_t *handle)
{
    esp_err_t ret = ESP_OK;
    gt911_handle_t gt911 = NULL;

    ESP_GOTO_ON_FALSE(config && handle, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

    gt911 = calloc(1, sizeof(struct gt911_handle));
    ESP_GOTO_ON_FALSE(gt911, ESP_ERR_NO_MEM, err, TAG, "no mem for gt911 handle");

    gt911->i2c_port = config->i2c_port;
    gt911->i2c_addr = config->i2c_addr;
    gt911->int_pin = config->int_pin;
    gt911->rst_pin = config->rst_pin;
    gt911->width = config->width;
    gt911->height = config->height;

    // Configure interrupt pin
    if (gt911->int_pin >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << gt911->int_pin,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for INT failed");
    }

    // Configure reset pin
    if (gt911->rst_pin >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << gt911->rst_pin,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST failed");
    }

    // Hardware reset
    ESP_GOTO_ON_ERROR(gt911_hardware_reset(gt911), err, TAG, "hardware reset failed");

    // Wait for device to be ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check if device is responding
    uint8_t product_id[4];
    ret = gt911_i2c_read_reg(gt911, GT911_REG_PRODUCT_ID, product_id, sizeof(product_id));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read product ID, trying software reset");
        ESP_GOTO_ON_ERROR(gt911_software_reset(gt911), err, TAG, "software reset failed");
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = gt911_i2c_read_reg(gt911, GT911_REG_PRODUCT_ID, product_id, sizeof(product_id));
    }

    ESP_GOTO_ON_ERROR(ret, err, TAG, "device not responding");

    ESP_LOGI(TAG, "GT911 Product ID: %02X %02X %02X %02X", 
             product_id[0], product_id[1], product_id[2], product_id[3]);

    gt911->initialized = true;
    *handle = gt911;

    ESP_LOGI(TAG, "GT911 initialized successfully");
    return ESP_OK;

err:
    if (gt911) {
        if (gt911->int_pin >= 0) {
            gpio_reset_pin(gt911->int_pin);
        }
        if (gt911->rst_pin >= 0) {
            gpio_reset_pin(gt911->rst_pin);
        }
        free(gt911);
    }
    return ret;
}

esp_err_t gt911_deinit(gt911_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

    if (handle->int_pin >= 0) {
        gpio_reset_pin(handle->int_pin);
    }
    if (handle->rst_pin >= 0) {
        gpio_reset_pin(handle->rst_pin);
    }

    free(handle);
    ESP_LOGI(TAG, "GT911 deinitialized");
    return ESP_OK;
}

esp_err_t gt911_read_touch(gt911_handle_t handle, gt911_touch_data_t *touch_data)
{
    ESP_RETURN_ON_FALSE(handle && touch_data, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(handle->initialized, ESP_ERR_INVALID_STATE, TAG, "device not initialized");

    uint8_t status;
    esp_err_t ret = gt911_i2c_read_reg(handle, GT911_REG_TOUCH_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Check if touch is valid
    if (!(status & GT911_STATUS_TOUCH_VALID)) {
        touch_data->point_count = 0;
        return ESP_OK;
    }

    // Get number of touch points
    uint8_t point_count = status & 0x0F;
    if (point_count > GT911_MAX_TOUCH_POINTS) {
        point_count = GT911_MAX_TOUCH_POINTS;
    }

    touch_data->point_count = point_count;

    // Read touch points
    for (int i = 0; i < point_count; i++) {
        uint8_t touch_data_raw[8];
        uint16_t reg_addr = GT911_CMD_READ_COORD + 1 + (i * 8);
        
        ret = gt911_i2c_read_reg(handle, reg_addr, touch_data_raw, sizeof(touch_data_raw));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read touch point %d", i);
            continue;
        }

        // Parse touch data
        touch_data->points[i].id = touch_data_raw[0];
        touch_data->points[i].x = touch_data_raw[1] | (touch_data_raw[2] << 8);
        touch_data->points[i].y = touch_data_raw[3] | (touch_data_raw[4] << 8);
        touch_data->points[i].size = touch_data_raw[5] | (touch_data_raw[6] << 8);

        // Validate coordinates
        if (touch_data->points[i].x > handle->width) {
            touch_data->points[i].x = handle->width;
        }
        if (touch_data->points[i].y > handle->height) {
            touch_data->points[i].y = handle->height;
        }
    }

    // Clear touch buffer
    uint8_t clear_cmd = 0;
    gt911_i2c_write_reg(handle, GT911_CMD_CLEAR_BUFFER, &clear_cmd, 1);

    return ESP_OK;
}

bool gt911_is_interrupt_active(gt911_handle_t handle)
{
    if (!handle || handle->int_pin < 0) {
        return false;
    }
    return gpio_get_level(handle->int_pin) == 0; // Active low
}

esp_err_t gt911_reset(gt911_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    return gt911_hardware_reset(handle);
}

static esp_err_t gt911_i2c_read_reg(gt911_handle_t handle, uint16_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg_addr >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg_addr & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(GT911_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t gt911_i2c_write_reg(gt911_handle_t handle, uint16_t reg_addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg_addr >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg_addr & 0xFF, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(GT911_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t gt911_hardware_reset(gt911_handle_t handle)
{
    if (handle->rst_pin < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Reset sequence
    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

static esp_err_t gt911_software_reset(gt911_handle_t handle)
{
    // Send software reset command
    uint8_t reset_cmd = 0x02;
    return gt911_i2c_write_reg(handle, 0x8040, &reset_cmd, 1);
}
