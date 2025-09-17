
#include "libEspnow.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "SERVER";
#define BUTTON_GPIO GPIO_NUM_0   // Pulsante attivo basso
#define ESPNOW_CHANNEL 1

static void server_rx_cb(const libespnow_msg_t *msg) {
    if (msg->type == LIBESPNOW_MSG_TYPE_DATA && msg->data) {
        ESP_LOGI(TAG, "RX da %02X:%02X:%02X:%02X:%02X:%02X -> %.*s",
                 msg->src_mac[0], msg->src_mac[1], msg->src_mac[2],
                 msg->src_mac[3], msg->src_mac[4], msg->src_mac[5],
                 (int)msg->len, (char*)msg->data);
    }
    libespnow_msg_free((libespnow_msg_t*)msg);
}

static void button_task(void *arg) {
    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            char msg[] = "SERVER_DISCOVERY";
            if(libespnow_send((uint8_t*)LIBESPNOW_BROADCAST_MAC, msg, strlen(msg)+1, true)!=ESP_OK)
                ESP_LOGE(TAG, "Errore invio broadcast");
            else    
             ESP_LOGI(TAG, "Broadcast inviato");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void status_task(void *arg) {
    while (1) {
        libespnow_status_t st;
        libespnow_get_status(&st);
        if(st.state== LIBESPNOW_STATE_PAIRED)
        {
            ESP_LOGI(TAG, "STATE=%d RSSI=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                 st.state, st.last_rssi,
                 st.mac[0], st.mac[1], st.mac[2],
                 st.mac[3], st.mac[4], st.mac[5]);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Avvio SERVER ESP-NOW");
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    if (libespnow_init(server_rx_cb, ESPNOW_CHANNEL) != ESP_OK) {
        ESP_LOGE(TAG, "Errore init libEspnow");
        return;
    }
    xTaskCreate(button_task, "button", 4096, NULL, 4, NULL);
    xTaskCreate(status_task, "status", 4096, NULL, 4, NULL);
}
