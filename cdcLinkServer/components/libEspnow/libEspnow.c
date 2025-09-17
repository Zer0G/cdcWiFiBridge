#include "libEspnow.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdlib.h>

#define TAG "libEspnow"

static libespnow_peer_t peers[LIBESPNOW_MAX_PEERS];
static int peer_count = 0;
static SemaphoreHandle_t peer_mutex;
static QueueHandle_t tx_queue;
static libespnow_recv_cb_t user_cb;
static TaskHandle_t discovery_handle = NULL;

/* Stato globale */
static libespnow_status_t g_status = {
    .state = LIBESPNOW_STATE_IDLE,
    .last_rssi = 0,
    .last_seen_ms = 0
};
static uint8_t g_channel = 0;
typedef struct {
    uint8_t dst_mac[ESP_NOW_ETH_ALEN];
    bool broadcast;
    uint8_t *data;
    size_t len;
} tx_msg_t;

/* --- Forward declarations --- */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status);
static void sender_task(void *arg);
static void discovery_task(void *arg);
static esp_err_t ensure_peer_registered(const uint8_t *mac);

esp_err_t libespnow_init(libespnow_recv_cb_t cb, uint8_t channel) {
    if (channel < 1 || channel > 14) {
        return ESP_ERR_INVALID_ARG;
    }
    g_channel = channel;
    user_cb = cb;
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "ESP-NOW using channel %u", g_channel);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    uint8_t bcast_addr[ESP_NOW_ETH_ALEN];
    memset(bcast_addr, 0xFF, sizeof(bcast_addr));
    esp_err_t bcast_ret = ensure_peer_registered(bcast_addr);
    if (bcast_ret != ESP_OK) {
        ESP_ERROR_CHECK(bcast_ret);
    }

    peer_mutex = xSemaphoreCreateMutex();
    tx_queue   = xQueueCreate(10, sizeof(tx_msg_t));
    xTaskCreate(sender_task, "espnow_sender", 4096, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t ensure_peer_registered(const uint8_t *mac) {
    if (!mac) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = g_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                 esp_err_to_name(err));
    }
    return err;
}

esp_err_t libespnow_send(const uint8_t *dst_mac, const void *data, size_t len, bool broadcast) {
    tx_msg_t msg;
    memcpy(msg.dst_mac, dst_mac, ESP_NOW_ETH_ALEN);
    msg.broadcast = broadcast;
    msg.len = len;

    if (!broadcast) {
        esp_err_t peer_err = ensure_peer_registered(dst_mac);
        if (peer_err != ESP_OK) {
            return peer_err;
        }
    }
    msg.data = malloc(len);
    if (!msg.data) return ESP_ERR_NO_MEM;
    memcpy(msg.data, data, len);
    if (xQueueSend(tx_queue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
        free(msg.data);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void sender_task(void *arg) {
    tx_msg_t msg;
    uint8_t bcast[ESP_NOW_ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    while (1) {
        if (xQueueReceive(tx_queue, &msg, portMAX_DELAY)) {
            const uint8_t *target = msg.broadcast ? bcast : msg.dst_mac;
            esp_now_send(target, msg.data, msg.len);
            free(msg.data);
        }
    }
}

esp_err_t libespnow_start_discovery(uint32_t duration_ms) {
    if (discovery_handle != NULL) {
        return ESP_ERR_INVALID_STATE; // già in corso
    }
    if (xTaskCreate(discovery_task, "espnow_disc", 4096,
                    (void*)(uintptr_t)duration_ms, 4,
                    &discovery_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void discovery_task(void *arg) {
    uint32_t duration_ms = (uint32_t)(uintptr_t)arg;
    int64_t start = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Discovery avviato per %d ms", duration_ms);
    uint8_t my_mac[ESP_NOW_ETH_ALEN];
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    g_status.state = LIBESPNOW_STATE_DISCOVERY;
    while ((esp_timer_get_time()/1000 - start) < duration_ms) {
        uint8_t payload[8];
        memcpy(payload, my_mac, 6);
        payload[6] = LIBESPNOW_MSG_TYPE_DISCOVERY;
        payload[7] = 0;
        libespnow_send(my_mac, payload, sizeof(payload), true);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    g_status.state = (peer_count > 0) ? LIBESPNOW_STATE_PAIRED : LIBESPNOW_STATE_IDLE;
    ESP_LOGI(TAG, "Discovery terminato");
    discovery_handle = NULL;
    vTaskDelete(NULL);
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len <= 0) return;

    esp_err_t peer_err = ensure_peer_registered(info->src_addr);
    if (peer_err != ESP_OK) {
        ESP_LOGW(TAG, "ensure_peer_registered failed for %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 info->src_addr[0], info->src_addr[1], info->src_addr[2],
                 info->src_addr[3], info->src_addr[4], info->src_addr[5],
                 esp_err_to_name(peer_err));
    }

    int64_t now = esp_timer_get_time()/1000;
    // Aggiorna stato globale
    memcpy(g_status.mac, info->src_addr, ESP_NOW_ETH_ALEN);
    if (info->rx_ctrl) {
        g_status.last_rssi = info->rx_ctrl->rssi;
    }
    g_status.last_seen_ms = now;
    g_status.state = LIBESPNOW_STATE_PAIRED;
    // Aggiorna peer list
    xSemaphoreTake(peer_mutex, 0);
    int i;
    for (i=0;i<peer_count;i++) {
        if (memcmp(peers[i].mac, info->src_addr, ESP_NOW_ETH_ALEN)==0) {
            peers[i].last_seen_ms = now;
            peers[i].active = true;
            break;
        }
    }
    if (i == peer_count && peer_count < LIBESPNOW_MAX_PEERS) {
        memcpy(peers[peer_count].mac, info->src_addr, ESP_NOW_ETH_ALEN);
        peers[peer_count].last_seen_ms = now;
        peers[peer_count].active = true;
        peer_count++;
    }
    xSemaphoreGive(peer_mutex);
    libespnow_msg_type_t msg_type = LIBESPNOW_MSG_TYPE_DATA;
    if (len >= 7) {
        if (data[6] == LIBESPNOW_MSG_TYPE_DISCOVERY) {
            msg_type = LIBESPNOW_MSG_TYPE_DISCOVERY;
        } else if (data[6] == LIBESPNOW_MSG_TYPE_WELCOME) {
            msg_type = LIBESPNOW_MSG_TYPE_WELCOME;
        }
    }
    // Risposta automatica al DISCOVERY
    if (msg_type == LIBESPNOW_MSG_TYPE_DISCOVERY) {
        uint8_t my_mac[ESP_NOW_ETH_ALEN];
        esp_wifi_get_mac(WIFI_IF_STA, my_mac);
        uint8_t welcome[8];
        memcpy(welcome, my_mac, 6);
        welcome[6] = LIBESPNOW_MSG_TYPE_WELCOME;
        welcome[7] = 0;
        libespnow_send(info->src_addr, welcome, sizeof(welcome), false);
    }
    if (user_cb) {
        libespnow_msg_t msg = {
            .type = msg_type,
            .len = len,
            .is_broadcast = (info->des_addr[0]==0xFF)
        };
        memcpy(msg.src_mac, info->src_addr, ESP_NOW_ETH_ALEN);
        memcpy(msg.dst_mac, info->des_addr, ESP_NOW_ETH_ALEN);
        msg.data = malloc(len);
        if (msg.data) {
            memcpy(msg.data, data, len);
            user_cb(&msg);
        } else {
            ESP_LOGE(TAG, "malloc fallita in recv_cb");
        }
    }
}
static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status) {
    const uint8_t *mac_addr = (tx_info && tx_info->des_addr) ? tx_info->des_addr : NULL;  // esp_now_send_info_t uses des_addr
    if (mac_addr) {
        ESP_LOGD(TAG, "Send cb to %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5],
                 status==ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    } else {
        ESP_LOGD(TAG, "Send cb with unknown dest: %s",
                 status==ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    }
}

size_t libespnow_get_peers(libespnow_peer_t *out, size_t max_peers) {
    size_t n=0;
    xSemaphoreTake(peer_mutex, portMAX_DELAY);
    for (int i=0;i<peer_count && n<max_peers;i++) {
        if (peers[i].active) out[n++] = peers[i];
    }
    xSemaphoreGive(peer_mutex);
    return n;
}

esp_err_t libespnow_deinit(void) {
    esp_now_deinit();
    vQueueDelete(tx_queue);
    vSemaphoreDelete(peer_mutex);
    return ESP_OK;
}

void libespnow_msg_free(libespnow_msg_t *msg) {
    if (msg && msg->data) {
        free(msg->data);
        msg->data = NULL;
        msg->len = 0;
    }
}

void libespnow_get_status(libespnow_status_t *out) {
    if (!out) return;
    xSemaphoreTake(peer_mutex, portMAX_DELAY);
    *out = g_status;
    xSemaphoreGive(peer_mutex);
}