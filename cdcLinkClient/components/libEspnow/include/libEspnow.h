
#pragma once
#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIBESPNOW_MAX_PEERS     20
#define LIBESPNOW_BROADCAST_MAC "\xFF\xFF\xFF\xFF\xFF\xFF"

typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    int64_t last_seen_ms;
    bool active;
} libespnow_peer_t;

typedef enum {
    LIBESPNOW_MSG_TYPE_DISCOVERY,
    LIBESPNOW_MSG_TYPE_WELCOME,
    LIBESPNOW_MSG_TYPE_DATA
} libespnow_msg_type_t;

typedef struct {
    libespnow_msg_type_t type;
    uint8_t src_mac[ESP_NOW_ETH_ALEN];
    uint8_t dst_mac[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    size_t len;
    bool is_broadcast;
} libespnow_msg_t;

/* Stato globale della connessione */
typedef enum {
    LIBESPNOW_STATE_IDLE = 0,
    LIBESPNOW_STATE_DISCOVERY,
    LIBESPNOW_STATE_PAIRED
} libespnow_state_t;

typedef struct {
    libespnow_state_t state;
    uint8_t mac[ESP_NOW_ETH_ALEN];
    int8_t last_rssi;
    int64_t last_seen_ms;
} libespnow_status_t;

typedef void (*libespnow_recv_cb_t)(const libespnow_msg_t *msg);

esp_err_t libespnow_init(libespnow_recv_cb_t cb, uint8_t channel);
esp_err_t libespnow_send(const uint8_t *dst_mac, const void *data, size_t len, bool broadcast);
esp_err_t libespnow_start_discovery(uint32_t duration_ms);
size_t libespnow_get_peers(libespnow_peer_t *out, size_t max_peers);
esp_err_t libespnow_deinit(void);
void libespnow_msg_free(libespnow_msg_t *msg);
void libespnow_get_status(libespnow_status_t *out);

#ifdef __cplusplus
}
#endif
