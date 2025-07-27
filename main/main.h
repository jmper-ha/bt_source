#ifndef __MAIN_H__
#define __MAIN_H__

#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"


/* A2DP global states */
enum {
    APP_AV_STATE_IDLE,
    APP_AV_STATE_DISCOVERING,
    APP_AV_STATE_DISCOVERED,
    APP_AV_STATE_UNCONNECTED,
    APP_AV_STATE_CONNECTING,
    APP_AV_STATE_CONNECTED,
    APP_AV_STATE_DISCONNECTING,
};

/* sub states of APP_AV_STATE_CONNECTED */
enum {
    APP_AV_MEDIA_STATE_IDLE,
    APP_AV_MEDIA_STATE_STARTING,
    APP_AV_MEDIA_STATE_STARTED,
    APP_AV_MEDIA_STATE_STOPPING,
};

typedef enum {
    APP_AUDIO_BT,
    APP_AUDIO_SPEAKER,
} audio_m_t;

static const char *TAG = "BT_MAIN";
static const char *dev_name = "YORADIO_BT";

static esp_bd_addr_t s_peer_bda = {0};  
static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static int s_a2d_state = APP_AV_STATE_IDLE;
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;

static uint8_t  disc_time = 7;//11;//23;

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void scan_result(esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

static void bt_app_av_state_unconnected_hdlr(uint16_t event, void *param);
static void bt_app_av_state_connecting_hdlr(uint16_t event, void *param);
static void bt_app_av_state_connected_hdlr(uint16_t event, void *param);
static void bt_app_av_state_disconnecting_hdlr(uint16_t event, void *param);

static void i2s_init(void);
static void i2s_app_task_handler(void *arg);

static void audio_output(audio_m_t state);
static void save_eir(void);
#endif /* __MAIN_H__ */