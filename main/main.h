#ifndef __MAIN_H__
#define __MAIN_H__

#include "esp_gap_bt_api.h"
#include "esp_avrc_api.h"
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
    APP_MODE_IDLE,
    APP_MODE_SOURCE,
    APP_MODE_SINK,
} app_mode_t;

typedef enum {
    APP_AUDIO_BT,
    APP_AUDIO_SPEAKER,
} audio_m_t;

typedef enum {
    SR_44100,
    SR_22050,
} samplerate_t;

typedef struct {
    samplerate_t    sr;
} uart_rx_msg_t;

static const char *TAG = "BT_MAIN";
static const char *TAG_GAP = "BT_MAIN_APP_GAP";
static char *dev_name = "YORADIO_BT";
static char *dev_alias = NULL;

static esp_bd_addr_t s_peer_bda = {0};  

static uint8_t app_mode = APP_MODE_IDLE;
static int s_a2d_state = APP_AV_STATE_IDLE;
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;

static uint8_t  disc_time = 7;//11;//23;
void i2s_app_task_start_up(void);

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void scan_result(esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_app_a2d_sink_data_cb(const uint8_t *data, uint32_t len);
static void resample(size_t size,const uint8_t *in,uint8_t *out);
static void change_volume(uint8_t* data,size_t size);
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

static void bt_app_av_state_unconnected_hdlr(uint16_t event, void *param);
static void bt_app_av_state_connecting_hdlr(uint16_t event, void *param);
static void bt_app_av_state_connected_hdlr(uint16_t event, void *param);
static void bt_app_av_state_disconnecting_hdlr(uint16_t event, void *param);
static void bt_app_av_sink_mode_hdlr(uint16_t event, void *param);

static void i2s_init(void);
static void i2s_app_task_handler(void *arg);

static void audio_output(audio_m_t state);
static void save_eir(void);
static char* set_dev_name();

static void source_init();
static void sink_init();
static void set_scan_mode();
void collect_and_send_meta(esp_avrc_ct_cb_param_t *rc);

#endif /* __MAIN_H__ */