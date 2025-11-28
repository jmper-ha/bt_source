#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "bt_app_core.h"
#include "bt_app_avrc_ct.h"
#include "uart_app.h"
#include "utils.h"

meta_t  meta;
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
static bool volume_control;
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
static bool cover_art_connected = false;
static bool cover_art_getting = false;
static uint32_t cover_art_image_size = 0;
static uint8_t image_handle_old[7];
uint8_t* image_handler = NULL;
uint16_t buff_size = 510;
uint8_t* buff;
uint16_t data_to_send = 0;
uint32_t image_size;

static bool image_handle_check(uint8_t *image_handle, int len);
#endif
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
static void meta_init();
static void collect_and_send_meta(esp_avrc_ct_cb_param_t *rc);
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
static void bt_av_new_track(void);
static void bt_av_playback_changed(void);
static void bt_av_volume_changed(void);
static void volume_set_by_controller(uint8_t volume);

void avrc_ct_init(){
    meta_init();
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);
}

static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param){
//    ESP_LOGI(BT_AVRC_CT_TAG, "%s event: %d", __func__, event);

    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);
        /* fall through */
//    case ESP_AVRC_CT_PROF_STATE_EVT:
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
    case ESP_AVRC_CT_COVER_ART_STATE_EVT:
    case ESP_AVRC_CT_COVER_ART_DATA_EVT:{
        bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AVRC_CT_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param){
//    ESP_LOGI(BT_AVRC_CT_TAG, "%s event: %d", __func__, event);

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_AVRC_CT_TAG, "AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            /* get remote supported event_ids of peer AVRCP Target */
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            /* clear peer notification capability record */
            s_avrc_peer_rn_cap.bits = 0;
        }
        volume_control = false;
        break;
    }
    /* when passthrough response, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_AVRC_CT_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d", rc->psth_rsp.key_code,
                    rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
        break;
    }
    /* when metadata response, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_AVRC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, len %d, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_length,rc->meta_rsp.attr_text);
        collect_and_send_meta(rc);
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        if(rc->meta_rsp.attr_id == 0x80 && cover_art_connected && cover_art_getting == false) {
            /* check image handle is valid and different with last one, wo dont want to get an image repeatedly */
            if(image_handle_check(rc->meta_rsp.attr_text, rc->meta_rsp.attr_length)) {
                esp_avrc_ct_cover_art_get_linked_thumbnail(rc->meta_rsp.attr_text);
                cover_art_getting = true;
            }
        }
#endif
        free(rc->meta_rsp.attr_text);
        break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_AVRC_CT_TAG, "AVRC event notification: %d", rc->change_ntf.event_id);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_AVRC_CT_TAG, "AVRC remote features %"PRIx32", TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        if ((rc->rmt_feats.tg_feat_flag & ESP_AVRC_FEAT_FLAG_TG_COVER_ART) && !cover_art_connected) {
            ESP_LOGW(BT_AVRC_CT_TAG, "Peer support Cover Art feature, start connection...");
            /* set mtu to zero to use a default value */
            esp_avrc_ct_cover_art_connect(0);
        }
#endif
        break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(BT_AVRC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,ESP_AVRC_RN_VOLUME_CHANGE)){
            uart_send_dparam(APP_UART_SEND_BT_VOLUM,1);
            volume_control = true;
        }
        bt_av_playback_changed();
        bt_av_new_track();
        bt_av_volume_changed();
        break;
    }
    /* when set absolute volume responded, this event comes */
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT: {
        ESP_LOGI(BT_AVRC_CT_TAG, "Set absolute volume response: volume %d", rc->set_volume_rsp.volume);
        break;
    }
    case ESP_AVRC_CT_COVER_ART_STATE_EVT: {
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        if (rc->cover_art_state.state == ESP_AVRC_COVER_ART_CONNECTED) {
            cover_art_connected = true;
            ESP_LOGW(BT_AVRC_CT_TAG, "Cover Art Client connected");
        }
        else {
            cover_art_connected = false;
            ESP_LOGW(BT_AVRC_CT_TAG, "Cover Art Client disconnected, reason:%d", rc->cover_art_state.reason);
        }
#endif
        break;
    }
    case ESP_AVRC_CT_COVER_ART_DATA_EVT: {
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        /* when rc->cover_art_data.final is true, it means we have received the entire image or get operation failed */
        if(rc->cover_art_data.status == ESP_BT_STATUS_SUCCESS){
            image_handler = (uint8_t *)realloc(image_handler, cover_art_image_size + rc->cover_art_data.data_len);
            memcpy(image_handler + cover_art_image_size, rc->cover_art_data.p_data, rc->cover_art_data.data_len);
            cover_art_image_size += rc->cover_art_data.data_len;
            if (rc->cover_art_data.final){
                ESP_LOGI(BT_AVRC_CT_TAG, "Cover Art Client final data event, image size: %lu bytes", cover_art_image_size);
                uart_send_dparam(APP_UART_SEND_ART_IMG,cover_art_image_size);

                cover_art_image_size = 600; //for test
                cover_art_image_size = 0; // block sent image
                image_size = cover_art_image_size;
                buff = malloc(buff_size + 2);
                while (image_size){
                    data_to_send = MIN(buff_size,image_size);
                    sprintf((char*)buff,"%d:",APP_UART_SEND_ART_IMG_DATA);
                    memcpy(buff+3,image_handler+cover_art_image_size-image_size,data_to_send);
                    uart_send_data_len((char*)buff,data_to_send+2);
                    image_size -= data_to_send;
                }
                free(buff);
                cover_art_image_size = 0;
                cover_art_getting = false;
            }
        }

#endif
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AVRC_CT_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter){
    switch (event_id) {
    /* when new track is loaded, this event comes */
    case ESP_AVRC_RN_TRACK_CHANGE:
        bt_av_new_track();
        break;
    /* when track status changed, this event comes */
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        ESP_LOGI(BT_AVRC_CT_TAG, "Playback status changed: 0x%x", event_parameter->playback);
        meta.audio_stat = event_parameter->playback;
        uart_send_dparam(APP_UART_SEND_PLAYBACK,meta.audio_stat);
        bt_av_playback_changed();
        break;
    case ESP_AVRC_RN_VOLUME_CHANGE: {
        ESP_LOGI(BT_AVRC_CT_TAG, "Volume changed: %d", event_parameter->volume);
        volume_set_by_controller(event_parameter->volume);
        esp_avrc_ct_send_set_absolute_volume_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, event_parameter->volume);
        bt_av_volume_changed();
        break;
    }
    /* others */
    default:
        ESP_LOGI(BT_AVRC_CT_TAG, "unhandled event: %d", event_id);
        break;
    }
}

static void bt_av_new_track(void){
    /* request metadata */
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM;// |
//                        ESP_AVRC_MD_ATTR_GENRE;
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
    if (cover_art_connected) {
        attr_mask |= ESP_AVRC_MD_ATTR_COVER_ART;
    }
#endif
     esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,ESP_AVRC_RN_TRACK_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

static void bt_av_playback_changed(void){
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

static void bt_av_volume_changed(void){
    // register notification if peer support the event_id /
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,ESP_AVRC_RN_VOLUME_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, ESP_AVRC_RN_VOLUME_CHANGE, 10);
    }
}

static void collect_and_send_meta(esp_avrc_ct_cb_param_t *rc){
    switch (rc->meta_rsp.attr_id){
    case ESP_AVRC_MD_ATTR_TITLE:
        meta.title_len = rc->meta_rsp.attr_length;
        if(meta.title_len){
            meta.title = malloc(meta.title_len+1);
            memcpy(meta.title,rc->meta_rsp.attr_text,meta.title_len);
            meta.title[meta.title_len] = 0;
        }
        break;
    case ESP_AVRC_MD_ATTR_ARTIST:
        meta.artist_len = rc->meta_rsp.attr_length;
        if(meta.artist_len){
            if(meta.title_len){
                meta.artist = malloc(meta.title_len + meta.artist_len + 4);
                sprintf(meta.artist,"%s - %s",rc->meta_rsp.attr_text,meta.title);
            }else{
                meta.artist = malloc(meta.artist_len+1);
                memcpy(meta.artist,rc->meta_rsp.attr_text,meta.artist_len);
                meta.artist[meta.artist_len] = 0;
            }
            uart_send_sparam(APP_UART_SEND_BT_NAME,meta.artist);
        } else if(meta.title_len) uart_send_sparam(APP_UART_SEND_BT_NAME,meta.title);
          else uart_send_sparam(APP_UART_SEND_BT_NAME,get_audio_state(meta.audio_stat));
        break;
    case ESP_AVRC_MD_ATTR_ALBUM:
        meta.album_len = rc->meta_rsp.attr_length;
        if(meta.album_len){
            meta.album = malloc(meta.album_len+1);
            memcpy(meta.album,rc->meta_rsp.attr_text,meta.album_len);
            meta.album[meta.album_len] = 0;
            uart_send_sparam(APP_UART_SEND_BT_TITLE,meta.album);
        } else uart_send_sparam(APP_UART_SEND_BT_TITLE,get_peer_name());
        if(meta.title_len) free(meta.title);
        if(meta.artist_len) free(meta.artist);
        if(meta.album_len) free(meta.album);
        break;
    default:
        break;
    }
}

static void meta_init(){
    meta.title = NULL;
    meta.artist = NULL;
    meta.album = NULL;
    meta.title_len = 0;
    meta.artist_len = 0;
    meta.album_len = 0;
    meta.audio_stat = 0;
};

static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param){
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);

    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    rc->meta_rsp.attr_text = attr_text;
}
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
static bool image_handle_check(uint8_t *image_handle, int len){
    /* Image handle length must be 7 */
    if (len == 7 && memcmp(image_handle_old, image_handle, 7) != 0) {
        memcpy(image_handle_old, image_handle, 7);
        return true;
    }
    return false;
}
#endif

void volume_set_by_local_host_source_mode(uint8_t volume){
    if(!volume_control) return;
    ESP_LOGI(BT_AVRC_CT_TAG, "Volume is set locally to: %"PRIu32"%%", (uint32_t)volume * 50 / 0x7f);
    set_local_volume(volume);
    esp_avrc_ct_send_set_absolute_volume_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, volume / 2);
}

static void volume_set_by_controller(uint8_t volume){
    ESP_LOGI(BT_AVRC_CT_TAG, "Volume is set by remote controller to: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    set_local_volume(volume * 2);
    timer_start();
}