#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "bt_app_core.h"
#include "bt_app_avrc_tg.h"
#include "uart_app.h"
#include "utils.h"

static bool s_volume_notify; 

static void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);

static void volume_set_by_controller(uint8_t volume);

void avrc_tg_init(){
    esp_avrc_tg_init();
    esp_avrc_tg_register_callback(bt_app_rc_tg_cb);
    timer_init();
    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);
}

static void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param){
//    ESP_LOGI(BT_AVRC_TG_TAG, "%s event: %d", __func__, event);
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
        bt_app_work_dispatch(bt_av_hdl_avrc_tg_evt, event, param, sizeof(esp_avrc_tg_cb_param_t), NULL);
        break;
    default:
        ESP_LOGE(BT_AVRC_TG_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param){
//    ESP_LOGI(BT_AVRC_TG_TAG, "%s event: %d", __func__, event);
    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);
    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_AVRC_TG_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (rc->conn_stat.connected) {
            esp_avrc_psth_bit_mask_t psth_set = {0};
            esp_avrc_tg_get_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_ALLOWED_CMD, &psth_set);
            if (!esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD,
                                                &psth_set) != ESP_OK) {
                ESP_LOGE(BT_AVRC_TG_TAG, "esp_avrc_tg_set_psth_cmd_filter");
            }
            ESP_LOGI(BT_AVRC_TG_TAG, "Set passthru capabilities: 0x%0x", psth_set);
        }
        break;
    }
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        ESP_LOGI(BT_AVRC_TG_TAG, "AVRC passthrough cmd: key_code 0x%x, key_state %d", rc->psth_cmd.key_code, rc->psth_cmd.key_state);
        if(rc->psth_cmd.key_state) uart_send_dparam(APP_UART_SEND_PT_CMD,rc->psth_cmd.key_code);
        break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        ESP_LOGI(BT_AVRC_TG_TAG, "AVRC set absolute volume: %d%%", (int)rc->set_abs_vol.volume * 100 / 0x7f);
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        ESP_LOGI(BT_AVRC_TG_TAG, "AVRC register event notification: %d, param: 0x%"PRIx32, rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = get_local_volume() / 2;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_AVRC_TG_TAG, "AVRC remote features: %"PRIx32", CT features: %x", rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AVRC_TG_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void volume_set_by_controller(uint8_t volume){
    ESP_LOGI(BT_AVRC_TG_TAG, "Volume is set by remote controller to: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    set_local_volume(volume * 2);
    timer_start();
}

void volume_set_by_local_host_sink_mode(uint8_t volume){
    ESP_LOGI(BT_AVRC_TG_TAG, "Volume is set locally to: %"PRIu32"%%", (uint32_t)volume * 50 / 0x7f);
    set_local_volume(volume);
    if (s_volume_notify) {
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = get_local_volume() / 2;
        printf("send volume local %d sent %d\n",get_local_volume(),rn_param.volume);
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
}
