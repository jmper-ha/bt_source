
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_avrc_ct.h"
#include "bt_app_avrc_tg.h"
#include "main.h"
#include "uart_app.h"
#include "utils.h"
#include "adc_app.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "settings.h"

#define TX_PACKET_SIZE                 ( 2 * 1024)
#define RINGBUF_HIGHEST_WATER_LEVEL    (32 * 1024)
#define RINGBUF_PREFETCH_WATER_LEVEL   (20 * 1024)

enum {
    RINGBUFFER_MODE_PROCESSING,    /* ringbuffer is buffering incoming audio data, I2S is working */
    RINGBUFFER_MODE_PREFETCHING,   /* ringbuffer is buffering incoming audio data, I2S is waiting */
    RINGBUFFER_MODE_DROPPING       /* ringbuffer is not buffering (dropping) incoming audio data, I2S is working */
};

static TaskHandle_t         s_i2s_app_task_handler = NULL;
static QueueHandle_t        uart_rx_queue;

static i2s_chan_handle_t    rx_i2s_chan;
static i2s_chan_handle_t    tx_i2s_chan;

static RingbufHandle_t s_ringbuf_i2s = NULL;     /* handle of ringbuffer for I2S */
static SemaphoreHandle_t s_i2s_write_semaphore = NULL;
static uint16_t ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;

arr_t   eir_saved;
arr_t   eir_discovered;

static const char* sink_cfg = "bt_snk.conf";
static const char* source_cfg = "bt_src.conf";

static void audio_output(audio_m_t state){
    gpio_set_level(A_MUTE, state);
    uart_send_dparam(APP_UART_SEND_BT_STATUS,state?APP_AUDIO_BT:APP_AUDIO_SPEAKER);
}

static void gpio_init(void){
    gpio_reset_pin(A_MUTE);
    gpio_reset_pin(CHARGE_MODE_SETUP);
    gpio_reset_pin(CHARGE_MODE_CONTROL);
    gpio_reset_pin(CHARGE_MODE_LED);
    gpio_reset_pin(PIN_RESET);
    gpio_set_direction(A_MUTE, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHARGE_MODE_SETUP, GPIO_MODE_INPUT);
    gpio_set_direction(CHARGE_MODE_CONTROL, GPIO_MODE_INPUT);
    gpio_set_direction(CHARGE_MODE_LED, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(CHARGE_MODE_SETUP,GPIO_PULLUP_ONLY);
    gpio_set_direction(PIN_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RESET,true);
}

static void i2s_init(void){
    // RX CHANNEL FOR SOURCE
    if(app_mode == APP_MODE_SOURCE){
        i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE); //I2S_NUM_AUTO
        ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_i2s_chan));
        i2s_std_config_t rx_std_cfg = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = STD_BCLK,
                .ws   = STD_WS,
                .dout = I2S_GPIO_UNUSED,
                .din  = STD_DIN,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv   = false,
                },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_i2s_chan, &rx_std_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(rx_i2s_chan));
    }
    // TX CHANNEL FOR SINK
    if(app_mode == APP_MODE_SINK){
        i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER); //I2S_NUM_AUTO
        ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_i2s_chan, NULL));
        
        i2s_std_config_t tx_std_cfg = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = STD_BCLK,
                .ws   = STD_WS,  
                .dout = STD_DIN, 
                .din  = I2S_GPIO_UNUSED,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv   = false,
                },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_i2s_chan, &tx_std_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(tx_i2s_chan));
    }
    uart_send_dparam(APP_UART_SEND_DRV_READY,app_mode);
    i2s_app_task_start_up();
}

void i2s_app_task_start_up(void){
    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    s_i2s_write_semaphore = xSemaphoreCreateBinary();
    s_ringbuf_i2s = xRingbufferCreate(RINGBUF_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF);
    xTaskCreatePinnedToCore(i2s_app_task_handler, "I2SAppTask", 1024*4, NULL, configMAX_PRIORITIES - 3, &s_i2s_app_task_handler, tskNO_AFFINITY );
}

static void i2s_app_task_handler(void *arg){
    uint8_t*        data = NULL;
    uint8_t*        in_mem = NULL;
    size_t          item_size = 0;
    const size_t    item_size_upto = 240 * 6;
    size_t          bytes_written = 0;
    uart_rx_msg_t   msg;
    samplerate_t    samplerate = SR_44100;

    if(app_mode == APP_MODE_SINK){
        for (;;) {
            if (pdTRUE == xSemaphoreTake(s_i2s_write_semaphore, portMAX_DELAY)) {
                for (;;) {
                    item_size = 0;
                    data = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size, (TickType_t)pdMS_TO_TICKS(20), item_size_upto);
                    if (item_size == 0) {
                        ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING");
                        ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                        break;
                    }
                    change_volume(data,item_size);
                    i2s_channel_write(tx_i2s_chan, data, item_size, &bytes_written, portMAX_DELAY);
                    vRingbufferReturnItem(s_ringbuf_i2s, (void *)data);
                }
            }
        }
    }
    if(app_mode == APP_MODE_SOURCE){
        data = malloc( TX_PACKET_SIZE );
	    in_mem = malloc( TX_PACKET_SIZE/2 );
        for(;;){
            if(xQueueReceive(uart_rx_queue, &msg, 0)){
                if(samplerate != msg.sr) samplerate = msg.sr;
            }
            if(samplerate == SR_44100){
                i2s_channel_read(rx_i2s_chan, data, TX_PACKET_SIZE, &item_size, portMAX_DELAY );
            } else {
                i2s_channel_read(rx_i2s_chan, in_mem, TX_PACKET_SIZE/2, &item_size, portMAX_DELAY );
                resample(item_size,in_mem,data);
            }
            if(!xRingbufferSend(s_ringbuf_i2s, (void *)data, item_size, portMAX_DELAY)){
                ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING");
            }
        }
    }
}

static void change_volume(uint8_t* data,size_t size){
    if (data == NULL || size <= 0) return;
    uint8_t vol = get_local_volume();
    int16_t tmp;
    for(uint16_t i=0; i<size;i+=2){
        tmp = data[i+1]<<8 | data[i];
        tmp = ((int32_t)(tmp*vol))>>8;
        data[i+1] = tmp>>8;
        data[i] = tmp & 0xFF;
    }
}

static void resample(size_t size,const uint8_t *in,uint8_t *out){
    for(uint16_t i=0; i<size;i+=2){
        out[2*i]   = in[i];
        out[2*i+1] = in[i+1];
        out[2*i+2] = in[i];
        out[2*i+3] = in[i+1];      
    }
}

static void bt_app_a2d_sink_data_cb(const uint8_t *data, uint32_t len){
    size_t item_size = 0;
    if (ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer is full, drop this packet!");
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        if (item_size <= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer data decreased! mode changed: RINGBUFFER_MODE_PROCESSING");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer size %d water level %d",item_size,RINGBUF_PREFETCH_WATER_LEVEL);
        return;
    }
    if (!xRingbufferSend(s_ringbuf_i2s, (void *)data, len, (TickType_t)0)) { //portMAX_DELAY)){ //
        ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING");
        ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
    }

    if (ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
            if (pdFALSE == xSemaphoreGive(s_i2s_write_semaphore)) {
                ESP_LOGE(BT_APP_CORE_TAG, "semphore give failed");
            }
        }
    }
}

static int32_t bt_app_a2d_source_data_cb(uint8_t *data, int32_t len){
    if (data == NULL || len < 0) return 0;
    size_t item_size = 0;

    uint8_t *buff = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size, (TickType_t)pdMS_TO_TICKS(20), len);
    if (item_size == 0) {
        ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING");
        ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    } else {
        memcpy( data, buff, item_size );
        vRingbufferReturnItem(s_ringbuf_i2s, buff );
    }
    return item_size; 
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
//    if(event>1)
//        ESP_LOGW(TAG_GAP, "%s event: %d a2d_state: %d mode %d", __func__, event,s_a2d_state,app_mode);
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        if (s_a2d_state == APP_AV_STATE_DISCOVERING) {
            if(app_mode == APP_MODE_SOURCE) scan_result(param);
            if(app_mode == APP_MODE_SINK){ //check_source_peer(param);
                if(check_stored(&eir_discovered,(char*)param->disc_res.bda)){
                    del_line(&eir_discovered,(char*)param->auth_cmpl.bda);
                    s_a2d_state = APP_AV_STATE_DISCOVERED;
                    esp_bt_gap_cancel_discovery();
                }
            }
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
//        ESP_LOGI(TAG_GAP, "%s param->disc_st_chg.state: %d a2d_state: %d", __func__, param->disc_st_chg.state,s_a2d_state);
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (s_a2d_state == APP_AV_STATE_DISCOVERED) {
                s_a2d_state = APP_AV_STATE_CONNECTING;
                ESP_LOGI(TAG_GAP, "Device discovery stopped.");
                ESP_LOGI(TAG_GAP, "a2dp connecting to peer: %s", get_peer_name());
                if(app_mode == APP_MODE_SOURCE) esp_a2d_source_connect(s_peer_bda);
                if(app_mode == APP_MODE_SINK) esp_a2d_sink_connect(s_peer_bda);
            } else if (s_a2d_state == APP_AV_STATE_DISCOVERING){
                if(app_mode == APP_MODE_SOURCE) send_json_eir(&eir_saved,&eir_discovered,true);
                ESP_LOGI(TAG_GAP, "Device discovery failed, continue to discover...");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
            } else
                ESP_LOGI(TAG_GAP, "Discovery stoped.");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG_GAP, "Discovery started.");
            if(app_mode == APP_MODE_SINK){
                uart_send_sparam(APP_UART_SEND_BT_NAME,"[discovery]");
                send_json_eir(&eir_saved,&eir_discovered,false);
            } 
        }
        break;
    }
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG_GAP, "authentication success: %s", param->auth_cmpl.device_name);
//            ESP_LOG_BUFFER_HEX(TAG_GAP, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            memcpy(s_peer_bda, param->auth_cmpl.bda, ADDR_SIZE);
            set_peer_name((char*)param->auth_cmpl.device_name);
            if(app_mode == APP_MODE_SINK){
                uart_send_sparam(APP_UART_SEND_BT_TITLE,get_peer_name());
                uart_send_dparam(APP_UART_SEND_BT_A2DP,1);
            }
        } else {
            ESP_LOGE(TAG_GAP, "authentication failed, status: %d", param->auth_cmpl.stat);
            if(check_stored(&eir_saved,(char*)param->auth_cmpl.bda)){
                del_line(&eir_saved,(char*)param->auth_cmpl.bda);
                save_eir();
            }
        }
        ESP_LOGI(TAG_GAP, "link key type of current link is: %d", param->auth_cmpl.lk_type);
        if ((param->auth_cmpl.lk_type == ESP_BT_LINK_KEY_AUTHED_COMB_P192) ||
            (param->auth_cmpl.lk_type == ESP_BT_LINK_KEY_AUTHED_COMB_P256)){
                uart_send_dparam(APP_UART_SEND_BT_NUM_CFM,0);
            }
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG_GAP, "Please compare the numeric value: %06"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        if(app_mode == APP_MODE_SINK) uart_send_fparam(APP_UART_SEND_BT_NUM_CFM,"%d:%06ld\n",param->cfm_req.num_val);
        break;
    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT: break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        ESP_LOGW(TAG_GAP,"REMOTE_NAME stat: %d, name: %s",param->read_rmt_name.stat,param->read_rmt_name.rmt_name);
        if(!param->read_rmt_name.stat) set_peer_name((char*)param->read_rmt_name.rmt_name);
        break;
    case ESP_BT_GAP_MODE_CHG_EVT: break;    
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG_GAP, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT stat: %d", param->acl_conn_cmpl_stat.stat);
        ESP_LOG_BUFFER_HEX(TAG_GAP, param->acl_conn_cmpl_stat.bda, ADDR_SIZE);
        memcpy(s_peer_bda, param->acl_conn_cmpl_stat.bda, ADDR_SIZE);
        esp_bt_gap_read_remote_name(s_peer_bda);
        if(param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS){
            s_a2d_state = APP_AV_STATE_CONNECTED;
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
            stop_eir_scan(&eir_saved);
        } else if(param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_HCI_PAGE_TIMEOUT){
            if(app_mode == APP_MODE_SINK) {
                if(set_eir_scan(&eir_saved,(char*)s_peer_bda)){
                    esp_a2d_sink_connect(s_peer_bda);
                    uart_send_scan(APP_UART_SEND_BT_NAME,get_peer_name());
                } else set_scan_mode();
            } else esp_a2d_source_connect(s_peer_bda);//set_scan_mode();
        }
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        audio_output(APP_AUDIO_SPEAKER);
        if((s_a2d_state == APP_AV_STATE_UNCONNECTED)||(s_a2d_state == APP_AV_STATE_CONNECTED)){
            s_a2d_state = APP_AV_STATE_DISCOVERING;
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
        }
        break;
    default: {
        ESP_LOGI(TAG_GAP, "%s default event: %d,  a2d_state: %d, ", __func__, event, s_a2d_state);
        break;
    }
    }
    return;
}

static void scan_result(esp_bt_gap_cb_param_t *param){
    uint32_t cod = 0;
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;
    uint8_t *rmt_bdname = NULL;
    uint8_t bdname_len;

    for (int i = 0; i < param->disc_res.num_prop; i++){
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD: cod = *(uint32_t *)(p->val); break;
        case ESP_BT_GAP_DEV_PROP_EIR: eir = (uint8_t *)(p->val);   break;
        default: break;
        }
    }
    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) {
        return;
    }
    if (eir){
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &bdname_len);
        if (!rmt_bdname) {
            rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &bdname_len);
        }
    }
    if (rmt_bdname && (bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN))
        bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
    if (rmt_bdname && bdname_len){
        rmt_bdname[bdname_len] = 0;
        if(add_new_eir(&eir_discovered,(char*)param->disc_res.bda, (char*)rmt_bdname, bdname_len))
            send_json_eir(&eir_saved,&eir_discovered,false);
        if(check_stored(&eir_saved,(char*)param->disc_res.bda)){
            memcpy(s_peer_bda, param->disc_res.bda, ADDR_SIZE);
            set_peer_name((char*)rmt_bdname);
            s_a2d_state = APP_AV_STATE_DISCOVERED;
            esp_bt_gap_cancel_discovery();
        }
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
}

static void bt_app_av_sm_hdlr(uint16_t event, void *param){
//    ESP_LOGW(TAG, "%s a2d_state: %d event: %d mode %d", __func__, s_a2d_state,event,app_mode);
    if(app_mode == APP_MODE_SINK)
        bt_app_av_sink_mode_hdlr(event, param);
    else if(app_mode == APP_MODE_SOURCE){
        switch (s_a2d_state) {
            case APP_AV_STATE_IDLE:
            case APP_AV_STATE_DISCOVERING:
            case APP_AV_STATE_DISCOVERED:
            case APP_AV_STATE_UNCONNECTED:
                bt_app_av_state_unconnected_hdlr(event, param);   break;
            case APP_AV_STATE_CONNECTING:
                bt_app_av_state_connecting_hdlr(event, param);    break;
            case APP_AV_STATE_CONNECTED:
                bt_app_av_state_connected_hdlr(event, param);     break;
            case APP_AV_STATE_DISCONNECTING:
                bt_app_av_state_disconnecting_hdlr(event, param); break;
            default:
                ESP_LOGE(TAG, "%s invalid state: %d", __func__, s_a2d_state);
                break;
        }
    }
}

static void bt_app_av_state_unconnected_hdlr(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case ESP_A2D_PROF_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (ESP_A2D_DEINIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
            audio_output(APP_AUDIO_SPEAKER);
        }
        break;
    }
    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT: break;
    default: {
        ESP_LOGE(TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
    }
}

static void bt_app_av_state_connecting_hdlr(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "a2dp connected");
            s_a2d_state =  APP_AV_STATE_CONNECTED;
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            s_a2d_state =  APP_AV_STATE_UNCONNECTED;
            ESP_LOGI(TAG, "a2dp disconnected");
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        ESP_LOGI(TAG, "%s, delay value: %u * 1/10 ms", __func__, a2d->a2d_report_delay_value_stat.delay_value);
        break;
    }
    default:
        ESP_LOGE(TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_app_av_state_connected_hdlr(uint16_t event, void *param){
//    ESP_LOGI(TAG, "%s event: %d s_a2d_state %d", __func__, event,s_a2d_state);
    esp_a2d_cb_param_t *a2d = NULL;
    a2d = (esp_a2d_cb_param_t *)(param);
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "a2dp disconnected");
            s_a2d_state = APP_AV_STATE_UNCONNECTED;
            audio_output(APP_AUDIO_SPEAKER);
        }
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "a2dp media ready checking ...");
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:      break;
    case ESP_A2D_AUDIO_CFG_EVT:        break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(TAG, "a2dp media ready, starting ...");
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            s_media_state = APP_AV_MEDIA_STATE_STARTING;
        }
        if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START &&
                a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(TAG, "a2dp media started");
            s_media_state = APP_AV_MEDIA_STATE_STARTED;
            audio_output(APP_AUDIO_BT);
            send_json_eir_connected((char*)s_peer_bda,get_peer_name());
            if(!check_stored(&eir_saved,(char*)s_peer_bda)){
                add_new_eir(&eir_saved,(char*)s_peer_bda, get_peer_name(), strlen(get_peer_name()));
                save_eir();
            }
        }
        if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_SUSPEND &&
                a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(TAG, "a2dp media suspend");
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
            esp_a2d_source_disconnect(s_peer_bda);
        }
        break;
    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT: {
        ESP_LOGI(TAG, "%s, delay value: %u * 1/10 ms", __func__, a2d->a2d_report_delay_value_stat.delay_value);
        break;
    }
    default: {
        ESP_LOGE(TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
    }
}

static void bt_app_av_state_disconnecting_hdlr(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "a2dp disconnected");
            s_a2d_state =  APP_AV_STATE_UNCONNECTED;
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT: {
        break;
    }
    default: {
        ESP_LOGE(TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
    }
}

static void bt_app_av_sink_mode_hdlr(uint16_t event, void *param){
    static const char *s_a2d_audio_state_str[] = {"Suspended", "Started"};
    static const char *s_a2d_conn_state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
//    ESP_LOGI(TAG, "%s event: %d", __func__, event);

    esp_a2d_cb_param_t *a2d = NULL;

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
            s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            ESP_LOGI(TAG, "A2DP disconnection reason: %d",a2d->conn_stat.disc_rsn);
            if(a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL){
                if(!is_eir_scan(&eir_saved)) del_line(&eir_saved,(char*)s_peer_bda);}
            else 
                add_new_eir(&eir_discovered,(char*)s_peer_bda,get_peer_name(),get_peer_name_len());
            uart_send_sparam(APP_UART_SEND_BT_TITLE,"");
            uart_send_sparam(APP_UART_SEND_BT_NAME,"[disconnected]");
            uart_send_dparam(APP_UART_SEND_BT_A2DP,0);
            send_json_eir(&eir_saved,&eir_discovered,false);            
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            uart_send_sparam(APP_UART_SEND_BT_TITLE,get_peer_name());
            uart_send_sparam(APP_UART_SEND_BT_NAME,"[connected]");
            uart_send_dparam(APP_UART_SEND_BT_A2DP,1);
            if(!check_stored(&eir_saved,(char*)s_peer_bda)){// && !is_eir_scan(&eir_saved)
                add_new_eir(&eir_saved,(char*)s_peer_bda, get_peer_name(), strlen(get_peer_name()));
                save_eir();
            }
            send_json_eir_connected((char*)s_peer_bda,get_peer_name());
        } 
        break;
    }
    /* when audio stream transmission state changed, this event comes */
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        ESP_LOGI(TAG, "A2DP audio state: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
        break;
    }
    /* when audio codec is configured, this event comes */
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        ESP_LOGI(TAG, "A2DP audio stream configuration, codec type: %d", a2d->audio_cfg.mcc.type);
        /* for now only SBC stream is supported */
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            int sample_rate = 16000;
            int ch_count = 2;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                sample_rate = 32000;
            } else if (oct0 & (0x01 << 5)) {
                sample_rate = 44100;
            } else if (oct0 & (0x01 << 4)) {
                sample_rate = 48000;
            }

            if (oct0 & (0x01 << 3)) {
                ch_count = 1;
            }
            ESP_LOGI(TAG, "Configure audio player: %x-%x-%x-%x",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            ESP_LOGI(TAG, "Audio player configured, sample rate: %d ch:%d", sample_rate,ch_count);
        }
        break;
    }
    /* when a2dp init or deinit completed, this event comes */
    case ESP_A2D_PROF_STATE_EVT:  break;
    /* When protocol service capabilities configured, this event comes */
    case ESP_A2D_SNK_PSC_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        ESP_LOGI(TAG, "protocol service capabilities configured: 0x%x ", a2d->a2d_psc_cfg_stat.psc_mask);
        if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT) {
            ESP_LOGI(TAG, "Peer device support delay reporting");
        } else {
            ESP_LOGI(TAG, "Peer device unsupported delay reporting");
        }
        break;
    }
    /* when set delay value completed, this event comes */
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (ESP_A2D_SET_INVALID_PARAMS == a2d->a2d_set_delay_value_stat.set_state) {
            ESP_LOGI(TAG, "Set delay report value: fail");
        } else {
            ESP_LOGI(TAG, "Set delay report value: success, delay_value: %u * 1/10 ms", a2d->a2d_set_delay_value_stat.delay_value);
        }
        break;
    }
    /* when get delay value completed, this event comes */
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        ESP_LOGI(TAG, "Get delay report value: delay_value: %u * 1/10 ms", a2d->a2d_get_delay_value_stat.delay_value);
        /* Default delay value plus delay caused by application layer */
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + 50);//APP_DELAY_VALUE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

void get_addr_to_connect(char* addr){
    char* p_end;
    s_peer_bda[0] = strtol(addr, &p_end, 16);
    for(int i=1;i<ADDR_SIZE;i++)
        s_peer_bda[i] = strtol(++p_end, &p_end, 16);
    set_peer_name(++p_end);
}

void uart_hdl_evt(uint16_t event, void *p_param){
   char*   comm_suf;
   uint8_t pt_cmd;
//   printf("***UART Receive %s\n",(char*)p_param);
    uint8_t comm = (uint8_t)strtol((char*)p_param, &comm_suf, 10);
    switch (comm) {
    case APP_UART_GET_STATUS:
        if(s_a2d_state == APP_AV_STATE_CONNECTED)
            send_json_eir_connected((char*)s_peer_bda, get_peer_name());
        else
            send_json_eir(&eir_saved,&eir_discovered,false);
        break;
    case APP_UART_START_DISCOVERY:
        if((s_a2d_state == APP_AV_STATE_IDLE)){
            s_a2d_state = APP_AV_STATE_DISCOVERING;
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
        }
        break;
    case APP_UART_STOP_DISCOVERY:
        if(s_a2d_state == APP_AV_STATE_DISCOVERING){
            s_a2d_state = APP_AV_STATE_IDLE;
            esp_bt_gap_cancel_discovery();
        }
        break;
    case APP_UART_CONNECT:
        get_addr_to_connect(comm_suf+1);
        s_a2d_state = APP_AV_STATE_DISCOVERED;
        esp_bt_gap_cancel_discovery();
        break;
    case APP_UART_DISCONNECT:
        if(app_mode == APP_MODE_SOURCE){
            if(s_media_state == APP_AV_MEDIA_STATE_STARTED){
                s_media_state = APP_AV_MEDIA_STATE_STOPPING;
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
            }
            del_line(&eir_saved,(char*)s_peer_bda);
        } else if(app_mode == APP_MODE_SINK){
            esp_a2d_sink_disconnect(s_peer_bda);
        }
        break;
    case APP_UART_FORGET_SINK:
        if(app_mode == APP_MODE_SOURCE){
            if(s_media_state == APP_AV_MEDIA_STATE_STARTED){
                s_media_state = APP_AV_MEDIA_STATE_STOPPING;
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
            }
        } else if(app_mode == APP_MODE_SINK){
            esp_a2d_sink_disconnect(s_peer_bda);
        }
        esp_bt_gap_remove_bond_device(s_peer_bda);
        del_line(&eir_saved,(char*)s_peer_bda);
        save_eir();
        break;
    case APP_UART_I2S_SAMPLE_RATE:
        uart_rx_msg_t msg;
        if(!strncmp(comm_suf+1,"44100",5))  msg.sr = SR_44100;
        else                                msg.sr = SR_22050;
        xQueueSend(uart_rx_queue, &msg, 0);//portMAX_DELAY);//
        break;
    case APP_UART_REINIT_CMD:
        printf("************* REINIT_CMD **************\n");
        app_mode = atoi(comm_suf+1);
        if(app_mode == APP_MODE_SOURCE){
            ESP_ERROR_CHECK(i2s_channel_disable(tx_i2s_chan));
            ESP_ERROR_CHECK(i2s_del_channel(tx_i2s_chan));
            gpio_reset_pin(STD_BCLK);
            gpio_reset_pin(STD_WS);
            gpio_reset_pin(STD_DIN);
        }
        esp_restart();
        break;
    case APP_UART_GET_INIT:
        app_mode = atoi(comm_suf+1);
        size_t n_len = strchr(comm_suf, '\n') - comm_suf-3;
        if(n_len > 1){
            dev_alias = malloc(n_len);
            memcpy(dev_alias,comm_suf+3,n_len);
            dev_alias[n_len] = 0;
        }
        break;
    case APP_UART_VOLUME:
        if(app_mode == APP_MODE_SINK) volume_set_by_local_host_sink_mode(atoi(comm_suf+1));
        if(app_mode == APP_MODE_SOURCE) volume_set_by_local_host_source_mode(atoi(comm_suf+1));
        break;
    case APP_UART_PT_CMD:
        pt_cmd = atoi(comm_suf+1);
        esp_avrc_ct_send_passthrough_cmd(APP_RC_CT_TL_RN_PT_CMD, pt_cmd, ESP_AVRC_PT_CMD_STATE_PRESSED);
        break;
    case APP_UART_ONLY_FORGET:
        get_addr_to_connect(comm_suf+1);
        esp_bt_gap_remove_bond_device(s_peer_bda);
        del_line(&eir_saved,(char*)s_peer_bda);
        save_eir();        
        break;
    default:
        ESP_LOGE(TAG, "%s unhandled command: %d", __func__, comm);
        break;
    }
}

void get_saved_eir(){
    nvs_handle handle;
    size_t blen;
    destroy_eir(&eir_saved);
    destroy_eir(&eir_discovered);
    const char* key = (app_mode == APP_MODE_SINK)?"sink":"eir";
    if(ESP_OK == nvs_open(dev_name, NVS_READONLY, &handle)){
        if(ESP_OK == nvs_get_blob(handle, key, NULL, &blen)){
            uint8_t *buff = (uint8_t*)malloc(blen);
            nvs_get_blob(handle, key, buff, &blen);
            deserialize(&eir_saved, &buff);
            eir_print(&eir_saved);
            free(buff);
        }
        nvs_close(handle);
    }
}

static void save_eir(){
    eir_print(&eir_saved);
    nvs_handle handle;
    size_t blen;
    const char* key = (app_mode == APP_MODE_SINK)?"sink":"eir";
    if(ESP_OK == nvs_open(dev_name, NVS_READWRITE, &handle)){
        if(eir_saved.arr == NULL){
            nvs_erase_key(handle, key);
            nvs_commit(handle);
        } else {
            char*  buff = serialize(&eir_saved, &blen);
            if(ESP_OK == nvs_set_blob(handle, key, buff, blen)) nvs_commit(handle);
            free(buff);
        }
        nvs_close(handle);
    }
}

static char* set_dev_name(){
    return (dev_alias == NULL)?dev_name:dev_alias;
}

void app_main(void){

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_init();
    audio_output(APP_AUDIO_SPEAKER);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    bt_app_task_start_up();

    uart_rx_queue = xQueueCreate( 5, sizeof(uart_rx_msg_t) );
    uart_init();
    adc_init();

    ESP_LOGI(TAG,"Wait for main controller give mode");
    while (app_mode == APP_MODE_IDLE){
        uart_send_sparam(APP_UART_SEND_ASK_MODE,"");
//        printf(".");
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    printf("Init mode: %d\n",app_mode);

    i2s_init();
    get_saved_eir(); 

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    esp_bt_config_file_path_update((app_mode == APP_MODE_SOURCE)?source_cfg:sink_cfg);

    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#if 1
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
#endif
/*
    int dev_num = esp_bt_gap_get_bond_device_num();
    printf("paired device num %d\n", dev_num);
    esp_bd_addr_t *dev_list = (esp_bd_addr_t *)malloc(sizeof(esp_bd_addr_t) * dev_num);
    esp_bt_gap_get_bond_device_list(&dev_num, dev_list);
	for (int i = 0; i < dev_num; i++) {
    	printf("\t| %02x:%02x:%02x:%02x:%02x:%02x |\n", ESP_BD_ADDR_HEX(dev_list[i]));
	}
    free(dev_list);   
*/
    send_json_eir(&eir_saved,&eir_discovered,false);
    audio_output(APP_AUDIO_SPEAKER);

    esp_bt_gap_set_device_name(set_dev_name());
    if(app_mode == APP_MODE_SOURCE){
        source_init();
        set_scan_mode();
    }else{
        sink_init();
        uart_send_sparam(APP_UART_SEND_BT_TITLE,"");
        if(set_eir_scan(&eir_saved,(char*)s_peer_bda)){
            uart_send_scan(APP_UART_SEND_BT_NAME,get_peer_name());
            esp_a2d_sink_connect(s_peer_bda);
        } else {
            set_scan_mode();
        }
    }
}

static void set_scan_mode(){
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    ESP_LOGI(TAG, "Starting device discovery...");
    s_a2d_state = APP_AV_STATE_DISCOVERING; 
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
}

static void source_init(){
    esp_bt_cod_t cod;
    cod.major   = ESP_BT_COD_MAJOR_DEV_AV; 
    cod.minor   = 0;
    cod.service = ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO;
    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

    esp_bt_gap_register_callback(bt_app_gap_cb);
    esp_bt_gap_get_cod(&cod);
    printf("cod.major %d,cod.minor %d,cod.service %d\n",cod.major,cod.minor,cod.service);

    avrc_ct_init();
    avrc_tg_init();
    esp_a2d_source_init();
    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_source_register_data_callback(bt_app_a2d_source_data_cb);
}

static void sink_init(){
    esp_bt_cod_t cod;
    cod.major   = ESP_BT_COD_MAJOR_DEV_AV; 
    cod.minor   = 0;
    cod.service = ESP_BT_COD_SRVC_CAPTURING | ESP_BT_COD_SRVC_AUDIO;
    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

    esp_bt_gap_register_callback(bt_app_gap_cb);
    esp_bt_gap_get_cod(&cod);
    printf("cod.major %d,cod.minor %d,cod.service %d\n",cod.major,cod.minor,cod.service);

    avrc_ct_init();
    avrc_tg_init();
    esp_a2d_sink_init();
    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_sink_register_data_callback(bt_app_a2d_sink_data_cb);
    esp_a2d_sink_get_delay_value();
}