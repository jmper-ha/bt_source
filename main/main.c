
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
#include "main.h"
#include "uart_app.h"
#include "utils.h"
#include "adc_app.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "settings.h"

#define I2S_RINGBUF_SIZE 512

static TaskHandle_t     s_i2s_app_task_handler = NULL;
static RingbufHandle_t  i2s_buf = NULL;
static QueueHandle_t    uart_rx_queue;
i2s_chan_handle_t       rx_chan;

arr_t   eir_saved;
arr_t   eir_discovered;

static void audio_output(audio_m_t state)
{
    gpio_set_level(A_MUTE, state);
    uart_send_data(state?"2:\n":"3:\n");
}

static void gpio_init(void){
    gpio_reset_pin(A_MUTE);
    gpio_reset_pin(CHARGE_MODE_SETUP);
    gpio_reset_pin(CHARGE_MODE_CONTROL);
    gpio_reset_pin(CHARGE_MODE_LED);
    gpio_set_direction(A_MUTE, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHARGE_MODE_SETUP, GPIO_MODE_INPUT);
    gpio_set_direction(CHARGE_MODE_CONTROL, GPIO_MODE_INPUT);
    gpio_set_direction(CHARGE_MODE_LED, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(CHARGE_MODE_SETUP,GPIO_PULLUP_ONLY);
}

void i2s_app_task_start_up(void)
{
    i2s_buf = xRingbufferCreateNoSplit( I2S_RINGBUF_SIZE, 16);
    xTaskCreatePinnedToCore( i2s_app_task_handler, "I2SAppTask", 4096, NULL, configMAX_PRIORITIES - 3, &s_i2s_app_task_handler, tskNO_AFFINITY );
}

static void i2s_init(void)
{
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
//        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(22050),
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
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    i2s_app_task_start_up();
}

static void i2s_app_task_handler(void *arg)
{
	size_t          bytes_read = 0;
    uart_rx_msg_t   msg;
    samplerate_t    samplerate = SR_44100;

	unsigned char* in_mem = malloc( I2S_RINGBUF_SIZE/2 );
    unsigned char* out_mem = malloc( I2S_RINGBUF_SIZE );
	
	if( !in_mem){
		printf(" i2s0 task malloc err\n ");
		for(;;)vTaskDelay( 1000 / portTICK_PERIOD_MS );
	}
	for(;;){
        if(xQueueReceive(uart_rx_queue, &msg, 0)){
            if(samplerate != msg.sr) samplerate = msg.sr;
        }
        if(samplerate == SR_44100){
    		if( i2s_channel_read( rx_chan, out_mem, I2S_RINGBUF_SIZE, &bytes_read, portMAX_DELAY ) )printf("i2s0 read err\n");
    		if( xRingbufferSend( i2s_buf, out_mem, I2S_RINGBUF_SIZE, portMAX_DELAY ) == pdFALSE )printf("i2s0 ringBufferSend err\n");
        } else {
            if( i2s_channel_read( rx_chan, in_mem, I2S_RINGBUF_SIZE/2, &bytes_read, portMAX_DELAY ) )printf("i2s0 read err\n");
            for(uint16_t i=0; i<bytes_read;i+=2){
    //            uint16_t tmp = ((in_mem[i]<<8|in_mem[i+1])+(in_mem[i+2]<<8|in_mem[i+3]))/2;
                out_mem[2*i] = in_mem[i];
                out_mem[2*i+1] = in_mem[i+1];
                out_mem[2*i+2] = in_mem[i];//0;//tmp>>8;
                out_mem[2*i+3] = in_mem[i+1];//0;//0x0f&tmp;             
            }
        if( xRingbufferSend( i2s_buf, out_mem, I2S_RINGBUF_SIZE, portMAX_DELAY ) == pdFALSE )printf("i2s0 ringBufferSend err\n");
        }
    }
}

static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len)
{
    if (len < 0 || data == NULL)return 0;
 
    size_t itemSize = 0;
    unsigned char* ringBufferItem = xRingbufferReceive( i2s_buf, &itemSize, 0 ); //portMAX_DELAY
   
    if( ringBufferItem == NULL ){
   	    return 0;
    }
    if( itemSize != len ){
        printf( "%s itemSize != len (%d != %d)\n", __func__, itemSize, (int)len );
   	    vRingbufferReturnItem( i2s_buf, ringBufferItem );
   	    return 0;
    }
    memcpy( data, ringBufferItem, itemSize );
    vRingbufferReturnItem( i2s_buf, ringBufferItem );
    return itemSize; 
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
//    if(event>0)
//        ESP_LOGI(TAG, "%s event: %d a2d_state: %d", __func__, event,s_a2d_state);
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        if (s_a2d_state == APP_AV_STATE_DISCOVERING) 
            scan_result(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
//        ESP_LOGI(TAG, "%s param->disc_st_chg.state: %d a2d_state: %d", __func__, param->disc_st_chg.state,s_a2d_state);
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (s_a2d_state == APP_AV_STATE_DISCOVERED) {
                s_a2d_state = APP_AV_STATE_CONNECTING;
                ESP_LOGI(TAG, "Device discovery stopped.");
                ESP_LOGI(TAG, "a2dp connecting to peer: %s", s_peer_bdname);
                esp_a2d_source_connect(s_peer_bda);
            } else if (s_a2d_state == APP_AV_STATE_DISCOVERING){
                send_json_eir(&eir_discovered,true);
                ESP_LOGI(TAG, "Device discovery failed, continue to discover...");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
            } else
                ESP_LOGI(TAG, "Discovery stoped.");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "Please compare the numeric value: %06"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:     
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        break;    
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT stat: %d", param->acl_conn_cmpl_stat.stat);
        if(param->acl_conn_cmpl_stat.stat) {
            esp_a2d_source_connect(s_peer_bda);
            s_a2d_state = APP_AV_STATE_CONNECTING;
        } else {
            s_a2d_state = APP_AV_STATE_CONNECTED;
            s_media_state = APP_AV_MEDIA_STATE_IDLE;
        }
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        s_a2d_state = APP_AV_STATE_DISCOVERING;
        audio_output(APP_AUDIO_SPEAKER);
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
        break;
    default: {
        ESP_LOGI(TAG, "%s default event: 0x%x,  a2d_state: %d, ", __func__, event, s_a2d_state);
        break;
    }
    }
    return;
}

static void scan_result(esp_bt_gap_cb_param_t *param)
{
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
            send_json_eir(&eir_discovered,false);
        if(check_stored(&eir_saved,(char*)param->disc_res.bda)){
            memcpy(s_peer_bda, param->disc_res.bda, ADDR_SIZE);
            strcpy((char *)s_peer_bdname, (char*)rmt_bdname);
            s_peer_bdname[bdname_len] = 0;
            s_a2d_state = APP_AV_STATE_DISCOVERED;
            esp_bt_gap_cancel_discovery();
        }
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
}

static void bt_app_av_sm_hdlr(uint16_t event, void *param)
{
    ESP_LOGI(TAG, "%s a2d_state: %d", __func__, s_a2d_state);
    switch (s_a2d_state) {
    case APP_AV_STATE_DISCOVERING:
    case APP_AV_STATE_DISCOVERED:
        break;
    case APP_AV_STATE_UNCONNECTED:
        bt_app_av_state_unconnected_hdlr(event, param);
        break;
    case APP_AV_STATE_CONNECTING:
        bt_app_av_state_connecting_hdlr(event, param);
        break;
    case APP_AV_STATE_CONNECTED:
        bt_app_av_state_connected_hdlr(event, param);
        break;
    case APP_AV_STATE_DISCONNECTING:
        bt_app_av_state_disconnecting_hdlr(event, param);
        break;
    default:
        ESP_LOGE(TAG, "%s invalid state: %d", __func__, s_a2d_state);
        break;
    }
}

static void bt_app_av_state_unconnected_hdlr(uint16_t event, void *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT:
        break;
    default: {
        ESP_LOGE(TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
    }
}
static void bt_app_av_state_connecting_hdlr(uint16_t event, void *param)
{
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

static void bt_app_av_state_connected_hdlr(uint16_t event, void *param)
{
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
    case ESP_A2D_AUDIO_STATE_EVT:
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        break;
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
            send_json_eir_connected((char*)s_peer_bda,(char*)s_peer_bdname);
            if(!check_stored(&eir_saved,(char*)s_peer_bda)){
                add_new_eir(&eir_saved,(char*)s_peer_bda, (char*)s_peer_bdname, strlen((char*)s_peer_bdname));
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

static void bt_app_av_state_disconnecting_hdlr(uint16_t event, void *param)
{
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

void get_addr_to_connect(char* addr)
{
    char* p_end;
    s_peer_bda[0] = strtol(addr, &p_end, 16);
    for(int i=1;i<6;i++)
        s_peer_bda[i] = strtol(++p_end, &p_end, 16);
    strcpy((char*)s_peer_bdname,++p_end);
}

void uart_hdl_evt(uint16_t event, void *p_param)
{
   char*   comm_suf;
    uint8_t comm = (uint8_t)strtol((char*)p_param, &comm_suf, 10);
   switch (comm) {
    case APP_UART_GET_STATUS:
        if(s_a2d_state == APP_AV_STATE_CONNECTED)
            send_json_eir_connected((char*)s_peer_bda,(char*)s_peer_bdname);
        else
            send_json_eir(&eir_discovered,false);
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
        if(s_media_state == APP_AV_MEDIA_STATE_STARTED){
            s_media_state = APP_AV_MEDIA_STATE_STOPPING;
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
            del_line(&eir_saved,(char*)s_peer_bda);
        }
        break;
    case APP_UART_FORGET_SINK:
            if(s_media_state == APP_AV_MEDIA_STATE_STARTED){
            s_media_state = APP_AV_MEDIA_STATE_STOPPING;
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
            del_line(&eir_saved,(char*)s_peer_bda);
            save_eir();
        }
        break;
    case APP_UART_I2S_PARAM:
        uart_rx_msg_t msg;
        if(!strncmp(comm_suf+1,"44100",5))  msg.sr = SR_44100;
        else                                msg.sr = SR_22050;
        xQueueSend(uart_rx_queue, &msg, 0);//portMAX_DELAY);//
        break;
    default:
        ESP_LOGE(TAG, "%s unhandled command: %d", __func__, comm);
        break;
    }
}

void get_saved_eir(){
    nvs_handle handle;
    size_t blen;

    if(ESP_OK == nvs_open(dev_name, NVS_READONLY, &handle)){
        if(ESP_OK == nvs_get_blob(handle, "eir", NULL, &blen)){
            uint8_t *buff = (uint8_t*)malloc(blen);
            nvs_get_blob(handle, "eir", buff, &blen);
            deserialize(&eir_saved, &buff);
            eir_print(&eir_saved);
            free(buff);
        }
        nvs_close(handle);
    }
}

static void save_eir(){
    nvs_handle handle;
    size_t blen;
    if(ESP_OK == nvs_open(dev_name, NVS_READWRITE, &handle)){
        if(eir_saved.arr == NULL){
            nvs_erase_key(handle, "eir");
            nvs_commit(handle);
        } else {
            char*  buff = serialize(&eir_saved, &blen);
            if(ESP_OK == nvs_set_blob(handle, "eir", buff, blen))
                nvs_commit(handle);
            
            free(buff);
        }
        nvs_close(handle);
    }
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
    uart_rx_queue = xQueueCreate( 5, sizeof(uart_rx_msg_t) );
    uart_init();
    get_saved_eir();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    
    bt_app_task_start_up();
    i2s_init();
    

    esp_bt_gap_set_device_name(dev_name);
    esp_bt_gap_register_callback(bt_app_gap_cb);

/*    no need avrc here
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));
*/
    esp_a2d_source_init();
    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_source_register_data_callback(bt_app_a2d_data_cb);

    adc_init();

    ESP_LOGI(TAG, "Starting device discovery...");
    s_a2d_state = APP_AV_STATE_DISCOVERING;
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, disc_time, 0);
}
