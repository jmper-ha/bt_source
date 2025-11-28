#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "uart_app.h"
#include "bt_app_core.h"
#include "settings.h"

typedef  struct {
    int     data_len;
    char*   data;
} uart_tx_msg_t;

extern void uart_hdl_evt(uint16_t event, void *p_param);
static const int RX_BUF_SIZE = 256;

QueueHandle_t uart_tx_queue;
static void rx_task(void *arg);
static void tx_task(void *arg);

void uart_init(void){
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_tx_queue = xQueueCreate( 16, sizeof(uart_tx_msg_t) );

    xTaskCreate(rx_task, "uart_rx_task", 1024*3, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, tskIDLE_PRIORITY, NULL);
}

void uart_deinit(void){
    uart_driver_delete(UART_NUM_1);
}

int uart_send_data(const char* data){
    uart_send_data_len(data,strlen(data)+1);
    return 0;
}

int uart_send_data_len(const char* data,int len){
    if(uart_tx_queue==NULL) return 1;
    uart_tx_msg_t msg;
    msg.data_len = len;
    if ((msg.data = malloc(msg.data_len)) != NULL){
        memcpy(msg.data, data, msg.data_len);
        xQueueSend(uart_tx_queue, &msg, 0);
    }
    return 0;
}

void uart_send_sparam(uint8_t param,const char* data){
    char *buff= (char*)malloc(255);
    sprintf(buff,"%d:%s\n",param,data);
    uart_send_data(buff);
    free(buff);
}

void uart_send_dparam(uint8_t param,uint16_t data){
    char *buff= (char*)malloc(255);
    sprintf(buff,"%d:%d",param,data);
    uart_send_data(buff);
    free(buff);
}

void uart_send_fparam(uint8_t param,const char *format,uint32_t data){
    char *buff= (char*)malloc(255);
    sprintf(buff,format,param,data);
    uart_send_data(buff);
    free(buff);
}

void uart_send_scan(uint8_t param,const char* data){
    char *buff= (char*)malloc(255);
    sprintf(buff,"%d:[quick scan] - %s\n",param,data);
    uart_send_data(buff);
    free(buff);
}

static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 200 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            bt_app_work_dispatch(uart_hdl_evt, 244, data, rxBytes+1, NULL);
        }
    }
    free(data);
}

static void tx_task(void *arg){
    uart_tx_msg_t msg;
    while (1){
        if(xQueueReceive(uart_tx_queue, &msg, 0)){
            uart_write_bytes(UART_NUM_1, msg.data, msg.data_len);
//            if(msg.data_len<254) printf("***UART send %d bytes %s\n",msg.data_len,msg.data);
//            else printf("***UART send %d bytes\n",msg.data_len);
            free(msg.data);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}