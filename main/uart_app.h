#ifndef __UART_APP_H__
#define __UART_APP_H__

enum {
    APP_UART_IDLE,
    APP_UART_GET_STATUS,
    APP_UART_START_DISCOVERY,
    APP_UART_STOP_DISCOVERY,
    APP_UART_CONNECT,
    APP_UART_DISCONNECT,
    APP_UART_FORGET_SINK,
    APP_UART_PIN_SINK,
    APP_UART_I2S_SAMPLE_RATE,
    APP_UART_GET_INIT,
    APP_UART_REINIT_CMD,
    APP_UART_VOLUME,
    APP_UART_PT_CMD,
    APP_UART_ONLY_FORGET,
};

enum {
    APP_UART_SEND_IDLE,
    APP_UART_SEND_EIR_DATA,
    APP_UART_SEND_BT_STATUS,
    APP_UART_SEND_DRV_READY,
    APP_UART_SEND_BATTERY,
    APP_UART_SEND_ASK_MODE,
    APP_UART_SEND_BT_TITLE,
    APP_UART_SEND_BT_NUM_CFM,
    APP_UART_SEND_BT_NAME,
    APP_UART_SEND_BT_A2DP,
    APP_UART_SEND_VOLUME,
    APP_UART_SEND_PLAYBACK,
    APP_UART_SEND_BT_VOLUM,
    APP_UART_SEND_PT_CMD,
    APP_UART_SEND_ART_IMG,
    APP_UART_SEND_ART_IMG_DATA,
};

void uart_init(void);
void uart_deinit(void);
int uart_send_data(const char* data);
int uart_send_data_len(const char* data,int len);
void uart_send_sparam(uint8_t param,const char* data);
void uart_send_fparam(uint8_t param,const char *format,uint32_t data);
void uart_send_dparam(uint8_t param,uint16_t data);
void uart_send_scan(uint8_t param,const char* data);

#endif /* __UART_APP_H__ */