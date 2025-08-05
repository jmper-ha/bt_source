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
    APP_UART_I2S_PARAM,
};

void uart_init(void);
int uart_send_data(const char* data);

#endif /* __UART_APP_H__ */