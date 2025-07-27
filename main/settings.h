#ifndef __SETTINGS_H__
#define __SETTINGS_H__

// AUDIO
#define STD_BCLK            GPIO_NUM_26
#define STD_WS              GPIO_NUM_25
#define STD_DIN             GPIO_NUM_27
#define A_MUTE              GPIO_NUM_23

//UART
#define TXD_PIN             GPIO_NUM_4
#define RXD_PIN             GPIO_NUM_5
#define BAUD_RATE           115200

//ADC
#define R1                  47
#define R2                  120
#define VMIN                2750
#define VMAX                4150

#define ADC_UNIT            ADC_UNIT_1
#define ADC_CHANNEL         ADC_CHANNEL_4 // GPIO_32

#define CHARGE_MODE_SETUP   GPIO_NUM_12
#define CHARGE_MODE_CONTROL GPIO_NUM_13
#define CHARGE_MODE_LED     GPIO_NUM_14 

#endif /* __SETTINGS_H__ */