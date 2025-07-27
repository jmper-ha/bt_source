#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h" // For adc_cali_check_scheme and scheme creation functions
#include "hal/adc_types.h"
#include "driver/gpio.h"
#include "uart_app.h"
#include "adc_app.h"
#include "settings.h"

#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_BITWIDTH    ADC_BITWIDTH_DEFAULT // Or ADC_BITWIDTH_12 etc.

// Oversampling Configuration
#define NO_OF_SAMPLES   128  // Number of samples to average
#define PREV_COUNT      16

static const char *TAG = "ADC_CALI";

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t cali_handle = NULL;
static bool do_calibration = false;

static void adc_task(void *arg);

const float k_volt = 0.15;
//const float k_chrg = 0.5;
float filtered_mv = 0;

uint16_t prev_voltage = 0;
uint8_t prev_voltage_idx = PREV_COUNT;
int32_t diff=0;
uint32_t v_sum,v_sum_prew;

// Function to initialize ADC unit and channel
void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));
    ESP_LOGI(TAG, "ADC1 Channel %d configured", ADC_CHANNEL);

    xTaskCreate(adc_task, "adc_task", 1024*2, NULL, tskIDLE_PRIORITY, NULL);
}

 // Function to attempt ADC calibration
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_bitwidth_t bitwidth, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    bool calibrated = false;

    ESP_LOGI(TAG, "Attempting calibration for ADC_UNIT_%d, Atten: %d, Bitwidth: %d", unit, atten, bitwidth);

    adc_cali_line_fitting_config_t cali_config_lf = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = bitwidth,
    };
    if (ESP_OK == adc_cali_create_scheme_line_fitting(&cali_config_lf, &handle)) calibrated = true;
    *out_handle = handle;
    if (calibrated) ESP_LOGI(TAG, "Calibration successful. Handle: %p", handle);
    else ESP_LOGW(TAG, "ADC Calibration failed for all known schemes.");

    return calibrated;
}

uint8_t percent(int v){
    if(v<=VMIN) return 0;
    if(v>=VMAX) return 100;
    return (v-VMIN)/((VMAX-VMIN)/100);
}

bool check_charge(){
    bool charge;
    if(gpio_get_level(CHARGE_MODE_SETUP)) charge = diff>0?true:false;   //SOFT MODE
    else charge = gpio_get_level(CHARGE_MODE_CONTROL);                  //HARDWARE MODE
    gpio_set_level(CHARGE_MODE_LED, charge);
   return charge;
}



void send_data(int volt){

    char *batt = (char*)malloc(15);
    memset(batt,0,15);
    uint16_t bat_voltage_mv = volt * (R1+R2)/R2;

//    if(bat_voltage_mv>4000) bat_voltage_mv-=10;

    if(!filtered_mv){ 
        filtered_mv = bat_voltage_mv;
        v_sum = 0;
        v_sum_prew = 0;
    }
    filtered_mv = filtered_mv*(1-k_volt) + bat_voltage_mv*k_volt;
    uint16_t bat_filtered_mv = (uint16_t)filtered_mv;

    if(prev_voltage_idx == PREV_COUNT) v_sum = 0;
    v_sum += bat_filtered_mv;
    if(!prev_voltage_idx--){
//        printf("-------------------------------------------------------------------------------\n");
        if(!v_sum_prew) v_sum_prew = v_sum;
        diff = v_sum-v_sum_prew;
        v_sum_prew = v_sum;
//        v_sum = 0;
        prev_voltage_idx = PREV_COUNT;

    } 

//    printf("raw: %d filter: %d volt: %3.2f filter: %3.2f ",bat_voltage_mv,bat_filtered_mv,(float)bat_voltage_mv/1000,filtered_mv/1000);
//    printf("persent: %d filter: %d, diff: %ld charge %d\n",percent(bat_voltage_mv),percent((uint16_t)filtered_mv),diff,check_charge());
    uint16_t prs = (check_charge()<<8)|percent(bat_filtered_mv);    
    sprintf(batt,"4:%d:%d",bat_filtered_mv,prs);
    uart_send_data(batt);
    free(batt);
}

void adc_task(void *arg)
{
    do_calibration = adc_calibration_init(ADC_UNIT, ADC_ATTEN, ADC_BITWIDTH, &cali_handle);

    int adc_raw_reading;
    int voltage_mv;
    
    uint32_t adc_reading_sum = 0;

    while (1) {
        adc_reading_sum = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw_reading) == ESP_OK) {
                adc_reading_sum += adc_raw_reading;
            }
        }
        adc_raw_reading = adc_reading_sum / NO_OF_SAMPLES; // Averaged raw reading
        if (do_calibration && cali_handle) {
            if(ESP_OK == adc_cali_raw_to_voltage(cali_handle, adc_raw_reading, &voltage_mv))
                send_data(voltage_mv);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
