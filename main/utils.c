
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "utils.h"
#include "uart_app.h"

size_t get_size(arr_t *ss);

static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static char *audio_state_str[] = {"[stop]","[play]","[pause]"};

static uint8_t s_volume = 0;                 /* local volume value */
static TimerHandle_t handle_tmr;
static _lock_t s_volume_lock;

void set_local_volume(uint8_t vol){
    _lock_acquire(&s_volume_lock);
    s_volume = vol;
    _lock_release(&s_volume_lock);
}

uint8_t get_local_volume(){
    return s_volume;
}

static void send_uart_volume(TimerHandle_t tmr){
    uart_send_dparam(APP_UART_SEND_VOLUME,get_local_volume());
}

void timer_init(){
    handle_tmr = xTimerCreate("timer", pdMS_TO_TICKS( 200 ),
                pdFALSE, 0, send_uart_volume);
}

void timer_start(){
    xTimerReset(handle_tmr, 10);
}



bool set_eir_scan(arr_t *ss, char* adr){
//    printf("---> %s idx %d current_idx %d\n",__func__,ss->idx,ss->current_idx);
    if((ss->idx)&&(ss->current_idx<ss->idx)){
        memcpy(adr, ss->arr[ss->current_idx]->addr, ADDR_SIZE);
        set_peer_name(ss->arr[ss->current_idx++]->name);
        return true;
    } else {
        ss->current_idx = 255;
        memset(adr,0,ADDR_SIZE);
        set_peer_name("");
        return false;
    }
}

void stop_eir_scan(arr_t *ss){
    if(ss->idx!=ss->scan_idx) ss->scan_idx = 0;
}

bool is_eir_scan(arr_t *ss){
    bool rez = (ss->idx)&&(0<ss->scan_idx--);
    printf("---> %s rez %d idx %d scan_idx %d\n",__func__,rez,ss->idx,ss->scan_idx+1);
    return rez;
}

char* get_audio_state(uint8_t state){
    return audio_state_str[state];
}

char* get_peer_name(){
    return (char *)s_peer_bdname;
}

uint8_t get_peer_name_len(){
    return strlen((char *)s_peer_bdname);
}

void set_peer_name(char* name){
    strcpy((char *)s_peer_bdname,name);
    s_peer_bdname[strlen(name)] = 0;
}

char* get_name(arr_t *ss, const char* adr){
    uint8_t i;
    for(i = 0; i<ss->idx;i++)
        if(!memcmp((char*)ss->arr[i]->addr, adr,ADDR_SIZE)) break;
    return (i==ss->idx)?NULL:ss->arr[i]->name;
}

bool add_new_eir(arr_t *ss, const char* adr, const char* nam, uint8_t nam_len){
    bool unique = true;
//    printf("-----> here add line\n");
//    printf("new: %02x:%02x:%02x:%02x:%02x:%02x\n",adr[0],adr[1],adr[2],adr[3],adr[4],adr[5]);
    for(uint8_t i = 0; i<ss->idx;i++){
        if(!memcmp((char*)ss->arr[i]->addr, adr,ADDR_SIZE)){
            unique = false;
            break;
        }
    }    
    if(unique){
        ss->arr = (eir_t**)realloc(ss->arr, (ss->idx + 1) * sizeof(eir_t*));
        ss->arr[ss->idx] = (eir_t*)malloc(sizeof(eir_t));   
        memcpy((char*)ss->arr[ss->idx]->addr, adr,ADDR_SIZE);
        ss->arr[ss->idx]->name = (char*)malloc(nam_len+1);
        strncpy((char*)ss->arr[ss->idx]->name, nam, nam_len);
        ss->arr[ss->idx++]->name[nam_len] = 0;
    }
    return unique;
}

void del_line(arr_t *ss, const char* adr){
//    printf("-----> here delete line\n");
//    printf("del: %02x:%02x:%02x:%02x:%02x:%02x\n",adr[0],adr[1],adr[2],adr[3],adr[4],adr[5]);
    bool found = false;
    for(uint8_t i = 0; i<ss->idx;i++){
        if(found){
            ss->arr[i-1] = ss->arr[i];
        } else {
            if(!memcmp(ss->arr[i]->addr, adr,ADDR_SIZE)){
                found = true;
                free(ss->arr[i]->name);
                free(ss->arr[i]);
            }
        }
    }
    if(found){
        if(--ss->idx)
            ss->arr = (eir_t**)realloc(ss->arr, ss->idx * sizeof(eir_t*));
        else {
           free(ss->arr);
           ss->arr = NULL;
        }
    }
}

void destroy_eir(arr_t *ss){
    if(ss->arr == NULL) return;
//    printf("-----> destroy_eir\n");
    for(int8_t i = ss->idx-1; i>=0; i--){
        free(ss->arr[i]->name);
        free(ss->arr[i]);
    }
    free(ss->arr); 
    ss->arr = NULL;
    ss->idx = 0;
    ss->current_idx = 0;
    ss->scan_idx = 0;
}
#if 1
void send_json_eir(arr_t *svd, arr_t *dsc, bool clean){
    char *eir_line = (char*)malloc(100);
    char *eir_json = (char*)malloc(255 + (svd->idx + dsc->idx)*64);
    strcpy(eir_json, "1:{\"eirs\":[");
    for(uint8_t i = 0; i<dsc->idx;i++){
        sprintf(eir_line,"{\"addr\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"name\":\"%s\",\"stat\":1},",
            dsc->arr[i]->addr[0],dsc->arr[i]->addr[1],dsc->arr[i]->addr[2],dsc->arr[i]->addr[3],
            dsc->arr[i]->addr[4],dsc->arr[i]->addr[5],dsc->arr[i]->name);
        strcat(eir_json, eir_line);
    }
    for(uint8_t i = 0; i<svd->idx;i++){
        if(!check_stored(dsc,(char*)svd->arr[i]->addr)){
            sprintf(eir_line,"{\"addr\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"name\":\"%s\",\"stat\":2},",
                svd->arr[i]->addr[0],svd->arr[i]->addr[1],svd->arr[i]->addr[2],svd->arr[i]->addr[3],
                svd->arr[i]->addr[4],svd->arr[i]->addr[5],svd->arr[i]->name);
            strcat(eir_json, eir_line);
        }
    }
    uint8_t str_len = strlen(eir_json);
    if(str_len > 12) eir_json[str_len-1] = 0;
    strcat(eir_json, "]}\n");
    uart_send_data(eir_json);
    if(clean) destroy_eir(dsc);
    free(eir_line);
    free(eir_json);
}


#else
void send_json_eir(arr_t *ss, bool clean){
    char *eir_json = (char*)malloc(255);
    char *eir_line = (char*)malloc(100);

    strcpy(eir_json, "1:{\"eirs\":[");
    for(uint8_t i = 0; i<ss->idx;i++){
        sprintf(eir_line,"{\"addr\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"name\":\"%s\",\"stat\":1},",ss->arr[i]->addr[0]
                                                                                          ,ss->arr[i]->addr[1]
                                                                                          ,ss->arr[i]->addr[2]
                                                                                          ,ss->arr[i]->addr[3]
                                                                                          ,ss->arr[i]->addr[4]
                                                                                          ,ss->arr[i]->addr[5]
                                                                                          ,ss->arr[i]->name);
        strcat(eir_json, eir_line);
    }
    uint8_t str_len = strlen(eir_json);
    if(str_len > 12)
        eir_json[str_len-1] = 0;

    strcat(eir_json, "]}\n");
    uart_send_data(eir_json);
    if(clean) destroy_eir(ss);
    free(eir_line);
    free(eir_json);
}
#endif
void send_json_eir_connected(const char* addr,const char* name){
    char *eir_line = (char*)malloc(100);
    sprintf(eir_line,"1:{\"eirs\":[{\"addr\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"name\":\"%s\",\"stat\":0}]}\n",addr[0]
                                                                        ,addr[1],addr[2],addr[3],addr[4],addr[5],name);
    uart_send_data(eir_line);
    free(eir_line);
}

void eir_print(arr_t *ss){
    printf("===== saved_eirs ======\n");
    for(uint8_t i = 0; i<ss->idx;i++){
        printf("addr: %02x:%02x:%02x:%02x:%02x:%02x name %s\n",ss->arr[i]->addr[0]
                                                              ,ss->arr[i]->addr[1]
                                                              ,ss->arr[i]->addr[2]
                                                              ,ss->arr[i]->addr[3]
                                                              ,ss->arr[i]->addr[4]
                                                              ,ss->arr[i]->addr[5]
                                                              ,ss->arr[i]->name);
    }
}

size_t get_size(arr_t *ss){
    size_t size = 1; // for idx
    for(uint8_t i = 0; i<ss->idx;i++)
        size += ADDR_SIZE + strlen(ss->arr[i]->name) + 1; // 1 for name len
    return size;
}

bool check_stored(arr_t *ss, char* adr){
    bool res = false;
    if(ss->arr == NULL) return res;
    for(uint8_t i = 0; i<ss->idx;i++){
        if(!memcmp((char*)ss->arr[i]->addr, adr,ADDR_SIZE)){
            res = true;
            break;
        }
    }    
    return res;
}

char* serialize(arr_t *ss, size_t *mlen){
    
    char *mem,*p_mem;

    *mlen = get_size(ss);
    mem = (char*)malloc(*mlen); 
    memset(mem,0,*mlen);
    p_mem = mem;
    memcpy(p_mem++, &ss->idx, sizeof ss->idx);
    for(uint8_t i = 0; i<ss->idx;i++){
        memcpy(p_mem, ss->arr[i]->addr, ADDR_SIZE);
        p_mem += ADDR_SIZE;
        uint8_t nlen = strlen(ss->arr[i]->name);
        memcpy(p_mem++, &nlen, sizeof nlen);
        memcpy(p_mem, ss->arr[i]->name, nlen);
        p_mem += nlen;
    }
    return mem;
}

void deserialize(arr_t *ss, uint8_t **mem){

    uint8_t *p_mem = *mem;
    uint8_t nlen;
    memcpy(&ss->idx, p_mem++, sizeof ss->idx);
    ss->scan_idx = ss->idx;
    ss->arr = (eir_t**)realloc(ss->arr, (ss->idx + 1) * sizeof(eir_t*));
    for(int8_t i = 0; i<ss->idx;i++){
        ss->arr[i] = (eir_t*)malloc(sizeof(eir_t));
        memcpy(ss->arr[i]->addr, p_mem, ADDR_SIZE);
        p_mem += ADDR_SIZE;
        memcpy(&nlen, p_mem++, sizeof nlen);
        ss->arr[i]->name = (char*)malloc(nlen+1);
        memcpy(ss->arr[i]->name, p_mem, nlen);
        ss->arr[i]->name[nlen] = 0;
        p_mem += nlen;
    }
}
