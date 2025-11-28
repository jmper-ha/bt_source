#ifndef __UTILS_H__
#define __UTILS_H__

#define ADDR_SIZE    6  //ESP_BD_ADDR_LEN

typedef struct {
    uint8_t addr[ADDR_SIZE];
    char*   name;
} eir_t;

typedef struct {
    eir_t**     arr;
    uint8_t     idx;
    uint8_t     current_idx;
    int8_t     scan_idx;
} arr_t;

char* get_peer_name();
uint8_t get_peer_name_len();
void set_peer_name(char* name);
char* get_audio_state(uint8_t state);

bool add_new_eir(arr_t *ss, const char* adr, const char* nam, uint8_t nam_len);
void send_json_eir_connected(const char* addr,const char* name);
void send_json_eir(arr_t *svd, arr_t *dsc, bool clean);
char* serialize(arr_t *ss, size_t *mlen);
void deserialize(arr_t *ss, uint8_t **mem);
bool check_stored(arr_t *ss, char* adr);
void del_line(arr_t *ss, const char* adr);
void eir_print(arr_t *ss);
void destroy_eir(arr_t *ss);
char* get_name(arr_t *ss, const char* adr);

bool set_eir_scan(arr_t *ss, char* adr);
bool is_eir_scan(arr_t *ss);
void stop_eir_scan(arr_t *ss);

void set_local_volume(uint8_t vol);
uint8_t get_local_volume();
void timer_init();
void timer_start();

#endif /* __UTILS_H__ */