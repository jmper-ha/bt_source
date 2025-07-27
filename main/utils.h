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
} arr_t;


bool add_new_eir(arr_t *ss, const char* adr, const char* nam, uint8_t nam_len);
void send_json_eir_connected(const char* addr,const char* name);
void send_json_eir(arr_t *ss, bool clean);
char* serialize(arr_t *ss, size_t *mlen);
void deserialize(arr_t *ss, uint8_t **mem);
bool check_stored(arr_t *ss, char* adr);
void del_line(arr_t *ss, const char* adr);
void eir_print(arr_t *ss);
#endif /* __UTILS_H__ */