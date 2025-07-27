#ifndef __BT_APP_CORE_H__
#define __BT_APP_CORE_H__

#define BT_APP_CORE_TAG             "BT_APP_CORE"
#define BT_APP_SIG_WORK_DISPATCH    (0x01)

typedef void (* bt_app_cb_t) (uint16_t event, void *param);
typedef void (* bt_app_copy_cb_t) (void *p_dest, void *p_src, int len);

/* message to be sent */
typedef struct {
    uint16_t             sig;      /*!< signal to bt_app_task */
    uint16_t             event;    /*!< message event id */
    bt_app_cb_t          cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} bt_app_msg_t;


bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback);
void bt_app_task_start_up(void);
void bt_app_task_shut_down(void);

#endif /* __BT_APP_CORE_H__ */