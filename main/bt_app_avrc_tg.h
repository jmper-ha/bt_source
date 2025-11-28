#ifndef __BT_APP_AVRC_TG_H__
#define __BT_APP_AVRC_TG_H__

#include "esp_avrc_api.h"

#define BT_AVRC_TG_TAG             "BT_APP_AVRC_TG"

void avrc_tg_init();
void volume_set_by_local_host_sink_mode(uint8_t volume);

#endif /* __BT_APP_AVRC_TG_H__ */