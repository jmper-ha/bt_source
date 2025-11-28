#ifndef __BT_APP_AVRC_CT_H__
#define __BT_APP_AVRC_CT_H__

#include "esp_avrc_api.h"

#define BT_AVRC_CT_TAG             "BT_APP_AVRC_CT"

#define APP_RC_CT_TL_GET_CAPS            (0)
#define APP_RC_CT_TL_GET_META_DATA       (1)
#define APP_RC_CT_TL_RN_TRACK_CHANGE     (2)
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE  (3)
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE  (4)
#define APP_RC_CT_TL_RN_VOLUME_CHANGE    (5)
#define APP_RC_CT_TL_RN_PT_CMD           (6)

#define CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE false //true //
#define MIN(a,b) ((a) < (b) ? (a) : (b))

typedef struct {
    char*   title;
    char*   artist;
    char*   album;
    uint8_t title_len;
    uint8_t artist_len;
    uint8_t album_len;
    uint8_t audio_stat;
} meta_t;

void avrc_ct_init();
void volume_set_by_local_host_source_mode(uint8_t volume);

#endif /* __BT_APP_AVRC_CT_H__ */