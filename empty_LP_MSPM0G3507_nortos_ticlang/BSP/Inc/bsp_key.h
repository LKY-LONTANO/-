#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "ti_msp_dl_config.h"

typedef enum {
	USEKEY_stateless,
	USEKEY_single_click,
	USEKEY_double_click,
	USEKEY_long_click
}UserKeyState_t;

typedef struct {
    UserKeyState_t (*getKeyState)(uint16_t freq);
}KeyInterface_t,*pKeyInterface_t;

extern KeyInterface_t UserKey;

#endif /* __BSP_LED_H */


