/**********************************************************************************/
/* File:         led.h                                                            */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the LED node                  */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_LED_H
#define WB_LED_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_led_set(WbDeviceTag tag, int value);  /* 0 for off, 1, 2, etc. for on with a color */
int  wb_led_get(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_LED_H */
