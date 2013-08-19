/**********************************************************************************/
/* File:         emitter.h                                                        */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Emitter node              */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_EMITTER_H
#define WB_EMITTER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WB_CHANNEL_BROADCAST
#define WB_CHANNEL_BROADCAST -1
#endif

int    wb_emitter_send(WbDeviceTag tag, const void *data, int size);
int    wb_emitter_get_buffer_size(WbDeviceTag tag);
void   wb_emitter_set_channel(WbDeviceTag tag, int channel);
int    wb_emitter_get_channel(WbDeviceTag tag);
double wb_emitter_get_range(WbDeviceTag tag);
void   wb_emitter_set_range(WbDeviceTag tag, double range);

#ifdef __cplusplus
}
#endif

#endif /* WB_EMITTER_H */
