/**********************************************************************************/
/* File:         receiver.h                                                       */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Receiver node             */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_RECEIVER_H
#define WB_RECEIVER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WB_CHANNEL_BROADCAST
#define WB_CHANNEL_BROADCAST -1
#endif

/* device functions */
void          wb_receiver_enable(WbDeviceTag tag, int ms);
void          wb_receiver_disable(WbDeviceTag tag);
int           wb_receiver_get_sampling_period(WbDeviceTag tag);

void          wb_receiver_set_channel(WbDeviceTag tag, int channel);
int           wb_receiver_get_channel(WbDeviceTag tag);
int           wb_receiver_get_queue_length(WbDeviceTag tag);
void          wb_receiver_next_packet(WbDeviceTag tag);
int           wb_receiver_get_data_size(WbDeviceTag tag);
const void   *wb_receiver_get_data(WbDeviceTag tag);
double        wb_receiver_get_signal_strength(WbDeviceTag tag);
const double *wb_receiver_get_emitter_direction(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_RECEIVER_H */
