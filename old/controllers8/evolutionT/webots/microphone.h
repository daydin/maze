/**********************************************************************************/
/* File:         microphone.h                                                     */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Microphone node           */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_MICROPHONE_H
#define WB_MICROPHONE_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// device functions
void wb_microphone_enable(WbDeviceTag tag, int ms);
void wb_microphone_disable(WbDeviceTag tag);
int  wb_microphone_get_sampling_period(WbDeviceTag t);

// data packet functions
const void *wb_microphone_get_sample_data(WbDeviceTag tag);
int wb_microphone_get_sample_size(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_MICROPHONE_H */
