/**********************************************************************************/
/* File:         speaker.h                                                        */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Speaker node              */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_SPEAKER_H
#define WB_SPEAKER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_speaker_emit_sample(WbDeviceTag tag, const void *data, int size);

#ifdef __cplusplus
}
#endif

#endif /* WB_SPEAKER_H */
