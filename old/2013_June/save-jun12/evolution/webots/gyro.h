/**********************************************************************************/
/* File:         gyro.h                                                           */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Gyro node                 */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_GYRO_H
#define WB_GYRO_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void          wb_gyro_enable(WbDeviceTag tag, int ms);
void          wb_gyro_disable(WbDeviceTag tag);
int           wb_gyro_get_sampling_period(WbDeviceTag tag);

const double *wb_gyro_get_values(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_GYRO_H */
