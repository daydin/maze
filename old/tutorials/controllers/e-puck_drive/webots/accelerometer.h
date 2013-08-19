/**********************************************************************************/
/* File:         accelerometer.h                                                  */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Accelerometer node        */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_ACCELEROMETER_H
#define WB_ACCELEROMETER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void          wb_accelerometer_enable(WbDeviceTag tag, int ms);
void          wb_accelerometer_disable(WbDeviceTag tag);
int           wb_accelerometer_get_sampling_period(WbDeviceTag tag);

const double *wb_accelerometer_get_values(WbDeviceTag tag); /* return a pointer to an array of 3 double for X, Y and Z accelerations */

#ifdef __cplusplus
}
#endif

#endif /* WB_ACCELEROMETER_H */
