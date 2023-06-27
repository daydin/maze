/**********************************************************************************/
/* File:         distance_sensor.h                                                */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the DistanceSensor node       */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_DISTANCE_SENSOR_H
#define WB_DISTANCE_SENSOR_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void   wb_distance_sensor_enable(WbDeviceTag tag, int ms);
void   wb_distance_sensor_disable(WbDeviceTag tag);
int    wb_distance_sensor_get_sampling_period(WbDeviceTag tag);
double wb_distance_sensor_get_value(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_DISTANCE_SENSOR_H */
