/**********************************************************************************/
/* File:         light_sensor.h                                                   */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the LightSensor node          */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_LIGHT_SENSOR_H
#define WB_LIGHT_SENSOR_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void   wb_light_sensor_enable(WbDeviceTag tag, int ms);
void   wb_light_sensor_disable(WbDeviceTag tag);
int    wb_light_sensor_get_sampling_period(WbDeviceTag tag);

double wb_light_sensor_get_value(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_LIGHT_SENSOR_H */
