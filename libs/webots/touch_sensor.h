/**********************************************************************************/
/* File:         touch_sensor.h                                                   */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the TouchSensor node          */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_TOUCH_SENSOR_H
#define WB_TOUCH_SENSOR_H

#define WB_USING_C_API

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void          wb_touch_sensor_enable(WbDeviceTag tag,int ms);
void          wb_touch_sensor_disable(WbDeviceTag tag);
int           wb_touch_sensor_get_sampling_period(WbDeviceTag tag);

double        wb_touch_sensor_get_value(WbDeviceTag tag);
const double *wb_touch_sensor_get_values(WbDeviceTag tag);

#define WB_TOUCH_SENSOR_BUMPER  0
#define WB_TOUCH_SENSOR_FORCE   1
#define WB_TOUCH_SENSOR_FORCE3D 2
int wb_touch_sensor_get_type(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_TOUCH_SENSOR_H */
