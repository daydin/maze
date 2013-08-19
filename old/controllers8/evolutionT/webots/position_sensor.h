/**********************************************************************************/
/* File:         position_sensor.h                                                          */
/* Date:         Feb 11, 2013                                                     */
/* Description:  Webots C programming interface for the Motor node                */
/* Authors:      Luc Guyot                                                        */
/*                                                                                */
/* Copyright (c) 2013 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_MOTOR_H
#define WB_MOTOR_H

#define WB_USING_C_API
#include "types.h"

#ifndef WB_MATLAB_LOADLIBRARY
#include <math.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* set the control mode  */

void   wb_position_sensor_enable_position(WbDeviceTag tag, int ms);               /* milliseconds      */
void   wb_position_sensor_disable_position(WbDeviceTag tag);
int    wb_position_sensor_get_position_sampling_period(WbDeviceTag tag);
double wb_position_sensor_get_position(WbDeviceTag tag);                          /* rad or meters     */

#define WB_ANGULAR 0
#define WB_LINEAR  1
int wb_position_sensor_get_type(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_MOTOR_H */
