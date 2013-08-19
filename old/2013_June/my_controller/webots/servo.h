/**********************************************************************************/
/* File:         servo.h                                                          */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Servo node                */
/* Authors:      Olivier Michel, Yvan Bourquin                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_SERVO_H
#define WB_SERVO_H

#define WB_USING_C_API
#include "types.h"

#ifndef WB_MATLAB_LOADLIBRARY
#include <math.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void   wb_servo_set_position(WbDeviceTag tag, double position);         /* rad or meters     */
void   wb_servo_set_acceleration(WbDeviceTag tag, double acceleration); /* rad/s^2 or m/s^2  */
void   wb_servo_set_velocity(WbDeviceTag tag, double velocity);         /* rad/s or m/s      */
void   wb_servo_set_force(WbDeviceTag tag, double force);               /* N or N*m          */
void   wb_servo_set_motor_force(WbDeviceTag tag, double motor_force);   /* N or N*m          */
void   wb_servo_set_control_p(WbDeviceTag tag, double p);               /* set the controlP  */

void   wb_servo_enable_position(WbDeviceTag tag, int ms);               /* milliseconds      */
void   wb_servo_disable_position(WbDeviceTag tag);
int    wb_servo_get_position_sampling_period(WbDeviceTag tag);
double wb_servo_get_position(WbDeviceTag tag);                          /* rad or meters     */

void   wb_servo_enable_motor_force_feedback(WbDeviceTag tag, int ms);
void   wb_servo_disable_motor_force_feedback(WbDeviceTag tag);
int    wb_servo_get_motor_force_feedback_sampling_period(WbDeviceTag tag);
double wb_servo_get_motor_force_feedback(WbDeviceTag tag);

#define WB_SERVO_ROTATIONAL 0
#define WB_SERVO_LINEAR     1
int         wb_servo_get_type(WbDeviceTag tag);

double      wb_servo_get_target_position(WbDeviceTag tag);
double      wb_servo_get_min_position(WbDeviceTag tag);
double      wb_servo_get_max_position(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_SERVO_H */
