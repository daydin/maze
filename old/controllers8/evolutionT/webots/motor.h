/**********************************************************************************/
/* File:         motor.h                                                          */
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

void   wb_motor_set_position(WbDeviceTag tag, double position);         /* rad or meters     */
void   wb_motor_set_acceleration(WbDeviceTag tag, double acceleration); /* rad/s^2 or m/s^2  */
void   wb_motor_set_velocity(WbDeviceTag tag, double velocity);         /* rad/s or m/s      */
void   wb_motor_set_force(WbDeviceTag tag, double force);               /* N                 */
void   wb_motor_set_torque(WbDeviceTag tag, double torque);             /* N*m               */
void   wb_motor_set_motor_force(WbDeviceTag tag, double motor_force);   /* N                 */
void   wb_motor_set_motor_torque(WbDeviceTag tag, double motor_force);  /* N*m               */
void   wb_motor_set_control_p(WbDeviceTag tag, double p);               /* set the controlP  */
void   wb_motor_set_control_mode(WbDeviceTag tag, int mode);            /* set the control mode  */

void   wb_motor_enable_position(WbDeviceTag tag, int ms);               /* milliseconds      */
void   wb_motor_disable_position(WbDeviceTag tag);
int    wb_motor_get_position_sampling_period(WbDeviceTag tag);
double wb_motor_get_position(WbDeviceTag tag);                          /* rad or meters     */

void   wb_motor_enable_motor_force_feedback(WbDeviceTag tag, int ms);
void   wb_motor_disable_motor_force_feedback(WbDeviceTag tag);
int    wb_motor_get_motor_force_feedback_sampling_period(WbDeviceTag tag);
double wb_motor_get_motor_force_feedback(WbDeviceTag tag);

void   wb_motor_enable_motor_torque_feedback(WbDeviceTag tag, int ms);
void   wb_motor_disable_motor_torque_feedback(WbDeviceTag tag);
int    wb_motor_get_motor_torque_feedback_sampling_period(WbDeviceTag tag);
double wb_motor_get_motor_torque_feedback(WbDeviceTag tag);

int wb_motor_get_type(WbDeviceTag tag);

double  wb_motor_get_target_position(WbDeviceTag tag);
double  wb_motor_get_min_position(WbDeviceTag tag);
double  wb_motor_get_max_position(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_MOTOR_H */
