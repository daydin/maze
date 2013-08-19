/**********************************************************************************/
/* File:         differential_wheels.h                                            */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the DifferentialWheels node   */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_DIFFERENTIAL_WHEELS_H
#define WB_DIFFERENTIAL_WHEELS_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void   wb_differential_wheels_set_speed(double left, double right);
double wb_differential_wheels_get_left_speed();
double wb_differential_wheels_get_right_speed();
double wb_differential_wheels_get_max_speed();
double wb_differential_wheels_get_speed_unit();

void   wb_differential_wheels_enable_encoders(int ms);
void   wb_differential_wheels_disable_encoders();
int    wb_differential_wheels_get_encoders_sampling_period();
double wb_differential_wheels_get_left_encoder();
double wb_differential_wheels_get_right_encoder();
void   wb_differential_wheels_set_encoders(double left,double right);

#ifdef __cplusplus
}
#endif

#endif /* WB_DIFFERENTIAL_WHEELS_H */
