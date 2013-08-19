/**********************************************************************************/
/* File:         inertial_unit.h                                                           */
/* Date:         Oct 29, 2012                                                     */
/* Description:  Webots C programming interface for the InertialUnit node         */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2012 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_INERTIAL_UNIT_H
#define WB_INERTIAL_UNIT_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_inertial_unit_enable(WbDeviceTag tag, int ms);
void wb_inertial_unit_disable(WbDeviceTag tag);
int  wb_inertial_unit_get_sampling_period(WbDeviceTag tag);

const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_INERTIAL_UNIT_H */
