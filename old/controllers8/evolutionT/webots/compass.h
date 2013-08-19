/**********************************************************************************/
/* File:         compass.h                                                        */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Compass node              */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_COMPASS_H
#define WB_COMPASS_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void          wb_compass_enable(WbDeviceTag tag, int ms);
void          wb_compass_disable(WbDeviceTag tag);
int           wb_compass_get_sampling_period(WbDeviceTag tag);

const double *wb_compass_get_values(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_COMPASS_H */
