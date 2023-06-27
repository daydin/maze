/**********************************************************************************/
/* File:         gps.h                                                            */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the GPS node                  */
/* Authors:      Olivier Michel, Yvan Bourquin                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_GPS_H
#define WB_GPS_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void          wb_gps_enable(WbDeviceTag tag,int ms);
void          wb_gps_disable(WbDeviceTag tag);
int           wb_gps_get_sampling_period(WbDeviceTag tag);

const double *wb_gps_get_values(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_GPS_H */
