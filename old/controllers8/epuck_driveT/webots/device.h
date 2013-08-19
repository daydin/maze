/**********************************************************************************/
/* File:         device.h                                                         */
/* Date:         Mar 3th, 2010                                                    */
/* Description:  Abstraction of a robot device                                    */
/* Authors:      Fabien Rohrer                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_DEVICE_H
#define WB_DEVICE_H

#define WB_USING_C_API
#include "types.h"
#include "nodes.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *wb_device_get_name(WbDeviceTag dt);
WbNodeType  wb_device_get_type(WbDeviceTag dt);

#ifdef __cplusplus
}
#endif

#endif /* WB_DEVICE_H */
