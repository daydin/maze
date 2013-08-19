/**********************************************************************************/
/* File:         connector.h                                                      */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Connector node            */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_CONNECTOR_H
#define WB_CONNECTOR_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_connector_enable_presence(WbDeviceTag tag, int ms);
void wb_connector_disable_presence(WbDeviceTag tag);
int  wb_connector_get_presence(WbDeviceTag tag);
void wb_connector_lock(WbDeviceTag tag);
void wb_connector_unlock(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_CONNECTOR_H */
