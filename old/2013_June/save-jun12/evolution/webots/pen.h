/**********************************************************************************/
/* File:         pen.h                                                            */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Pen node                  */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_PEN_H
#define WB_PEN_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_pen_write(WbDeviceTag tag, bool write);               /* switch on/off writing */
void wb_pen_set_ink_color(WbDeviceTag tag, int color, double density); /* set ink color */

#ifdef __cplusplus
}
#endif

#endif /* WB_PEN_H */
