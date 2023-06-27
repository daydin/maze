/**********************************************************************************/
/* File:         motion.h                                                         */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for Motion file playback          */
/* Authors:      Yvan Bourquin                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef MOTION_H
#define MOTION_H

#include "../types.h"

#ifdef __cplusplus
extern "C" {
#endif

WbMotionRef wbu_motion_new(const char *filename);
void        wbu_motion_delete(WbMotionRef motion);

void        wbu_motion_play(WbMotionRef motion);
void        wbu_motion_stop(WbMotionRef motion);
void        wbu_motion_set_loop(WbMotionRef motion, bool loop);
void        wbu_motion_set_reverse(WbMotionRef motion, bool reverse);

bool        wbu_motion_is_over(WbMotionRef motion);
int         wbu_motion_get_duration(WbMotionRef motion);
int         wbu_motion_get_time(WbMotionRef motion);
void        wbu_motion_set_time(WbMotionRef motion, int time);

#ifdef __cplusplus
}
#endif

#endif
