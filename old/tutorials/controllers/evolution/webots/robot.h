/**********************************************************************************/
/* File:         robot.h                                                          */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Robot node                */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_ROBOT_H
#define WB_ROBOT_H

#define WB_USING_C_API
#include "types.h"

#if defined(__VISUALC__) || defined (_MSC_VER)
#include "stdio.h"
#endif

#include "nodes.h"

#ifdef KROS_COMPILATION
#define main() _kros_main()
#endif

typedef void *WbMutexRef;  /* identifier of a mutex */

typedef enum {
  WB_MODE_SIMULATION=0,
  WB_MODE_CROSS_COMPILATION=1,
  WB_MODE_REMOTE_CONTROL=2
} WbRobotMode;

/* cart function headers */
#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__VISUALC__) && !defined (_MSC_VER)
int          wb_robot_init();

/* In the visual studio case, the buffer size of the standard output and
 * the standard error cannot be modified from a dll
 */
#else
int wb_robot_init_msvc(); /* internally, this function just calls wb_robot_init() */
#define wb_robot_init() (setvbuf( stdout, NULL, _IONBF, 0 ), \
                         setvbuf( stderr, NULL, _IONBF, 0 ), \
                         wb_robot_init_msvc())
#endif

int          wb_robot_step(int ms); /* ms = time step in milliseconds */
void         wb_robot_cleanup();
double       wb_robot_get_time();
const char  *wb_robot_get_name();
const char  *wb_robot_get_model();
int          wb_robot_get_mode();
void         wb_robot_set_mode(int mode, void *args);
bool         wb_robot_get_synchronization();
const char  *wb_robot_get_project_path();
double       wb_robot_get_basic_time_step();
WbDeviceTag  wb_robot_get_device(const char *name);

/* Controller API */
const char  *wb_robot_get_controller_name();
const char  *wb_robot_get_controller_arguments();

/* Introspection API */
int          wb_robot_get_number_of_devices();
WbDeviceTag  wb_robot_get_device_by_index(int index);
WbNodeType   wb_robot_get_type();

/* robot keyboard API */
void    wb_robot_keyboard_enable(int ms);
void    wb_robot_keyboard_disable();
int     wb_robot_keyboard_get_key();
#define WB_ROBOT_KEYBOARD_KEY     0x000ffff
#define WB_ROBOT_KEYBOARD_SHIFT   0x0010000
#define WB_ROBOT_KEYBOARD_CONTROL 0x0020000
#define WB_ROBOT_KEYBOARD_ALT     0x0040000
#define WB_ROBOT_KEYBOARD_LEFT          314 /* the following key codes are taken from include/wx/defs.h */
#define WB_ROBOT_KEYBOARD_UP            315
#define WB_ROBOT_KEYBOARD_RIGHT         316
#define WB_ROBOT_KEYBOARD_DOWN          317
#define WB_ROBOT_KEYBOARD_PAGEUP        366
#define WB_ROBOT_KEYBOARD_PAGEDOWN      367
#define WB_ROBOT_KEYBOARD_HOME          313
#define WB_ROBOT_KEYBOARD_END           312

/* robot battery API */
void   wb_robot_battery_sensor_enable(int ms);
void   wb_robot_battery_sensor_disable();
int    wb_robot_battery_sensor_get_sampling_period();
double wb_robot_battery_sensor_get_value();

/* robot multi-thread API */
#ifndef WB_MATLAB_LOADLIBRARY
void        wb_robot_task_new(void (*task)(void *),void *param); /* create a task */
WbMutexRef  wb_robot_mutex_new();
void        wb_robot_mutex_lock(WbMutexRef);
void        wb_robot_mutex_unlock(WbMutexRef);
void        wb_robot_mutex_delete(WbMutexRef);
#endif

#ifdef __cplusplus
}
#endif

#endif /* WB_ROBOT_H */
