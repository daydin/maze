/**********************************************************************************/
/* File:         camera.h                                                         */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Camera node               */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_CAMERA_H
#define WB_CAMERA_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void                 wb_camera_enable(WbDeviceTag tag, int ms);
void                 wb_camera_disable(WbDeviceTag tag);
int                  wb_camera_get_sampling_period(WbDeviceTag tag);

const unsigned char *wb_camera_get_image(WbDeviceTag tag);
const float         *wb_camera_get_range_image(WbDeviceTag tag);
int                  wb_camera_get_width(WbDeviceTag tag);
int                  wb_camera_get_height(WbDeviceTag tag);
double               wb_camera_get_fov(WbDeviceTag tag);
void                 wb_camera_set_fov(WbDeviceTag tag, double fov); /* fov specified in rad */
int                  wb_camera_get_type(WbDeviceTag tag);
double               wb_camera_get_near(WbDeviceTag tag);
double               wb_camera_get_max_range(WbDeviceTag tag);
int                  wb_camera_save_image(WbDeviceTag tag, const char *filename, int quality);
void                 wb_camera_move_window(WbDeviceTag tag, int x, int y) WB_DEPRECATED;

/* possible return values of wb_camera_get_type() */
#define WB_CAMERA_COLOR           99
#define WB_CAMERA_RANGE_FINDER    114
#define WB_CAMERA_BOTH            98

/* useful macros to get pixel colors from the image data, width and coords *
 *
 *  ^ y
 *  |    (height)
 *  |===============
 *  |=============== *: pixel@(x,y)
 *  |---------*=====
 *  |=========|===== (width)
 *  |=========|=====
 *  |=========|=====
 *  |=========|=====
 * -+-----------------> x
 * o|
 */

#define wb_camera_image_get_red(image,width,x,y)   (image[4*((y)*(width)+(x))+2])
#define wb_camera_image_get_green(image,width,x,y) (image[4*((y)*(width)+(x))+1])
#define wb_camera_image_get_blue(image,width,x,y)  (image[4*((y)*(width)+(x))])

#ifdef KROS_COMPILATION
#define wb_camera_image_get_grey(image,width,x,y) (image[(y)*(width)+(x)])
#else
#define wb_camera_image_get_grey(image,w,x,y) ((image[4*((y)*(w)+(x))+2]+image[4*((y)*(w)+(x))+1]+image[4*((y)*(w)+(x))])/3)
#endif

/* range finder functions */
#define wb_camera_range_image_get_depth(image,width,x,y) (image[(y)*(width)+(x)])

/* Deprecated functions */
float wb_camera_range_image_get_value(const float *image,double n,double f,int width,int x,int y) WB_DEPRECATED; // please use wb_camera_range_image_get_depth() instead

#ifdef __cplusplus
}
#endif

#endif /* WB_CAMERA_H */
