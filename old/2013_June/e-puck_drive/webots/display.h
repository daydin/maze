/**********************************************************************************/
/* File:         display.h                                                        */
/* Date:         Jan 27, 2009                                                     */
/* Description:  Webots C programming interface for the Display node              */
/* Authors:      Rohrer Fabien                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_DISPLAY_H
#define WB_DISPLAY_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

int            wb_display_get_width(WbDeviceTag tag);
int            wb_display_get_height(WbDeviceTag tag);

// drawing properties
void           wb_display_set_color(WbDeviceTag tag, int color);
void           wb_display_set_alpha(WbDeviceTag tag, double alpha);
void           wb_display_set_opacity(WbDeviceTag tag, double opacity);

// draw primitive
void           wb_display_draw_pixel(WbDeviceTag tag,int x,int y);
void           wb_display_draw_line(WbDeviceTag tag,int x1,int y1,int x2,int y2);
void           wb_display_draw_rectangle(WbDeviceTag tag,int x,int y,int width,int height);
void           wb_display_draw_oval(WbDeviceTag tag,int cx,int cy,int a,int b);
void           wb_display_draw_polygon(WbDeviceTag tag,const int *x,const int *y,int size);
void           wb_display_draw_text(WbDeviceTag tag,const char *txt,int x,int y);
void           wb_display_fill_rectangle(WbDeviceTag tag,int x,int y,int width,int height);
void           wb_display_fill_oval(WbDeviceTag tag,int cx,int cy,int a,int b);
void           wb_display_fill_polygon(WbDeviceTag tag,const int *x,const int *y,int size);

// WbImageRef handle functions
#define WB_IMAGE_RGB  3
#define WB_IMAGE_RGBA 4
#define WB_IMAGE_ARGB 5
#define WB_IMAGE_BGRA 6

WbImageRef     wb_display_image_new(WbDeviceTag tag,int width,int height,const void *data,int format);
WbImageRef     wb_display_image_copy(WbDeviceTag tag,int x,int y,int width,int height);
WbImageRef     wb_display_image_load(WbDeviceTag tag,const char *filename);
void           wb_display_image_delete(WbDeviceTag tag,WbImageRef ir);
void           wb_display_image_paste(WbDeviceTag tag,WbImageRef ir,int x,int y);
void           wb_display_image_save(WbDeviceTag tag,WbImageRef ir,const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* WB_DISPLAY_H */
