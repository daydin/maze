/**********************************************************************************/
/* File:         remote_control.h                                                 */
/* Date:         Sept 7, 2010                                                     */
/* Description:  Webots C programming interface for communicating with the        */
/*               remote control library                                           */
/* Authors:      Rohrer Fabien                                                    */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_REMOTE_CONTROL_H
#define WB_REMOTE_CONTROL_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void *wb_remote_control_custom_function(void *);

//Sensor functions (values read by the controller)
void wbr_robot_battery_sensor_set_value   (double value);
void wbr_differential_wheels_set_encoders (double left, double right);

void wbr_accelerometer_set_values         (WbDeviceTag tag, const double *values);
void wbr_compass_set_values               (WbDeviceTag tag, const double *values);
void wbr_distance_sensor_set_value        (WbDeviceTag tag, double value);
void wbr_gps_set_values                   (WbDeviceTag tag, const double *values);
void wbr_gyro_set_values                  (WbDeviceTag tag, const double *values);
void wbr_light_sensor_set_value           (WbDeviceTag tag, double value);
void wbr_microphone_set_buffer            (WbDeviceTag tag, const unsigned char* buffer, int size);
void wbr_servo_set_position_feedback      (WbDeviceTag tag, double value);
void wbr_servo_set_motor_force_feedback   (WbDeviceTag tag, double value);
void wbr_touch_sensor_set_value           (WbDeviceTag tag, double value);
void wbr_touch_sensor_set_values          (WbDeviceTag tag, const double *values);

//TODO doc required
void wbr_display_save_image               (WbDeviceTag t, int id, int width, int height, unsigned char *im);

void wbr_camera_set_image                 (WbDeviceTag tag, const unsigned char* img);
unsigned char *wbr_camera_get_image_buffer(WbDeviceTag tag);

//Actuator functions (values written by the controller)
typedef struct WbrInterface {
  //mandatory functions :
  struct {
    bool (*wbr_start)          (void *);
    void (*wbr_stop)           ();
    bool (*wbr_has_failed)     ();
    void (*wbr_stop_actuators) ();
    int  (*wbr_robot_step)     (int);
  } mandatory;

  // user custom function to communicate data
  void *(*wbr_custom_function) (void *);

  //optional functions (if they are used but not defined it will print a warning) :
  void (*wbr_robot_battery_set_refresh_rate) (int ms);

  void (*wbr_differential_wheels_encoders_set_refresh_rate) (int ms);
  void (*wbr_differential_wheels_set_speed)                 (double left, double right);
  void (*wbr_differential_wheels_set_encoders)              (double left, double right);

  void (*wbr_set_refresh_rate)     (WbDeviceTag tag, int ms);
  void (*wbr_camera_set_fov)       (WbDeviceTag tag, double fov);
  void (*wbr_led_set)              (WbDeviceTag tag, int state);
  void (*wbr_pen_set_ink_color)    (WbDeviceTag tag, int color, double density);
  void (*wbr_pen_write)            (WbDeviceTag tag, bool write); 
  void (*wbr_receiver_set_channel) (WbDeviceTag tag, int channel);
  void (*wbr_speaker_emit_sample)  (WbDeviceTag tag, const void *data, int size);

  void (*wbr_servo_set_motor_force_refresh_rate) (WbDeviceTag tag, int ms);
  void (*wbr_servo_set_position)     (WbDeviceTag tag, double position);
  void (*wbr_servo_set_acceleration) (WbDeviceTag tag, double acceleration);
  void (*wbr_servo_set_velocity)     (WbDeviceTag tag, double velocity);
  void (*wbr_servo_set_force)        (WbDeviceTag tag, double force);
  void (*wbr_servo_set_motor_force)  (WbDeviceTag tag, double motor_force);
  void (*wbr_servo_set_control_p)    (WbDeviceTag tag, double p);

  void (*wbr_display_set_color)      (WbDeviceTag tag, int color);
  void (*wbr_display_set_alpha)      (WbDeviceTag tag, double alpha);
  void (*wbr_display_set_opacity)    (WbDeviceTag tag, double opacity);
  
  void (*wbr_display_draw_pixel)     (WbDeviceTag tag, int x, int y);
  void (*wbr_display_draw_line)      (WbDeviceTag tag, int x1, int y1, int x2, int y2);
  void (*wbr_display_draw_rectangle) (WbDeviceTag tag, int x, int y, int width,int height);
  void (*wbr_display_draw_oval)      (WbDeviceTag tag, int cx, int cy, int a, int b);
  void (*wbr_display_draw_polygon)   (WbDeviceTag tag, const int *x, const int *y, int size);
  void (*wbr_display_draw_text)      (WbDeviceTag tag, const char *txt, int x, int y);
  void (*wbr_display_fill_rectangle) (WbDeviceTag tag, int x, int y, int width, int height);
  void (*wbr_display_fill_oval)      (WbDeviceTag tag, int cx, int cy, int a, int b);
  void (*wbr_display_fill_polygon)   (WbDeviceTag tag, const int *x, const int *y, int size);
  
  //TODO these functions are not like the API, documentation required
  void (*wbr_display_image_new)      (WbDeviceTag tag, int id, int width, int height, const void *data, int format);
  void (*wbr_display_image_copy)     (WbDeviceTag tag, int id, int x, int y, int width, int height);
  void (*wbr_display_image_delete)   (WbDeviceTag tag, int id);
  void (*wbr_display_image_paste)    (WbDeviceTag tag, int id ,int x ,int y);
  void (*wbr_display_image_save)     (WbDeviceTag tag, int id);
  
} WbrInterface;

#ifdef __cplusplus
}
#endif

#endif /* WB_REMOTE_CONTROL_H */
