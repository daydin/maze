/*
 * File:         void.c
 * Description:  This is an empty robot controller, the robot does nothing. 
 * Author:       www.cyberbotics.com
 * Note:         !!! PLEASE DO NOT MODIFY THIS SOURCE FILE !!!
 *               This is a system file that Webots needs to work correctly.
 */
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/differential_wheels.h>

#include <webots/distance_sensor.h>


int main() {
  wb_robot_init();
  int time_step = (int) wb_robot_get_basic_time_step();
  for (;;) {
    wb_robot_step(time_step);
    wb_differential_wheels_set_speed(1000,1000);
  }
  return 0;
}
