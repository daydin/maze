
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <emitter.h>
#include <webots/receiver.h>
//#include <webots/node.h>


int main() {
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  
  for (;;) {
    wb_robot_step(time_step);
    wb_differential_wheels_set_speed(1000,1000);
  }
  
  return 0;
}