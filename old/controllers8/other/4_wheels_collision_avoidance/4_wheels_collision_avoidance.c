#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/servo.h>

// time in [ms] of a simulation step
#define TIME_STEP 64

// entree point of the controller
int main(int argc, char **argv)
{
  // initialise the Webots API
  wb_robot_init();

  // internal variables
  int i;
  bool avoid_obstacle_counter = 0;
  
  // initialise distance sensors
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i=0; i<2 ; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  // initialise servos
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {
    "wheel1", "wheel2", "wheel3", "wheel4"
  };
  for (i=0; i<4 ; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_servo_set_position(wheels[i], INFINITY);
  }
  
  // feedback loop
  while (wb_robot_step(TIME_STEP) != -1) {
    
    // init speeds
    double left_speed  = 1.0;
    double right_speed = 1.0;
    
    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      left_speed  =  1.0;
      right_speed = -1.0;
    }
    else {
      // read sensors outputs
      double ds_values[2];
      for (i=0; i<2 ; i++)
        ds_values[i] = wb_distance_sensor_get_value(ds[i]);
      
      // increase counter in case of obstacle
      if (ds_values[0] < 950.0 ||
          ds_values[1] < 950.0)
        avoid_obstacle_counter = 100;
    }
    
    // write actuators inputs
    wb_servo_set_velocity(wheels[0], left_speed);
    wb_servo_set_velocity(wheels[1], right_speed);
    wb_servo_set_velocity(wheels[2], left_speed);
    wb_servo_set_velocity(wheels[3], right_speed);
  }
  
  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
