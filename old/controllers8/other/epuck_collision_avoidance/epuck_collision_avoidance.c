#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>

// time in [ms] of a simulation step
#define TIME_STEP 64

// entree point of the controller
int main(int argc, char **argv)
{
  // initialise the Webots API
  wb_robot_init();

  // internal variables
  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  // initialise devices
  for (i=0; i<8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  // feedback loop
  while (1) {
    // step simulation
    int delay = wb_robot_step(TIME_STEP);
    if (delay == -1) // exit event from webots
      break;
  
    // read sensors outputs
    double ps_values[8];
    for (i=0; i<8 ; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    
    // detect obsctacles
    bool left_obstacle =
      ps_values[0] > 100.0 ||
      ps_values[1] > 100.0 ||
      ps_values[2] > 100.0;
    bool right_obstacle =
      ps_values[5] > 100.0 ||
      ps_values[6] > 100.0 ||
      ps_values[7] > 100.0;

    // init speeds
    double left_speed  = 500;
    double right_speed = 500;
    
    // modify speeds according to obstacles
    if (left_obstacle) {
      // turn right
      left_speed  -= 500;
      right_speed += 500;
    }
    else if (right_obstacle) {
      // turn left
      left_speed  += 500;
      right_speed -= 500;
    }
    
    // write actuators inputs
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
  
  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
