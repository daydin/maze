/*
 * File:          my_controller.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include "webots/robot.h"
#include "my_controller.h"
/*
 * You may want to add macros here.
 */


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();
  reset();
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(time_step) != -1) {
    
    /* 
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    
    wb_differential_wheels_set_speed(100.0,100.0);
    save_data();
    /* Process sensor data here */
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
  };
  
  /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  close_files();
  wb_robot_cleanup();
  
  return 0;
}

void reset(void){
  emitter = wb_robot_get_device("emitter");
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,time_step);
  
  int i;
  
    for(i=0; i<NB_DIST_SENS; i++){
      sprintf(fName,"sensor%d.txt",i);
      f[i]= fopen(fName,"w+");
    }
  
    for(i=0; i<NB_FLOOR_SENS; i++){
      sprintf(fName,"fl_sensor%d.txt",i);
      fps[i]= fopen(fName,"w+");
    }
  
  char text[5]="led0";
  
  for(i=0; i<NB_LEDS; i++){
    led[i]=wb_robot_get_device(text);
    wb_led_set(led[i],on);
    text[3]++;
  }
  
  char t[4]="ps0";
  ps[0]=wb_robot_get_device(t);
  t[2]='7';
  ps[1]=wb_robot_get_device(t);
  t[2]='1';
  ps[2]=wb_robot_get_device(t);
  t[2]='6';
  ps[3]=wb_robot_get_device(t);
  t[2]='2';
  ps[4]=wb_robot_get_device(t);
  t[2]='5';
  ps[5]=wb_robot_get_device(t);
  t[2]='3';
  ps[6]=wb_robot_get_device(t);
  t[2]='4';
  ps[7]=wb_robot_get_device(t);  
  
  for(i=0; i<NB_DIST_SENS; i++){
    wb_distance_sensor_enable(ps[i],time_step);
  }
  
  t[0]='f';
  t[2]='0';


  fs[0]=wb_robot_get_device(t);
  t[2]='1';
  fs[1]=wb_robot_get_device(t);
  t[2]='2';
  fs[2]=wb_robot_get_device(t);

  for(i=0; i<NB_FLOOR_SENS; i++){
    wb_distance_sensor_enable(fs[i],time_step);
  } 


  //if this sensor is active increase the fitness
  return;
}

void save_data(void){
  double sv[NB_DIST_SENS];
  double floor_sens[NB_FLOOR_SENS];
  int i;
  const double *pos;
  pos = wb_gps_get_values(gps);
  double x = pos[0];
  double y = pos[1];
  double z = pos[2];
  printf("x: %f y: %f z: %f \n", x,y,z);
  
    for(i=0; i<NB_DIST_SENS; i++){
      sv[i]=wb_distance_sensor_get_value(ps[i]);
    }
  
    for(i=0; i<NB_FLOOR_SENS; i++){
      floor_sens[i]=wb_distance_sensor_get_value(fs[i]);
    }

    for(i=0; i<NB_DIST_SENS; i++){  
      assert(f[i]!=NULL);  
      fprintf(f[i],"%f\t%f\n", z,sv[i]);
      fflush(f[i]);
    }


    for(i=0; i<NB_FLOOR_SENS; i++){  
      assert(fps[i]!=NULL);   
      fprintf(fps[i],"%f\n",floor_sens[i]);
      fflush(fps[i]);
    }
    

  return;
}

void close_files(void){
  int i;
  for(i=0; i<NB_DIST_SENS; i++){
    fclose(f[i]);
  }
  
    for(i=0; i<NB_FLOOR_SENS; i++){
    fclose(fps[i]);
  }
  return;
}