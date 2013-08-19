/***************************************************************************

  e-puck_drive -- E-puck robot evolves to drive in an arena and avoid walls
  This program is free software; any publications presenting results
  obtained with this program must mention it and its origin. You
  can redistribute it and/or modify it under the terms of the GNU
  General Public License as published by the Free Software
  Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
  USA.

***************************************************************************/

#include "drive.h"

int main(int argc, char **argv)
{
  //robot_live(reset);
 // robot_run(run);

  /* necessary to initialize webots stuff */
  wb_robot_init();
  //TIME_STEP = (int)wb_robot_get_basic_time_step();
  int n=reset();
  printf("did reset: %d\n",n);


  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    run(TIME_STEP);
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

//
// Reset the robot controller and initiate the sensors and emitter/receiver
//
static int reset()
{
  int i;
  mode =1;
  emitter = wb_robot_get_device("emitter");
  //no emitter_enable here on purpose!

  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  camera= wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  char text[5]="led0";
  for(i=0;i<NB_LEDS;i++) {
    led[i]=wb_robot_get_device(text); // get a handler to the sensor
    text[3]++; // increase the device name to "ps1", "ps2", etc.
  }

  text[0]='p';
  text[1]='s';
  text[3]='\0';

  text[2]='0';
  ps[0] = wb_robot_get_device(text); // proximity sensors
  text[2]='7';
  ps[1] = wb_robot_get_device(text); // proximity sensors
  text[2]='1';
  ps[2] = wb_robot_get_device(text); // proximity sensors
  text[2]='6';
  ps[3] = wb_robot_get_device(text); // proximity sensors
  text[2]='2';
  ps[4] = wb_robot_get_device(text); // proximity sensors
  text[2]='5';
  ps[5] = wb_robot_get_device(text); // proximity sensors
  text[2]='3';
  ps[6] = wb_robot_get_device(text); // proximity sensors
  text[2]='4';
  ps[7] = wb_robot_get_device(text); // proximity sensors

  // Enable proximity and floor sensors
  for(i=0;i<NB_DIST_SENS;i++) {
    wb_distance_sensor_enable(ps[i],TIME_STEP);
    printf("ps[%d] is active\n",i);
  }
/*
  //text[0]='f';
  //text[2]='0';


  //fs[0]=wb_robot_get_device(text);
  //text[2]='1';
  //fs[1]=wb_robot_get_device(text);
  //text[2]='2';
  //fs[2]=wb_robot_get_device(text);


  //wb_distance_sensor_enable(fs[1],TIME_STEP); //enable center floor sensor
  //printf("fs[%d] is active\n",1);
  //if this sensor is active increase the fitness

  // Enable GPS sensor to determine position
  //gps=wb_robot_get_device("gps");
  //wb_gps_enable(gps, TIME_STEP);
  //printf("gps is active\n");
*/
  return 1;
}

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
static int run(int ms)
{
  int i;
  const unsigned char *image;

  if (mode!=wb_robot_get_mode())
  {
    mode = wb_robot_get_mode();
    if (mode == SIMULATION) {
      for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_SIMULATION[i];
      puts("Switching to SIMULATION.\n\n");
    }
    else if (mode == REALITY) {
      for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_REALITY[i];
      puts("\nSwitching to REALITY.\n\n");
    }
  }

  // if we're testing a new genome, receive weights and initialize trial
  if (step == 0) {
    int n = wb_receiver_get_queue_length(receiver);
    printf("queue length %d\n",n);
    //wait for new genome
    if (n) {
      const double *genes = (double *) wb_receiver_get_data(receiver);
      //set neural network weights
      for (i=0;i<NB_WEIGHTS;i++) {
        weights[i]=genes[i];
        printf("wt[%d]: %g\n",i,weights[i]);
      }
      wb_receiver_next_packet(receiver);
    }


    else {

      return TIME_STEP;}


    fitness = 0;
  }

  step++;
  printf("Step: %d\n", step);

  image=wb_camera_get_image(camera);
  int *grey = (int *)malloc(sizeof(int)*width);


  for (i = 0; i < width; i++) {
    grey[i] = 255-wb_camera_image_get_grey(image, width, i, 0);
  }
  //send this row of grayscale values to the reward searcher!

  if(step < TRIAL_DURATION/(double)TIME_STEP) {
    //drive robot
    fitness+=run_trial();
    //send message with current fitness
    double msg[2] = {fitness, 0.0};
    wb_emitter_send(emitter, (void *)msg, 2*sizeof(double));
    reward_find(grey);
  }
  else {
    //stop robot
    wb_differential_wheels_set_speed(0, 0);
    //send message to indicate end of trial
    double msg[2] = {fitness, 1.0};
    wb_emitter_send(emitter, (void *)msg, 2*sizeof(double));
    //reinitialize counter
    step = 0;
  }

  return TIME_STEP;
  return TIME_STEP;
}

//
// Trial to test the robot's behavior (according to NN) in the environment
//
double run_trial() {

  double inputs[NB_INPUTS+NB_FS+NB_HIDDEN_NEURONS+NB_OUTPUTS];
  double outputs[NB_OUTPUTS];

  //Get position of the e-puck
  //static double position[3]={0.0,0.0,0.0};
  //const double *gps_matrix = wb_gps_get_values(gps);
  //position[0] = gps_matrix[0];//gps_position_x(gps_matrix);
  //position[1] = gps_matrix[2];//gps_position_z(gps_matrix);
  //position[2] = gps_matrix[1];//gps_position_y(gps_matrix);

  //Speed of the robot's wheels within the +SPEED_RANGE and -SPEED_RANGE values
  int speed[2]={0,0};
  //Maximum activation of all IR proximity sensors [0,1]
  double maxIRActivation = 0;

  //get sensor data, i.e., NN inputs
  int i, j;
  for(j=0;j<NB_INPUTS;j++) {
    printf("sensor %d reads %g\n",j,wb_distance_sensor_get_value(ps[j]));
    inputs[j]=(((double)wb_distance_sensor_get_value(ps[j])-ps_offset[j])<0)?0:((double)wb_distance_sensor_get_value(ps[j])-ps_offset[j])/((double)PS_RANGE);
    //get max IR activation
    if(inputs[j]>maxIRActivation) maxIRActivation=inputs[j];
    printf("sensor #: %d, sensor read: %g, maxIR: %g\n", j, inputs[j], maxIRActivation);
  }
  //printf("max IR activation: %f\n", maxIRActivation);

  // Run the neural network and computes the output speed of the robot
  run_neural_network(inputs, outputs);

  speed[LEFT]  = SPEED_RANGE*outputs[0];
  speed[RIGHT] = SPEED_RANGE*outputs[1];
  printf("speed l: %d, speed r: %d\n",speed[LEFT],speed[RIGHT]);

  // If you are running against an obstacle, stop the wheels
 if ( maxIRActivation > 0.09 ) {
    speed[LEFT]=0;
    speed[RIGHT]=0;
    printf("Wheels stopped.\n");
  }

  // Set wheel speeds to output values
  wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]); //left, right
  printf("Set wheels to outputs.\n");

  // Stop the robot if it is against an obstacle
  //this seems unnecessary, just do it above
  for (i=0;i<NB_DIST_SENS;i++) {
    int tmpps=(((double)wb_distance_sensor_get_value(ps[i])-ps_offset[i])<0)?0:((double)wb_distance_sensor_get_value(ps[i])-ps_offset[i]);

    if(OBSTACLE_THRESHOLD<tmpps ) {// proximity sensors
      //printf("%d \n",tmpps);
      speed[LEFT]  = 0;
      speed[RIGHT] = 0;
      break;
    }
  }

  return compute_fitness(speed, maxIRActivation);
  return compute_fitness(speed, maxIRActivation);
}

//
// Computes the fitness of the robot at each timestep.
//
double compute_fitness(int speed[2], double maxIRActivation)
{



  fitness= (speed[0]*speed[1]/pow(SPEED_RANGE,2))/(maxIRActivation+1);

  /*
  Input parameters:

  speed[0] //leftSpeed
  speed[1] //rightSpeed

  SPEED_RANGE //all speed outputs are within the +SPEED_RANGE and -SPEED_RANGE values

  position[0] //current X coordinate
  position[1] //current Y coordinate

  robot_initial_position[0] //initial X coordinate
  robot_initial_position[1] //initial Y coordinate

  maxIRActivation //highest IR sensor activation [0,1]
  */
  printf("fitness: %g\n", fitness);
  return fitness;
}

//
// Run the neural network
//
//void run_neural_network(double* inputs, double* outputs)
//{
//  int i,j;
//  int weight_counter=0;
//
//  if(NB_HIDDEN_NEURONS>0){
//    double hidden_neuron_out[(int)NB_HIDDEN_NEURONS];
//
//    for(i=0;i<NB_HIDDEN_NEURONS;i++) {
//      double sum=0;
//      for(j=0;j<NB_INPUTS;j++) {
//        sum+=inputs[j]*weights[weight_counter];
//        weight_counter++;
//      }
//      hidden_neuron_out[i]=tanh(sum+weights[weight_counter]);
//      weight_counter++;
//    }
//
//    for(i=0;i<NB_OUTPUTS;i++) {
//      double sum=0;
//      for(j=0;j<NB_HIDDEN_NEURONS;j++)
//      {
//        sum+=hidden_neuron_out[j]*weights[weight_counter];
//        weight_counter++;
//      }
//      outputs[i]=tanh(sum+weights[weight_counter]);
//      weight_counter++;
//    }
//  }
//  else
//  {
//    for(i=0;i<NB_OUTPUTS;i++) {
//      double sum=0.0;
//      for(j=0;j<NB_INPUTS;j++)
//      {
//        sum+=inputs[j]*weights[weight_counter];
//        weight_counter++; //useful to access the bias weight elements
//      }
//      outputs[i]=tanh(sum+weights[weight_counter]);
//      weight_counter++;
//    }
//  }
//
//  return;
//}

void run_neural_network(double* inputs, double* outputs)
{

    //double activation[NB_TOT_OUT]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int i,j;
    int weight_counter=0;

            fprintf(f3a,"Network step\n");
            fflush(f3a);

    //double hidden_neuron_out[(int)NB_HIDDEN_NEURONS];

    for(i=0;i<NB_TOT_OUT;i++) {
      double sum=0.0;
      double sum_tmp=0.0; //previous time step inputs


    for(j=0;j<NB_IN;j++)
    {
       fprintf(f1a,"input[%d]:%lf\n",j,inputs[j]);
        fflush(f1a);
        sum+=inputs[j]*weights[weight_counter];
        weight_counter++;
    } //weighted sum of 11

      outputs[i]+= ((sum-outputs[i])*(1/weights[weight_counter])); //11th weight is tau
        fprintf(f3a,"out[%d]:%lf\n", i, outputs[i]);
        fflush(f3a);

    //+weights[weight_counter];
      weight_counter++;

      activation[i]=sigmoid(outputs[i]-(weights[weight_counter])); //outputs are yi: activations
        fprintf(f3a,"activation[%d]:%lf\n", i,activation[i]);
        fflush(f3a);


        weight_counter++;
    }

        fflush(f3a);

        for(i=0;i<NB_TOT_OUT;i++){
            inputs[i+5]=activation[i];
        }


  return;
}

double sigmoid(double x){

    double tmp;

    tmp= 1+exp(-x);

    return 1/tmp;

}



