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
  
  #include "epuck_driveT.h"
  
  int main()
  {
    //initialize robot controller library
    wb_robot_init();
    int i;
    mode=1;
    //get devices handles using epuck device names listed at http://www.cyberbotics.com/dvd/common/doc/webots/guide/section8.1.html
    emitter = wb_robot_get_device("emitter");//emitterepuck?
    //buffer = (float *) emitter_get_buffer(emitter); //returns a pointer to the buffer used by the emitter to send data
   
    receiver = wb_robot_get_device("receiver");
    
    wb_receiver_enable(receiver, TIME_STEP);
    
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
    }
    
    // Enable GPS sensor to determine position
    gps=wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
    
    
    //get device tags 
   // robot_live(reset); //why no die?
    //robot_run(run);
    
  while(wb_robot_step(TIME_STEP) != -1){
              // sense and actuate
    if (mode!=wb_robot_get_mode())
    {
      mode = wb_robot_get_mode();
      if (mode == SIMULATION) {
        for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_SIMULATION[i];
        printf("Switching to SIMULATION.\n\n");
      } 
      else if (mode == REALITY) { 
        for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_REALITY[i];
        printf("\nSwitching to REALITY.\n\n");
      }
    }
    //    printf("\n");
    // if we're testing a new genome, receive weights and initialize trial
    if (step == 0) {
      int n = wb_receiver_get_queue_length(receiver); //get the num of packets waiting
      //wait for new genome
      printf("n= %d\n",n);
      if (n) {
        const double *genes = (double *) wb_receiver_get_data(receiver); 
        //set neural network weights
        for (i=0;i<NB_WEIGHTS;i++) weights[i]=genes[i];
        wb_receiver_next_packet(receiver);
      }
      else return TIME_STEP;
      
      const double *pos = wb_gps_get_values(gps);
      robot_initial_position[0] = pos[0];
      robot_initial_position[1] = pos[1]; //changed types to double!
      fitness = 0;
    }
    
    step++;
    printf("Step: %d/%d\n", step, TRIAL_DURATION/TIME_STEP);
    
      //here is dies
    if(step < (int)(TRIAL_DURATION/(float)TIME_STEP)) {
    //  printf("Stepping time");
      fitness+=run_trial();
      //send message with current fitness
      double msg[2] = {fitness, 0.0};
      wb_emitter_send(emitter, (void *)msg, 2*sizeof(double)); //why twice the byte size = you're sending 2 floats as args. to allow for enough space
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
    
    //return TIME_STEP;
    //return TIME_STEP; //why two?
  }
  
   // wb_robot_cleanup();
    return 0;
  }
  
  
  // Trial to test the robot's behavior (according to NN) in the environment
  //
  double run_trial() {
  
    double inputs[NB_INPUTS];
    double outputs[NB_OUTPUTS];
    
    //Get position of the e-puck
    static double position[3]={0.0,0.0,0.0};
    const double *gps_vals = wb_gps_get_values(gps);
    position[0] = gps_vals[0];
    position[1] = gps_vals[1];
    position[2] = gps_vals[2];
    
    //Speed of the robot's wheels within the +SPEED_RANGE and -SPEED_RANGE values
    int speed[2]={0,0};
    //Maximum activation of all IR proximity sensors [0,1]
    double maxIRActivation = 0;
  
    //get sensor data, i.e., NN inputs
    int i, j;
    for(j=0;j<NB_INPUTS;j++) {
      inputs[j]=((wb_distance_sensor_get_value(ps[j])-ps_offset[j])<0)?0:(wb_distance_sensor_get_value(ps[j])-ps_offset[j])/((double)PS_RANGE);
      //get max IR activation
      if(inputs[j]>maxIRActivation) maxIRActivation=inputs[j];
    }
    //robot_console_printf("max IR activation: %f\n", maxIRActivation);
  
    // Run the neural network and computes the output speed of the robot
    run_neural_network(inputs, outputs); 
    
    speed[LEFT]  = SPEED_RANGE*outputs[0];
    speed[RIGHT] = SPEED_RANGE*outputs[1];
    
    // If you are running against an obstacle, stop the wheels
    if ( maxIRActivation > 0.9 ) { //what's 0.9?
      speed[LEFT]=0;
      speed[RIGHT]=0;
    }
    // Set wheel speeds to output values
    wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]); //left, right
    
    // Stop the robot if it is against an obstacle
    for (i=0;i<NB_DIST_SENS;i++) {
      int tmpps=((wb_distance_sensor_get_value(ps[i])-ps_offset[i])<0)?0:(wb_distance_sensor_get_value(ps[i])-ps_offset[i]);
     // seems like tmpps can at max be 1000...?
      if(OBSTACLE_THRESHOLD<tmpps ) {// proximity sensors 
        //robot_console_printf("%d \n",tmpps);
        speed[LEFT]  = 0;
        speed[RIGHT] = 0;
        break;
      }
    }
    
    return compute_fitness(speed, position, maxIRActivation);
  }
  
  //
  // Computes the fitness of the robot at each timestep.
  //
  double compute_fitness(int speed[2], double position[3], double maxIRActivation)
  {
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
  
    return 0.0;
  }
  
  //
  // Run the neural network
  //
  void run_neural_network(double* inputs, double* outputs)
  {
    int i,j;
    int weight_counter=0;
    
    if(NB_HIDDEN_NEURONS>0){
      double hidden_neuron_out[(int)NB_HIDDEN_NEURONS];
      
      for(i=0;i<NB_HIDDEN_NEURONS;i++) {
        double sum=0;
        for(j=0;j<NB_INPUTS;j++) {
          sum+=inputs[j]*weights[weight_counter];
          weight_counter++;
        }
        hidden_neuron_out[i]=tanh(sum+weights[weight_counter]);
        weight_counter++;
      }
      
      for(i=0;i<NB_OUTPUTS;i++) {
        double sum=0;
        for(j=0;j<NB_HIDDEN_NEURONS;j++)
        {
          sum+=hidden_neuron_out[j]*weights[weight_counter];
          weight_counter++;
        }
        outputs[i]=tanh(sum+weights[weight_counter]);
        weight_counter++;
      }
    }
    else
    {
      for(i=0;i<NB_OUTPUTS;i++) {
        double sum=0.0;
        for(j=0;j<NB_INPUTS;j++)
        {
          sum+=inputs[j]*weights[weight_counter];
          weight_counter++;
        } 
        outputs[i]=tanh(sum+weights[weight_counter]);
        weight_counter++;
      }
    }
  
    return;
  }
  