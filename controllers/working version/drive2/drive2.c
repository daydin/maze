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

#include "drive2.h"

int main(int argc, char **argv)
{
	//robot_live(reset);
	// robot_run(run);

	/* necessary to initialize webots stuff */
	wb_robot_init();
	
	int n = reset();
//	printf("did reset: %d\n",n);


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
		run();
		/* Process sensor data here */

		/*
		 * Enter here functions to send actuator commands, like:
		 * wb_differential_wheels_set_speed(100.0,100.0);
		 */
	};

	/* Enter your cleanup code here */
	end();
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
	text[2]='6';
	ps[2] = wb_robot_get_device(text); // proximity sensors
	text[2]='5';
	ps[3] = wb_robot_get_device(text);
	text[2]='4';
	ps[4] = wb_robot_get_device(text);
	text[2]='3';
	ps[5] = wb_robot_get_device(text);
	text[2]='2';
	ps[6] = wb_robot_get_device(text);
	text[2]='1';
	ps[7] = wb_robot_get_device(text);

	// Enable proximity and floor sensors
	for(i=0;i<NB_DIST_SENS;i++) {
		wb_distance_sensor_enable(ps[i],TIME_STEP);
		//printf("ps[%d] is active\n",i);
	}

	//get handles for all the floor sensors
	text[0]='f';
	text[2]='1';



	fs[0]=wb_robot_get_device(text); //note that fs[0] is a 1x1 array that holds the center floor sensor DeviceTag


	//enable the center fs just to try

	wb_distance_sensor_enable(fs[0],TIME_STEP); //enable center floor sensor
	//printf("fs[%d] is active\n",1);
	//if this sensor is active increase the fitness

	// open files for writing
	//pF1=fopen("act_in.txt","w+");
//	pF2=fopen("decoded_pop.txt","w+");
//	pF3=fopen("nn_values.txt","w+");

	return 1;
}

static int run(int ms)
{
  int i;


  if (mode!=wb_robot_get_mode())
  {
    mode = wb_robot_get_mode();
    if (mode == SIMULATION) {
      for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_SIMULATION[i]; //offset/baseline noise is ~35 in test world
      puts("Switching to SIMULATION.\n");
    }
    else if (mode == REALITY) {
      for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_REALITY[i];
      puts("\nSwitching to REALITY.\n");
    }
  }

  // if we're testing a new genome, receive weights and initialize trial
  if (step == 0) {
    int n = wb_receiver_get_queue_length(receiver);
   // printf("queue length %d\n",n);
    reset_neural_network();
    //wait for new genome
    if (n) {
      const _Bool *genes_bin = (_Bool *) wb_receiver_get_data(receiver);
      //decode obtained boolean genes into real numbers and set the NN weights.

      double *genes= popDecoder(genes_bin); //is const necessary here?


      //set neural network weights
      for (i=0;i<NB_WEIGHTS;i++) {
        weights[i]=genes[i];
      //  printf("wt[%d]: %g\n",i,weights[i]);
      }
      wb_receiver_next_packet(receiver);
    }


    else {

      return TIME_STEP;}


    fitness = 0;
  }

  step++;
  //printf("Step: %d\n", step);

  //first trial has 360 steps available, subsequent trials have 180.
  //At each epoch, first 360 steps (e.g. the first trial) have the number step_max=360 associated with them.

  if(step < TRIAL_STEPS) {
        //try the robot
           fitness+= run_trial();
	//printf("Fitness in step %d: %g\n",step,fitness);
	double msg[2] = {fitness, 0.0};
	wb_emitter_send(emitter, (void *)msg, 2*sizeof(double));
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

double* popDecoder (const _Bool *genome_binary){
	int j=0,flag,k;
	int val[2];
	_Bool bin_array[BIT_SIZE];
	int pop_int[NB_WEIGHTS];

	//_Bool pop_bin[POP_SIZE][GENOME_LENGTH]; //10-by-390

	for(j=0; j<NB_WEIGHTS; j++){//split genome into 5 bit strings, set option flag for each segment of the genome
		if(j<NB_ONLY_WT){
			flag=0;
		}else if(j>=NB_ONLY_WT && j<NB_ONLY_WT+NB_TOT_TC){
			flag=1;
		}else if(j>=NB_ONLY_WT+NB_TOT_TC && j<NB_WEIGHTS){
			flag=2;
		}else {
			return NULL;
		}
		for(k=0;k<BIT_SIZE;k++){
			bin_array[k]=genome_binary[(BIT_SIZE*j)+k];
		}
//		for(k=0;k<BIT_SIZE;k++){
//			fprintf(pF2,"%d",(bin_array[k])?1:0);
//			fflush(pF2);
//		}
		val[0]=0;
		val[1]=0;		
		decodeOne(val,BIT_SIZE,bin_array,flag); //obtain your integer in val array, first is sign bit and second is the number
		pop_int[j]= (1==val[0])?(-val[1]):val[1];
//		fprintf(pF2,"\npop_int[%d]:%d\n",j,pop_int[j]);
//		fflush(pF2);
		genome_real[j]=itor(val, flag);
	}

//	fflush(pF2);
	return genome_real;
}


void decodeOne(int *sum, int bits, _Bool* binary, int option){ //option is probably redundant here, remove it later
	//take in pointer to bit array, calculate integer value
	if(0==option || 2==option){ //if you have negatives, use the first bit as a sign bit!
		int k;
		int carry=0;

		//if all bits processed, return the sign flag and the value
		if(0==bits){
			return;
		}

		if(1==binary[0]){
			//if your num is negative: subtract 1 and flip bits, calc number, number = -number

			carry=1;
			sum[0]=1;
			for(k=BIT_SIZE-1; k>-1; k--){
				if(binary[k]&carry){
					binary[k]=binary[k]^carry;
					carry=0;
				}else{
					binary[k]=binary[k]^carry;
				}
			}

			for(k=BIT_SIZE-1; k>-1; k--){
				binary[k] = sum[0]^(binary[k]); //flip all the bits and repopulate the array with compl. nums
				// printf("bin[%d]:%d\n",k,binary[k]);
			}
		}

		sum[1]+=(int)pow(2.0,(double)BIT_SIZE-(bits))*(int)binary[bits-1];
		//printf("sum %d bin[%d]:%d\n",sum[1], bits-1,binary[bits-1]);
		--bits;
		decodeOne(sum, bits, binary,option);
	}else{
		if(0==bits){
			return;
		}
		sum[1]+=(int)pow(2.0,(double)BIT_SIZE-(bits))*(int)binary[bits-1];
		//printf("sum %d bin[%d]:%d\n",sum[1], bits-1,binary[bits-1]);
		--bits;
		decodeOne(sum, bits, binary,option);

	}
}

double itor(int* value, int flag){

	if(0==flag){
		if(1==value[0]){
			return -(GENE_MIN + ((double)value[1]/(double)INT_MAX_PN)*(GENE_MAX-GENE_MIN));
		}else return (GENE_MIN + ((double)value[1]/(double)INT_MAX_PN)*(GENE_MAX-GENE_MIN));
	}
	else if(1==flag){
		return (TAU_MIN + ((double)value[1]/(double)INT_MAX_P)*(TAU_MAX-TAU_MIN));
	}
	else if(2==flag){
		if(1==value[0]){
			return -(BIAS_MIN + ((double)value[1]/(double)INT_MAX_PN)*(BIAS_MAX-BIAS_MIN));
		}else return (BIAS_MIN + ((double)value[1]/(double)INT_MAX_PN)*(BIAS_MAX-BIAS_MIN));
	}else {
		printf("Something is wrong with int to decimal conversion.\n");
		return 900.0;
	}
	return 900.0;
}

void end(){
//	fclose(pF3);
//	fclose(pF2);
	//fclose(pF1);
	return;
}

//
// Trial to test the robot's behavior (according to NN) in the environment
//

int clip_value(double val, int limit){
int n = round(val);
if (n>limit) return limit;
else if (-limit>n) return -limit;
else return val;

}
double run_trial() {

	int speed[2]={0,0};


	//Maximum activation of all IR proximity sensors [0,1]
	//double maxIRActivation = 0;

	//get sensor data, i.e., NN inputs
	int j;
	for(j=0;j<NB_DIST_SENS;j++) {
		//printf("sensor %d reads %g\n",j,wb_distance_sensor_get_value(ps[j]));
		temp[j]=(((double)wb_distance_sensor_get_value(ps[j])-ps_offset[j])<0)?0:(((double)wb_distance_sensor_get_value(ps[j])-ps_offset[j])/((double)PS_RANGE));
	}

	for(j=0;j<NB_PS;j++) {
		//printf("sensor %d reads %g\n",j,wb_distance_sensor_get_value(ps[j]));
		inputs[j]=(temp[2*j]+temp[(2*j)+1])/2.0;

	}
            inputs[4]= wb_distance_sensor_get_value(fs[0])-fs_offset[0];
            //printf("floor sensor:%g\n",inputs[4]);
            
          
	// Run the neural network and computes the output speed of the robot
	run_neural_network(inputs, outputs);

	speed[LEFT]  =  clip_value(outputs[0],max_speed); //scale  outputs between 0 and 1
	speed[RIGHT] =  clip_value(outputs[1],max_speed);
	//printf("L: %d; R: %d\n",speed[LEFT],speed[RIGHT]);

	// Set wheel speeds to output values
	wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]); //left, right
	//printf("Set wheels to outputs.\n");

	return compute_fitness(inputs[4]);
	return compute_fitness(inputs[4]);
}


// Computes the fitness of the robot at each timestep.

double compute_fitness(double val)
{
	double fit= (val<100)?1:0;
	//printf("fitness: %g\n", fitness);
	return fit;
}

void run_neural_network(double* inputs, double* outputs)
{

	//double activation[NB_TOT_OUT]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	int i,j;
	int weight_counter=0;
	int tau_counter=0;
	int bias_counter=0;
	int tmp;

//	fprintf(pF3,"\n Network step\n");
//	fflush(pF3);

	//double hidden_neuron_out[(int)NB_HIDDEN_NEURONS];

	for(i=0;i<NB_TOT_OUT;i++) {
		double sum=0.0;
		tmp=weight_counter;

		for(j=tmp;j<NB_IN+tmp;j++)
		{
			//fprintf(pF1,"input[%d]:%lf\n",j-tmp,inputs[j-tmp]);
//			fflush(pF1);
			sum+=inputs[j-tmp]*weights[j];
			++weight_counter;
		} //weighted sum of 11

		outputs[i]+= ((sum-outputs[i])*(1/weights[tau_counter+66])); //11th weight is tau
//		fprintf(pF3,"out[%d]:%lf\n", i, outputs[i]);
//		fflush(pF3);


		++tau_counter;

		activation[i]=sigmoid(outputs[i]-(weights[bias_counter+72])); //outputs are yi: activations
//		fprintf(pF3,"activation[%d]:%lf\n", i,activation[i]);
//		fflush(pF3);


		++bias_counter;
	}

	fflush(pF3);

	for(i=0;i<NB_TOT_OUT;i++){
		inputs[i+5]=activation[i];
	}

	//fflush(pF1);
	fflush(pF3);

	return;
}

void reset_neural_network(){
//	static double inputs[NB_IN]; //11
//	static double outputs[NB_TOT_OUT]; //6
//	static double activation[NB_TOT_OUT]; //6 activations

	int i;

	for(i=0;i<NB_IN;i++){
		inputs[i]=0.0;
	}
	for(i=0;i<NB_TOT_OUT;i++){
		outputs[i]=0.0;
		activation[i]=0.0;
	}
	return;
}

double sigmoid(double x){

	double tmp;

	tmp= 1+exp(-x);

	return 1/tmp;

}



