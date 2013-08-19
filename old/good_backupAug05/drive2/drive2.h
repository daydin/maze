/***************************************************************************

  e-puck_drive.h -- E-puck robot evolves to drive in an arena and avoid
  walls.
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
#ifndef DRIVE2
#define DRIVE2

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "../../old/2013_June/evolution/webots/robot.h"
#include "../../old/2013_June/evolution/webots/differential_wheels.h"
#include "../../old/2013_June/evolution/webots/distance_sensor.h"
#include "../../old/2013_June/evolution/webots/led.h"
#include "../../old/2013_June/evolution/webots/supervisor.h"
#include "../../old/2013_June/evolution/webots/receiver.h"
#include "../../old/2013_June/evolution/webots/emitter.h"
#include "../../old/2013_June/evolution/webots/gps.h"
#include "../../old/2013_June/evolution/webots/camera.h"

// Global defines
#define TRUE 1
#define FALSE 0
#define LEFT 0
#define RIGHT 1
#define SIMULATION 0            // for robot_get_mode() function
#define REALITY 2               // for robot_get_mode() function
//Simulation time step
#define TIME_STEP 128         // [ms]


//Evalutaion duration of one individual [ms]
//#define WHEEL_TURNS_1 360
//#define WHEEL_TURNS_2 180

#define TRIAL_STEPS 4320 //total of 552960 ms
//#define TRIAL_DURATION_1 WHEEL_TURNS_1*TIME_STEP
//#define TRIAL_DURATION_2 WHEEL_TURNS_2*TIME_STEP

//#define EPOCHS 4
//#define TRIALS 5
#define NB_LEDS    8
#define NB_DIST_SENS 8
/*
//Parameters of the neural controller
 */
//Number of inputs, 2 inputs correspond to the 2 front IR sensors,4 to the 4 front IR sensors, 6 to the front IR sensors and the sides and 8 inputs correspond to all the IR sensors of the e-puck.
#define NB_PS 4
//Number of hidden neurons
#define NB_HIDDEN_NEURONS 4
//Number of output neurons. The outputs here correspond to the speed of the left and right wheels of the e-puck.
#define NB_OUTPUTS 2
#define NB_FS 1 //one floor sensor
#define NB_BIAS 1 //one bias parameter per neuron
#define NB_TC 1 //one time const per neuron
#define NB_TOT_OUT (NB_OUTPUTS+NB_HIDDEN_NEURONS) //6 total neurons in the "output" layer at time t. Only 2 of these actuate wheels.
#define NB_TOT_TC (NB_TC*NB_TOT_OUT) //6 time constants for these 6 "output" neurons
#define NB_TOT_BIAS (NB_BIAS*NB_TOT_OUT) // 6 biases
#define NB_IN_S (NB_PS+NB_FS) //5 sensor inputs in total
#define NB_IN  (NB_PS+NB_FS+NB_TOT_OUT) //11 inputs total including outputs from previous time step
#define NB_WEIGHTS ((NB_TOT_OUT)*NB_IN)+(2*(NB_TOT_OUT)) //78
#define NB_ONLY_WT (NB_TOT_OUT)*NB_IN //66
#define GENOME_LENGTH 390

//Number of weights which need to be encoded in the neural controller genome.
//NB_WEIGHTS should be equal to NB_GENES in evolution.h ! (important when NB_HIDDEN_NEURONS > 0)

#define INT_MAX_PN 15
#define INT_MAX_P 31
#define BIT_SIZE 5


#define GENE_MIN -5
#define GENE_MAX 5
#define BIAS_MIN -1
#define BIAS_MAX 1
#define TAU_MIN 1
#define TAU_MAX 50

//#if (NB_HIDDEN_NEURONS==0)
//  #define NB_WEIGHTS NB_PS*NB_OUTPUTS+NB_OUTPUTS
//#else
//  #define NB_WEIGHTS ((NB_PS+NB_FS+NB_HIDDEN_NEURONS+NB_OUTPUTS)*(NB_OUTPUTS+NB_HIDDEN_NEURONS))+(NB_TC)+(NB_BIAS)
//#endif

//Wheel controllers receive commands between -SPEED_RANGE and SPEED_RANGE
#define SPEED_RANGE 1000
#define PS_RANGE 1100
#define OBSTACLE_THRESHOLD 100


//Initial position of the robot at each time step..
//static double robot_initial_position[3] = {0.0, 0.0, 0.0};

//static WbNodeRef robot;
WbDeviceTag ps[NB_DIST_SENS];	// proximity sensors
WbDeviceTag fs[NB_FS];
WbDeviceTag led[NB_LEDS];
WbDeviceTag gps;
//static WbNodeRef pattern;
//static WbFieldRef pattern_rotation;

static WbDeviceTag receiver;      // to receive coordinate information
//static WbDeviceTag camera;
static WbDeviceTag emitter;
//static double *buffer;
// GPS sensor


int height, width;

//static double robot_initial_position[3]={0.0,0.0,0.4};
//static int TIME_STEP;
double ps_offset[NB_DIST_SENS]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double fs_offset[NB_FS]={400.0};
static int mode;
static int step;

static double fitness;
static double genome_real[NB_WEIGHTS];
static double weights[NB_WEIGHTS];//78
static double temp[NB_DIST_SENS];//to combine sensor readouts use this temp array
static double inputs[NB_IN]; //11
static double outputs[NB_TOT_OUT]; //6
static double activation[NB_TOT_OUT]; //6 activations
const double PS_OFFSET_SIMULATION[NB_PS] = {35,35,35,35};
// *** TO BE ADAPTED TO YOUR ROBOT ***
const double PS_OFFSET_REALITY[NB_PS] = {0,0,0,0};

FILE *pF1, *pF2, *pF3, *pF4, *pF5;

static double pi= 3.14159;
static int epoch;
static int reset_position;
static int step_max;
static int tmp;

//Functions

// Default function called at initialisation
static int reset();
// Called every time-step
static int run();
// Trial to test the robot's behavior (according to NN) in the environment

double* popDecoder(const _Bool*);
// decodes the binary genome of all individuals to assess parameters of the NN
double itor(int*,int);
// needed for decoding -- converts int representation to a real-value
double clip_value(double, double);
void decodeOne(int*,int, _Bool*, int);
// decodes one binary string corresponding to one parameter
double run_trial();
// Computes the fitness of the robot at each timestep, edit this!!
double compute_fitness(double);
// Run the neural network
void run_neural_network(double* inputs, double* outputs);
void reset_neural_network();

double sigmoid(double x);
void end(void);

#endif //DRIVE
