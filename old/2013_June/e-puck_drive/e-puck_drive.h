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
#ifndef EPUCK_DRIVE
#define EPUCK_DRIVE

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "webots/robot.h"
#include "webots/differential_wheels.h"
#include "webots/distance_sensor.h"
#include "webots/led.h"
#include "webots/supervisor.h"  
#include "webots/receiver.h"
#include "webots/emitter.h"
#include "webots/gps.h"
#include "webots/camera.h"

// Global defines
#define TRUE 1
#define FALSE 0
#define LEFT 0
#define RIGHT 1
#define SIMULATION 0            // for robot_get_mode() function
#define REALITY 2               // for robot_get_mode() function
//Simulation time step
#define TIME_STEP 128         // [ms]

// 8 IR proximity sensors
#define NB_DIST_SENS 8

//Evalutaion duration of one individual [ms]
#define TRIAL_DURATION 51200

/*
//Parameters of the neural controller
*/
//Number of inputs, 2 inputs correspond to the 2 front IR sensors,4 to the 4 front IR sensors, 6 to the front IR sensors and the sides and 8 inputs correspond to all the IR sensors of the e-puck. 
#define NB_INPUTS 4
//Number of hidden neurons
#define NB_HIDDEN_NEURONS 0
//Number of output neurons. The outputs here correspond to the speed of the left and right wheels of the e-puck.
#define NB_OUTPUTS 2

//Number of weights which need to be encoded in the neural controller genome.
//NB_WEIGHTS should be equal to NB_GENES in evolution.h ! (important when NB_HIDDEN_NEURONS > 0)
#if (NB_HIDDEN_NEURONS==0)
  #define NB_WEIGHTS NB_INPUTS*NB_OUTPUTS+NB_OUTPUTS
#else
  #define NB_WEIGHTS (NB_INPUTS+1)*NB_HIDDEN_NEURONS+(NB_HIDDEN_NEURONS+1)*NB_OUTPUTS
#endif

//Wheel controllers receive commands between -SPEED_RANGE and SPEED_RANGE
#define SPEED_RANGE 1000

#define PS_RANGE 1100
#define OBSTACLE_THRESHOLD 135
#define NB_FS 3
//Initial position of the robot at each time step..
//static double robot_initial_position[3] = {0.0, 0.0, 0.0};

//static WbNodeRef robot;
WbDeviceTag ps[NB_DIST_SENS];	// proximity sensors
WbDeviceTag fs[NB_FS];
double ps_offset[NB_DIST_SENS]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
static int mode;
static double weights[NB_WEIGHTS];
static WbDeviceTag receiver;      // to receive coordinate information
static WbDeviceTag camera; 
static int step;
static double fitness;
//static double robot_initial_position[3]={0.0,0.0,0.4};
//static int TIME_STEP;
const double PS_OFFSET_SIMULATION[NB_DIST_SENS] = {35,35,35,35,35,35,35,35};
// *** TO BE ADAPTED TO YOUR ROBOT *** 
const double PS_OFFSET_REALITY[NB_DIST_SENS] = {0,0,0,0,0,0,0,0}; 

// LEDs
#define NB_LEDS    8
WbDeviceTag led[NB_LEDS];
static WbDeviceTag emitter;
//static double *buffer;

// GPS sensor
WbDeviceTag gps;

//Functions

// Default function called at initialisation
static int reset();
// Called evrey time-step
static int run(int ms);
// Trial to test the robot's behavior (according to NN) in the environment
double run_trial();
// Computes the fitness of the robot at each timestep, edit this!!
double compute_fitness(int speed[2], double maxIRActivation);
// Run the neural network
void run_neural_network(double* inputs, double* outputs);

#endif //EPUCK_DRIVE
