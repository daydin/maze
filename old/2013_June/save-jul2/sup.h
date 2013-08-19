#ifndef SUP
#define SUP

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../evolution/webots/robot.h"
#include "../evolution/webots/supervisor.h"
#include "../evolution/webots/emitter.h"
#include "../evolution/webots/receiver.h"
//#include "webots/node.h"

//////////////////////////
//Start Editing
//////////////////////////

/*
//Parameters of the genetic algorithm
*/

//Number of individuals in a population
#define POP_SIZE 10

//Range of genes: minimum value
#define GENE_MIN -5
//Range of genes: maximum value
#define GENE_MAX 5

//Proportion of best individuals which get directly copied to the next generation 
#define ELITISM_RATIO 0.1
//Probablity of mutating each weight-value in a genome 
#define MUTATION_PROBABILITY 0.1
//Mutations follow a Box-Muller distribution from the gene with this sigma
#define MUTATION_SIGMA 0.2
//Probablity of having a crossover 
#define CROSSOVER_PROBABILITY 0.5
//Size of genome
//NB_GENES should be equal to NB_WEIGHTS in e-puck_drive.h ! (important when NB_HIDDEN_NEURONS > 0)
#define NB_GENES 10
//Roulette wheel selection or not (truncation selection)
#define ROULETTE_WHEEL 0
//If not using roulette wheel (truncation selection), we need reproduction ratio
#define REPRODUCTION_RATIO 0.4

/*
//Parameter of the simulator
*/

//If 1, evolution takes place. If 0, then the best individual obtained during the previous evolution is tested for an undetermined amount of time.
#define EVOLVING 1 // i.e., !TEST_BEST

//Simulation time step
#define TIME_STEP 128
//////////////////////////
//Stop Editing
//////////////////////////
//Robots and devices
static WbNodeRef robot;
static WbDeviceTag emitter;
static WbDeviceTag receiver;
static WbFieldRef trans_field;
static WbFieldRef rot_field;
static WbFieldRef ctrl_field;
//static double *buffer;
static double robot_initial_position[3]={0.0,0.0,0.4};
//static double robot_xy[2];
//static double robot_rot[4]={0.0,1.0,0.0,0.0};
//static int TIME_STEP;
//Weights for each genome in population.
static double pop[POP_SIZE][NB_GENES];

//Fitness of each individual in a population.
static double fitness[POP_SIZE];
//Population sorted by fitness
double sortedfitness[POP_SIZE][2];

//evaluated individual and generation counter
static int evaluated_inds=0, generation=0;

//Log variables
static double avgfit=0.0, bestfit=-1.0, abs_bestfit=-1.0;
static int bestind=-1, abs_bestind=-1;

//Output/input files
FILE *f1; 
FILE *f2; 
FILE *f3;

//Functions
static void reset();
static int run(int ms);
//Initiaite genes of all individuals randomly, called at gen 0
void initializePopulation();
//resets the position of the robot before each trial
void resetRobotPosition();
//get the position of the robot at any time step
//static double* getRobotPosition();
// generate a new population of genomes for the next generation
void createNewPopulation();
//Mutate a given gene value within the min-max bounds
double mutate(double, double, double);
//Crossover operator for two individuals ind1 and ind2. Returns crossed individual
void crossover(int, int, double*);
//Sort population by fitness
void sortPopulation();
// Standard fast algorithm to sort population by fitness
void quickSort(double[][2], int, int);
//Write all genomes and fitnesses to file
void logPopulation();
//Write absolute best genome to file
void logBest();

#endif //SUP
