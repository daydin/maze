#ifndef SUPERVISOR
#define SUPERVISOR

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include "libs/webots/robot.h"
#include "libs/webots/supervisor.h"
#include "libs/webots/emitter.h"
#include "libs/webots/receiver.h"
//#include "libs/webots/node.h"

//////////////////////////
//Start Editing
//////////////////////////

/*
//Parameters of the genetic algorithm
 */

//Number of individuals in a population
#define POP_SIZE 200

#define INT_MAX_PN 15
#define INT_MAX_P 31
#define BIT_SIZE 5


#define GENE_MIN -5
#define GENE_MAX 5
#define BIAS_MIN -1
#define BIAS_MAX 1
#define TAU_MIN 1
#define TAU_MAX 50

#define NB_PS 4
#define NB_FS 1
#define NB_BIAS 1
#define NB_TC 1
#define NB_OUTPUTS 2
#define NB_HIDDEN_NEURONS 4
#define NB_TOT_OUT (NB_OUTPUTS+NB_HIDDEN_NEURONS) //6
#define NB_TOT_TC (NB_TC*NB_TOT_OUT) //6 time constants
#define NB_TOT_BIAS (NB_BIAS*NB_TOT_OUT) // 6 biases
#define NB_IN_S (NB_PS+NB_FS) //5
#define NB_IN  (NB_PS+NB_FS+NB_TOT_OUT) //11
#define NB_WEIGHTS ((NB_TOT_OUT)*NB_IN)+(2*(NB_TOT_OUT)) //78
#define NB_ONLY_WT (NB_TOT_OUT)*NB_IN //66
#define GENOME_LENGTH 390 


//#define NB_PS 4
//#define NB_FS 1
//#define NB_BIAS 1
//#define NB_TC 1
//#define NB_OUTPUTS 2
//#define NB_HIDDEN_NEURONS 4
//#define NB_TOT_OUT (NB_OUTPUTS+NB_HIDDEN_NEURONS) //6
//#define NB_IN_S (NB_PS+NB_FS) //5
//#define NB_IN  (NB_PS+NB_FS+NB_TOT_OUT) //11
//#define NB_WEIGHTS ((NB_TOT_OUT)*NB_IN)+(2*(NB_TOT_OUT)) //78
//#define NB_ONLY_WT (NB_TOT_OUT)*NB_IN //66

//Proportion of best individuals which get directly copied to the next generation
#define ELITISM_RATIO 0.025
//Probablity of mutating each weight-value in a genome
#define MUTATION_PROBABILITY 0.02
////Mutations follow a Box-Muller distribution from the gene with this sigma
//#define MUTATION_SIGMA 0.2
//Probablity of having a crossover
#define CROSSOVER_PROBABILITY 0.04
//Size of genome
//NB_GENES should be equal to NB_WEIGHTS in e-puck_drive.h ! (important when NB_HIDDEN_NEURONS > 0)
#define NB_GENES 78
//Roulette wheel selection or not (truncation selection)
#define ROULETTE_WHEEL 0
//If not using roulette wheel (truncation selection), we need reproduction ratio
#define REPRODUCTION_RATIO 0.2 
//40 out of 200 inds make 5 copies each

/*
//Parameter of the simulator
 */

//If 1, evolution takes place. If 0, then the best individual obtained during the previous evolution is tested for an undetermined amount of time.
#define EVOLVING 1 // i.e., !TEST_BEST

//Simulation time step
#define TIME_STEP 128
//robot device tags
static WbNodeRef robot;
static WbDeviceTag emitter;
static WbDeviceTag receiver;
static WbFieldRef trans_field;
static WbFieldRef rot_field;
static WbFieldRef ctrl_field;
static WbNodeRef pattern;
static WbFieldRef pattern_rotation;


static double robot_initial_position[3]={0.0,0.0,0.4};
static double robot_initial_rotation[4]={0.0,1.0,0.0,0.0};
//static double robot_xy[2];
//static double robot_rot[4]={0.0,1.0,0.0,0.0};
//static int TIME_STEP;
//Weights for each genome in population.
static double pop[POP_SIZE][NB_GENES];
static _Bool pop_bin[POP_SIZE][GENOME_LENGTH];
static _Bool info_bin[POP_SIZE][GENOME_LENGTH+2]; //allows sending of epoch and trial information
static int pop_int[POP_SIZE][NB_GENES];

//Fitness of each individual in a population.
static double fitness[POP_SIZE];
//Population sorted by fitness
double sortedfitness[POP_SIZE][2];

//evaluated individual and generation counter
static int evaluated_inds=0, generation=0;

//Log variables
static double avgfit=0.0, bestfit=-1.0, abs_bestfit=-1.0;
static int bestind=-1, abs_bestind=-1;
//static int step_counter=0;
static double pi= 3.14159;
//Output/input files
FILE *f1, *f2, *f3, *pF1; //pF2, *pR, *pF3
fpos_t position1, position2, position3, tmp1, tmp2;


//Functions
static void reset();
static int run();
//Initiaite genes of all individuals randomly, called at gen 0
void initializePopulation();
//resets the position of the robot before each trial
void resetRobotPosition();
//get the position of the robot at any time step
//static double* getRobotPosition();
// generate a new population of genomes for the next generation
void createNewPopulation();
//Mutate a given gene value within the min-max bounds
_Bool mutate(_Bool);
//Crossover operator for two individuals ind1 and ind2. Returns crossed individual
void crossover(int, int, _Bool*);
//Sort population by fitness
void sortPopulation();
// Standard fast algorithm to sort population by fitness
void quickSort(double[][2], int, int);
//Write all genomes and fitnesses to file
void logPopulation();
//Write absolute best genome to file
void logBest();
void closeFiles();
void rtoi(void);
void popEncoder(void);
double itor(int*,int);
void encodeOne(int, int, int, _Bool *);
void decodeOne(int*,int, _Bool*, int);
#endif //SUP
