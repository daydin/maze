/*
 * File:         evolution.c
 */

#include "sup.h"

int main()
{

    /* necessary to initialize webots stuff */
  wb_robot_init();
  //TIME_STEP = (int)wb_robot_get_basic_time_step();
  reset();


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
//The reset function is called at the beginning of an evolution.
//

/*
* The call to wb_robot_step is mandatory for the function
* supervisor_node_was_found to be able to work correctly.
*/
static void reset(void)
{
  int i;
  for(i=0; i<POP_SIZE; i++) fitness[i]=-1;
  srand(time(0));

  // Initiate the emitter used to send genomes to the experiment
  emitter = wb_robot_get_device("emittersupervisor");

  // Initiate the receiver used to receive fitness
  receiver = wb_robot_get_device("receiversupervisor");
  wb_receiver_enable(receiver, TIME_STEP);

  // Create a supervised node for the robot
  robot = wb_supervisor_node_get_from_def("EPUCK");

  trans_field = wb_supervisor_node_get_field(robot,"translation");
  rot_field = wb_supervisor_node_get_field(robot,"rotation");
  ctrl_field= wb_supervisor_node_get_field(robot,"controller");

  wb_robot_step(0);
  wb_robot_step(0); //this is magic

  // set the robot controller to nn
  const char *controller_name = "drive";
   wb_supervisor_field_set_sf_string(ctrl_field,controller_name);

  //check whether robot was found
  if (wb_supervisor_node_get_type(robot) == WB_NODE_NO_NODE)
    puts("Error: node EPUCK not found!!!\n");

  if(EVOLVING) {
    //Open log files
    f1= fopen ("../../data/fitness.txt", "wt");
    f2= fopen ("../../data/genomes.txt", "wt");

    //initial weights randomly
    initializePopulation();


    puts("NEW EVOLUTION\n");
    puts("GENERATION 0\n");

    // send genomes to experiment
    resetRobotPosition();
    wb_emitter_send(emitter, (void *)pop[evaluated_inds], NB_GENES*sizeof(double));

    puts("Genes sent.\n");
  }
  else {
	// testing best individual
    // Read best genome from bestgenome.txt and initialize weights.
    f3= fopen ("../../data/bestgenome.txt", "rt");
    fscanf(f3,"%d %d", &generation, &evaluated_inds);
    for(i=0;i<NB_GENES;i++) fscanf(f3,"%lf ",&pop[0][i]);

    printf("TESTING INDIVIDUAL %d, GENERATION %d\n", evaluated_inds, generation);

    // send genomes to experiment
    resetRobotPosition();
    wb_emitter_send(emitter, (void *)pop[0], NB_GENES*sizeof(double));
  }
  return;
}

//
//main controller of the evolution, this function never returns
//
static int run(int ms)
{
  const double *fit;
  double finished=0;

  // as long as individual is being evaluated, print current fitness and return
  int n = wb_receiver_get_queue_length(receiver);
  if (n) {
    fit = (double *)wb_receiver_get_data(receiver); //gets msg[2]={fitness, 0.0}
    fitness[evaluated_inds]=fit[0];
    finished=fit[1];
    // print stuff on screen
    char message[100];
    sprintf(message, "Gen: %d Ind: %d Fit: %.2f", generation, evaluated_inds, fit[0]);
    wb_supervisor_set_label(0, message, 0, 0, 0.1, 0x000000,0);
    wb_receiver_next_packet(receiver);
  }

  // when evaluation is done, an extra flag is returned in the message
  // if packets to be received are done but you didn't get the flag to finish evaluation, return
  if (!finished) return TIME_STEP;

  if(EVOLVING) {
    // if whole population has been evaluated
    if ((evaluated_inds+1) == POP_SIZE ) {
      // sort population by fitness
      sortPopulation();
      // find and log current and absolute best individual
      bestfit=sortedfitness[0][0];
      bestind=(int)sortedfitness[0][1];
      if (bestfit > abs_bestfit) {
        abs_bestfit=bestfit;
        abs_bestind=bestind;
        logBest();
      }
      printf("best fit: %f\n", bestfit);

      //write data to files
      logPopulation();

      //rank population, select best individuals and create new generation
      createNewPopulation();

      generation++;
      printf("\nGENERATION %d\n", generation);
      evaluated_inds = 0;
      avgfit = 0.0;
      bestfit = -1.0;
      bestind = -1.0;

      resetRobotPosition();
      wb_emitter_send(emitter, (void *)pop[evaluated_inds], NB_GENES*sizeof(double));
    }
    else {
      // assign received fitness to individual
      printf("fitness: %f\n", fitness[evaluated_inds]);
      evaluated_inds++;

      // send next genome to experiment
      resetRobotPosition();
      wb_emitter_send(emitter, (void *)pop[evaluated_inds], NB_GENES*sizeof(double));
    }
  }

  return TIME_STEP;
}

//
//Initiaite genes of all individuals randomly
//
//void initializePopulation(void)
//{
//  int i, j;
//  for(i=0;i<POP_SIZE;i++) {
//    for(j=0;j<NB_GENES;j++) {
//      // all genes must be in the range of [-1, 1]
//      pop[i][j]=(GENE_MAX-GENE_MIN)*(double)rand()/(double)RAND_MAX
//               -(GENE_MAX-GENE_MIN)/2.0;
//    }
//  }
//  return;
//}

void initializePopulation()
{
  int i,j,weight_counter;
    weight_counter=0; //allows us to separate different segments of the genome
    FILE *pF;
    pF= fopen("initPop.txt","w+");
    for(i=0;i<POP_SIZE;i++){
    	//initialize all 78 parameters: 66 weights + 6 biases + 6 time constants
        for(j=0;j<NB_ONLY_WT;j++) {
        	// all genes must be in the range of [-5, 5]
        	pop[i][j]=(GENE_MAX-GENE_MIN)*(double)rand()/(double)RAND_MAX
        			-(GENE_MAX-GENE_MIN)/2.0;
        	fprintf(pF,"pop[%d][%d]: %g\n",i,j,pop[i][j]);
        	fflush(pF);
        	weight_counter++;
        }
        tmp = weight_counter; //next loop run between 66-71
        //set time const
        for(j=tmp;j<tmp+NB_TOT_TC;j++){
            pop[i][j]=(double)(rand()%50) +1; //init a time constant
        	fprintf(pF,"pop[%d][%d]: %g\n",i,j,pop[i][j]);
        	fflush(pF);
            weight_counter++;
        }

        tmp= weight_counter; // final loop between 72-77
        //set init bias
        for(j=tmp;j<tmp+NB_TOT_BIAS;j++){
            pop[i][j]=(BIAS_MAX-BIAS_MIN)*(double)rand()/(double)RAND_MAX
            		-(BIAS_MAX-BIAS_MIN)/2.0;
        	fprintf(pF,"pop[%d][%d]: %g\n",i,j,pop[i][j]);
        	fflush(pF);
            weight_counter++;
        }
    }
    fclose(pF);

  return;
}


void rtoi(){

	double res;
	if(0==flag){
		res= (n-GENE_MIN)*((double)INT_MAX_PN/(GENE_MAX-GENE_MIN));
		printf("\n n: %g res0: %g\n",n, res);
		return ((int) round(res));
	}
	else if(1==flag){
		res= (n-TAU_MIN)*((double)INT_MAX_P/(TAU_MAX-TAU_MIN));
		printf("\n n: %g res1: %g\n",n, res);
		return ((int) round(res));
	}
	else{
		res= (n-BIAS_MIN)*((double)INT_MAX_PN/(BIAS_MAX-BIAS_MIN));
		printf("\n n: %g res2: %g\n",n, res);
		return ((int) round(res));
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

void encode(int value, int bits, int s, _Bool* binary){

        int k;
        int carry=0;

        if(0==bits && 1==s) {
            carry=1;
            for(k=BIT_SIZE-1; k>-1; k--){
            	if(binary[k]&carry){
            		binary[k]=binary[k]^carry;
            	}else{
            		binary[k]=binary[k]^carry;
            		carry=0;
            	}
            }
            return;
        }

        if(0==bits && 0==s) {
            return;
        }

        binary[bits-1] = s^(value %2); //XOR sign flag with the remainder. Flag ==1 if number negative; obtain the complement.


     //   printf("val:%d rem:%d\n",value,binary[bits-1]);
        value/=2;
        --bits;
        encode(value, bits, s, binary);
}

void decode(int *sum, int bits, _Bool* binary, int option){
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
            decode(sum, bits, binary,option);
	}else{
			if(0==bits){
				return;
			}
			sum[1]+=(int)pow(2.0,(double)BIT_SIZE-(bits))*(int)binary[bits-1];
			//printf("sum %d bin[%d]:%d\n",sum[1], bits-1,binary[bits-1]);
			--bits;
			decode(sum, bits, binary,option);

	}
}



//
//resets the position of the robot before each trial
//
void resetRobotPosition(void)
{ //TODO: Add random rotation angle about z-axis

  if (wb_supervisor_node_get_type(robot) == WB_NODE_NO_NODE)
    printf("Error: node %s not found\n", "EPUCK");

  //Initial position of the robot at each time step..
  int i;
  for (i=0; i<2; i++) robot_initial_position[i] = 0.0;//0.6*(double)rand()/(double)RAND_MAX-0.3;
  robot_initial_position[2] = 0.4;

  wb_supervisor_field_set_sf_vec3f(trans_field, robot_initial_position);

  return;
}

//
//get the position of the robot at any time step
//
/*
double* getRobotPosition(void)
{
  double position[3];
  supervisor_field_get(robot,SUPERVISOR_FIELD_TRANSLATION_X | SUPERVISOR_FIELD_TRANSLATION_Z | SUPERVISOR_FIELD_ROTATION_ANGLE, &position, TIME_STEP);
  return position;
}
*/

//
//Based on the fitness of the last generation, generate a new population of
//genomes for the next generation.
//
void createNewPopulation(void)
{
  double newpop[POP_SIZE][NB_GENES];
  int elitism_counter = POP_SIZE*ELITISM_RATIO;
  double total_fitness = 0;
  // find minimum fitness to subtract it from sum
  double min_fitness = sortedfitness[POP_SIZE-1][0];
  if (min_fitness<0) min_fitness=0;
  int i, j;

  // calculate total of fitness, used for roulette wheel selection
  for(i=0; i<POP_SIZE; i++) total_fitness+=fitness[i];
  total_fitness-=min_fitness*POP_SIZE;

  //create new population
  for(i=0; i<POP_SIZE; i++) {

    //the elitism_counter best individuals are simply copied to the new population
    if(i<elitism_counter) {
      for(j=0;j<NB_GENES;j++)
        newpop[i][j]=pop[(int)sortedfitness[i][1]][j];
    }
    //the other individuals are generated through the crossover of two parents
    else {

      //select non-elitist individual
      int ind1=0;
      if (ROULETTE_WHEEL==1) {
        double r=(double)rand()/(double)RAND_MAX;
        double fitness_counter=(sortedfitness[ind1][0]-min_fitness)/total_fitness;
        while( r > fitness_counter) {
          ind1++;
          fitness_counter+=(sortedfitness[ind1][0]-min_fitness)/total_fitness;
        }
      }
      else ind1=floor((double)rand()/(double)RAND_MAX*POP_SIZE*REPRODUCTION_RATIO);

      //if we will do crossover, select a second individual
      if ((double)rand()/(double)RAND_MAX < CROSSOVER_PROBABILITY) {
        int ind2=0;
        if (ROULETTE_WHEEL==1)
          do {
            double r=(double)rand()/(double)RAND_MAX;
            double fitness_counter=(sortedfitness[ind2][0]-min_fitness)/total_fitness;
            while(( r > fitness_counter) && (ind2 < POP_SIZE)){
              ind2++;
              fitness_counter+=(sortedfitness[ind2][0]-min_fitness)/total_fitness;
            }
          } while (ind1==ind2);
        else
          do {
            ind2=floor((double)rand()/(double)RAND_MAX*POP_SIZE*REPRODUCTION_RATIO);
          } while (ind1==ind2);
        ind1=(int)sortedfitness[ind1][1];
        ind2=(int)sortedfitness[ind2][1];
        crossover(ind1, ind2, newpop[i]);
      }
      else { //if no crossover was done, just copy selected individual directly
        for(j=0;j<NB_GENES;j++) newpop[i][j]=pop[(int)sortedfitness[ind1][1]][j];
      }
    }
  }

  //mutate new population and copy back to pop
  for(i=0; i<POP_SIZE; i++) {
    if(i<elitism_counter) { //no mutation for elitists
      for(j=0;j<NB_GENES;j++)
        pop[i][j]=newpop[i][j];
    }
    else { //mutate others with probability per gene
      for(j=0;j<NB_GENES;j++)
        if((double)rand()/(double)RAND_MAX<MUTATION_PROBABILITY)
          pop[i][j]=mutate(GENE_MIN,GENE_MAX,newpop[i][j]);
        else
          pop[i][j]=newpop[i][j];
    }

    //reset fitness
    fitness[i]=-1;
  }
  return;
}

//
//Mutate a given gene value within the min-max bounds
//
double mutate(double min, double max, double gene)
{
  double  x1, x2, w, y1;

  do {
//    x1 = GENE_MAX-GENE_MIN * ((double)rand()/(double)RAND_MAX) - (GENE_MAX-GENE_MIN)/2.0;
//    x2 = GENE_MAX-GENE_MIN * ((double)rand()/(double)RAND_MAX) - (GENE_MAX-GENE_MIN)/2.0;
    x1 = (GENE_MAX-GENE_MIN) * ((double)rand()/(double)RAND_MAX) - (GENE_MAX-GENE_MIN)/2.0;
    x2 = (GENE_MAX-GENE_MIN) * ((double)rand()/(double)RAND_MAX) - (GENE_MAX-GENE_MIN)/2.0;
    w = x1 * x1 + x2 * x2;
  } while ( w > 1.0 || w == 0 );

  y1 = gene + MUTATION_SIGMA * x1 * sqrt( (-2.0 * log( w ) ) / w );

  if (y1>max) return max;
  if (y1<min) return min;
  return y1;
}

//
//Crossover operator for two individuals ind1 and ind2. Returns crossed individual
//
void crossover(int ind1, int ind2, double* new_ind) {
  int crossover_point = 0, i;
  double nb_genes=NB_GENES;

  crossover_point=floor(nb_genes*(double)rand()/(double)RAND_MAX);

  for(i=0;i<NB_GENES;i++) {
    if(i<=crossover_point) new_ind[i]=pop[ind1][i];
    else                   new_ind[i]=pop[ind2][i];
  }
}

//
//Sort population by fitness
//
void sortPopulation() {
  int i;
  //sort population by fitness
  for (i=0; i<POP_SIZE; i++) {
    sortedfitness[i][0]=fitness[i];
    sortedfitness[i][1]=(double)i; //keep index
  }
  quickSort(sortedfitness, 0, POP_SIZE - 1);
}

//
// Standard fast algorithm to sort population by fitness
//
void quickSort(double fitness[][2], int left, int right)
{
  double pivot[2];
  int l_hold, r_hold;

  l_hold = left;
  r_hold = right;
  pivot[0] = fitness[left][0];
  pivot[1] = fitness[left][1];
  while (left < right)
  {
    while ((fitness[right][0] <= pivot[0]) && (left < right))
      right--;
    if (left != right) {
      fitness[left][0] = fitness[right][0];
      fitness[left][1] = fitness[right][1];
      left++;
    }
    while ((fitness[left][0] >= pivot[0]) && (left < right))
      left++;
    if (left != right) {
      fitness[right][0] = fitness[left][0];
      fitness[right][1] = fitness[left][1];
      right--;
    }
  }
  fitness[left][0] = pivot[0];
  fitness[left][1] = pivot[1];
  pivot[0] = left;
  left = l_hold;
  right = r_hold;
  if (left < (int)pivot[0])  quickSort(fitness, left, (int)pivot[0]-1);
  if (right > (int)pivot[0]) quickSort(fitness, (int)pivot[0]+1, right);
}

//
//Write all genomes and fitnesses to file
//
void logPopulation() {
  int i, j;
  //calculate average fitness
  for (i=0; i<POP_SIZE; i++) {
    avgfit+=fitness[i];
  }
  avgfit/=(double)POP_SIZE;

  //fitness
  fprintf(f1, "%d\t %f\t %f\n",
              generation, avgfit, bestfit);
  fflush(f1);

  //all genomes
  fprintf(f2, "gen: %d\n", generation);
  for(i=0; i<POP_SIZE; i++) {
    fprintf(f2, "ind: %d\t fitness: %f\tgenes:\t", i, fitness[i]);
    for(j=0; j<NB_GENES; j++)
      fprintf(f2, "%f\t", pop[i][j]);
    fprintf(f2, "\n");
    fflush(f2);
  }
  fprintf(f2, "best ind: %d\tbest fitness: %f\n\n", bestind, bestfit);
  fflush(f2);
}

//
//Write absolute best genome to file
//
void logBest() {
  int j;
  //best genome
  f3= fopen ("../../data/bestgenome.txt", "wt");
  fprintf(f3, "%d %d ", generation, abs_bestind);
  for(j=0; j<NB_GENES; j++)
    fprintf(f3, "%f ", pop[abs_bestind][j]);
  fprintf(f3, "\n");
  fflush(f3);
}
