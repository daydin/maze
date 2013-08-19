/*
* File:         evolution.c
*/

#include "evolutionT.h"

int main()
{

	wb_robot_init();

	//init and reset functionalities

	int i;
	int fin=0;

	for(i=0; i<POP_SIZE; i++) fitness[i]=-1;
	srand(time(0));  //seed the pseudo-random number generator

	// Initiate the emitter used to send genomes to the experiment
	emitter = wb_robot_get_device("emitter");


	// Initiate the receiver used to receive fitness
	receiver = wb_robot_get_device("receiversupervisor");
	wb_receiver_enable(receiver, TIME_STEP);

	// Create a supervised node for the robot 
	robot = wb_supervisor_node_get_from_def("EPUCK");
	controller_field = wb_supervisor_node_get_field(robot,"controller");
	trans_field = wb_supervisor_node_get_field(robot, "translation");
	/*
	* The call to this function is mandatory for the function
	* supervisor_node_was_found to be able to work correctly.
	*/
	wb_robot_step(0);

	// set the robot controller to nn
	const char *controller_name = "e-puck_drive";
	wb_supervisor_field_set_sf_string(controller_field, controller_name); //robot_set_controller(robot, controller_name);

	//check whether robot was found
	if (wb_supervisor_node_get_type(robot) == WB_NODE_NO_NODE) 
		printf("Error: node EPUCK not found!!!\n");

	if(EVOLVING) {
		//Open log files
		f1= fopen ("../../data/fitness.txt", "wt");
		f2= fopen ("../../data/genomes.txt", "wt");
		//initial weights randomly
		initializePopulation();
		printf("NEW EVOLUTION\n");
		printf("GENERATION 0\n");
		// send genomes to experiment
		resetRobotPosition();
		wb_emitter_send(emitter, (void *)pop[evaluated_inds], NB_GENES*sizeof(double));
	}
	else { // testing best individual
		//Read best genome from bestgenome.txt and initialize weights.
		f3= fopen ("../../data/bestgenome.txt", "rt");
		fscanf(f3,"%d %d", &generation, &evaluated_inds); //return integers read from file stored at generation and e_ind addresses
		for(i=0;i<NB_GENES;i++) fscanf(f3,"%lf ",&pop[0][i]); //return weights (float) for the best individual stored in pop[0] row.

		printf("TESTING INDIVIDUAL %d, GENERATION %d\n", evaluated_inds, generation);

		// send genomes to experiment
		resetRobotPosition();
		wb_emitter_send(emitter, (void *)pop[0], NB_GENES*sizeof(double));
	}

	

	//
	//main controller of the evolution, this function never returns
	//
	while(!fin){
		const double *fit;
		double finished=0;


		// as long as individual is being evaluated, print current fitness and return
		int n = wb_receiver_get_queue_length(receiver); 
		if (n) {
			fit = (double *)wb_receiver_get_data(receiver); //gets msg[2]={fitness, 0.0}
			fitness[evaluated_inds]=fit[0];
			finished=fit[1]; //message to end evals
			fin=(1==(int) finished)?1:0;
			
			// print stuff on screen
			char* message=NULL;
			sprintf(message, "Gen: %d Ind: %d Fit: %.2f", generation, evaluated_inds, fit[0]); //compose a string to be printed, pointer "message"
			wb_supervisor_set_label(0, message, 0, 0, 0.1, 0x000000,0); //int id, const char *text, double x, double y, double size, int color, double transparency
			wb_receiver_next_packet(receiver); //increment receiver pckg
		}


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
	}
	// wb_robot_cleanup();
	return 0; 
}

//
//Initiaite genes of all individuals randomly
//
void initializePopulation(void)
{
  int i, j;
  for(i=0;i<POP_SIZE;i++) {
    for(j=0;j<NB_GENES;j++) {
      // all genes must be in the range of [-1, 1]
      pop[i][j]=(GENE_MAX-GENE_MIN)*(double)rand()/(double)RAND_MAX
               -(GENE_MAX-GENE_MIN)/2.0;
    }
  }
  return;
}

//
//resets the position of the robot before each trial EDIT HERE
//
void resetRobotPosition(void)
{
  if (wb_supervisor_node_get_type(robot) == WB_NODE_NO_NODE)
    printf("Error: node %s not found\n", "EPUCK");

  //Initial position of the robot at each time step..
  int i;
  for (i=0; i<2; i++) robot_initial_position[i] = 0.6*(double)rand()/(double)RAND_MAX-0.3;
  robot_initial_position[2] = 6.28*(double)rand()/(double)RAND_MAX;
  wb_supervisor_field_set_sf_vec3f(trans_field,robot_initial_position);
  //might need some additional constraints
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
  fitness[left][0] = pivot[0]; //add the pivot entry to its right location
  fitness[left][1] = pivot[1];
  pivot[0] = left; //why assign the index to something meant to hold values? this is weird...
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
