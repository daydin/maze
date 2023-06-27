/*
 * File:         evolution.c
 */

#include "supervisor.h"

int main()
{

	/* necessary to initialize webots stuff */
	wb_robot_init();

	reset();
	int result=0;


	/* main loop
	 * Perform simulation steps of TIME_STEP milliseconds
	 * and leave the loop when the simulation is over
	 */
	while (wb_robot_step(TIME_STEP) != -1) {

		result=run();
		if (result!=TIME_STEP) break;
	};
	closeFiles();
	//	quit the sim
	wb_supervisor_simulation_quit(EXIT_SUCCESS);
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
	pF1= fopen("initPop.txt","w+");
	//pF2= fopen("real2IntGenome.txt","w+");
	//pF3= fopen("encodedPop.txt","w+");
	// Initiate the emitter used to send genomes to the experiment
	emitter = wb_robot_get_device("emittersupervisor");

	// Initiate the receiver used to receive fitness
	receiver = wb_robot_get_device("receiversupervisor");
	wb_receiver_enable(receiver, TIME_STEP);

	// Create a supervised node for the robot
	robot = wb_supervisor_node_get_from_def("EPUCK");
	assert(robot!=NULL);
	pattern=wb_supervisor_node_get_from_def("FLOOR");
	pattern_rotation = wb_supervisor_node_get_field(pattern,"rotation");
	assert(pattern!=NULL);
	assert(pattern_rotation!=NULL);


	trans_field = wb_supervisor_node_get_field(robot,"translation");
	rot_field = wb_supervisor_node_get_field(robot,"rotation");
	ctrl_field= wb_supervisor_node_get_field(robot,"controller");

	wb_robot_step(0);
	wb_robot_step(0); //this is magic

	// set the robot controller to nn
	const char *controller_name = "drive2";
	wb_supervisor_field_set_sf_string(ctrl_field,controller_name);

	//check whether robot was found
	if (wb_supervisor_node_get_type(robot) == WB_NODE_NO_NODE)
		puts("Error: node EPUCK not found!!!");

	if(EVOLVING) {
		//Open log files
		f1= fopen ("../../data/fitness.txt", "wt");
		f2= fopen ("../../data/genomes.txt", "wt");
		//pR= fopen("savePop.txt","w+");

		//initial weights randomly
		initializePopulation();
		//rtoi(); //real to int conversion before encoding
		popEncoder();


		//puts("NEW EVOLUTION");
		//puts("GENERATION 0");

		// send genomes to experiment
		resetRobotPosition();
		wb_emitter_send(emitter, (void *)pop_bin[evaluated_inds], GENOME_LENGTH*sizeof(_Bool));
		//instead save binary genomes to a file
		//puts("Genes sent.");
	}
	else {
		// testing best individual
		// Read best genome from bestgenome.txt and initialize weights.
		f3= fopen ("../../data/bestgenome.txt", "rt");
		fscanf(f3,"%d %d", &generation, &evaluated_inds);

		//TODO either read binary genome or make sure it is decoded prior to storage
		for(i=0;i<GENOME_LENGTH;i++) fscanf(f3,"%d ",&pop_bin[0][i]);

		//printf("TESTING INDIVIDUAL %d, GENERATION %d\n", evaluated_inds, generation);

		// send genomes to experiment
		resetRobotPosition();
		// wb_emitter_send(emitter, (void *)pop[0], NB_GENES*sizeof(double));
	}
	return;
}

//
//main controller of the evolution, this function never returns
//
static int run()
{
	const double *fit;
	double finished=0;
	//double values[4]= {0.0,1.0,0.0,0.0};
	//int epoch, reset_position;

	// as long as individual is being evaluated, print current fitness and return
	int n = wb_receiver_get_queue_length(receiver);
	if (n) {
		fit = (double *)wb_receiver_get_data(receiver); //gets msg[4]={fitness, finished,epoch,reset}
		fitness[evaluated_inds]=fit[0];
		finished=fit[1];
		//epoch= (int) fit[2];
		//reset_position= (int) fit[3];
		//if(reset_position) resetRobotPosition();
		//if(1  == epoch || 3 == epoch){
			//wb_supervisor_field_set_sf_rotation(pattern_rotation, values);
			//printf("Flag: %d, world is set.\n", epoch);
		//}else{
            //values[3]=pi;
			//wb_supervisor_field_set_sf_rotation(pattern_rotation, values);
			//printf("\n Flag: %d, world is rotated.\n", epoch);
		//}
		// print generational info on screen
		char message[100];
		sprintf(message, "Gen: %d Ind: %d Fit: %.2f", generation, evaluated_inds, fit[0]);
		wb_supervisor_set_label(0, message, 0, 0, 0.1, 0xff0000,0);
		wb_receiver_next_packet(receiver);
	}


	// if still evaluating the same individual, return.

	if (!finished) return TIME_STEP; //if not finished, !finished ==1.

	// when evaluation is done, an extra flag is returned in the message

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
			//printf("best fit: %f\n", bestfit);

			//write data to files
			logPopulation();

			//rank population, select best individuals and create new generation
			createNewPopulation();

			generation++;
			if(200==generation) return 0;
			//printf("\nGENERATION %d\n", generation);
			evaluated_inds = 0;
			avgfit = 0.0;
			bestfit = -1.0;
			bestind = -1.0;

			resetRobotPosition();
			wb_emitter_send(emitter, (void *)pop_bin[evaluated_inds], GENOME_LENGTH*sizeof(_Bool));
		}
		else {
			// assign received fitness to individual
			//printf("fitness: %f\n", fitness[evaluated_inds]);
			evaluated_inds++;

			// send next genome to experiment
			resetRobotPosition();
			wb_emitter_send(emitter, (void *)pop_bin[evaluated_inds], GENOME_LENGTH*sizeof(_Bool));
		}
	}

	return TIME_STEP;
}

void closeFiles(){
	fclose(f3);
	fclose(f2);
	fclose(f1);
	//fclose(pF3);
	//fclose(pF2);
	fclose(pF1);
	return;
}

//

//Initiate genes of all individuals randomly
//


void initializePopulation()
{
	int i,j,weight_counter,tmp;
	weight_counter=0; //allows us to separate different segments of the genome



	for(i=0;i<POP_SIZE;i++){
		//initialize all 78 parameters: 66 weights + 6 biases + 6 time constants
		weight_counter=0;
		for(j=0;j<NB_ONLY_WT;j++) {
			// all genes must be in the range of [-5, 5]
			pop[i][j]=(GENE_MAX-GENE_MIN)*(double)rand()/(double)RAND_MAX
					-(GENE_MAX-GENE_MIN)/2.0;
			fprintf(pF1,"pop[%d][%d]: %g\n",i,j,pop[i][j]);
			fflush(pF1);
			weight_counter++;
		}
		tmp = weight_counter; //next loop run between 66-71
		//set time const
		for(j=tmp;j<tmp+NB_TOT_TC;j++){
			pop[i][j]=(double)(rand()%50) +1; //init a time constant
			fprintf(pF1,"pop[%d][%d]: %g\n",i,j,pop[i][j]);
			fflush(pF1);
			weight_counter++;
		}

		tmp= weight_counter; // final loop between 72-77
		//set init bias
		for(j=tmp;j<tmp+NB_TOT_BIAS;j++){
			pop[i][j]=(BIAS_MAX-BIAS_MIN)*(double)rand()/(double)RAND_MAX
					-(BIAS_MAX-BIAS_MIN)/2.0;
			fprintf(pF1,"pop[%d][%d]: %g\n",i,j,pop[i][j]);
			fflush(pF1);
			weight_counter++;
		}
	}


	return;
}


void rtoi(){
	int i,j;
	int weight_counter=0, tmp;
	double res,tmp2;


	for(i=0;i<POP_SIZE;i++){
		weight_counter=0;
		for(j=0;j<NB_ONLY_WT;j++){ //this might be an issue
			tmp2=(pop[i][j]<0)?(-pop[i][j]):pop[i][j];
			res= (tmp2-GENE_MIN)*((double)INT_MAX_PN/(GENE_MAX-GENE_MIN));
			pop_int[i][j]=(int) round(res);
			//fprintf(pF2,"\n n: %g res0: %g pop_int[%d][%d]: %d \n",pop[i][j], res,i,j,pop_int[i][j]);
			//fflush(pF2);
			++weight_counter;
		}
		tmp=weight_counter;
		for(j=tmp; j<NB_ONLY_WT+NB_TOT_TC; j++){
			res= (pop[i][j]-TAU_MIN)*((double)INT_MAX_P/(TAU_MAX-TAU_MIN));
			pop_int[i][j]=(int) round(res);
			//fprintf(pF2,"\n n: %g res1: %g pop_int[%d][%d]: %d \n",pop[i][j], res,i,j,pop_int[i][j]);
			//fflush(pF2);
			++weight_counter;
		}
		tmp=weight_counter;
		for(j=tmp; j<NB_WEIGHTS;j++){
			tmp2=(pop[i][j]<0)?(-pop[i][j]):pop[i][j];
			res= (tmp2-BIAS_MIN)*((double)INT_MAX_PN/(BIAS_MAX-BIAS_MIN));
			pop_int[i][j]=(int) round(res);
			//fprintf(pF2,"\n n: %g res2: %g pop_int[%d][%d]: %d \n",pop[i][j], res,i,j,pop_int[i][j]);
			//fflush(pF2);
		}

	}
	return;
}

void popEncoder (void){
	int i,j,s,k;
	_Bool bin_array[BIT_SIZE];

	rtoi();
	for(i=0; i<POP_SIZE; i++){
		//fprintf(pF3,"\nNEW IND %d\n",i);
		//fflush(pF3);
		for(j=0; j<NB_WEIGHTS; j++){
			s=0;
			if(pop[i][j]<0){
				s=1;
			}
			encodeOne(pop_int[i][j],BIT_SIZE,s,bin_array);
			for(k=0;k<BIT_SIZE;k++){
				pop_bin[i][(BIT_SIZE*j)+k]=bin_array[k];
				//fprintf(pF3,"\n pop_bin[%d][%d]: %d\n",i,(BIT_SIZE*j)+k,(pop_bin[i][(BIT_SIZE*j)+k])?1:0);
				//fflush(pF3);
			}
			for(k=0;k<BIT_SIZE;k++){
				//fprintf(pF3,"%d",(pop_bin[i][(BIT_SIZE*j)+k])?1:0);
				//fflush(pF3);
			}
		}
	}
	return;
}



void encodeOne(int value, int bits, int s, _Bool* binary){

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
	encodeOne(value, bits, s, binary);
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
             robot_initial_rotation[3]=((pi/2)*((double) rand()/(double) RAND_MAX))-pi/4;
	wb_supervisor_field_set_sf_vec3f(trans_field, robot_initial_position);
            wb_supervisor_field_set_sf_rotation(rot_field,robot_initial_rotation);
	return;
}

/*void createNewPopulation(void)
{
	//double newpop[POP_SIZE][NB_GENES];
	_Bool newpop_bin[POP_SIZE][GENOME_LENGTH];

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
			for(j=0;j<GENOME_LENGTH;j++)
				newpop_bin[i][j]=pop_bin[(int)sortedfitness[i][1]][j];
		}
		//the other individuals are generated through the crossover of two parents
		else {

			//select non-elitist individual
			int ind1=0;
			ind1= rand() % (int)(POP_SIZE*REPRODUCTION_RATIO);

			//if we will do crossover, select a second individual
			if ((double)rand()/(double)RAND_MAX < CROSSOVER_PROBABILITY) {
				int ind2=0;
					do {
						ind2= rand() % (int)(POP_SIZE*REPRODUCTION_RATIO); //floor((double)rand()/(double)RAND_MAX*POP_SIZE*REPRODUCTION_RATIO);
					} while (ind1==ind2);

				ind1=(int)sortedfitness[ind1][1];
				ind2=(int)sortedfitness[ind2][1];
				crossover(ind1, ind2, newpop_bin[i]);
			}else{ //if no crossover was done, just copy selected individual directly
				for(j=0;j<GENOME_LENGTH;j++) newpop_bin[i][j]=pop_bin[(int)sortedfitness[ind1][1]][j];
			}
		}
	}

	//mutate new population and copy back to pop
	for(i=0; i<POP_SIZE; i++) {
		if(i<elitism_counter) { //no mutation for elitists
			for(j=0;j<GENOME_LENGTH;j++)
				pop_bin[i][j]=newpop_bin[i][j];
		}
		else { //mutate others with probability per gene
			for(j=0;j<GENOME_LENGTH;j++)
				if((double)rand()/(double)RAND_MAX<MUTATION_PROBABILITY)
					pop_bin[i][j]=mutate(newpop_bin[i][j]);
				else
					pop_bin[i][j]=newpop_bin[i][j];
		}

		//reset fitness
		fitness[i]=-1;
	}
	return;
}*/

void createNewPopulation(void)
{
  //double newpop[POP_SIZE][NB_GENES];
  _Bool newpop_bin[POP_SIZE][GENOME_LENGTH];


  int elitism_counter = round(POP_SIZE*ELITISM_RATIO);
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
      for(j=0;j<GENOME_LENGTH;j++)
        newpop_bin[i][j]=pop_bin[(int)sortedfitness[i][1]][j];
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
        crossover(ind1, ind2, newpop_bin[i]);
      }
      else { //if no crossover was done, just copy selected individual directly
        for(j=0;j<GENOME_LENGTH;j++) newpop_bin[i][j]=pop_bin[(int)sortedfitness[ind1][1]][j];
      }
    }
  }

  //mutate new population and copy back to pop
  for(i=0; i<POP_SIZE; i++) {
    if(i<elitism_counter) { //no mutation for elitists
      for(j=0;j<GENOME_LENGTH;j++)
        pop_bin[i][j]=newpop_bin[i][j];
    }
    else { //mutate others with probability per gene
      for(j=0;j<GENOME_LENGTH;j++)
        if(((double)rand()/(double)RAND_MAX)<MUTATION_PROBABILITY)
          pop_bin[i][j]=mutate(newpop_bin[i][j]);
        else
          pop_bin[i][j]=newpop_bin[i][j];
    }

    //reset fitness
    fitness[i]=-1;
  }
  return;
}

//
//Mutate a given gene value within the min-max bounds
//
_Bool mutate(_Bool gene)
{
            //puts("Did mutation on gene.");
	return gene?0:1;	
}

//
//Crossover operator for two individuals ind1 and ind2. Returns crossed individual
//
void crossover(int ind1, int ind2, _Bool* new_ind) {
	int crossover_point = 0, i;

	crossover_point=rand() % GENOME_LENGTH; //floor(nb_genes*(double)rand()/(double)RAND_MAX); //can be anywhere between 0 and 389

	for(i=0;i<GENOME_LENGTH;i++) {
		if(i<=crossover_point) new_ind[i]=pop_bin[ind1][i]; //first part of the genome comes from mom (ind1), second part from dad(ind2)
		else                   new_ind[i]=pop_bin[ind2][i];
	}
	//puts("Did crossover.");
	return;
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
	return;
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
            return;
}

//
//Write all genomes and fitnesses to file
//
void logPopulation() {
	int i, j;
	//calculate average fitness
	//TODO Either decode binary genome or store binary genome
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
		for(j=0; j<GENOME_LENGTH; j++)
			fprintf(f2, "%d\t", pop_bin[i][j]?1:0);
		fprintf(f2, "\n");
		fflush(f2);
	}
	fprintf(f2, "best ind: %d\tbest fitness: %f\n\n", bestind, bestfit);
	fflush(f2);
            return;
}

//
//Write absolute best genome to file
//
void logBest() {
	int j;
	//best genome
	//TODO Either decode binary genome or store binary genome
	f3= fopen ("../../data/bestgenome.txt", "wt");
	fprintf(f3, "%d %d ", generation, abs_bestind);
	fflush(f3);
	for(j=0; j<GENOME_LENGTH; j++)
		fprintf(f3, "%d ", pop_bin[abs_bestind][j]?1:0);
	fprintf(f3, "\n");
	fflush(f3);
	return;
}

