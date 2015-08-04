// std::vector<double> ;

const int number_sample = 17; 
const int state_size = 6; 
double sigma_sample[number_sample][state_size];
double prob_belief[number_sample];

double mean_state[2][state_size];
//This corresponds to the mean and standard deviation / covariance of each state variable. 

// States are [rx, ry, vxo, vyo, vxi, vyi, dx, dy]

void read_state_sample(double *currentState, int numDims, double timeNow, int sample_no)
{
    // Read from MAVLink/autopilot
    // if (READSTATELOCATION == 2)
    //     {   if ( not numDims==6 )
	   //           {   printf("ERROR: numDims not 6, is %i",numDims);
	   //  	           throw 1;
    //            }

		// --------------------------------------------------------------------------
		//   GET STATES
		// --------------------------------------------------------------------------
            //ONLY FOR THE DEMO: 
		        //vxi = 0; 
		        // vyi = 0; 
		        // xi = 1.5;
		        // yi = 0;

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		        // printf("OWNSHIP XY: [ % .4f , % .4f ]  " , xo,yo);
		        // printf("OWNSHIP UV: [ % .4f , % .4f ] \n" , vxo,vyo);
		        // printf("INTRUDER XY:[ % .4f , % .4f ]  " , xi,yi);
		        // printf("INTRUDER UV:[ % .4f , % .4f ] \n" , vxi,vyi);

                
		        // currentState[0] = (double) (xi-xo); // rx
		        // currentState[1] = (double) (yi-yo); // ry
		        // currentState[2] = (double) vxo;     // vxo
		        // currentState[3] = (double) vyo;     // vyo
		        // currentState[4] = (double) vxi;     // vxi
		        // currentState[5] = (double) vyi;     // vyi

		        for (int i=0; i<state_size; i++)
		        	currentState[i] = sigma_sample[sample_no][i];
		        	
		        printf("\n");
      // }

  // return 0;

}



void update_sample_points()
	{	
		// States are [rx, ry, vxo, vyo, vxi, vyi, dx, dy]
		mean_state[0][0] = xi-xo;
		mean_state[1][0] = ;//cov of rx
		
		mean_state[0][1] = yi-yo;
		mean_state[1][1] = ;//cov of ry;

		mean_state[0][2] = vxo; 
		mean_state[1][2] = vxo_cov; 

		mean_state[0][3] = vyo; 
		mean_state[1][3] = vyo_cov; 

		mean_state[0][4] = vxi; 
		mean_state[1][4] = vxi_cov; 

		mean_state[0][5] = vyi; 
		mean_state[1][5] = vyi_cov; 


		// mean_state[]

		for (int i=0; i<number_sample; i++)
			{	for (int j=0; j<state_size; j++)
					{	sigma_sample[i][j] = mean_state[0][j];
					}
			}

		for (int i=0; i<state_size; i++)
			{	
				// sigma_sample[i][0] = mean_state[0][j] + //Formula for standard deviation/
				sigma_sample[2*i+1][i] = mean_state[0][i] + mean_state[1][i];//stand dev
				sigma_sample[2*i+2][i] = mean_state[0][i] - mean_state[1][i];//stand dev
			}
	}


// simga = sum (x - mu)^2

void parse_state()
	{
	}

//Calling get neighbors and interpolate on sigma_sample
//Call another function to retrieve probabilities

void get_probabity_belief(int sample)
	{	

	}


for (int state_iter=0; state_iter<number_sample; i++)
	{	
		for (int i=0; i<state_size; i++)
        	currentState[i] = sigma_sample[state_iter][i];
		prob_belief[state_iter] = get_probability_belief(state_iter);
		// prob_belief[state_iter] = 

	}







