#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "interpolate.h"
#include <sys/stat.h>

// #include "autopilot_setup.h"

#define VERBOSE 0
#define READSTATELOCATION 2   // 0 is a canned state, 1 is from a file, 2 is from MAVLink
#define EXTRAPMODE 0      // Use 0 for the interpolation function to use nearest neighbor 
                          //  beyond the grid
#define THREATRANGE 15    // Range in meters at which an intruder aircraft triggers calls to CA
#define MAXSIMSTEPS 100    // If running in canned sim mode, stop after this number of time steps
#define SIMSTATERAND 0    // If != 0, returns random states upon calls to readState() (must be in 
                          // READSTATELOCATION==0 mode). For debugging.  Random states are within
                          // + or - SIMSTATERAND
#define NOMINAL_VX -1.0   // Was originally -2.5, but I want to make sure it's consistent with my policy
#define NOMINAL_VY  0.0

#define STATESCALE 0.5    // Factor by which to scale all states and actions.  Was originally 0.2 when Vx_nom was -2.5
#define VMAX  1.5         // Maximum absolute allowable horizontal velocity

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

float vyi, vxi, vxo, vyo, vx, vy, xi, yi, xo, yo, xt, yt;   // xt and yt are the absolute locations of the desired trajectory point
float dt = 0.01; //Based on the frequency of publishing data on the IMU topic.

//Defining ROS subscribers to retrieve data. 
ros::Subscriber own_imu_sub, own_pose_sub, intruder_imu_sub, intruder_pose_sub;
ros::Subscriber own_vel_sub, intruder_vel_sub;
//Defining ROS publisher to publish velocity setpoint data. 
ros::Publisher set_pt_vel_pub, set_pt_vel_pub_1, set_pt_vel_pub_2;

//Callback from intruder SLAM pose estimate.










// void intruder_pose_callback(const nav_msgs::Odometry::ConstPtr& intruder_pose_var)
// { 
//     nav_msgs::Odometry intruder_pose = *intruder_pose_var;
//     xi = intruder_pose.pose.pose.position.x;
//     yi = intruder_pose.pose.pose.position.y;
// }

// //Callback from intruder IMU velocity estimate.
// void intruder_vel_callback(const sensor_msgs::Imu::ConstPtr& intruder_acc_var)
// {   
//     //Must write the integral inside the main function. 
//     sensor_msgs::Imu intruder_acc=*intruder_acc_var;

//     vxi += intruder_acc.linear_acceleration.x * dt;
//     vyi += intruder_acc.linear_acceleration.y * dt;
// }

// //Callback from ownship SLAM pose estimate.
// void own_pose_callback(const nav_msgs::Odometry::ConstPtr& own_pose_var)
// { 
//     nav_msgs::Odometry own_pose = *own_pose_var;
//     yo = own_pose.pose.pose.position.y;
//     xo = own_pose.pose.pose.position.x;
// }

// void own_pose_demo_callback(const geometry_msgs::Vector3::ConstPtr& dummy_pose)
// { 
//  	geometry_msgs::Vector3 dummyhaha = *dummy_pose; 
// 	xo = dummyhaha.x;
// 	yo = dummyhaha.y;
// //    nav_msgs::Odometry own_pose = *own_pose_var;
// //    yo = own_pose.pose.pose.position.y;
// //    xo = own_pose.pose.pose.position.x;
// 	// std::cerr<<"THe callback is here"<<std::endl;
// }

// //Callback from ownship IMU velocity estimate. 
// void own_vel_callback(const sensor_msgs::Imu::ConstPtr& own_acc_var)
// {
// 	// vxo += *own_acc_var.linear_acceleration.x * dt;
// 	// vyo += *own_acc_var.linear_acceleration.y * dt;
//   sensor_msgs::Imu own_acc = *own_acc_var;
//   vxo += own_acc.linear_acceleration.x * dt;
//   vyo += own_acc.linear_acceleration.y * dt;

//   std::cout<<""
   
// }











// void gps_intruder_pose_callback(const nav_msgs::Odometry::ConstPtr& intruder_pose_var)
void gps_intruder_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& intruder_pose_var)
{ 
    // nav_msgs::Odometry intruder_pose = *intruder_pose_var;
    geometry_msgs::PoseStamped intruder_pose = *intruder_pose_var;
    xi = intruder_pose.pose.position.x / STATESCALE;
    yi = intruder_pose.pose.position.y / STATESCALE;
}

//Callback from intruder IMU velocity estimate.
// void gps_intruder_vel_callback(const sensor_msgs::Imu::ConstPtr& intruder_acc_var)
void gps_intruder_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& intruder_acc_var)
{   
    //Must write the integral inside the main function. 
    // sensor_msgs::Imu intruder_acc=*intruder_acc_var;
    geometry_msgs::Vector3Stamped intruder_vel = *intruder_acc_var;
 
    // vxi += intruder_acc.linear_acceleration.x * dt;
    // vyi += intruder_acc.linear_acceleration.y * dt;

    vyi = intruder_vel.vector.y / STATESCALE;
    vxi = intruder_vel.vector.x / STATESCALE;
}

//Callback from ownship SLAM pose estimate.
void gps_own_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& own_pose_var)
{ 
    // nav_msgs::Odometry own_pose = *own_pose_var;
    // yo = own_pose.pose.pose.position.y;
    // xo = own_pose.pose.pose.position.x;

    geometry_msgs::PoseStamped own_pose = *own_pose_var;
    xo = own_pose.pose.position.x / STATESCALE;
    yo = own_pose.pose.position.y / STATESCALE;
}

// void gps_own_pose_demo_callback(const geometry_msgs::Vector3::ConstPtr& dummy_pose)
// { 
//   geometry_msgs::Vector3 dummyhaha = *dummy_pose; 
//   xo = dummyhaha.x;
//   yo = dummyhaha.y;
// //    nav_msgs::Odometry own_pose = *own_pose_var;
// //    yo = own_pose.pose.pose.position.y;
// //    xo = own_pose.pose.pose.position.x;
//   // std::cerr<<"THe callback is here"<<std::endl;
// }

//Callback from ownship IMU velocity estimate. 
void gps_own_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& own_acc_var)
{
  // vxo += *own_acc_var.linear_acceleration.x * dt;
  // vyo += *own_acc_var.linear_acceleration.y * dt;
  // sensor_msgs::Imu own_acc = *own_acc_var;
  // vxo += own_acc.linear_acceleration.x * dt;
  // vyo += own_acc.linear_acceleration.y * dt;

  geometry_msgs::Vector3Stamped own_vel = *own_acc_var;
 
    // vxi += intruder_acc.linear_acceleration.x * dt;
    // vyi += intruder_acc.linear_acceleration.y * dt;

  vyo = own_vel.vector.y / STATESCALE;
  vxo = own_vel.vector.x / STATESCALE;

  // std::cout<<""
   
}

void set_velocity_ownship(float vx, float vy, float yaw)
{
  // unscale
  vx *= STATESCALE;
  vy *= STATESCALE;

  // speed limit
  if (vx >  VMAX)  vx =  VMAX;
  if (vx < -VMAX)  vx = -VMAX;
  if (vy >  VMAX)  vy =  VMAX;
  if (vy < -VMAX)  vy = -VMAX;


  ///NEW INTERFACE TO PUBLISH COMMANDS

  //On topic cmd_vel 
  //Creating object to publish. 
  geometry_msgs::TwistStamped setpoint_vel_ob; 

  //Setting the velocities to the object. 
  setpoint_vel_ob.twist.linear.x = vx; 
  setpoint_vel_ob.twist.linear.y = vy; 
  setpoint_vel_ob.twist.linear.z = 0; 
  // float yaw = (2.*M_PI) - ( atan2( currentState[1], currentState[0] ) - (M_PI/4.) );
  setpoint_vel_ob.twist.angular.z = yaw;


  set_pt_vel_pub.publish(setpoint_vel_ob);
  set_pt_vel_pub_1.publish(setpoint_vel_ob);
  set_pt_vel_pub_2.publish(setpoint_vel_ob);

}

// States are [rx, ry, vxo, vyo, vxi, vyi, dx, dy]
// The states coming from the autopilot will not be these, I'll need to calculate at least
// the ranges and perhaps some others in this function.

int readState(double *currentState, int numDims, double timeNow)
{
    // Read from MAVLink/autopilot
    if (READSTATELOCATION == 2)
        {   if ( not numDims==8 )
	             {   printf("ERROR: numDims not 6, is %i",numDims);
	    	           throw 1;
               }

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
		        currentState[0] = (double) (xi-xo); // rx
		        currentState[1] = (double) (yi-yo); // ry
		        currentState[2] = (double) vxo;     // vxo
		        currentState[3] = (double) vyo;     // vyo
		        currentState[4] = (double) vxi;     // vxi
		        currentState[5] = (double) vyi;     // vyi
            currentState[6] = (double) (xt-xo);      // dx
            currentState[7] = (double) (yt-yo);      // dy

		        printf("\n");
      }

  return 0;

}

// Sends command corresponding to actionInd to autopilot/MAVLink.  
// Actions are the following:
// 0  nothing
// 1  -x
// 2  +x
// 3  -y
// 4  +y

int writeCAAction(int actionInd, double *currentState, int numDims, FILE *fpOut)
{   double ax, ay;
    double vx_cmd, vy_cmd;
    double dt;
    struct tm *t;
    time_t timeVar;
    char str_time[10];

    timeVar = time(NULL);
    t = localtime(&timeVar);
    strftime(str_time, sizeof(str_time), "%H%M%S",t);

    fprintf(fpOut, "%s: ",str_time);

    dt = 1;

    switch (actionInd)
      {   case 0:   // Send command for zero acceleration
                    ax=0;
                    ay=0;
                    break;
          case 1:   // Send command for -x acceleration
                    ax=-1;
                    ay=0;          
                    break;
          case 2:   // Send command for +x acceleration
                    ax=1;
                    ay=0;       
                    break;
          case 3:   // Send command for -y acceleration
                    ax=0;
                    ay=-1;      
                    break;
          case 4:   // Send command for +y acceleration
                    ax=0;
                    ay=1;
                    break;
          default:
                    printf("Non supported action\n");
          return 1;
      }

  vx_cmd = (currentState[2]+ax*dt);
  vy_cmd = (currentState[3]+ay*dt);

  // Update nominal desired trajectory
  xt += NOMINAL_VX*dt;
  yt += NOMINAL_VY*dt;

  // Log the scaled commands (what the algorithm sees):
  fprintf(fpOut, "%d, %lf, %lf, ", actionInd, vx_cmd, vy_cmd);

  float vx=vx_cmd, vy=vy_cmd;
  float yaw = (2.*M_PI) - ( atan2( currentState[1], currentState[0] ) - (M_PI/4.) );

  set_velocity_ownship( vx, vy, yaw );

  printf("AVOID U-V-Psi    [ % .4f , % .4f, % .4f ]\n\n", vx, vy, yaw);

  // Log the actual output (which is passed to the autopilot)
  fprintf(fpOut, "%lf, %lf\n", vx_cmd, vy_cmd);
  
  /*************************************************************************/
  /****         Send vx_cmd and vy_cmd here                             ****/
  /*************************************************************************/
  return 0;

}


// Simply command a position, hard coded for now.
int writeNominalAction(FILE *fpOut)
{
    // Send nominal position command (non-scaled)
    float vx=NOMINAL_VX, vy=NOMINAL_VY;
    float yaw = 180.;


    // Probably not necessary to do this since it's not being used for nominal actions, 
    // but I'm going to do it so we don't end up with some huge error.
    xt = xo;
    yt = yo;

    set_velocity_ownship( vx, vy, yaw );
    printf("NOMINAL U-V-Psi  [ % .4f , % .4f , % .4f]\n\n", vx, vy, yaw);
    

    fprintf(fpOut, "-1\n");
    return 0;
}


// Returns true if the measurement indicate that an intruder is a threat, in which case the
// CA algorithm should be called.
int intruderThreat(double *currentState)
{   double range;
    range = sqrt(currentState[0]*currentState[0] + currentState[1]*currentState[1]);

    // std::cout<<"Deciding what action"<<std::endl;

    return (range<THREATRANGE);


}


// Writes key states to a log:
// [currentStep, commanded action, states[0:N]]
// These are the values seen by the algorithm, not the autopilot
int writeLogs(FILE *fpLog, int stepCounter, double* currentState, int numDims, int actionInd)
{
    int i;
    struct tm *t;
    time_t timeVar;
    char str_time[10];

    timeVar = time(NULL);
    t = localtime(&timeVar);
    strftime(str_time, sizeof(str_time), "%H%M%S",t);

    fprintf(fpLog, "%s: ",str_time);

    fprintf(fpLog, "%d, ", stepCounter);
    fprintf(fpLog, "%d, ", actionInd);

    for (i=0; i<numDims-1; i++) 
      {   fprintf(fpLog, "%lf, ", currentState[i]);
      }   
      fprintf(fpLog, "%lf, ", currentState[numDims-1]);

      // Also write the absolute positions
    fprintf(fpLog, "%lf, %lf, %lf, %lf, %lf, %lf\n", xi, yi, xo, yo, xt, yt);

    

    return 0;
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
  { 
	    // This program uses throw, wrap one big try/catch here
      ros::init(argc, argv, "ros_ca_flight");

	    vxo=0;
	    vyo=0;
	    vyi=0;
	    vxi=0;
	    xi=0;
	    yi=0;
	    yo=0;
	    xo=0;
      xt=0;
      yt=0;

		  std::cerr<<"MY god"<<std::endl;
      // ros::nodeHandler
      ros::NodeHandle nh_;
	    // own_imu_sub = nh_.subscribe("odroid1/mavros/imu/data",1,own_vel_callback);
      // own_pose_sub = nh_.subscribe("odometry",1,own_pose_callback);
	    // own_pose_sub = nh_.subscribe("/demo_odom",1,own_pose_demo_callback);
      //	own_pose_sub = nh_.subscribe("odroid1/odometry",1,own_pose_callback);
      // intruder_imu_sub = nh_.subscribe("odroid2/mavros/imu/data",1,intruder_vel_callback);
      // intruder_pose_sub = nh_.subscribe("odroid2/odometry",1,intruder_pose_callback);

      // own_pose_sub = nh_.subscribe("mavros/local_position/local",1,gps_own_pose_callback);
      own_pose_sub = nh_.subscribe("mavros/position/local",1,gps_own_pose_callback);
      own_vel_sub = nh_.subscribe("mavros/global_position/gps_vel",1,gps_own_vel_callback);
      // intruder_pose_sub = nh_.subscribe("odroid1/mavros/local_position/local",1,gps_intruder_pose_callback);
      intruder_pose_sub = nh_.subscribe("odroid1/mavros/position/local",1,gps_intruder_pose_callback);
      intruder_vel_sub = nh_.subscribe("odroid1/mavros/global_position/gps_vel",1,gps_intruder_vel_callback);

    	set_pt_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/cmd_vel",1);    
      set_pt_vel_pub_1 = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);  
      set_pt_vel_pub_2 = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint/cmd_vel",1);  

      FILE *fpIn, *fpData;
      int numVerticies, numElements; 

      // Needed in  decide_Algorithm() but allocated or evaluated 
      // in start_Algorithm():
      int numDims, numActions, *elementsDim;
      FILE *fpOut, *fpLog;
      double *neighY;
      double **discMat, **neighX;
      struct cd_grid **gridQsa;  // This should hold a vector of grids, one for each action
      struct stat st = {0};
      struct tm *t;
      time_t timeVar;
      char str_time[100];
      char log_destination[150];
      char results_destination[150];


      // Variables simply used in the algorithm functions, may be redeclared in each function:
      int i, j, k;
      int err, keyinput, exitCondition;

      if (VERBOSE) 
         printf("Reading parameter file...\n");    

      /* Open and check the files */
      // fpIn    = fopen("CA2_params.prm","r");
      // fpData  = fopen("CA2_data.dat","r");

      fpIn = fopen(argv[1],"r");
      fpData = fopen(argv[2],"r");
      // fpIn  = fopen("paramtemp.txt","r");
      // fpData  = fopen("datatemp.txt","r");
      // fpIn  = fopen("test.txt","r");
      // fpData  = fopen("data.txt","r");
      // fpIn  = fopen("param8Dim141201.txt","r");
      // fpData  = fopen("data8Dim141201.txt","r");
      if (stat("logs", &st) == -1)
        mkdir("logs", 0777);
      timeVar = time(NULL);
      t = localtime(&timeVar);
      strftime(str_time, sizeof(str_time), "%H%M%S",t);
      strcpy(results_destination,"./logs/CA2_results_");
      strcat(results_destination,str_time);
      strcat(results_destination,".out");
      fpOut = fopen(results_destination,"w");
      strcpy(log_destination,"./logs/CA2_results_");
      strcat(log_destination,str_time);
      strcat(log_destination,".log");
      fpLog = fopen(log_destination,"w");
      //printf("Output file name: %s", results_destination);
      /* Check for errors opening files */
      if (fpIn == NULL) 
        { fprintf(stderr, "Can't open input file param\n");
          exit(1);
        }
      if (fpData == NULL) 
        { fprintf(stderr, "Can't open data file data\n");
          exit(1);
        }
      if (fpLog == NULL) 
        { fprintf(stderr, "Can't open log file: %s\n", log_destination);
          exit(1);
        }
      if (fpOut == NULL) 
        { fprintf(stderr, "Can't open output file results out: %s\n", results_destination);
          exit(1);
        }
      

      /* Read in parameter file */

      // Number of dimensions:
      fscanf(fpIn, "%d", &numDims);

      // Number of grid points (states) in each dimension:
      numElements = 1;
      elementsDim = (int *)malloc(numDims*sizeof(int));
      discMat = (double **)malloc(numDims*sizeof(double));
      for (i=0; i<numDims; i++) 
        {  fscanf(fpIn, "%d", &elementsDim[i]);
           //printf("Dimension %d = %d\n", i, elementsDim[i]);
           numElements *= elementsDim[i];
          discMat[i] = (double *)malloc(elementsDim[i]*sizeof(double));
        }

      // Values of states at each grid point:
      for (i=0; i<numDims; i++) 
        {   for (j=0; j<elementsDim[i]; j++) 
              {   fscanf(fpIn, "%lf", &discMat[i][j]);
              }
        }

      // Number of actions:
      fscanf(fpIn, "%d", &numActions);
      fclose(fpIn);

      if (VERBOSE) 
         printf("Done reading parameter file\n");
  
      /* Finished reading parameter file */

      /* Create the grid data structure to hold contents as specified in the parameter file */
      //cell_init = (void *)malloc(numElements*sizeof(double));
      // gridInst = (struct cd_grid **) malloc(sizeof(struct cd_grid));
      double var;
      struct cd_grid **gridInst;
      int actionInd;
      int *subs; 
      gridInst = (struct cd_grid **) malloc(sizeof(struct cd_grid));
      gridQsa = (struct cd_grid **)malloc(numActions*sizeof(struct cd_grid));
      subs = (int *)malloc(numDims*sizeof(int));
 
      // To make this a single block of text for every possible value of numDims, need to be able to 
      // fill the "grid" with elements directly using the index and subscripts.  There may already be
      // a function in cd_grid to do this...
      
      if (VERBOSE) 
          printf("Reading data file\n");
  
      if (numDims==3) 
        {  double Q[elementsDim[0]][elementsDim[1]][elementsDim[2]];
      
      /* Using the contents of the parameter file, read and parse the data file */  
          for (actionInd=0; actionInd<numActions; actionInd++)
            {
                // I should probably be able to read in the elements directly to Q if it's allocated into a contiguous
                // memory block.  Could just use the index value, rather than going from  an index to a subscript and
                // back to and index? Just need to verify that this works, then make sure my revision matches this version.
                for (i=0;i<numElements;i++)
                  {  fscanf(fpData, "%lf", &var);
                     err = ind2subs(elementsDim, numDims, i, subs);
                     Q[subs[0]][subs[1]][subs[2]] = var;
                  }

                // After the number of dimensions, the size of each dimension must be passed in as a series of arguments.
                cd_grid_create_fill(gridInst, Q, sizeof(double), numDims, elementsDim[0], elementsDim[1], elementsDim[2]);
                gridQsa[actionInd] = *gridInst;
            }
        }

    if (numDims==6) 
      {   double Q[elementsDim[0]][elementsDim[1]][elementsDim[2]][elementsDim[3]][elementsDim[4]][elementsDim[5]];  
          for (actionInd=0; actionInd<numActions; actionInd++) 
            { for (i=0;i<numElements;i++)
                { fscanf(fpData, "%lf", &var);
                  err = ind2subs(elementsDim, numDims, i, subs);
                  Q[subs[0]][subs[1]][subs[2]][subs[3]][subs[4]][subs[5]] = var;
                }
              cd_grid_create_fill(gridInst, Q, sizeof(double), numDims, elementsDim[0], elementsDim[1], elementsDim[2], elementsDim[3], elementsDim[4], elementsDim[5]);
              gridQsa[actionInd] = *gridInst;
            } 
      }    

    if (numDims==8) 
      {   double Q[elementsDim[0]][elementsDim[1]][elementsDim[2]][elementsDim[3]][elementsDim[4]][elementsDim[5]][elementsDim[6]][elementsDim[7]];  
          for (actionInd=0; actionInd<numActions; actionInd++) 
            { for (i=0;i<numElements;i++)
                { fscanf(fpData, "%lf", &var);
                  err = ind2subs(elementsDim, numDims, i, subs);
                  Q[subs[0]][subs[1]][subs[2]][subs[3]][subs[4]][subs[5]][subs[6]][subs[7]] = var;
                }
              cd_grid_create_fill(gridInst, Q, sizeof(double), numDims, elementsDim[0], elementsDim[1], elementsDim[2], elementsDim[3], elementsDim[4], elementsDim[5], elementsDim[6], elementsDim[7]);
              gridQsa[actionInd] = *gridInst;
            }      
      }    

      // // Test the read-in data:
      // double testArray[numElements];
      // for (i=0;i<numElements;i++){
      //       err = ind2subs(elementsDim, numDims, i, subs);
      //       testArray[i] = *(double *)cd_grid_get_subs(gridQsa[0], subs);
      //       printf("%lf\n", testArray[i]);
      //     }

      fclose(fpData);
      free(gridInst);
      free(subs);
      
      if (VERBOSE) 
        printf("Done reading data file\n");
  
      // Allocate space for the interpolations needed to decide actions:
      numVerticies = pow(2,numDims);
      neighX = (double **)malloc(numDims*sizeof(double));
      
      if (neighX) 
        for (i=0;i<numDims;i++) 
          neighX[i]=(double *)malloc(numVerticies*sizeof(double));
    
      neighY = (double *)malloc(numVerticies*sizeof(double)); 

         /***********************************************************
          ***   The following would be in decide_Algorithm()       ***
          ************************************************************/
  
        int bestActionInd;
        double maxQsa;
        int stepCounter;
        double *qVals, *currentState, *indicies;

        // These allocations may be done on every call to decide_Algorithm(), they're not large
        qVals = (double *)malloc(numActions*sizeof(double));
        currentState = (double *)malloc(numDims*sizeof(double));
        indicies = (double *)malloc(numDims*sizeof(double));
        exitCondition = 0;
        stepCounter = 0;

        printf("BEGINNING SIMULATION\n");
        printf("\n");





    	// try
	    //    {	int result = top(argc,argv);
		   //      return result;
	    //    }
    	// catch ( int error )
	    //    {	fprintf(stderr,"main threw exception %i \n" , error);
		   //      return error;
	    //    }

	    // if (!(nh_.ok()))	  
		    // ros::spin();

      // if ((nh_.ok())&&(!exitCondition))
      int NOMINAL_MODE_LAST;
      NOMINAL_MODE_LAST = 1;
      while (!exitCondition)
        {           

        // while ((!exitCondition)&&(nh_.ok()))
             // read the current state from MAVLink (or a file, for now)
              err = readState(currentState, numDims, (double)stepCounter);
              // std::cerr<<"It actually reached here"<<std::endl;
              // find indicies into the grid using the current state
              err = getIndicies(indicies, currentState, numDims, elementsDim, discMat);

              // for each action:
              int actionInd;
              for (actionInd=0; actionInd<numActions; actionInd++) 
                {   
                    // read in the neighbor verticies to the data structure and read in the values at the verticies
                    err = getNeighbors(indicies, &gridQsa[actionInd], neighX, neighY, numDims, discMat);
                    // interpolate within the data file using the current state (call interpN()) and store
                    qVals[actionInd] = interpN(numDims, currentState, neighX, neighY, EXTRAPMODE);

                    if (VERBOSE) 
                      {   printf("Indicies:\n");
                          printf("[");
                          for (i=0;i<numDims;i++) 
                            {   printf("%lf, ", indicies[i]);
                            }
                          printf("]\n");

                          printf("Neighbors:\n");
                          printf("[");
                          for (i=0;i<numDims;i++) 
                            {   for(j=0;j<numVerticies; j++)
                                  {   printf("%lf, ", neighX[i][j]);
                                  }
                                printf("\n");
                            }

                          printf("Neighbor values:\n");
                          printf("[");
                          for (i=0;i<numVerticies;i++) 
                            {   printf("%lf, ", neighY[i]);
                            }
                          printf("]\n");

                          printf("Interpolated Value: %lf\n", qVals[actionInd]);
                      }
                }

              // calculate the best action
              bestActionInd = 0;
              maxQsa = qVals[0];
              
              for (i=1;i<numActions;i++) 
                { if (qVals[i] > maxQsa) 
                    {   maxQsa = qVals[i];
                        bestActionInd = i;
                    }

                }

              // write the action to MAVLink (or a file, for now)
              if (intruderThreat(currentState)) 
              {
                  err = writeCAAction(bestActionInd, currentState, numDims, fpOut); 
                  if (NOMINAL_MODE_LAST)
                  {
                    // We were just in nominal mode, so record the current position as the start of the desired trajectory
                    xt = xo;
                    yt = yo;
                    NOMINAL_MODE_LAST = 0;
                  }
              }
              else
              {
                  err = writeNominalAction(fpOut);
                  NOMINAL_MODE_LAST = 1;
              }
  
              // log the current state and action to a log file
              err = writeLogs(fpLog, stepCounter, currentState, numDims, bestActionInd);

              // Increment the counter (proxy for the current time when writing to file)
              stepCounter++;

              // While debugging, stop after a predetermined number of steps:
              // if (stepCounter>=MAXSIMSTEPS)
              //   { printf("ENDING SIMULATION\n\n");
              //     exitCondition = 1;
              //   }
    
              // if (time_to_exit){
              // exitCondition = 1;
              // }
              if (nh_.ok())
                ros::spinOnce();
                // ros::spin();
              else
                exitCondition = 1;
        }

      free(currentState);
      free(qVals);
      free(indicies);
  
      for (i=0;i<numActions;i++) 
        {  cd_grid_destroy(gridQsa[i]);
        }
      
      free(gridQsa);
      for (i=0; i<numDims; i++) 
        {  free(neighX[i]);
           free(discMat[i]);
        }
      free(discMat);
      free(elementsDim);
      free(neighX);
      free(neighY);
    

      /* close the log and output files */
      fclose(fpOut);
      fclose(fpLog);
      printf("Exiting\n");

      return 0;

  }