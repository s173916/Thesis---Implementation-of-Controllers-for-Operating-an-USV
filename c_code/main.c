# include <stdio.h>
# include <stdlib.h>
# include <sys/time.h>
# include <sys/types.h>
# include <unistd.h>
# include <signal.h>
# include <pthread.h>
#include "struct.h"
#include "initial.h"
#include "orient_a_45.h"
#include "vel_2_thrust.h"
//#include "ctr.h"
#include "mix.h"
#include "lim.h"
#include <pid.h>
#include <time.h>
#include <sys/time.h>
#include "/home/frederik/Documents/speciale/rhd-mbzirc-master/include/rhd.h"



// defines structs
thrustType rawData;     //Define raw data  
flagType flagData;      //Define flag data 
controlType controlData;//Define control data  
sensorType sensorData;
simType simData;        //Define sim data 
// defines controller structs 
stateType surge;
stateType sway;
stateType rot;
stateType yaw;
// interval for sample time 
struct itimerval it;
struct timeval tv;
sigset_t signalset;
int sig;




struct timeval t1;





// Prototypes
//void* controller_tick(void* args);
//void timer_setup();
void end(int);
int connectRhd(char *hostname);


// ####################### input references #######################
#define Ref_surge 2*0.5144
#define Ref_sway 2*0.5144
#define Ref_yaw 0.0


// ############################## main ############################
int main(int argc, char *argv){ 
    //pthread_t tick; // treath 
    signal(SIGTERM, end);
    signal(SIGINT, end);
    signal(SIGKILL, end);

    init(); // initializing
    //time start = clock();
    //pthread_create(&tick, NULL, controller_tick, NULL);
    
    
    if (connectRhd("127.0.0.1") < 0) {
      exit(0);
    }



    //clock_t start = clock(); // timer start
    //clock_t old = start; 

    //printf("Clock per sec = %d \n", CLOCKS_PER_SEC);

	int count = 0;
    // main loop


    // sensor
    int count_G = 0;
    double gyro_val = readValueNamed("kvhpos",0)/10000.00;
    double gyro_val_old = gyro_val;
    //off_pr_sec = 0.0000060*10000000 = 60;
    int off_pr_sec = 60;
    writeValueNamed("kvhoffset", 0 , off_pr_sec ); // updating Gyro offset in rhd
    double gyro_cal = readValueNamed("kvhpos",0)/10000.00; // callibration value for the gyro

    while(1){
    //for(;;) {    
        //update_val();
        //clock_t start = clock(); // timer start
        // Execuatable code


	if (count == 10){
	count = 0;
	//printf("---------------10 iterationer--------------------- \n");
	}
        count ++;



        
        // //float*motor_set = trust(F_x, F_y, tau );  
        
        // // Prints out the solution of the trust and angles
        // //cout << "Index = " << index_out;
        
        // /*
        // printf("Alpha_1 = //d \n", alpha1);
        // printf("raw->F1 = //d \n", F1);
        // printf("Alpha_2 = //d \n", alpha2);
        // printf("raw->F2 = //d \n", F2);
        // */


        // // waiting for signal (to ensure sampletime for the program)
        // sigwait(&signalset, &sig);
        // // rhd sync
        char rhd_sync = rhdSync();
     //   printf("rhd sync return = %x \n", rhd_sync);
        gettimeofday(&t1,NULL);
        printf("micro seconds : %lf \n", t1.tv_usec/1000000.00);
        //start = clock();
        //double elapsed = (double)(start - old) * 1000.0 / CLOCKS_PER_SEC;
        //printf("Time elapsed in ms: %f \n", elapsed);
        //old = start;
        
        
         printf("\n------------------Sensor val.---------------- \n");
        // reading from the rhd client 
        sensorData.yaw_USV = readValueNamed("kvhpos",0)/10000.00 - gyro_cal;
        printf("gyro val = %f \n",sensorData.yaw_USV);
        sensorData.psi_local = readValueNamed("kvhrate",0)/10000000.00;
        printf("gyro vel = %f \n",sensorData.psi_local);
        sensorData.u_local = (readValueNamed("GPSspeed",0)/1000.00) * sin(sensorData.yaw_USV);
        sensorData.v_local = (readValueNamed("GPSspeed",0)/1000.00) * cos(sensorData.yaw_USV);
        printf("u_vel = %f \n",sensorData.u_local);
        printf("v_vel = %f \n",sensorData.v_local );


        if (count_G == 100){
            gyro_val = readValueNamed("kvhpos",0)/10000.00;
            double gyro_val_pr_ittr = (gyro_val-gyro_val_old)/100;
            double val_pr_sec = gyro_val_pr_ittr*10;
            
            printf("\n------------------------------------------------------------------------------------- \n" );

            printf("gyro stamp pr sec. = %f  ,  gyro stamp pr ittr. = %f \n" , val_pr_sec, gyro_val_pr_ittr);
            // 0.000006 pr ittr. og 0.000060 pr sec
            gyro_val_old = gyro_val;
            count_G = 0;
        }
        count_G ++;


        // connecting the ref to the control_struct
        controlData.ref_u = Ref_surge;
        controlData.ref_v = Ref_sway;
        controlData.ref_yaw = Ref_yaw;    

        // printf("F_x = %f \n", rawData.F_x);
        // printf("F_y = %f \n", rawData.F_y);
        // printf("tau = %f \n", rawData.tau);
        // printf("ref Theta = %f \n", Ref_yaw);
        // printf("psi_con = %f \n", controlData.psi_con);

        // orientation reference overwriter when going above 45 degrees 
        orientation_above_45(&controlData, &sensorData, &flagData);

        printf("\n------------------References---------------- \n");
        printf("ref_u_con = %f \n", controlData.ref_u);
        printf("ref_v_con = %f \n", controlData.ref_v);
        printf("res_yaw_con = %f \n", controlData.ref_yaw);

    
        // Controllers
        // Connecting values from controller_struct and sensor_struct to PID 
        surge.ref = controlData.ref_u;
        surge.measurement = sensorData.u_local;
        sway.ref = controlData.ref_v;
        sway.measurement = sensorData.v_local;
        yaw.ref = controlData.ref_yaw;
        yaw.measurement = sensorData.yaw_USV;
        rot.measurement = sensorData.psi_local;


        // printf("mes_u = %f \n", surge.measurement);
        // printf("ref_u = %f \n", surge.ref);
        // printf("mes_v = %f \n", sway.measurement);
        // printf("ref_v = %f \n", sway.ref);
        // printf("mes_yaw = %f \n", yaw.measurement);
        // printf("res_yaw = %f \n", yaw.ref);
        // printf("mes_psi = %f \n", rot.measurement);
        
       

        printf("\n------------------Controller---------------- \n");
        //surge_min_max(&surge, sway.ref); // setting flag for surge controller         
        pid_out(&surge,sway.ref); // Surge velocity controller 
        pid_out(&sway,0);         // Sway velocity controller 
        pid_out(&yaw,0);          // Position controller for Yaw (input for Rotational)
        rot.ref = yaw.out;        // inputs the output from yaw controller to the rot controller
        //printf("res_psi = %f \n", rot.ref);
        pid_out(&rot,0);          // Rotational velocity controller 
        // connectiong the control_strucht again
        controlData.u_con = surge.out;
        controlData.v_con = sway.out;
        controlData.psi_con = rot.out;

       
        //controlData.u_con = 0;
        printf("con_Fx_out = %f \n", controlData.u_con);
        printf("con_Fy_out = %f \n", controlData.v_con);
        printf("con_tau_out = %f \n", controlData.psi_con);

        


        // controlData.u_con = 2;
        // controlData.v_con = 2;
        // controlData.psi_con = 0.1;
        
        printf("\n------------------Mixer in--------------- \n");
        //update(); // Update controller vals 


        // Velocity to Thrust (-1000 -> 1000)
        vel_2_thrust(&controlData, &rawData); 

        printf("\n-------------------Mixer----------------- \n");


        // Mixer matrice
        mixer(&controlData, &rawData, &sensorData, &flagData);

        printf("\n---------------Limiter/Results------------- \n");
        // Limiter 
        limmitter_angle_thrust(&rawData, &controlData, &flagData);

        

        // Updating input for the USV
        // rawData.F_1_res = rawData.F1;
        // rawData.F_2_res = rawData.F2;
        // rawData.alpha_1_res = rawData.alpha_1;
        // rawData.alpha_2_res = rawData.alpha_2;
        // int i=0;
        // if (i == 0){
        //     return; 
        // }

        // // timer stop
        // clock_t stop = clock();
        // double elapsed = (double)(stop - start) * 1000.0 / CLOCKS_PER_SEC;
        // printf("Time elapsed in ms: %f \n", elapsed);

       
    }
return 0;
}


// ############### timer  function for sampletime ################
void timer_setup(){
    sigemptyset(&signalset);
    sigaddset(&signalset, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &signalset, NULL);

    getitimer(ITIMER_REAL, &it);
    tv.tv_sec = 0.0;
    tv.tv_usec = 1.0/10.0;
    it.it_interval = tv;
    it.it_value = tv;
    setitimer(ITIMER_REAL, &it, NULL);
}



// #################### thread program (main) ####################
void* controller_tick(void* args){
    
    while (true){
        // waiting for signal (to ensure sampletime for the program)
        sigwait(&signalset, &sig);
        // rhd sync


        // reading from the rhd client 
        sensorData.yaw_USV = readValueNamed("kvhpos",0);
        sensorData.psi_local = readValueNamed("kvhrate",0);
        



        // connecting the ref to the control_struct
        controlData.ref_u = Ref_surge;
        controlData.ref_v = Ref_sway;
        controlData.ref_yaw = Ref_yaw;    


        // orientation reference overwriter when going above 45 degrees 
        orientation_above_45(&controlData, &sensorData, &flagData);

               
        // Controllers
        // Connecting values from controller_struct and sensor_struct to PID 
        surge.ref = controlData.ref_u;
        surge.measurement = sensorData.u_local;
        sway.ref = controlData.ref_v;
        sway.measurement = sensorData.v_local;
        yaw.ref = controlData.ref_yaw;
        yaw.measurement = sensorData.yaw_USV;
        rot.measurement = sensorData.psi_local;
        //surge_min_max(&surge, sway.ref); // setting flag for surge controller         
        pid_out(&surge,sway.ref); // Surge velocity controller 
        pid_out(&sway,0);         // Sway velocity controller 
        pid_out(&yaw,0);          // Position controller for Yaw (input for Rotational)
        rot.ref = yaw.out;        // inputs the output from yaw controller to the rot controller
        pid_out(&rot,0);          // Rotational velocity controller 
        // connectiong the control_strucht again
        controlData.u_con = surge.out;
        controlData.v_con = sway.out;
        controlData.psi_con = rot.out;
        
        //update(); // Update controller vals 


        // Velocity to Thrust (-1000 -> 1000)
        vel_2_thrust(&controlData, &rawData); 


        // Mixer matrice
        mixer(&controlData, &rawData, &sensorData, &flagData);


        // Limiter 
        limmitter_angle_thrust(&rawData, &controlData, &flagData);


        // Updating input for the USV
        rawData.F_1_res = rawData.F1;
        rawData.F_2_res = rawData.F2;
        rawData.alpha_1_res = rawData.alpha_1;
        rawData.alpha_2_res = rawData.alpha_2;
    }
    
}

int connectRhd(char *hostname) {
    char rhd_con = rhdConnect('w',hostname,DEFAULTPORT);
    printf("rhd connect return %x \n" , rhd_con);
  if (rhd_con <= 0) {
      printf("Error: Failed to connect to RHD on host: %s\n",hostname);
      return -1;
   } else {
      printf("Successfully connected to RHD\n");
   }
  return 1;

}

void end (int sig) {
    printf("Shutting down usr client!!!!!!!\n");
    exit(1);
}
