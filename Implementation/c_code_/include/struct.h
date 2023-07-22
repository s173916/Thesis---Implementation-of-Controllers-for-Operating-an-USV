
#include <time.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>

#ifndef STRUCT_H
#define STRUCT_H

//typedef enum {true, false} bool;


// global
//time_t start,update;
#define M_M_PI 3.14159265359
#define D 1.3395

//void pid_out(state *st)

typedef struct{ //input signals
    double F1, F2, alpha1, alpha2, alpha_1, alpha_2, F_old_1, F_old_2, alpha_1_old, alpha_2_old, F_1_res, F_2_res, alpha_1_res, alpha_2_res;


    double F_x, F_y, tau; 
    int count_sway;

    int count, count_old;
    int time_s;
} thrustType;
//thrustType rawData;    //Define raw data  


typedef struct{ //input signals
    bool turn_above_45_flag, rot_thrust, alpha_flag, sway_flag, turn_yes;
} flagType;
//flagType flagData;    //Define flag data  


typedef struct{ //input signals
    double ref_u, ref_v, ref_yaw, yaw_USV, u_con, v_con, yaw_con, psi_con;
} controlType;
//controlType controlData;    //Define control data  


typedef struct{
    double u_local, v_local, psi_local, yaw_USV;
} sensorType;
//sensorType sensorData;


typedef struct{ //input signals
    double pos_x, pos_y, theta;
} simType;
//simType simData;    //Define sim data 


/*
thrustType rawData;    //Define raw data  
flagType flagData;    //Define flag data 
controlType controlData;    //Define control data  
sensorType sensorData;
simType simData;    //Define sim data 
*/

#endif