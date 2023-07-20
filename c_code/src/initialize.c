
#include"struct.h"
#include"initial.h"
#include <pid.h>
//#include "extern_struct.h"


extern thrustType rawData;      //Define raw data  
extern flagType flagData;       //Define flag data 
extern controlType controlData; //Define control data  
extern sensorType sensorData;
extern simType simData;         //Define sim data 

extern stateType surge;
extern stateType sway;
extern stateType rot;
extern stateType yaw;




void initRawData(thrustType * raw) { 
	raw -> F1 = 0;
    raw -> F2 = 0;
    raw -> alpha1 = 0;
    raw -> alpha2 = 0;
    raw-> alpha_1 = 0;
    raw-> alpha_2 = 0;	
    raw -> F_x = 0;
    raw -> F_y = 0;
    raw -> tau = 0;
    raw -> F_old_1 = 0;
    raw -> F_old_2 = 0;
    raw -> alpha_1_old = 0;
    raw -> alpha_2_old = 0;
    raw -> F_1_res = 0;
    raw -> F_2_res = 0;
    raw -> alpha_1_res = 0;
    raw -> alpha_2_res =0;



    raw -> count_sway = 0;

    raw -> count = 0;
    raw -> count_old = 0;
    raw -> time_s = 0;
}


void initFlagData(flagType * flag) { 
    flag -> turn_above_45_flag = 0;
    flag -> rot_thrust = 0;
    flag -> alpha_flag = 0;
    flag -> sway_flag = 0;
    flag -> turn_yes = 0;
}


void initSensorData(sensorType * sensor) { 
    sensor -> u_local = 0;
    sensor -> v_local = 0;
    sensor -> psi_local = 0;
    sensor -> yaw_USV = 0;
}


void initControlData(controlType * con) { 
    con -> ref_u = 0;
    con -> ref_v = 0;
    con -> ref_yaw = 0;
    con -> u_con = 0;
    con -> v_con = 0;
    con -> yaw_con = 0;
    con -> psi_con = 0;
}


void initSimData(simType * sim) { 
	sim -> pos_x = 0;
    sim -> pos_y = 0;
    sim -> theta = 0;
}


void init() {
    // cals the initialize functions
	initRawData(&rawData);// Initialyse rawData
    initSimData(&simData);
    initFlagData(&flagData);
    initSensorData(&sensorData);
    initControlData(&controlData);

    // PID init
    // pid_init(struct State *st, int id, double ts, double Kp, double Tau_i, double Tau_d, double Alpha, double min, double max, double min_vel_surge,double max_vel_surge);
    pid_init(&surge, 1, 1.0/10.0, 1.43, 0.333, 1.4907, 0.05, 0.0, 0.0, -8*0.54133, 8*0.54133);
    pid_init(&sway, 2, 1.0/10.0, 1.58, 0.333, 1.4907, 0.05, -2*0.54133-0.5, 2*0.54133+0.5, 0.0, 0.0);
    pid_init(&rot, 3, 1.0/10.0, 1.2, 3.0303, 0.0, 0.0, -0.2, 0.2, 0.0, 0.0);
    pid_init(&yaw, 4, 1.0/10.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0);

	printf("initiliseret \n");
}

