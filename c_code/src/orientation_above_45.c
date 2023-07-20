
#include"struct.h"
#include "orient_a_45.h"
//#include "extern_struct.h"

extern thrustType rawData;      //Define raw data  
extern flagType flagData;       //Define flag data 
extern controlType controlData; //Define control data  
extern sensorType sensorData;
extern simType simData;         //Define sim data 


//void orientation_above_45((con->ref_u),(con->ref_v), (con->ref_yaw), (sensor->yaw_USV), (flag -> turn_above_45_flag), (flag -> rot_thrust)){
void orientation_above_45(controlType * con, sensorType * sensor, flagType * flag){
   
    int direction = 0;
    if ((con -> ref_yaw) <= (sensor -> yaw_USV)){
        direction = -1;
    }else{
        direction = 1;
    } 
        
    if ((fabs((con -> ref_yaw) - (sensor -> yaw_USV)) > 0.2) && (fabs((con -> ref_v))>=fabs((con -> ref_u))) || (flag -> turn_above_45_flag) == 1){
        flag -> turn_above_45_flag = 1;
        con -> ref_u = 0.0;  
        con -> ref_v = 0.0;
        con -> ref_yaw =  ((sensor -> yaw_USV) + 0.2*direction); //ref_yaw; 
       
        if (fabs((con -> ref_yaw) - (sensor -> yaw_USV)) < 0.05){
             flag -> turn_above_45_flag = 0;      
        }
        
    }else{ 
        (con -> ref_u) = (con -> ref_u);  
        (con -> ref_v) = (con -> ref_v);
        (con -> ref_yaw) = (con -> ref_yaw);
        (flag -> turn_above_45_flag) = 0;
    }
   
    //rot_thrust_out = rot_thrust;
}
