#include "struct.h"
//#include "extern_struct.h"

extern thrustType rawData;    //Define raw data  
extern flagType flagData;    //Define flag data 
extern controlType controlData;    //Define control data  
extern sensorType sensorData;
extern simType simData;    //Define sim data 


void vel_2_thrust(controlType *con, thrustType * raw){
    
 // Force surge (u)
    int k_u = 0;
    if ((con -> u_con) < 0){
        k_u = -1;
    }else{
        k_u = 1;
    }
    
    if ((con -> u_con) == 0){
        raw -> F_x = 0; 
    }else if(fabs(con -> u_con) < 1){   
        raw -> F_x = 2*59.7 * fabs(con ->u_con) * k_u;
    }else{
        //T = 2 * (19.2 *fabs(u_con)^2 + 1.5* fabs(u_con) + 0)*k;
        raw -> F_x = 2 * (19.2 * pow(fabs(con -> u_con),2) + 1.5 * fabs(con->u_con) + 39)*k_u;
        //T = ((x_u + x_uu * fabs(u)) * fabs(u))*k;
    }
 
    if (raw -> F_x == 0.0){
        raw->F_x = 0.00000000000000000001;
    }


 // Force sway (v)
    int x_u = 51.3;
    int x_uu = 72.4 ;
    //knob_2_ms = 0.5144;
    int k_side = 4; // factor for vel sideway
    int k_v = 0;

    if (con -> v_con < 0){
        k_v = -1;
    }else{
        k_v = 1;
    }

    raw -> F_y = k_side*((x_u + x_uu * fabs(con -> v_con)) * fabs(con -> v_con))*k_v;
    
    if (raw -> F_y == 0.0){
        raw -> F_y = 0.00000000000000000001;
    }


 // Tau
    raw -> tau = con -> psi_con * 5450;

}
