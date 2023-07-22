

#include "struct.h"
//#include "ctr.h"
//#include "extern_struct.h"



extern thrustType rawData;    //Define raw data  
extern flagType flagData;    //Define flag data 
extern controlType controlData;    //Define control data  
extern sensorType sensorData;
extern simType simData;    //Define sim data 



/*

float PI_ctr(controlType *con, thrustType * raw, sensorType * sensor){
   if(tick_count == 0)
    {
        if (yaw_control == false)
        {
        // Controller input, minus lead output, times kp.
        ui[0] = (ref - yl[0]) * kp;
        }
        else
            ui[0] = angle_diff(ref, yl[0]) * kp;
    }
    else
    {
        ui[1] = ui[0];
        
        if (yaw_control == false)
            ui[0] = (ref - yl[0]) * kp;
        else
            ui[0] = angle_diff(ref, yl[0]) * kp;
        
        yi[1] = yi[0];
    }

    // Integral
    double tmp = 0.0;
    tmp = ((2 * tau_i * yi[1]) + (Ts * ui[1]) + (Ts * ui[0])) / (2 * tau_i);
    
    if(tmp > m_i_max)
        tmp = m_i_max;
    else if(tmp < m_i_min)
        tmp = m_i_min;

    yi[0] = tmp;

    return yi[0];
}


*/























// float error(char con_type[], controlType *con, sensorType * sensor){
//     float err = 0;
//     if(con_type == "PI_u"){
//         err = con->ref_u - sensor->u_local;
//     } else if(con_type == "PI_v"){
//         err = con->ref_v - sensor->v_local;
//     } else if(con_type == "P_yaw"){
//         err = con->ref_yaw- sensor->yaw_USV;
//     } else if(con_type == "PI_psi"){
//         err = sensor->yaw_USV - sensor->psi_local;
//     } else{
//         return 0;
//     }
//     return err;
// }


// void PI_u_con(controlType *con, thrustType * raw, sensorType * sensor){
//     // Integral
//     float tmp = 0.0;
//     float ts = 0.001;
//     float tau_i = 1.533;
//     float kp = 2.56;


//     tmp = ((2 * tau_i * yi[1]) + (Ts * ui[1]) + (Ts * ui[0])) / (2 * tau_i);
    
//     if(tmp > m_i_max)
//         tmp = m_i_max;
//     else if(tmp < m_i_min)
//         tmp = m_i_min;

//     yi[0] = tmp;

//     return yi[0];

// }

// void PI_v_con(controlType *con, thrustType * raw, sensorType * sensor){
    
// }

// void PI_psi_con(controlType *con, thrustType * raw, sensorType * sensor){
    
// }

// void PI_yaw_con(controlType *con, thrustType * raw, sensorType * sensor){
    
// }






// void controllers(controlType *con, thrustType * raw, sensorType * sensor){
//     // controllers 

//     con -> u_con; 
//     con -> v_con;
//     con -> psi_con;
//     con -> yaw_con;

//     PI_u_con(&controlData, &rawData, &sensorData);
//     PI_v_con(&controlData, &rawData, &sensorData);
//     PI_psi_con(&controlData, &rawData, &sensorData);
//     PI_yaw_con(&controlData, &rawData, &sensorData);

//}

