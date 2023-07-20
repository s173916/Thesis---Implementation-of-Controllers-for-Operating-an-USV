



#include <time.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>

//typedef enum {true, false} bool;



typedef struct{ //input signals
    double F1, F2, alpha1, alpha2, alpha_1, alpha_2, F_old_1, F_old_2, alpha_1_old, alpha_2_old;


    double F_x, F_y, tau; 
    int count_sway;

    int count, count_old;
    int time_s;
} thrustType;
thrustType rawData;    //Define raw data  


typedef struct{ //input signals
    bool turn_above_45_flag, rot_thrust, alpha_flag, sway_flag, turn_yes;
} flagType;
flagType flagData;    //Define flag data  



typedef struct{ //input signals
    double ref_u, ref_v, ref_yaw, yaw_USV, u_con, v_con, yaw_con, psi_con;
} controlType;
controlType controlData;    //Define control data  


typedef struct{
    double u_local, v_local, psi_local, yaw_USV;
} sensorType;
sensorType sensorData;



typedef struct{ //input signals
    double pos_x, pos_y, theta;
} simType;
simType simData;    //Define sim data 



// global
time_t start,update;
#define M_M_PI 3.14159265359
#define D 1.3395



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
	printf("initiliseret \n");
}




//void orientation_above_45((con->ref_u),(con->ref_v), (con->ref_yaw), (sensor->yaw_USV), (flag -> turn_above_45_flag), (flag -> rot_thrust)){
void orientation_above_45(controlType * con, sensorType * sensor, flagType * flag){
   
    int direction = 0;
    if ((con -> ref_yaw) <= (sensor -> yaw_USV)){
        direction = -1;
    }else{
        direction = 1;
    } 
        
    if ((abs((con -> ref_yaw) - (sensor -> yaw_USV)) > 0.2) && (abs((con -> ref_v))>=abs((con -> ref_u))) || (flag -> turn_above_45_flag) == 1){
        flag -> turn_above_45_flag = 1;
        con -> ref_u = 0.00001;  
        con -> ref_v = 0;
        con -> ref_yaw =  ((sensor -> yaw_USV) + 0.2*direction); //ref_yaw; 
       
        if (abs((con -> ref_yaw) - (sensor -> yaw_USV)) < 0.05){
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


void controllers(controlType *con, thrustType * raw, sensorType * sensor){
    // controllers 

    con -> u_con; 
    con -> v_con;
    con -> psi_con;
    con -> yaw_con;
}



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
    }else if(abs(con -> u_con) < 1){   
        raw -> F_x = 2*59.7 * abs(con ->u_con) * k_u;
    }else{
        //T = 2 * (19.2 *abs(u_con)^2 + 1.5* abs(u_con) + 0)*k;
        raw -> F_x = 2 * (19.2 * pow(abs(con -> u_con),2) + 1.5 * abs(con->u_con) + 39)*k_u;
        //T = ((x_u + x_uu * abs(u)) * abs(u))*k;
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

    raw -> F_y = k_side*((x_u + x_uu * abs(con -> v_con)) * abs(con -> v_con))*k_v;
    
    if (raw -> F_y == 0.0){
        raw -> F_y = 0.00000000000000000001;
    }


 // Tau
    raw -> tau = con -> psi_con * 5450;

}





void mixer(controlType * con, thrustType * raw, sensorType * sensor, flagType * flag){
    //limmitting the input signal 
    if ((( sensor -> u_local < 0.02 && sensor -> u_local > -0.02) && (abs(con -> ref_u)< 0.02 && abs(con -> ref_v)< 0.02) && abs(raw -> tau) > 0.01) && flag -> alpha_flag == 0) {
        raw -> F_x = 0.0001;
        raw -> F_y = 0.0001;
        
    }
    
    raw -> F1 = (0.5000*(raw ->F_x*raw->tau + pow(raw->F_x,2) * D - raw->tau*(raw->F_x - sqrt(pow(raw->F_x,2) + pow(raw->F_y,2))) - raw->F_x * D * (raw->F_x - sqrt(pow(raw->F_x,2) + pow(raw->F_y,2)))))/(raw->F_x*D);
    raw -> F2 = -(0.5000*(raw->F_x*raw->tau - pow(raw->F_x,2) * D - raw->tau*(raw->F_x - sqrt(pow(raw->F_x,2) + pow(raw->F_y,2))) + raw->F_x * D * (raw->F_x - sqrt(pow(raw->F_x,2) + pow(raw->F_y,2)))))/(raw->F_x*D);
    raw->alpha_1 = -2*atan((raw->F_x - sqrt(pow(raw->F_x,2) + pow(raw->F_y,2)))/raw->F_y);
    raw->alpha_2 = raw->alpha_1;
    
    // setting the pods when rotating 
    if ((raw->F_x == raw->F_y && raw->F_x < 0.001 && raw->F_x > -0.001)  && flag->alpha_flag == 0){
       raw->alpha_1 = 0;
       raw->alpha_2 = 0;
    }
    
    
   /* disp("================================================")
    disp("alpha_1 = ")
    disp(alpha_1)
    */

    // sideway: alpha > 70 degrees
    if (abs(raw->alpha_1) >= 7*M_PI/18 && abs(raw->alpha_1) <= 11*M_PI/18 || abs(raw->alpha_1) <= 25*M_PI/18 && abs(raw->alpha_1) >= 29*M_PI/18){  
        
        // between +-180 degrees
        if (raw->alpha_1 > M_PI){ 
            raw->alpha_1 = raw->alpha_1 - 2 * M_PI;
        }else if (raw->alpha_1 < -M_PI){
            raw->alpha_1 = raw->alpha_1 + 2 * M_PI;
        }       
        // determining the right angle and thrust    
        if (raw->alpha_1 > M_PI/2){       //(above 90 degrees)
            raw->alpha_1 = raw->alpha_1 - M_PI;
        }else if (raw->alpha_1 < -M_PI/2){  //(below -90 degrees)
            raw->alpha_1 = raw->alpha_1 + M_PI;
        }
        
        /*disp("==---====---===---===---===---===---===---===---===")
        disp("alpha_1 = ")
        disp(alpha_1)
        */
//         fprintf("Er HER alpha 1 :---------")
//         disp(alpha_1)
        // if alpha is positvie 
        if (raw -> alpha_1 >= 0){ //|| raw->alpha_1 == -M_PI/2 //&& raw->alpha_1 < M_PI/2
            raw -> F1 = 1.4618*raw->F_x + 1.0913*raw->tau;
            raw -> F2 = (0.5000*raw->F_x - 0.3733*raw->tau - (1.3736*raw->F_x*(2.0634e+18*raw->F_x - 1.5405e+18*raw->tau + 1.4142*sqrt(1.8196e+37*pow(raw->F_x,2) - 2.3394e+37*raw->F_x*raw->F_y + 2.0811e+37*raw->F_x*raw->tau + 8.5155e+36*pow(raw->F_y,2) - 1.7465e+37*raw->F_y*raw->tau + 1.0141e+37*pow(raw->tau,2))))/(5.6687e+18*raw->F_x - 4.1269e+18*raw->F_y + 4.2320e+18*raw->tau) + (raw->F_y*(2.0634e+18*raw->F_x - 1.5405e+18*raw->tau + 1.4142*sqrt(1.8196e+37*pow(raw->F_x,2) - 2.3394e+37*raw->F_x*raw->F_y + 2.0811e+37*raw->F_x*raw->tau + 8.5155e+36*pow(raw->F_y,2) - 1.7465e+37*raw->F_y*raw->tau + 1.0141e+37*pow(raw->tau,2))))/(5.6687e+18*raw->F_x - 4.1269e+18*raw->F_y + 4.2320e+18*raw->tau) - (1.0255*raw->tau*(2.0634e+18*raw->F_x - 1.5405e+18*raw->tau + 1.4142*sqrt(1.8196e+37*pow(raw->F_x,2) - 2.3394e+37*raw->F_x*raw->F_y + 2.0811e+37*raw->F_x*raw->tau + 8.5155e+36*pow(raw->F_y,2) - 1.7465e+37*raw->F_y*raw->tau + 1.0141e+37*pow(raw->tau,2))))/(5.6687e+18*raw->F_x - 4.1269e+18*raw->F_y + 4.2320e+18*raw->tau));
            raw -> alpha_1 = 7*M_PI/18; // 70 degrees
            raw -> alpha_2 = 2*atan((2.0634e+18*raw->F_x - 1.5405e+18*raw->tau + 1.4142*sqrt(1.8196e+37*pow(raw->F_x,2) - 2.3394e+37*raw->F_x*raw->F_y + 2.0811e+37*raw->F_x*raw->tau + 8.5155e+36*pow(raw->F_y,2) - 1.7465e+37*raw->F_y*raw->tau + 1.0141e+37*pow(raw->tau,2)))/(5.6687e+18*raw->F_x - 4.1269e+18*raw->F_y + 4.2320e+18*raw->tau));
            printf("Er HER alpha positiv");
            //fprintf("F1 =");
            //disp(F1);
            //fprintf("F2 =");
            //disp(F2);    
            //fprintf("alpha1 =");
            //disp(alpha_1);
            //fprintf("alpah2 =");
            //disp(alpha_2);
//             if raw->alpha_2 < 0 && raw->alpha_1 > 0
//                 raw->alpha_2 = M_PI/2;
//                 F2 = -F2;
//             end
            
        }else{ // if alpha is negative
            raw->F1 = 1.4618*raw->F_x + 1.0913*raw->tau;
            raw->F2 = 3.4268e-19*sqrt(1.8196e+37*pow(raw->F_x,2) + 2.3394e+37*raw->F_x*raw->F_y + 2.0811e+37*raw->F_x*raw->tau + 8.5155e+36*pow(raw->F_y,2) + 1.7465e+37*raw->F_y*raw->tau + 1.0141e+37*pow(raw->tau,2));
            raw->alpha_1 = -7*M_PI/18; // -70 degrees
            //alpha_2 = 2*atan((2.0634e+18*F_x - 1.5405e+18*tau + 1.4142*sqrt(1.8196e+37*pow(raw->F_x,2) - 2.3394e+37*F_x*F_y + 2.0811e+37*F_x*tau + 8.5155e+36*pow(raw->F_y,2) - 1.7465e+37*F_y*tau + 1.0141e+37*tau^(2)))/(5.6687e+18*F_x - 4.1269e+18*F_y + 4.2320e+18*tau));
            
            raw->alpha_2 = 2*atan((1.5405e+18*raw->tau - 2.0634e+18*raw->F_x + 1.4142*sqrt(1.8196e+37*pow(raw->F_x,2) + 2.3394e+37*raw->F_x*raw->F_y + 2.0811e+37*raw->F_x*raw->tau + 8.5155e+36*pow(raw->F_y,2) + 1.7465e+37*raw->F_y*raw->tau + 1.0141e+37*pow(raw->tau,2)))/(5.6687e+18*raw->F_x + 4.1269e+18*raw->F_y + 4.2320e+18*raw->tau));
            
            printf("Er HER alpha negativ");
            /*
            fprintf("F1 =")
            disp(F1)
            fprintf("F2 =")
            disp(F2)    
            fprintf("alpha1 =")
            disp(alpha_1)
            fprintf("alpah2 =")
            disp(alpha_2)
            */
        }
        
      
        // limitter for angle of pods of 88 degrees
                
        
//         // limmitter and angle determination
//         // angle raw->alpha_2
//         // keeps angle between +-180 degrees
//         if raw->alpha_2 > M_PI
//             raw->alpha_2 = raw->alpha_2 - 2 * M_PI;
//         elseif raw->alpha_2 < -M_PI
//             raw->alpha_2 = raw->alpha_2 + 2 * M_PI;
//         end       
//         // determining the right angle and thrust    
//         if raw->alpha_2 > M_PI/2 && raw->alpha_2 < 11*M_PI/18      //(between 90 and 120 degrees)
//             raw->alpha_2 = raw->alpha_2 - M_PI;
//             F2 = (-1)*F2;
//         elseif raw->alpha_2 < -M_PI/2 && raw->alpha_2 > 11*M_PI/18 //(between -90 and -120 degrees)
//             raw->alpha_2 = raw->alpha_2 + M_PI;
//             F2 = (-1)*F2;
//         end
// 
//         // limitter for the thrusters for going sideway
//         if abs(F1) > T_max
//             //F1 = T_max;            
//             F1_old = F1;
//             F1 = T_max;
//             F2 = F1/(F1_old/F2);   
//         //elseif F1 < -T_max
//         //    F1 = -T_max;    
//         end
//         if abs(F2) > T_max
//             //F2 = T_max;
//             F2_old = F2;
//             F2 = T_max;
//             F1 = F2/(F2_old/F1);  
//         //elseif F2 < -T_max
//         //   F2 = -T_max;    
//         end
    }

    
    // makes shure that the thrusters are straith if no input in surge andsway direction 
    //if F_x == F_y && F_x < 0.001 && F_x > -0.001 && alpha_flag == 0
//     if ((F_x == F_y && F_x < 0.001 && F_x > -0.001) || (abs(u)<0.02 && abs(v) < 0.02))  && alpha_flag == 0
//         raw->alpha_1 = 0;
//         raw->alpha_2 = 0;
//     end
/*tau_out = tau;
raw->F1 = F1;
raw->F2 = F2;
alpha1 = raw->alpha_1;
alpha2 = raw->alpha_2;
alpha_flag_out = alpha_flag;
*/


}









void limmitter_angle_thrust(thrustType * raw, controlType * con, flagType * flag){
 
   
    int T_max = 1000; // max for trust in both positiv and negativ direction 

    //if abs(raw->alpha_1) < 7*M_PI/18 && abs(raw->alpha_1) >= 0 
    // sway manuvering (abs(alpha > 70 degrees)) 
    if (abs(raw->alpha_1) >= 7*M_PI/18 && abs(raw->alpha_1) <= 11*M_PI/18 || abs(raw->alpha_1) <= 25*M_PI/18 && abs(raw->alpha_1) >= 29*M_PI/18){  // manuvering up to 
        
        // limmitter and angle determination
        // angle alpha_2
        // keeps angle between +-180 degrees
        if (raw->alpha_2 > M_PI){
            raw->alpha_2 = raw->alpha_2 - 2 * M_PI;
        }else if( raw->alpha_2 < -M_PI){
            raw->alpha_2 = raw->alpha_2 + 2 * M_PI;
        }       
        // determining the right angle and thrust    
        if (raw->alpha_2 > M_PI/2 && raw->alpha_2 <= 11*M_PI/18){      //(between 90 and 160 degrees)
            raw->alpha_2 = raw->alpha_2 - M_PI;
            raw->F2 = (-1)*raw->F2;
        }else if( raw->alpha_2 < -M_PI/2 && raw->alpha_2 >= -11*M_PI/18){ //(between -90 and -160 degrees)
            raw->alpha_2 = raw->alpha_2 + M_PI;
            raw->F2 = (-1)*raw->F2;
        }
        


        // limitter for the thrusters for going sideway
        if (abs(raw->F1) > T_max){
            if (raw->F1 < 0){
               T_max = (-1)*T_max; 
            }
            //F1 = T_max;            
            int F1_old = raw->F1;
            raw->F1 = T_max;
            raw->F2 = raw->F1/(F1_old/raw->F2);   
        //elseif F1 < -T_max
        //    F1 = -T_max;    
        }
        if (abs(raw->F2) > T_max){
            if (raw->F2 < 0){
               T_max = (-1)*T_max; 
            }
            //F2 = T_max;
            double F2_old = raw->F2;
            raw->F2 = T_max;
            raw->F1 = raw->F2/(F2_old/raw->F1);  
        //elseif F2 < -T_max
        //   F2 = -T_max;    
        }
  
        
        //pod angle limitter (88 degrees)
        if (raw->alpha_2 >= 22*M_PI/45){ 
            raw->alpha_2 = 22*M_PI/45;
        }else if (raw->alpha_2 <= -22*M_PI/45){ 
            raw->alpha_2 = -22*M_PI/45;
        }
        
        // solution for going straight in y direction
        if (raw->alpha_1 == 7*M_PI/18 && raw->alpha_2 < 0){
           raw->alpha_2 = -raw->alpha_2;
           raw->F2 = -raw->F2;
        }
        
 
 
        
        
        
        
        
    else // manuuvering alpha less than 70 degrees     
        
        // if F1 is larger than the limit and F2 is positive
        if (raw->F1 > T_max && abs(raw->F2) < abs(raw->F1)){
            double diff = raw->F1 - T_max;  

            // if diff is out of range and raw->F1 > 0
            if ((abs(diff) > T_max)&& ((raw->F2 > 0) && (((raw->F2 - diff) > T_max) || (raw->F2 - diff) < -T_max)) ||  (raw->F2 < 0) && (((raw->F2 + diff) > T_max) || (raw->F2 + diff) < -T_max)) {            
                double F_1_old = raw->F1;
                raw->F1 = T_max;
                raw->F2 = raw->F1/(F_1_old/raw->F2);
 //                 fprintf("Er HER 1_special")

            }else if (raw->F2 >= 0){      //F2 is positive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;
 //                 fprintf("Er HER 1_1")
            }else{                   //F2 is negative
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 + diff; 
 //                 fprintf("Er HER 1_2")
            }
    }    

        // if F2 is larger than the limit        
        if (raw->F2 > T_max && abs(raw->F2) > abs(raw->F1)) {         
            double diff = raw->F2 - T_max;  

            // if diff is out of range and raw->F2 > 0
            if ((abs(diff) > T_max)&& ((raw->F1 > 0) && (((raw->F1 - diff) > T_max) || (raw->F1 - diff) < -T_max)) ||  (raw->F1 < 0) && (((raw->F1 + diff) > T_max) || (raw->F1 + diff) < -T_max))  {
                double F_2_old = raw->F2;
                raw->F2 = T_max;
                raw->F1 = raw->F2/(F_2_old/raw->F1);
 //                 fprintf("Er HER 2_special")  

            }else if (raw->F1 >= 0){      //F1 is positive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;
 //                 fprintf("Er HER 2_1")
            }else{                   //F1 is negative
                raw->F1 = raw->F1 + diff;
                raw->F2 = raw->F2 - diff; 
 //                 fprintf("Er HER 2_2")
            }
        }

        // if F1 is below the limit
        if (raw->F1 < -T_max && abs(raw->F2) < abs(raw->F1)){
            double diff = raw->F1 - (-T_max); 

            // if diff is out of range and raw->F1 < 0
            if ((abs(diff) > T_max) && ((raw->F2 > 0) && (((raw->F2 + diff) > T_max) || (raw->F2 + diff) < -T_max)) ||  (raw->F2 < 0) && (((raw->F2 - diff) > T_max) || (raw->F2 - diff) < -T_max)) { 
                double F_1_old = raw->F1;
                raw->F1 = -T_max;
                raw->F2 = raw->F1/(F_1_old/raw->F2);
 //                 fprintf("Er HER 3_special")

            }else if (raw->F2 >= 0){      //F2 is positive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 + diff;
 //                 fprintf("Er HER 3_1")
            }else{                   //F2 is negattive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;
 //                 fprintf("Er HER 3_2")
            }
        }

        // if F2 is below the limit
        if (raw->F2 < -T_max && abs(raw->F2) > abs(raw->F1)){
            double diff = raw->F2 - (-T_max);

            // if diff is out of range and raw->F2 < 0
            if ((abs(diff) > T_max) && ((raw->F1 > 0) && (((raw->F1 - diff) > T_max) || (raw->F1 - diff) < -T_max)) ||  (raw->F1 < 0) && (((raw->F1 + diff) > T_max) || (raw->F2 + diff) < -T_max)){  
                double F_2_old = raw->F2;
                raw->F2 = -T_max;
                raw->F1 = raw->F2/(F_2_old/raw->F1);
 //                 fprintf("Er HER 4_special")

            }else if (raw->F1 >= 0){     //F1 is positive
                raw->F1 = raw->F1 + diff;
                raw->F2 = raw->F2 - diff;
 //                 fprintf("Er HER 4_1")
            }else{                  //F1 is negative
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;    
 //                 fprintf("Er HER 4_2")
            }
        }



        // angle alfa 
        // between +-180 degrees
        if (raw->alpha_1 > M_PI){ 
            raw->alpha_1 = raw->alpha_1 - 2 * M_PI;
        }else if (raw->alpha_1 < -M_PI){ 
            raw->alpha_1 = raw->alpha_1 + 2 * M_PI;
        }      
        /*
        printf("alpha = ")
        disp(raw->alpha_1)
        fprintf("F1 = ")
        disp(raw->F1)
        */
        // determining the right angle and thrust    
        if (raw->alpha_1 > M_PI/2){       //(above 90 degrees)
            raw->alpha_1 = raw->alpha_1 - M_PI;
            raw->F1 = (-1)*raw->F1;
            raw->F2 = (-1)*raw->F2;
 //             fprintf("1-------- alpha>M_PI/2 ---------")
        }else if (raw->alpha_1 < -M_PI/2){  //(below -90 degrees)
            raw->alpha_1 = raw->alpha_1 + M_PI;
            raw->F1 = (-1)*raw->F1;
            raw->F2 = (-1)*raw->F2;
 //             //fprintf("2-------- alpha<M_PI/2 ---------")
            
 //         if raw->alpha_1 > M_PI/2
 //             alpha_out = raw->alpha_1 - (2*M_PI);
 //             F_1_out = (-1)*raw->F1;
 //             F_2_out = (-1)*raw->F2;
 // 
 //         elseif raw->alpha_1 < -M_PI/2
 //             alpha_out = raw->alpha_1 + 180;
 //             F_1_out = (-1)*raw->F1;
 //             F_2_out = (-1)*raw->F2;

        }else{ 
            raw->alpha_1 = raw->alpha_1;
            raw->F1 = raw->F1;
            raw->F2 = raw->F2;
        }
       
        raw->alpha_1 = raw->alpha_1;
        raw->alpha_2 = raw->alpha_1;
     
    }
    
    
 
    
  
    
    
    
    //|| (abs(u_con_sig)<abs(v_con_sig) && ref_v_in == 0)
    
    
    
    // stops, turns and keep motor values    
    if ((cos(abs(con->ref_u)/abs(con->ref_v)) > 5*M_PI/12)  || (((raw->alpha_2 > 0 && raw->alpha_2_old < 0) || (raw->alpha_2 < 0 && raw->alpha_1 > 0)) && abs(raw->alpha_2 - raw->alpha_2_old )> 7*M_PI/18 ) ||  (flag->alpha_flag == 1)  || (flag->sway_flag == 1) ){ // Forces to 0 and turneing the pods (larger X Degrees)
    //    cacos
        
    //if abs(alpha_out_2 - alpha_2_old )> 7*M_PI/18 || (alpha_out_2 > 0 && alpha_2_old < 0) || (alpha_out_2 < 0 && alpha_2_old > 0) ||  alpha_flag == 1 // Forces to 0 and turneing the pods (larger X Degrees)
        //alpha_flag = 1;
        //turn_yes = 0;
        //if abs(alpha_out_2 - alpha_2_old) > M_PI/4 || (alpha_out_2 > 0 && alpha_2_old < 0) || (alpha_out_2 < 0 && alpha_2_old > 0)
         if ((flag->sway_flag == 0) && flag->alpha_flag ==1 ){
            /*disp("Venter på podvinkel");
            disp(alpha_2_old)
            disp(alpha_out_2)
            disp(alpha_out_2-alpha_2_old)
            disp("-------slukket::::::::::---------_________::::::::::::::")
            */
            raw->F1 = 0;
            raw->F2 = 0;
            raw->F_old_1 = raw->F1;
            raw->F_old_2 = raw->F2;
         }else{
            raw->F1 = raw->F1;
            raw->F2 = raw->F2;
             
         }
         flag -> turn_yes = 0;
         flag -> alpha_flag = 1;
 //             alpha_1 = alpha_1;
 //             alpha_2 = alpha_2;
        //else
            //alpha_flag=0;
 //          end
 //          if abs(abs(alpha_out_2) - abs(alpha_2_old)) < 0.2 //M_PI/18 
 //             F1 = F1;
 //             F2 = F2;
 //             alpha_1 = alpha_1;
 //             alpha_2 = alpha_2;
            

         if ((abs(raw->alpha_2 - raw->alpha_2_old) < 0.01) && flag->sway_flag == 0){
               //M_PI/18
               flag->alpha_flag = 0;
               flag->turn_yes = 1;
               //printf("indenfor x grader");
               /*disp(alpha_2_old);
               disp(alpha_out_2);*/
 //          else
 //              disp("Venter på podvinkel");
 //              disp(alpha_2_old);
 //              disp(alpha_out_2);
 //              disp(alpha_out_2-alpha_2_old);
 // //            F_1_out = 0;
 // //            F_2_out = 0;
               //disp("-------YEP::::::::::---------_________::::::::::::::");

          }
            
           
          //|| (ref_v_in == 0 && abs(v_con_sig) > abs(u_con_sig))  
        if (((flag->turn_yes == 1) || (flag->sway_flag == 1)) &&  (cos(abs(con->ref_u)/abs(con->ref_v)) > 5*M_PI/12) ){ // keeps the velocity/thrust
             //  cacos
               flag->sway_flag = 1;
               raw->count_sway = raw->count_sway + 1;
               
 //                F_1_out = raw->F1; // returning raw->F1
 //                F_2_out = raw->F2; // returning raw->F1
 //                alpha_out_1 = raw->alpha_1; // returning input from when the flag was set
 //                alpha_out_2 = raw->alpha_2; // returning input from when the flag was set
                
               raw->F1 = raw->F_old_1; // returning raw->F1
               raw->F2 = raw->F_old_2; // returning raw->F1
               raw->alpha_1 = raw->alpha_1_old; // returning input from when the flag was set
               raw->alpha_2 = raw->alpha_2_old; // returning input from when the flag was set
               //disp("-------turn_Sway_flag::::::::::---------_________::::::::::::::");
                
               if (raw->count_sway >= 10/0.02) { //(2sec 50Hz 5 sec 10Hz) //stops counter and goes on
                   flag->sway_flag = 0;
                   raw->count_sway = 0;
                   //disp("_________________// SEK overstået_____________-------------------_________::::::::::::::::");

               }
                    
        
            
            
        }
             
            
            
        //end
 //         disp("ALPHA_FLAG =");
 //         disp(alpha_flag)
    }
        //disp("ALPHA_FLAG =");
        //disp(alpha_flag)    

       
        
        
    //turn_above_45_flag_out = turn_above_45_flag;     
    //ref_yaw_old = ref_yaw;            
    //F1 = F_1_out;
    //F2 = F_2_out;
    //alpha1 = alpha_out_1;
    //alpha2 = alpha_out_2;
    //alpha_flag_out = alpha_flag; 

}




/*
// uppdates the trust and angle values
//float* trust( trustType *raw ){
void trust_angle(thrustType * raw ){

    //float D = 1.3395; // centerpoint to the middle of the hull
    //float D = 1.3395; //afstand mellem midt skrog til midt skrog divideret med 2
    //float M_PI = 3.14159265359; // M_PI

    // determines the trusts and angles fro the motors 
    raw -> F1 = (0.5000 * ((raw -> F_x) * (raw -> tau) + pow((raw -> F_x),2) * D - (raw -> tau)*((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))) - (raw -> F_x) * D * ((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))))) / ((raw -> F_x) * D); 
    raw -> F2 =  - (0.5000 * ((raw -> F_x) * (raw -> tau) - pow((raw -> F_x),2) * D - (raw -> tau) * ((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))) + (raw -> F_x) * D * ((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))))) / ((raw -> F_x) * D);
    raw -> alpha1 = -2 * atan(((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))) / (raw -> F_y));
    

    //raw -> F1 = 0.5000 * (raw -> F_x) + 0.3733 * (raw -> tau) - (1.8664e-04 * ((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))) * (2679 * (raw -> F_x) + 2000 * (raw -> tau))) / (raw -> F_x);
    //raw -> F2 = 0.5000 * (raw -> F_x) - 0.3733 * (raw -> tau) - (1.8664e-04 * ((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))) * (2679 * (raw -> F_x) - 2000 * (raw -> tau))) / (raw -> F_x);
    //raw -> alpha1  = -2 * atan(((raw -> F_x) - sqrt(pow((raw -> F_x),2) + pow((raw -> F_y),2))) / (raw -> F_y));
    raw -> alpha2 = (raw -> alpha1);




 /*   // keeps alpha witin the interval 
    if (alpha_1 > M_PI/2) {
        alpha = raw->alpha_1 - M_PI;
        F1 = raw->F1 * (-1);
        F2 = raw->F2 * (-1);
    }else if (alpha_1 < -M_PI/2){
        alpha = raw->alpha_1 + M_PI;
        F1 = raw->F1 * (-1);
        F2 = raw->F2 * (-1);
    }else{
        alpha = raw->alpha_1;
        F1 = raw->F1;
        F2 = raw->F2;
    }

 //   alpha1 = alpha;
 //   alpha2 = alpha;
 
    float *out = (float*) malloc(4*4);
    out[0] = F1;
    out[1] = F2;
    out[2] = alpha1;
    out[3] = alpha2;

    return out;   
*  
}







// updating the forces (need to be changed)

void comands(thrustType * raw){
        
        update = clock();
        raw -> time_s = (update-start)/1000;
        printf("Timer_s = (//d) \n", rawData.time_s);

        if ((raw -> time_s) < 10){
            raw -> F_x = 500;
            raw -> F_y = 0.00001; 
            raw -> tau = 0;
        } else if ((raw -> time_s) > 10 && (raw -> time_s) < 20){
            raw -> F_x = 200;
            raw -> F_y = 0.00000001; 
            raw -> tau = 50;
        } else if ((raw -> time_s) >= 20){
            raw -> F_x = 250;
            raw -> F_y = 250; 
            raw -> tau = 40;
            if ((raw -> time_s) == 30){
                raw -> count = 0;
            }
        }
        (raw -> count) = raw->count+1;
        //(raw -> count_old) = raw->count;
        printf("count = (%d) \n", rawData.count);
       

            raw -> F_x = 200;
            raw -> F_y = 200; 
            raw -> tau = 0;

}
*/



void update_val(){
    //trust_angle(&rawData);
    //comands(&rawData);
}



int time_r(){
    printf("Timer (s) = (%d) \n", rawData.time_s);
    return rawData.time_s;
}

// return count
int count_r(){
    return rawData.count;
} 

// returns trust 
int F1_trust() {
	printf("F1 = (%f) \n", rawData.F1);
	return rawData.F1*1000000;
}
// returns trust 
int F2_trust() {
	printf("F2 = (%f) \n", rawData.F2);
	return rawData.F2*1000000;
}
// returns trust 
int alpha1_angl() {
	printf("alpha1 = (%f) \n", rawData.alpha1);
	return rawData.alpha1*1000000;
}
int alpha2_angl() {
	printf("alpha2 = (%f) \n", rawData.alpha2);
	return rawData.alpha2*1000000;
}


/*
// loading data from simulator
void pos_dat(simType * sim, double X, double Y, double Theta ){
    sim -> pos_x = X;
    sim -> pos_y = Y;
    sim -> theta = Theta;
}
*/




// for the stop_func
int stop_func = 0;
int stopp1 = 0;
// The stop-value is returned from the python program
void stop(int Stopp1){
	stop_func = Stopp1;	
	//return Stopp1;
}

void* controller_tick(void* args){
    // Kode
    while (true){
        //update(); // Update controller vals 
    }
    
}


// ############################## main ############################

int main(int argc, char *argv){ 
    
    pthread_t tick; // treath 

    init(); // initializing
    start = clock();
    pthread_create(&tick, NULL, controller_tick, NULL);
    // main loop
    while(1) {    
        update_val();

        //float*motor_set = trust(F_x, F_y, tau );  
        
        // Prints out the solution of the trust and angles
        //cout << "Index = " << index_out;
        
        /*
        printf("Alpha_1 = //d \n", alpha1);
        printf("raw->F1 = //d \n", F1);
        printf("Alpha_2 = //d \n", alpha2);
        printf("raw->F2 = //d \n", F2);
        */


        			
		if (stop_func == 1){
		    printf("Program stopped");
			//sleep(4); // comment in 
			//exit;
			break;
		}
    }
return 0;
}