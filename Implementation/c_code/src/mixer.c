#include "struct.h"
#include "mix.h"
//#include "extern_struct.h"

extern thrustType rawData;    //Define raw data  
extern flagType flagData;    //Define flag data 
extern controlType controlData;    //Define control data  
extern sensorType sensorData;
extern simType simData;    //Define sim data 


void mixer(controlType * con, thrustType * raw, sensorType * sensor, flagType * flag){
    //limmitting the input signal 
    if ((( sensor -> u_local < 0.02 && sensor -> u_local > -0.02) && (fabs(con -> ref_u)< 0.02 && fabs(con -> ref_v)< 0.02) && fabs(raw -> tau) > 0.01) && flag -> alpha_flag == 0) {
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
    if (fabs(raw->alpha_1) >= 7*M_PI/18 && fabs(raw->alpha_1) <= 11*M_PI/18 || fabs(raw->alpha_1) <= 25*M_PI/18 && fabs(raw->alpha_1) >= 29*M_PI/18){  
        
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
            printf("Er HER alpha positiv \n");
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
            
            printf("Er HER alpha negativ \n");
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
//         if fabs(F1) > T_max
//             //F1 = T_max;            
//             F1_old = F1;
//             F1 = T_max;
//             F2 = F1/(F1_old/F2);   
//         //elseif F1 < -T_max
//         //    F1 = -T_max;    
//         end
//         if fabs(F2) > T_max
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
//     if ((F_x == F_y && F_x < 0.001 && F_x > -0.001) || (fabs(u)<0.02 && fabs(v) < 0.02))  && alpha_flag == 0
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
