#include "struct.h"
#include "lim.h"
//#include "extern_struct.h"


extern thrustType rawData;      //Define raw data  
extern flagType flagData;       //Define flag data 
extern controlType controlData; //Define control data  
extern sensorType sensorData;
extern simType simData;         //Define sim data 


void limmitter_angle_thrust(thrustType * raw, controlType * con, flagType * flag){
  
    int T_max = 1000; // max for trust in both positiv and negativ direction 

    //if fabs(raw->alpha_1) < 7*M_PI/18 && fabs(raw->alpha_1) >= 0 
    // sway manuvering (fabs(alpha > 70 degrees)) 
    if (fabs(raw->alpha_2) >= 7*M_PI/18 && fabs(raw->alpha_2) <= 11*M_PI/18 || fabs(raw->alpha_2) <= 25*M_PI/18 && fabs(raw->alpha_2) >= 29*M_PI/18){  // manuvering up to 
        
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
        if (fabs(raw->F1) > T_max){
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
        if (fabs(raw->F2) > T_max){
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
        /*if (raw->alpha_1 == 7*M_PI/18 && raw->alpha_2 < 0){
           raw->alpha_2 = -raw->alpha_2;
           raw->F2 = -raw->F2;
        }*/
              
        
    }else{ // manuuvering alpha less than 70 degrees     
        
        // if F1 is larger than the limit and F2 is positive
        if (raw->F1 > T_max && fabs(raw->F2) < fabs(raw->F1)){
            double diff = raw->F1 - T_max;  

            // if diff is out of range and raw->F1 > 0
            if ((fabs(diff) > T_max)&& ((raw->F2 > 0) && (((raw->F2 - diff) > T_max) || (raw->F2 - diff) < -T_max)) ||  (raw->F2 < 0) && (((raw->F2 + diff) > T_max) || (raw->F2 + diff) < -T_max)) {            
                double F_1_old = raw->F1;
                raw->F1 = T_max;
                raw->F2 = raw->F1/(F_1_old/raw->F2);


            }else if (raw->F2 >= 0){      //F2 is positive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;

            }else{                   //F2 is negative
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 + diff; 

            }
        }    

        // if F2 is larger than the limit        
        if (raw->F2 > T_max && fabs(raw->F2) > fabs(raw->F1)) {         
            double diff = raw->F2 - T_max;  

            // if diff is out of range and raw->F2 > 0
            if ((fabs(diff) > T_max)&& ((raw->F1 > 0) && (((raw->F1 - diff) > T_max) || (raw->F1 - diff) < -T_max)) ||  (raw->F1 < 0) && (((raw->F1 + diff) > T_max) || (raw->F1 + diff) < -T_max))  {
                double F_2_old = raw->F2;
                raw->F2 = T_max;
                raw->F1 = raw->F2/(F_2_old/raw->F1);
 

            }else if (raw->F1 >= 0){      //F1 is positive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;

            }else{                   //F1 is negative
                raw->F1 = raw->F1 + diff;
                raw->F2 = raw->F2 - diff; 

            }
        }

        // if F1 is below the limit
        if (raw->F1 < -T_max && fabs(raw->F2) < fabs(raw->F1)){
            double diff = raw->F1 - (-T_max); 

            // if diff is out of range and raw->F1 < 0
            if ((fabs(diff) > T_max) && ((raw->F2 > 0) && (((raw->F2 + diff) > T_max) || (raw->F2 + diff) < -T_max)) ||  (raw->F2 < 0) && (((raw->F2 - diff) > T_max) || (raw->F2 - diff) < -T_max)) { 
                double F_1_old = raw->F1;
                raw->F1 = -T_max;
                raw->F2 = raw->F1/(F_1_old/raw->F2);
 

            }else if (raw->F2 >= 0){      //F2 is positive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 + diff;
 
            }else{                   //F2 is negattive
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;
 
            }
        }

        // if F2 is below the limit
        if (raw->F2 < -T_max && fabs(raw->F2) > fabs(raw->F1)){
            double diff = raw->F2 - (-T_max);

            // if diff is out of range and raw->F2 < 0
            if ((fabs(diff) > T_max) && ((raw->F1 > 0) && (((raw->F1 - diff) > T_max) || (raw->F1 - diff) < -T_max)) ||  (raw->F1 < 0) && (((raw->F1 + diff) > T_max) || (raw->F2 + diff) < -T_max)){  
                double F_2_old = raw->F2;
                raw->F2 = -T_max;
                raw->F1 = raw->F2/(F_2_old/raw->F1);
 

            }else if (raw->F1 >= 0){     //F1 is positive
                raw->F1 = raw->F1 + diff;
                raw->F2 = raw->F2 - diff;
 
            }else{                  //F1 is negative
                raw->F1 = raw->F1 - diff;
                raw->F2 = raw->F2 - diff;    
 
            }
        }


        // angle alfa 
        // between +-180 degrees
        if (raw->alpha_1 > M_PI){ 
            raw->alpha_1 = raw->alpha_1 - 2 * M_PI;
        }else if (raw->alpha_1 < -M_PI){ 
            raw->alpha_1 = raw->alpha_1 + 2 * M_PI;
        }      

        // determining the right angle and thrust    
        if (raw->alpha_1 > M_PI/2){       //(above 90 degrees)
            raw->alpha_1 = raw->alpha_1 - M_PI;
            raw->F1 = (-1)*raw->F1;
            raw->F2 = (-1)*raw->F2;
            printf(" > chek 1= %f \n", raw->alpha_1);

        }else if (raw->alpha_1 < -M_PI/2){  //(below -90 degrees)
            raw->alpha_1 = raw->alpha_1 + M_PI;
            raw->F1 = (-1)*raw->F1;
            raw->F2 = (-1)*raw->F2;
            printf(" < chek 2= %f \n", raw->alpha_1);

        // }else if (fabs(raw->alpha_1) > 3.1415  && fabs(raw->alpha_1) < 3.1416){  
        //     raw->alpha_1 = raw->alpha_1 + M_PI;
        //     raw->F1 = (-1)*raw->F1;
        //     raw->F2 = (-1)*raw->F2;
        //     printf(" == chek 3= %f \n", raw->alpha_1);
 
            
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
            printf(" == chek else = %f \n", raw->alpha_1);
        }
       
        raw->alpha_1 = raw->alpha_1;
        raw->alpha_2 = raw->alpha_1;
     
    }
     
    
    //|| (fabs(u_con_sig)<fabs(v_con_sig) && ref_v_in == 0)
    
    // stops, turns and keep motor values    
    if ( (((raw->alpha_2 > 0 && raw->alpha_2_old < 0) || (raw->alpha_2 < 0 && raw->alpha_2_old > 0)) && fabs(raw->alpha_2 - raw->alpha_2_old )> 7*M_PI/18 ) ||  (flag->alpha_flag == 1)  || (flag->sway_flag == 1) ){ // Forces to 0 and turneing the pods (larger X Degrees)
    //(cos(fabs(con->ref_u)/fabs(con->ref_v)) > 5*M_PI/12)  ||
    //    cacos
        
    //if fabs(alpha_out_2 - alpha_2_old )> 7*M_PI/18 || (alpha_out_2 > 0 && alpha_2_old < 0) || (alpha_out_2 < 0 && alpha_2_old > 0) ||  alpha_flag == 1 // Forces to 0 and turneing the pods (larger X Degrees)
        //alpha_flag = 1;
        //turn_yes = 0;
        //if fabs(alpha_out_2 - alpha_2_old) > M_PI/4 || (alpha_out_2 > 0 && alpha_2_old < 0) || (alpha_out_2 < 0 && alpha_2_old > 0)
         if ((flag->sway_flag == 0) && flag->alpha_flag ==1 ){
 
            raw->F_old_1 = raw->F1;
            raw->F_old_2 = raw->F2;
            raw->F1 = 0;
            raw->F2 = 0;

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
 //          if fabs(fabs(alpha_out_2) - fabs(alpha_2_old)) < 0.2 //M_PI/18 
 //             F1 = F1;
 //             F2 = F2;
 //             alpha_1 = alpha_1;
 //             alpha_2 = alpha_2;
            
         if ((fabs(raw->alpha_2 - raw->alpha_2_old) < 0.01) && flag->sway_flag == 0){
               //M_PI/18
               flag->alpha_flag = 0;
               flag->turn_yes = 1;
               //printf("indenfor x grader");

 //          else

 // //            F_1_out = 0;
 // //            F_2_out = 0;
               
          }
            
           
          //|| (ref_v_in == 0 && fabs(v_con_sig) > fabs(u_con_sig))  
        if (((flag->turn_yes == 1) || (flag->sway_flag == 1)) &&  (fabs(atan(con->ref_v)/(con->ref_u)) )){ // keeps the velocity/thrust
             // &&  (cos(fabs(con->ref_u)/fabs(con->ref_v)) > 5*M_PI/12)
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
                
               if (raw->count_sway >= 2/0.1) { //(2sec 10Hz ) //stops counter and goes on
                   flag->sway_flag = 0;
                   raw->count_sway = 0;
                   flag->alpha_flag = 0;
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

    raw->alpha_1_old = raw->alpha_1;
    raw->alpha_2_old = raw->alpha_2;


    // just to be shure that the signal is limited
    if(raw->F1 > 1000){
        raw->F1 = 1000;
    } else if(raw->F1 < -1000){
        raw->F1 = -1000;
    }
    if(raw->F2 > 1000){
        raw->F2 = 1000;
    } else if(raw->F2 < -1000){
        raw->F2 = -1000;
    }

    // The final result thrust/anfle for the USV
    raw->F_1_res = raw->F1;
    raw->F_2_res = raw->F2;
    raw->alpha_1_res = raw->alpha_1;
    raw->alpha_2_res = raw->alpha_2;

    printf("F1_res =  %f \n", raw->F_1_res);
    printf("F2_res =  %f \n", raw->F_2_res);
    printf("alpha_1_res =  %f \n", raw->alpha_1_res);
    printf("alpha_2_res =  %f \n", raw->alpha_2_res);



}

