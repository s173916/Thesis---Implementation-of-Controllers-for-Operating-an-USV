#include <pid.h>
#include <struct.h>
#include <stdlib.h>
#include <math.h>

//extern controlType controlData;    //Define control data  



void integral(stateType *st, double ref_sway){
    
    
    surge_min_max(st, ref_sway);
    
    // setting flag
    if(st->tick == 0){
        // Controller input, minus lead output, times kp.
        st->ui[0] = (st->ref - st->yl[0]) * st->kp;


    }else{
        st->ui[1] = st->ui[0];
        
        st->ui[0] = (st->ref - st->yl[0]) * st->kp;
        
        st->yi[1] = st->yi[0];
    }

 
    // Integral
    // Integral disabled
    if(st->tau_i == 0){
        st->yi[0] = 0;//st->ui[0];
    
    // flag for surge and direction
    }else if (st->flag == 1){
        //double max = 8*0.54133;
        //double min = -max;
        double Kw = 0.9;
        double Delta = 0.0;
        double tmp = 0.0;

        st->ui[1] = st->ui[0];
        st->ui[0] = (st->ref - st->yl[0]) * st->kp;

        // if (st->yi[0] > st->max_vel_surge){
        //     Delta = st->max_vel_surge-st->yi[1]; // ændres til old out 
        // }else if(st->yi[1] < st->min_vel_surge){
        //     Delta = st->min_vel_surge-st->yi[1];
        // }else{
        //     Delta = st->yi[1];
        // }
        if (st->out > st->max_vel_surge){
            Delta = st->max_vel_surge-st->out; // ændres til old out 
        }else if(st->out < st->min_vel_surge){
            Delta = st->min_vel_surge-st->out;
        }else{
            Delta = st->out;
        }
        tmp = ((st->tau_i * st->yi[1]) + (st->Ts * st->ui[1]) + (st->Ts * st->ui[0]) + Kw*Delta*st->ui[1]) / (4 * st->tau_i);
        st->yi[0] = tmp;

    // Normal integral 
    }else{
        double tmp = 0.0;
        tmp = ((2 * st->tau_i * st->yi[1]) + (st->Ts * st->ui[1]) + (st->Ts * st->ui[0])) / (2 * st->tau_i);
        
        if(tmp > st->max){
            tmp = st->max;
        }else if(tmp < st->min){
            tmp = st->min;
        }
        st->yi[0] = tmp;
    }    
    
}
void flag_raised(stateType *st){
    st->yi[0] = 0;

    double pd_out = (st->ref - st->yl[0]) * st->kp;
}


void lead(stateType *st){
    if(st->tick == 0){
        st->ul[0] = st->measurement;
       
    }else{
        st->ul[1] = st->ul[0];
        st->ul[0] = st->measurement;
        st->yl[1] = st->yl[0];
        
    }

    if(st->tau_d == 0 && st->alpha == 0){
        st->yl[0] = st->measurement;
    }else{
        st->yl[0] = ((2 * st->alpha * st->tau_d * st->yl[1]) - (st->Ts * st->yl[1]) + (2 * st->tau_d * st->ul[1]) - (2 * st->tau_d * st->ul[1]) + 
            (st->Ts * st->ul[0]) + (st->Ts * st->ul[1])) / (2 * st->alpha * st->tau_d + st->Ts);   
                  
    }
    
}


void pid_out(stateType *st, double ref_sway){
    lead(st);
    integral(st, ref_sway);

/*    printf("kp_led = %f \n", (st->ref - st->yl[0]) * st->kp );
    printf("ref = %f \n", st->ref);
    printf("yl[0] = %f \n", st->yl[0]);
*/
    st->out = (st->ref - st->yl[0]) * st->kp + st->yi[0];
    if(st->flag == 1 && st->min_vel_surge > st->out){
        st->out = st->min_vel_surge;
        printf("jjfgjfujfujhhhhhhhhhhhhhhhhhhhf");
    }else if(st->flag == 1 && st->max_vel_surge < st->out){
        st->out = st->max_vel_surge;
    }

    st->tick = st->tick + 1; 
/*    printf("KP = %f \n" ,st->kp );
    // printf("integral_led = %f \n", st->yi[0]);
    // printf("Lead_led = %f \n",st->yl[0]);
    printf("tau_i = %f  , integral_out = %f \n", st->tau_i,st->yi[0]);   
    printf("alpha = %f  ,  tau_d = %f  , lead_out = %f \n",st->alpha, st->tau_d,st->yl[0]);  
    printf("integral min/max = %f/%f \n", st->min,st->max);
 */   

}


void surge_min_max(stateType *st, double ref_v){
    if(fabs(atan(st->ref/ref_v)) > M_M_PI/4 && st->id==1){    
    //if(fabs(atan(ref_v/st->ref)) > M_M_PI/4 && st->id==1){      
        st->flag = 1;
        printf("flag is set: val = %f \n",fabs(atan(st->ref/ref_v)));
    }else{
        st->flag = 0;
        printf("flag is NOT set: val = %f \n",fabs(atan(st->ref/ref_v)));
            // st->min = 0; 
            // st->max = 0;
            // st->flag = 1;
            // // slet i-led
    }
}

// void surge_min_max(stateType *st, double ref_v){
    
//     if(fabs(atan(ref_v/st->ref )) > M_M_PI/4 && st->id==1){      
//         st->flag = 1;
//         printf("flag is set: val = %f \n",fabs(atan(ref_v/st->ref)));
//     }else{
//         st->flag = 0;
//         printf("flag is NOT set: val = %f \n",fabs(atan(st->ref/ref_v)));
//             // st->min = 0; 
//             // st->max = 0;
//             // st->flag = 1;
//             // // slet i-led
//     }
// }


void pid_init(stateType *st, int id, double ts, double Kp, double Tau_i, double Tau_d, double Alpha, double min, double max, double min_vel_surge, double max_vel_surge){
    st->id = id;
    st->ref;
    st->measurement;
    st->min = min; 
    st->max = max;
    st->Ts = ts;
    st->tick = 0;
    st->yl[0] = 0.0;
    st->ul[0] = 0.0;
    st->yi[0] = 0.0;
    st->ui[0] = 0.0;
    st->yl[1] = 0.0;
    st->ul[1] = 0.0;
    st->yi[1] = 0.0;
    st->ui[1] = 0.0;
    st->kp = Kp;
    st->tau_i = Tau_i;
    st->tau_d = Tau_d;
    st->alpha = Alpha;
    st->out = 0.0;
    st->min_vel_surge = min_vel_surge;
    st->max_vel_surge = max_vel_surge;
}
