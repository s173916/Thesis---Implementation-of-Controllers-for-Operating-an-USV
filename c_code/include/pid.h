#ifndef PID_H
#define PID_H

typedef struct
{
    int id; //id for controllers (1=surge, 2=sway,3=rot, 4=yaw) 
    double ref; // Reference value
    double measurement;
    double min, max; // Integrator min and max
    double Ts; // Sample time
    int tick; // Tick counter for logic
    double yl[2]; // Lead output
    double ul[2]; // Lead input
    double yi[2]; // Integrator output
    double ui[2]; // Integrator input
    double kp, tau_i, tau_d, alpha; // Parameters
    int flag; // Flag: 1 = special case
    double limit; // Limit for special case
    double out; // Output
    double max_vel_surge; // max velocity limiter for surge 
    double min_vel_surge; // min velocity limiter for surge
}stateType;


// Prototypes
void integral(stateType *st, double ref_sway);
void lead(stateType *st);
void pid_out(stateType *st, double ref_sway);
void pid_init(stateType *st, int id, double ts, double Kp, double Tau_i, double Tau_d, double Alpha, double min, double max, double min_vel_surge, double max_vel_surge);
void surge_min_max(stateType *st, double ref_v);


// struct State surge;
// struct State sway;
// struct State rot;
// struct State yaw;



#endif // PID_H