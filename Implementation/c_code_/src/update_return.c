#include "struct.h"
//#include "extern_struct.h"

extern thrustType rawData;    //Define raw data  
extern flagType flagData;    //Define flag data 
extern controlType controlData;    //Define control data  
extern sensorType sensorData;
extern simType simData;    //Define sim data 


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
	int F1 = rawData.F1;
	return F1;
}
// returns trust 
int F2_trust() {
	printf("F1 = (%f) \n", rawData.F2);	
	int F2 = rawData.F1;
	return F2;
}
// returns trust 
int alpha1_angl() {
	printf("alpha1 = (%f) \n", rawData.alpha1);
	return rawData.alpha1;
}
int alpha2_angl() {
	printf("alpha2 = (%f) \n", rawData.alpha2);
	return rawData.alpha2;
}
