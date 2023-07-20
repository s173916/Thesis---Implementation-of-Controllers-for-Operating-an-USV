#ifndef CTR_H
#define CTR_H

#include"struct.h"



void PI_u_con(controlType *con, thrustType * raw, sensorType * sensor);

void PI_v_con(controlType *con, thrustType * raw, sensorType * sensor);

void PI_psi_con(controlType *con, thrustType * raw, sensorType * sensor);

void PI_yaw_con(controlType *con, thrustType * raw, sensorType * sensor);

void controllers(controlType *con, thrustType * raw, sensorType * sensor);

#endif 