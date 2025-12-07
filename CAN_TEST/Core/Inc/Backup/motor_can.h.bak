#ifndef __motor_can_H
#define __motor_can_H

#include "main.h"
#include "can.h"

#define P_MAX 12.5
#define V_MAX 30
#define T_MAX 10
#define KP_MAX 500
#define KD_MAX 5
#define P_MIN -12.5
#define V_MIN -30
#define T_MIN -10
#define KP_MIN 0
#define KD_MIN 0

void CANFilterConfig_Scale16_IdList(CAN_HandleTypeDef * hcan,uint32_t StdId1,uint32_t StdId2,uint32_t StdId3,uint32_t StdId4,uint8_t Filter_Nunber);
void CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_Start(CAN_HandleTypeDef *hcan);
float uint_to_float(int x_int,float x_min,float x_max,int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void motor_control(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);


#endif
