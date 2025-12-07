#include "motor_can.h"

void CANFilterConfig_Scale16_IdList(CAN_HandleTypeDef * hcan,uint32_t StdId1,uint32_t StdId2,uint32_t StdId3,uint32_t StdId4,uint8_t Filter_Nunber)  
{  
  CAN_FilterTypeDef   sFilterConfig;  
  sFilterConfig.FilterBank = Filter_Nunber;         //选择过滤器的号码 
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //16位模式
  sFilterConfig.FilterIdHigh = StdId1<<5;  
  sFilterConfig.FilterIdLow = StdId2<<5;  
  sFilterConfig.FilterMaskIdHigh = StdId3<<5;  
  sFilterConfig.FilterMaskIdLow = StdId4<<5;  
  sFilterConfig.FilterFIFOAssignment = 0;   //选择接收邮箱为FIFO0 
  sFilterConfig.FilterActivation = ENABLE;  
  sFilterConfig.SlaveStartFilterBank = 0;  //双CAN模式时才用到，不用就给0
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig); 
}

void CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
        
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

void CAN_Start(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_ActivateNotification(hcan ,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(hcan);
}

float uint_to_float(int x_int,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void motor_control(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	uint8_t Data[8];
	Data[0] = (pos_tmp >> 8);
	Data[1] = pos_tmp;
	Data[2] = (vel_tmp >> 4);
	Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Data[4] = kp_tmp;
	Data[5] = (kd_tmp >> 4);
	Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Data[7] = tor_tmp;
	
	CANx_SendStdData(hcan,id,Data,8);
}
