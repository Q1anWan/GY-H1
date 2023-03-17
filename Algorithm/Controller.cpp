#include "Controller.h"
#include "Flash_GD.h"
#include "MsgThread.h"
#include "IMU.h"
#include "arm_math.h"

/*控制指针*/
cCTR *qCtr;

/*系统设置进程控制指针*/
rt_thread_t Config_thread = RT_NULL;
rt_mailbox_t Config_mailbox = RT_NULL;

/*数据输出进程*/
rt_thread_t DataOutput_thread = RT_NULL;

/*
	系统设置控制进程
	接收到来自串口或者CDC的控制命令后，修改系统设置
*/
void ConfigThread(void* parameter)
{
	qCtr = new cCTR;
	uint32_t RecBuf = 0;
	uint32_t ConfigBuf[5]={0};
	for(;;)
	{
		rt_mb_recv(Config_mailbox,(rt_ubase_t*)&RecBuf,RT_WAITING_FOREVER);
		switch(RecBuf>>8)
		{
			case 0x00:
				switch(RecBuf&0xFF)
				{
					case 0x00://重启
						NVIC_SystemReset();
						while(1);
					break;
					case 0x01:
						IMU->Q[0]=1.0f;
						IMU->Q[1]=0.0f;
						IMU->Q[2]=0.0f;
						IMU->Q[3]=0.0f;
					break;				
					case 0x02:
						qCtr->EnableOutput = 1;
						rt_thread_resume(DataOutput_thread);
					break;
					case 0x03:
						qCtr->EnableOutput = 0;
					break;						
				}
			break;
			case 0x01:
				switch(RecBuf&0xFF)
				{
					case 0x00:
						fmc_read_u32(FLASH_USERDATA_ADDRESS,ConfigBuf,5);
						qCtr->ConfigFlag = 1;
					break;
					case 0x01:
						rt_enter_critical();
						qCtr->ConfigFlag = 0;
						crc_data_register_reset();
						ConfigBuf[4]=crc_block_data_calculate(ConfigBuf,4);
						fmc_erase_pages(FLASH_USERDATA_ADDRESS,1);
						fmc_program(FLASH_USERDATA_ADDRESS,ConfigBuf,5);
						NVIC_SystemReset();while(1);
						rt_exit_critical();
					break;
				}
			break;
				
			default:
			if(qCtr->ConfigFlag){
			switch(RecBuf>>8){
				
			case 0x02:
				switch(RecBuf&0xFF)
				{
					case 0x00://设置输出为USB-C
						ConfigBuf[0] &= (uint32_t)0xFFFFFF00U;
						ConfigBuf[0] |= (uint32_t)0x00000000U;
					break;
					case 0x01://设置输出为CAN
						ConfigBuf[0] &= (uint32_t)0xFFFFFF00U;
						ConfigBuf[0] |= (uint32_t)0x00000001U;
					break;
					case 0x02://对外输出四元数
						ConfigBuf[0] &= (uint32_t)0x00FFFFFFU;
						ConfigBuf[0] |= (uint32_t)0x00000000U;
					break;
					case 0x03://对外输出6轴原始数据
						ConfigBuf[0] &= (uint32_t)0x00FFFFFFU;
						ConfigBuf[0] |= (uint32_t)0x01000000U;
					break;
					case 0x04://对外输出6轴矫正数据
						ConfigBuf[0] &= (uint32_t)0x00FFFFFFU;
						ConfigBuf[0] |= (uint32_t)0x02000000U;
					break;
				}
			break;
			case 0x03://设置ODR
				if((RecBuf&0xFF)<=SYS_CONFIG_MAX_ODRK){
				ConfigBuf[0] &= (uint32_t)0xFF00FFFFU;
				ConfigBuf[0] |= (uint32_t)((RecBuf&0xFF)<<16);}
			break;
			case 0x04://设置ID
				if((RecBuf&0xFF)<=SYS_CONFIG_MAX_ID){
				ConfigBuf[0] &= (uint32_t)0xFFFF00FFU;
				ConfigBuf[0] |= (uint32_t)((RecBuf&0xFF)<<8);}
			break;
			case 0x10://设定校正值GyroX_H
				ConfigBuf[1] &= (uint32_t)0x00FFFFFFU;
				ConfigBuf[1] |= (uint32_t)((RecBuf&0xFF)<<24);
			break;
			case 0x11://设定校正值GyroX_L
				ConfigBuf[1] &= (uint32_t)0xFF00FFFFU;
				ConfigBuf[1] |= (uint32_t)((RecBuf&0xFF)<<16);
			break;
			case 0x12://设定校正值GyroY_H
				ConfigBuf[1] &= (uint32_t)0xFFFF00FFU;
				ConfigBuf[1] |= (uint32_t)((RecBuf&0xFF)<<8);		
			break;
			case 0x13://设定校正值GyroY_L
				ConfigBuf[1] &= (uint32_t)0xFFFFFF00U;
				ConfigBuf[1] |= (uint32_t)(RecBuf&0xFF);			
			break;
			case 0x14://设定校正值GyroZ_H
				ConfigBuf[2] &= (uint32_t)0x00FFFFFFU;
				ConfigBuf[2] |= (uint32_t)((RecBuf&0xFF)<<24);
			break;
			case 0x15://设定校正值GyroZ_L
				ConfigBuf[2] &= (uint32_t)0xFF00FFFFU;
				ConfigBuf[2] |= (uint32_t)((RecBuf&0xFF)<<16);
			break;
			case 0x16://设定校正值AccelX_H
				ConfigBuf[2] &= (uint32_t)0xFFFF00FFU;
				ConfigBuf[2] |= (uint32_t)((RecBuf&0xFF)<<8);
			break;
			case 0x17://设定校正值AccelX_L
				ConfigBuf[2] &= (uint32_t)0xFFFFFF00U;
				ConfigBuf[2] |= (uint32_t)(RecBuf&0xFF);				
			break;
			case 0x18://设定校正值AccelY_H
				ConfigBuf[3] &= (uint32_t)0x00FFFFFFU;
				ConfigBuf[3] |= (uint32_t)((RecBuf&0xFF)<<24);
			break;
			case 0x19://设定校正值AccelY_L
				ConfigBuf[3] &= (uint32_t)0xFF00FFFFU;
				ConfigBuf[3] |= (uint32_t)((RecBuf&0xFF)<<16);
			break;
			case 0x1A://设定校正值AccelZ_H
				ConfigBuf[3] &= (uint32_t)0xFFFF00FFU;
				ConfigBuf[3] |= (uint32_t)((RecBuf&0xFF)<<8);
			break;
			case 0x1B://设定校正值AccelZ_L
				ConfigBuf[3] &= (uint32_t)0xFFFFFF00U;
				ConfigBuf[3] |= (uint32_t)(RecBuf&0xFF);
			break;}}break;
		}
	}
}

/*
	数据输出
*/
void DataOutputThread(void* parameter)
{
	const uint16_t DelayTime = 1 << qCtr->ODR;
	rt_tick_t ticker;
	for(;;)
	{
		if(qCtr->EnableOutput==0)
		{rt_thread_suspend(DataOutput_thread);rt_schedule();}
		ticker = rt_tick_get();
		uint8_t a[8]={1,2,3};
		if(qCtr->OTSel==1)
		{Msg->CANTx(qCtr->CAN_ID,a,8);}
		else
		{Msg->USBTx(a,3,1);}
		
		rt_thread_delay_until(&ticker,DelayTime);
	}
}

void ConfigRead(void)
{
	uint32_t buf[5];
	fmc_read_u32(FLASH_USERDATA_ADDRESS,buf,5);
	
	qCtr->OTSel  	= (uint8_t)(buf[0]&(uint32_t)0x000000FFU);
	qCtr->CAN_ID 	= (uint16_t)(SYS_CAN_ID_BASE+(buf[0]&(uint32_t)0x0000FF00U));
	qCtr->ODR 		= (uint8_t)((buf[0]&(uint32_t)0x00FF0000U)>>16);
	qCtr->OutPutMode= (uint8_t)((buf[0]&(uint32_t)0xFF000000U)>>24);
	
	IMU->GyroCal[0] = (int16_t)(buf[1]&0xFFFF);
	IMU->GyroCal[1] = (int16_t)(buf[1]>>16);
	IMU->GyroCal[2] = (int16_t)(buf[2]&0xFFFF);
	IMU->AccelCal[0] = (int16_t)(buf[2]>>16);
	IMU->AccelCal[1] = (int16_t)(buf[2]&0xFFFF);
	IMU->AccelCal[2] = (int16_t)(buf[2]>>16);
}	
