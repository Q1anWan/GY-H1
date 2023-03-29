#include "Controller.h"
#include "Flash_GD.h"
#include "MsgThread.h"
#include "IMU.h"
#include "Data_Exchange.h"
#include "WS281x.h"
#include "CRC8.h"
#include "arm_math.h"

/*控制指针*/
cCTR *qCtr;
/*灯控制指针*/
cWS281x *LED;
/*内存读取*/
static void ConfigRead(void);
/*系统设置进程控制指针*/
rt_thread_t Config_thread = RT_NULL;
rt_mailbox_t Config_mailbox = RT_NULL;

/*数据输出进程*/
rt_thread_t DataOutput_thread = RT_NULL;

/*灯控制进程*/
extern rt_thread_t LEDCal_thread;
/*
	系统设置控制进程
	接收到来自串口或者CDC的控制命令后，修改系统设置
*/
void ConfigThread(void* parameter)
{
	qCtr = new cCTR;
	LED = new cWS281x;
	
	/*读取控制数据*/
	rt_enter_critical();
	ConfigRead();
	rt_exit_critical();
	
	/*LED发送缓冲区*/	
	uint8_t Color_buf[10]={0};
	LEDColor_t Color={0};
	
	LED->cSPI::SPI_Init(SPI2,GPIOB,GPIO_PIN_6,DMA1,DMA_CH1);
	LED->Init(Color_buf);
	
	uint32_t RecBuf = 0;
	uint32_t ConfigBuf[5]={0};
	fmc_read_u32(FLASH_USERDATA_ADDRESS,ConfigBuf,5);
	for(;;)
	{
		rt_mb_recv(Config_mailbox,(rt_ubase_t*)&RecBuf,RT_WAITING_FOREVER);
		switch(RecBuf>>8)
		{
			case 0x00:
				rt_enter_critical();
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
					break;
					case 0x03:
						qCtr->EnableOutput = 0;
					break;
					case 0x04:
						qCtr->OutPutModeLast = qCtr->OutPutMode;
						qCtr->OutPutMode = 0x02;
					break;	
				}
				rt_exit_critical();
			break;
			case 0x01:
				switch(RecBuf&0xFF)
				{
					case 0x00:
						//进入配置模式
						fmc_read_u32(FLASH_USERDATA_ADDRESS,ConfigBuf,5);
						//停止对外输出
						rt_thread_delete(DataOutput_thread);
						//关灯
						rt_thread_delete(LEDCal_thread);
						LED->LED_UpdateDMA(&Color,1);
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
			Color.GRB[0] = 50;Color.GRB[1] = 50;Color.GRB[2] = 50;LED->LED_UpdateDMA(&Color,1);
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
			break;}
			rt_thread_delay(3);
			Color.GRB[0] = 0;Color.GRB[1] = 0;Color.GRB[2] = 0;LED->LED_UpdateDMA(&Color,1);
			}break;
		}
	}
}

/*
	数据输出
*/
void DataOutputThread(void* parameter)
{
	rt_thread_delay(1500);
	const uint16_t DelayTime = 1 << qCtr->ODR;
	rt_tick_t ticker;
	for(;;)
	{		
		ticker = rt_tick_get();
		if(qCtr->EnableOutput==1)
		{
			if(qCtr->OTSel==1)
			{
				uint8_t TxBuf[8]={0};
				switch(qCtr->OutPutMode)
				{
					case 0x00://四元数输出
						Transform.Float_To_U8(IMU->Q,TxBuf,2);
						Msg->CANTx(qCtr->CAN_ID,TxBuf,8);
						Transform.Float_To_U8(IMU->Q+2,TxBuf,2);
						Msg->CANTx(qCtr->CAN_ID+1,TxBuf,8);
					break;
					case 0x01://原始6轴数据输出
						TxBuf[0]=IMU->Gyro[0]>>8;
						TxBuf[1]=IMU->Gyro[0]&0xFF;
						TxBuf[2]=IMU->Gyro[1]>>8;
						TxBuf[3]=IMU->Gyro[1]&0xFF;
						TxBuf[4]=IMU->Gyro[2]>>8;
						TxBuf[5]=IMU->Gyro[2]&0xFF;
						TxBuf[6]=0x03;
						Msg->CANTx(qCtr->CAN_ID,TxBuf,8);
						TxBuf[0]=IMU->Accel[0]>>8;
						TxBuf[1]=IMU->Accel[0]&0xFF;
						TxBuf[2]=IMU->Accel[1]>>8;
						TxBuf[3]=IMU->Accel[1]&0xFF;
						TxBuf[4]=IMU->Accel[2]>>8;
						TxBuf[5]=IMU->Accel[2]&0xFF;
						TxBuf[6]=0x03;
						Msg->CANTx(qCtr->CAN_ID,TxBuf,8);
					break;
				}
			}

			uint8_t TxBuf[18]={0};
			switch(qCtr->OutPutMode)
			{
				case 0x00://四元数输出
					TxBuf[0]=0x40;
					Transform.Float_To_U8(IMU->Q,TxBuf+1,4);
					TxBuf[17] = cal_crc8_table(TxBuf,17);
					if(!qCtr->OTSel){
					Msg->USBTx(TxBuf,18,RT_WAITING_NO);}
					#ifndef qwDbug//不设置调试模式UART才发
					Msg->UartTx(TxBuf,18,RT_WAITING_NO);
					#endif
				break;
				case 0x01://原始6轴数据输出
					TxBuf[0]=0x41;
					TxBuf[1]=IMU->Gyro[0]>>8;
					TxBuf[2]=IMU->Gyro[0]&0xFF;
					TxBuf[3]=IMU->Gyro[1]>>8;
					TxBuf[4]=IMU->Gyro[1]&0xFF;
					TxBuf[5]=IMU->Gyro[2]>>8;
					TxBuf[6]=IMU->Gyro[2]&0xFF;
					TxBuf[7]=IMU->Accel[0]>>8;
					TxBuf[8]=IMU->Accel[0]&0xFF;
					TxBuf[9]=IMU->Accel[1]>>8;
					TxBuf[10]=IMU->Accel[1]&0xFF;
					TxBuf[11]=IMU->Accel[2]>>8;
					TxBuf[12]=IMU->Accel[2]&0xFF;
					TxBuf[13]=cal_crc8_table(TxBuf,13);
					if(!qCtr->OTSel){
					Msg->USBTx(TxBuf,14,RT_WAITING_NO);}
					#ifndef qwDbug//不设置调试模式UART才发
					Msg->UartTx(TxBuf,14,RT_WAITING_NO);
					#endif
				break;
			}
		}
		/*6轴偏移量校正值输出*/
		if(qCtr->OutPutMode==0x02)
		{
			uint8_t TxBuf[14]={0};
			qCtr->OutPutMode = qCtr->OutPutModeLast;
			TxBuf[0]=0x42;
			TxBuf[1]=(int16_t)(IMU->GyroCal[0]*256.0f)>>8;
			TxBuf[2]=(int16_t)(IMU->GyroCal[0]*256.0f)&0xFF;
			TxBuf[3]=(int16_t)(IMU->GyroCal[1]*256.0f)>>8;
			TxBuf[4]=(int16_t)(IMU->GyroCal[1]*256.0f)&0xFF;
			TxBuf[5]=(int16_t)(IMU->GyroCal[2]*256.0f)>>8;
			TxBuf[6]=(int16_t)(IMU->GyroCal[2]*256.0f)&0xFF;
			TxBuf[7]=(int16_t)(IMU->AccelCal[0]*256.0f)>>8;
			TxBuf[8]=(int16_t)(IMU->AccelCal[0]*256.0f)&0xFF;
			TxBuf[9]=(int16_t)(IMU->AccelCal[1]*256.0f)>>8;
			TxBuf[10]=(int16_t)(IMU->AccelCal[1]*256.0f)&0xFF;
			TxBuf[11]=(int16_t)(IMU->AccelCal[2]*256.0f)>>8;
			TxBuf[12]=(int16_t)(IMU->AccelCal[2]*256.0f)&0xFF;
			TxBuf[13]=cal_crc8_table(TxBuf,13);
			if(!qCtr->OTSel){
			Msg->USBTx(TxBuf,14,RT_WAITING_NO);}
			#ifndef qwDbug//不设置调试模式UART才发
			Msg->UartTx(TxBuf,14,RT_WAITING_NO);
			#endif
		}
		rt_thread_delay_until(&ticker,DelayTime);
	}
}

static void ConfigRead(void)
{
	uint32_t buf;
	fmc_read_u32(FLASH_USERDATA_ADDRESS,&buf,1);
	
	qCtr->OTSel  	= (uint8_t)(buf&(uint32_t)0x000000FFU);
	qCtr->CAN_ID 	= (uint16_t)(SYS_CAN_ID_BASE+((buf&(uint32_t)0x0000FF00U)>>4));
	qCtr->ODR 		= (uint8_t)((buf&(uint32_t)0x00FF0000U)>>16);
	qCtr->OutPutMode= (uint8_t)((buf&(uint32_t)0xFF000000U)>>24);
}	

