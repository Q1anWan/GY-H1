#include "IMU.h"
#include "MsgThread.h"
#include "Controller.h"
#include "MahonyAHRS.h"
#include "Flash_GD.h"

cIMU *IMU;
/*IMU进程控制指针*/
rt_thread_t IMU_thread = RT_NULL;
rt_thread_t IMUSlaver_thread = RT_NULL;
rt_thread_t IMUAHRS_thread = RT_NULL;
rt_thread_t IMUHeat_thread = RT_NULL;
rt_sem_t IMU_INT1Sem = RT_NULL;	
rt_sem_t IMU_INT2Sem = RT_NULL;

static void IMU_PreInit(void);
static void IMU_Init(void);
static void IMUCalibRead(void);

void IMUThread(void* parameter)
{
	IMU = new cIMU();
	rt_enter_critical();
	IMUCalibRead();
	rt_exit_critical();
	uint8_t Test;
	IMU->SPI_Init(SPI0,GPIOB,GPIO_PIN_0);
	IMU_PreInit();
	
	while(IMU->Temperature<40.5f)
	{
		//50Hz温度读取
		IMU->ReadTem();
		rt_thread_delay(20);
	}
	
	IMU_Init();//预热完成后再正初始化
	qCtr->TemperatureOK=1;
	
	for(;;)
	{
		rt_sem_take(IMU_INT1Sem,RT_WAITING_FOREVER);
		/*读取中断位*/
		if(IMU->ReadReg(0x2D)&0x08)
		{
			IMU->ReadAccelGyro();
			rt_enter_critical();
			IMU->AccelCorrected[0] = ((float)IMU->Accel[0] + IMU->AccelCal[0]) * IMU->LSB_ACC_GYRO[0];
			IMU->AccelCorrected[1] = ((float)IMU->Accel[1] + IMU->AccelCal[1]) * IMU->LSB_ACC_GYRO[0];
			IMU->AccelCorrected[2] = ((float)IMU->Accel[2] + IMU->AccelCal[2]) * IMU->LSB_ACC_GYRO[0];
			IMU->GyroCorrected[0] = ((float)IMU->Gyro[0] + IMU->GyroCal[0]) * IMU->LSB_ACC_GYRO[1];
			IMU->GyroCorrected[1] = ((float)IMU->Gyro[1] + IMU->GyroCal[1]) * IMU->LSB_ACC_GYRO[1];
			IMU->GyroCorrected[2] = ((float)IMU->Gyro[2] + IMU->GyroCal[2]) * IMU->LSB_ACC_GYRO[1];			
			rt_exit_critical();
			IMU->ReadTem();
		}
	}
}

void IMU2Thread(void* parameter)
{
	for(;;)
	{
		rt_sem_take(IMU_INT2Sem,RT_WAITING_FOREVER);
	}
}

void IMUAHRSThread(void* parameter)
{
	rt_tick_t ticker;
	/*等待温度补偿OK*/
	while(!qCtr->TemperatureOK){rt_thread_delay(100);}
	/*等待滤波器稳定*/
	rt_thread_delay(100);
	for(;;)
	{
		ticker = rt_tick_get();
		rt_enter_critical();
		/*互补滤波迭代四元数*/
		MahonyAHRSupdateINS(IMU->Q,IMU->GyroCorrected[0],IMU->GyroCorrected[1],IMU->GyroCorrected[2],IMU->AccelCorrected[0],IMU->AccelCorrected[1],IMU->AccelCorrected[2]);
		rt_exit_critical();
		rt_thread_delay_until(&ticker,1);
	}
}	
	
void IMUHeatThread(void* parameter)
{
	rt_tick_t Ticker = 0;
	uint32_t PWM=0;
	/*等待IMU预初始化*/
	rt_thread_delay(20);
	
	for(;;)
	{
		Ticker = rt_tick_get();		
		PWM = (uint32_t)IMU->PID_Cal(IMU->Temperature);
		timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_3,PWM);
		rt_thread_delay_until(&Ticker,20);
	}
}

void EXTI3_IRQHandler(void)
{
	if(exti_flag_get(EXTI_3))
	{
		exti_flag_clear(EXTI_3);
		rt_sem_release(IMU_INT2Sem);
	}
	
}

void EXTI4_IRQHandler(void)
{
	if(exti_flag_get(EXTI_4))
	{
		exti_flag_clear(EXTI_4);
		rt_sem_release(IMU_INT1Sem);
	}
}
//IMU预初始化 用于完成加热动作
static void IMU_PreInit()
{
	uint8_t buf = 0;
	/*指定Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*软重启*/
	IMU->WriteReg(0x11,0x01);rt_thread_delay(5);
	/*读取中断位 切换SPI*/
	buf = IMU->ReadReg(0x2D);
	/*指定Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*Gyro设置*/
	IMU->WriteReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	IMU->WriteReg(0x50,0x06);//16G 1KHz
	/*电源管理*/
	IMU->WriteReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
}

//IMU初始化 用于正常工作
static void IMU_Init()
{
	uint8_t buf = 0;
	/*指定Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*软重启*/
	IMU->WriteReg(0x11,0x01);rt_thread_delay(5);
	/*读取中断位 切换SPI*/
	buf = IMU->ReadReg(0x2D);
	#ifdef qwDbug
	/*打印IMU信息*/
	rt_kprintf("\n/*****\nRST MSG: %d\nIMU Conect = %d\n*****/\n",buf,IMU->ReadReg(0x75));
	#endif
	
	/*指定Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*中断输出设置*/
	IMU->WriteReg(0x12,0x36);//INT1 INT2 脉冲模式，低有效
	/*Gyro设置*/
	IMU->WriteReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	IMU->WriteReg(0x50,0x06);//16G 1KHz
	/*LSB设置*/
	IMU->cICM42688::LSB_ACC_GYRO[0] = LSB_ACC_16G;
	IMU->cICM42688::LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem设置&Gyro_Config1*/
	IMU->WriteReg(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	IMU->WriteReg(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	IMU->WriteReg(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	IMU->WriteReg(0x63,0x00);//Null
	/*INT_CONFIG1*/
	IMU->WriteReg(0x64,0x00);//中断引脚正常启用
	/*INT_SOURCE0*/
	IMU->WriteReg(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	IMU->WriteReg(0x66,0x00);//Null
	/*INT_SOURCE3*/
	IMU->WriteReg(0x68,0x00);//Null
	/*INT_SOURCE3*/
	IMU->WriteReg(0x69,0x00);//Null
	
//	/*指定Bank1 信号链设置*/
//	IMU->WriteReg(0x76,0x01);
	
	exti_flag_clear(EXTI_3);
	exti_flag_clear(EXTI_4);
	exti_interrupt_enable(EXTI_3);
	exti_interrupt_enable(EXTI_4);
	/*指定Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*电源管理*/
	IMU->WriteReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
}

static void IMUCalibRead(void)
{	
	uint32_t buf[4];
	
	fmc_read_u32(FLASH_USERDATA_ADDRESS+4,buf,3);
	
	IMU->GyroCal[0] = ((float)((int16_t)(buf[0]>>16)))/1000.0f;
	IMU->GyroCal[1] = ((float)((int16_t)(buf[0]&0xFFFF)))/1000.0f;
	IMU->GyroCal[2] = ((float)((int16_t)(buf[1]>>16)))/1000.0f;
	IMU->AccelCal[0] = ((float)((int16_t)(buf[1]&0xFFFF)))/1000.0f;
	IMU->AccelCal[1] = ((float)((int16_t)(buf[2]>>16)))/1000.0f;
	IMU->AccelCal[2] = ((float)((int16_t)(buf[2]&0xFFFF)))/1000.0f;
}