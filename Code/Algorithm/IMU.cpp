#include "IMU.h"
#include "MsgThread.h"


cIMU *IMU;
/*IMU进程控制指针*/
rt_thread_t IMU_thread = RT_NULL;
rt_thread_t IMUSlaver_thread = RT_NULL;
rt_thread_t IMUHeat_thread = RT_NULL;
rt_sem_t IMU_INT1Sem = RT_NULL;	
rt_sem_t IMU_INT2Sem = RT_NULL;
static void IMU_Init();
void IMUThread(void* parameter)
{
	IMU = new cIMU();
	uint8_t Test;
	IMU->SPI_Init(SPI0,GPIOB,GPIO_PIN_0);
	rt_thread_delay(1000);
	IMU_Init();

	for(;;)
	{
		rt_sem_take(IMU_INT1Sem,RT_WAITING_FOREVER);
		/*读取中断位*/
		if(IMU->ReadReg(0x2D)&0x08)
		{
			IMU->ReadAccelGyro();
			IMU->AccelCorrected[0] =  IMU->Accel[0] - IMU->AccelCal[0];
			IMU->AccelCorrected[1] =  IMU->Accel[1] - IMU->AccelCal[1];
			IMU->AccelCorrected[2] =  IMU->Accel[2] - IMU->AccelCal[2];
			IMU->GyroCorrected[0] = IMU->Gyro[0] - IMU->GyroCal[0];
			IMU->GyroCorrected[1] = IMU->Gyro[1] - IMU->GyroCal[1];
			IMU->GyroCorrected[2] = IMU->Gyro[2] - IMU->GyroCal[2];			
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

void IMUHeatThread(void* parameter)
{
	rt_tick_t Ticker = 0;
	uint32_t PWM=0;

	rt_thread_delay(100);
	/*数据常为25度时测温异常，不对加热进行初始化*/
	while(IMU->Temperature==25.0f)
	{rt_thread_delay(100);}
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

static void IMU_Init()
{
	uint8_t buf = 0;
	/*指定Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*软重启*/
	IMU->WriteReg(0x11,0x01);rt_thread_delay(5);
	/*读取中断位*/
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
	exti_flag_clear(EXTI_3);
	exti_flag_clear(EXTI_4);
	exti_interrupt_enable(EXTI_3);
	exti_interrupt_enable(EXTI_4);
	/*电源管理*/
	IMU->WriteReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
}