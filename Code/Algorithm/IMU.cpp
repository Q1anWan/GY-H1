#include "IMU.h"
#include "MsgThread.h"
#include "Controller.h"
#include "MahonyAHRS.h"
#include "Flash_GD.h"

cIMU *IMU;
/*IMU���̿���ָ��*/
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
		//50Hz�¶ȶ�ȡ
		IMU->ReadTem();
		rt_thread_delay(20);
	}
	
	IMU_Init();//Ԥ����ɺ�������ʼ��
	qCtr->TemperatureOK=1;
	
	for(;;)
	{
		rt_sem_take(IMU_INT1Sem,RT_WAITING_FOREVER);
		/*��ȡ�ж�λ*/
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
	/*�ȴ��¶Ȳ���OK*/
	while(!qCtr->TemperatureOK){rt_thread_delay(100);}
	/*�ȴ��˲����ȶ�*/
	rt_thread_delay(5000);

	for(;;)
	{
		ticker = rt_tick_get();
		rt_enter_critical();
		/*�����˲�������Ԫ��*/
		MahonyAHRSupdateINS(IMU->Q,IMU->GyroCorrected[0],IMU->GyroCorrected[1],IMU->GyroCorrected[2],IMU->AccelCorrected[0],IMU->AccelCorrected[1],IMU->AccelCorrected[2]);
		rt_exit_critical();
		rt_thread_delay_until(&ticker,1);
	}
}	
	
void IMUHeatThread(void* parameter)
{
	rt_tick_t Ticker = 0;
	uint32_t PWM=0;
	/*�ȴ�IMUԤ��ʼ��*/
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
//IMUԤ��ʼ�� ������ɼ��ȶ���
static void IMU_PreInit()
{
	uint8_t buf = 0;
	/*ָ��Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*������*/
	IMU->WriteReg(0x11,0x01);rt_thread_delay(5);
	/*��ȡ�ж�λ �л�SPI*/
	buf = IMU->ReadReg(0x2D);
	/*ָ��Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*Gyro����*/
	IMU->WriteReg(0x4F,0x06);//2000dps 1KHz
	/*Accel����*/
	IMU->WriteReg(0x50,0x06);//16G 1KHz
	/*��Դ����*/
	IMU->WriteReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
}

//IMU��ʼ�� ������������
static void IMU_Init()
{
	uint8_t buf = 0;
	/*ָ��Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*������*/
	IMU->WriteReg(0x11,0x01);rt_thread_delay(5);
	/*��ȡ�ж�λ �л�SPI*/
	buf = IMU->ReadReg(0x2D);
	#ifdef qwDbug
	/*��ӡIMU��Ϣ*/
	rt_kprintf("\n/*****\nRST MSG: %d\nIMU Conect = %d\n*****/\n",buf,IMU->ReadReg(0x75));
	#endif
	
	/*ָ��Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*�ж��������*/
	IMU->WriteReg(0x14,0x12);//INT1 INT2 ����ģʽ������Ч
	/*Gyro����*/
	IMU->WriteReg(0x4F,0x06);//2000dps 1KHz
	/*Accel����*/
	IMU->WriteReg(0x50,0x06);//16G 1KHz
	/*LSB����*/
	IMU->cICM42688::LSB_ACC_GYRO[0] = LSB_ACC_16G;
	IMU->cICM42688::LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem����&Gyro_Config1*/
	IMU->WriteReg(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	IMU->WriteReg(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	IMU->WriteReg(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	IMU->WriteReg(0x63,0x00);//Null
	/*INT_CONFIG1*/
	IMU->WriteReg(0x64,0x00);//�ж�������������
	/*INT_SOURCE0*/
	IMU->WriteReg(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	IMU->WriteReg(0x66,0x00);//Null
	/*INT_SOURCE3*/
	IMU->WriteReg(0x68,0x00);//Null
	/*INT_SOURCE3*/
	IMU->WriteReg(0x69,0x00);//Null
	
/*****������˲���@536Hz*****/
	
	/*GYRO������˲�������*/
	/*ָ��Bank1*/
	IMU->WriteReg(0x76,0x01);
	/*GYRO������˲�������*/
	IMU->WriteReg(0x0B,0xA0);//������������ݲ��˲���
	IMU->WriteReg(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
	IMU->WriteReg(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
	IMU->WriteReg(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
	/*ACCEL������˲�������*/
	/*ָ��Bank2*/
	IMU->WriteReg(0x76,0x02);
	/*ACCEL������˲�������*/
	IMU->WriteReg(0x03,0x18);//�����˲��� ACCEL_AFF_DELT 12 (default 24)
	IMU->WriteReg(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
	IMU->WriteReg(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

/*****�Զ����˲���1��@111Hz*****/

	/*ָ��Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*�˲���˳��*/
	IMU->WriteReg(0x51,0x12);//GYRO�˲���1st
	IMU->WriteReg(0x53,0x05);//ACCEL�˲���1st
	/*�˲�������*/
	IMU->WriteReg(0x52,0x33);//111Hz 03

	
	exti_flag_clear(EXTI_3);
	exti_flag_clear(EXTI_4);
	exti_interrupt_enable(EXTI_3);
	exti_interrupt_enable(EXTI_4);
	
	/*ָ��Bank0*/
	IMU->WriteReg(0x76,0x00);
	/*��Դ����*/
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
	IMU->AccelCal[2] = ((float)((int16_t)(buf[2]&0xFFFF)))/800.0f;
}