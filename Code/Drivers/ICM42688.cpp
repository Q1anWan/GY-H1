#include "ICM42688.h"
uint8_t cICM42688::ReadReg(uint8_t Reg)
{
	uint8_t Read = 0xFF;
	Reg |= 0x80;
	this->CS_0();
	this->SPI_ExchangeOneByte(Reg);
	Read = this->SPI_ExchangeOneByte(0xFF);
	this->CS_1();
	return Read;
}

void cICM42688::ReadReg(uint8_t Reg, uint8_t* Data, uint8_t num)
{
	Reg |= 0x80;

	this->CS_0();
	this->SPI_ExchangeOneByte(Reg);
	for(uint8_t i = 0; i < num; i++)
	{Data[i] = this->SPI_ExchangeOneByte(0x00);}
	this->CS_1();
}

void cICM42688::WriteReg( uint8_t Reg, uint8_t  Data)
{
	this->CS_0();
	this->SPI_ExchangeOneByte(Reg);
	this->SPI_ExchangeOneByte(Data);
	this->CS_1();
}

void cICM42688::WriteReg( uint8_t Reg, uint8_t  *Data, uint8_t num)
{
	this->CS_0();
	this->SPI_ExchangeOneByte(Reg);
	for(uint8_t i = 0; i < num; i++)
	{this->SPI_ExchangeOneByte(Data[i]);}
	this->CS_1();
}

void cICM42688::ReadAccel(void)
{
	uint8_t AccelBuf[6];
	this->ReadReg(0x1F,AccelBuf,6);
	this->Accel[0] = (AccelBuf[0]<<8 | AccelBuf[1]);
	this->Accel[1] = (AccelBuf[2]<<8 | AccelBuf[3]);
	this->Accel[2] = (AccelBuf[4]<<8 | AccelBuf[5]);
}

void cICM42688::ReadGyro(void)
{
	uint8_t GyroBuf[6];
	this->ReadReg(0x25,GyroBuf,6);
	this->Gyro[0] = (GyroBuf[0]<<8 | GyroBuf[1]);
	this->Gyro[1] = (GyroBuf[2]<<8 | GyroBuf[3]);
	this->Gyro[2] = (GyroBuf[4]<<8 | GyroBuf[5]);
};

void cICM42688::ReadAccelGyro(void)
{
	uint8_t Buf[12];
	this->ReadReg(0x1F,Buf,12);
	this->Accel[0] = (int16_t)(Buf[0]<<8 | Buf[1]);
	this->Accel[1] = (int16_t)(Buf[2]<<8 | Buf[3]);
	this->Accel[2] = (int16_t)(Buf[4]<<8 | Buf[5]);
	this->Gyro[0]  = (int16_t)(Buf[6]<<8 | Buf[7]);
	this->Gyro[1]  = (int16_t)(Buf[8]<<8 | Buf[9]);
	this->Gyro[2]  = (int16_t)(Buf[10]<<8| Buf[11]);
}

void cICM42688::ReadTem(void)
{
	uint8_t buf[2]={0};
	int16_t raw_tmp;

	this->ReadReg(0x1D,buf,2);
	
	raw_tmp = (int16_t)((buf[0]<<8)|(buf[1]));
	//½ØÖ¹ÆµÂÊ:100Hz
	//this->Temperature = 0.1*(((float)raw_tmp/132.48f)+25.0f) + 0.9*this->Temperature;
	this->Temperature = ((float)raw_tmp/1324.8f)+ 2.5f + 0.9*this->Temperature;
}

