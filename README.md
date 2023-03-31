- [GY-H1](#gy-h1)
  - [基本配置](#基本配置)
  - [通讯接口](#通讯接口)
  - [数据输出](#数据输出)
  - [按钮](#按钮)
  - [LED](#led)
  - [上位机: GY-H1\_Assistance\_V1.0.exe](#上位机-gy-h1_assistance_v10exe)
- [附录](#附录)
  - [通讯协议](#通讯协议)
    - [UART与USB数据包：](#uart与usb数据包)
    - [UART与USB控制包：](#uart与usb控制包)
    - [CAN数据包：](#can数据包)
  - [用户配置信息存储](#用户配置信息存储)
  - [CRC-8算法](#crc-8算法)
    - [CRC-8表:](#crc-8表)
  - [IMU信号链路](#imu信号链路)

# GY-H1 
## 基本配置
- 主传感器使用**ICM42688-P**，配有避震孤岛结构设计及温度补偿电路。  
- 主控制器使用**GD32F303CGT6**，超频至**288MHz**使用。
 >288MHz主频下MCU可以稳定工作  

---

## 通讯接口
- GY-H1支持UART、USB-C CDC串口、CAN三种通讯方式，CAN-ID可调节。
>UART波特率864000Bit/s

>CAN波特率1MBit/S

>USB-C使用CDC串口

>USB-C与CAN不支持同时工作

---

## 数据输出
GY-H1支持输出四元数或原始6轴数据。当使用原始6轴数据时，用户还可以向GY-H1请求各轴偏移矫正量，从而对数据进行修正
>四元数以单精度浮点输出，1位浮点数被拆分为4 Bytes发送  

>输出原始6轴数据时，IMU的原始16位有符号整型数据被拆分为2个Bytes发送。此数据的单位是1/LSB

>输出6轴偏移矫正量时，矫正量被乘以1000成为16位有符号整型，并被拆分为2个Bytes发送。此数据的单位是1000/LSB   
>特别的是，加速度计Z轴(Accel-Z)被设计为800/LSB以获得更大的补偿范围  
---

## 按钮
GY-H1 有RST和Key两个按钮。更靠近边缘的是Key按钮。    
RST按钮用于复位单片机。  
Key按钮可切换输出模式，并在CAN模式下调整CAN-ID。
>无按压操作5S后用户设置将被写入内存，GY-H1将自动重启以应用新设置  

>任何模式下，长按按钮3S进入输出模式设置，短按按钮可切换输出模式。板载LED绿灯常亮表示当前为USB输出，蓝灯常亮表示CAN输出  

>CAN输出模式下，短按按钮进入ID设置模式，每一次按压后LED会闪烁1次，表示设置ID加1。  
如果需要将ID设置为0，则进入ID设置模式后不应当按压按钮  
>ID最大值为9，按压次数大于9次后ID仍被设置为9  

---
## LED
板载LED动态变换色彩指示系统正常运行。CAN通讯模式下，LED灯将通过闪烁次数指示ID。
>板载LED闪烁[N]次表示ID为[N-1]  
>板载LED搭载WS2812芯片，使用SPI控制，控制频率125Hz

---
## 上位机: GY-H1_Assistance_V1.0.exe
上位机V1.0具有**设置**和**校准**两大功能。可以帮助用户使用图形菜单完成对 GY-H1 各项参数的配置，并对加速度计和陀螺仪的稳态误差进行校准。
>加速度使用六向校准法，建议配合3D打印治具使用 

>上位机基于 Python-3.11.0 编写

---

---

# 附录

## 通讯协议
### UART与USB数据包：  
开启USB-C输出下，GY-H1将以指定速率向总线广播四元数或者原始6轴数据。用户还可以使用控制命令，使GY-H1向总线广播一次6轴偏移量校正值
>串口通讯波特率为 864000 Bits/s  
>UART GH1.25接口将始终按照用户设置对外输出数据   

>6轴偏移量校正值发送时被乘以1000，成为16位有符号整型  
>矫正数据 = 原始数据 + 偏移量校正值/1000   
>特别的是，加速度计Z轴偏移量(Accel-Z)被设计为800/LSB以获得更大的补偿范围  

输出旋转四元数时，数据包以下面的方式发送 :
| Data[00] | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    | Data[08]    |
|---------|------------|------------|------------|------------|------------|------------|------------|------------|
| 0x40 | 四元数A[0] | 四元数A[1] | 四元数A[2] | 四元数A[3] | 四元数B[0] | 四元数B[1] | 四元数B[2] | 四元数B[3] |  

| Data[09]    | Data[10]   | Data[11]   | Data[12]   | Data[13]   | Data[14]   | Data[15]   | Data[16]   | Data[17] |
|------------|------------|------------|------------|------------|------------|------------|------------|----------|
| 四元数C[0] | 四元数C[1] | 四元数C[2] | 四元数C[3] | 四元数D[0] | 四元数D[1] | 四元数D[2] | 四元数D[3] | CRC8  |

输出原始6轴数据时，数据包以下面的方式发送 :
| Data[00] | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    | Data[08]    |
|---------|------------|------------|------------|------------|------------|------------|------------|------------|
| 0x41 | GyX_H | GyX_L | GyY_H | GyY_L | GyZ_H | GyZ_H | AcX_H | AcX_L |  

| Data[09]    | Data[10]   | Data[11]   | Data[12]   | Data[13]|
|------------|------------|------------|------------|------------|
| AcY_H | AcY_L | AcZ_H | AcZ_L | CRC8  |
 
输出偏移量校正值时，数据包以下面的方式发送 :
| Data[00] | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    | Data[08]    |
|---------|------------|------------|------------|------------|------------|------------|------------|------------|
| 0x42 | GyXC_H | GyXC_L | GyYC_H | GyYC_L | GyZC_H | GyZC_H | AcXC_H | AcXC_L |  

| Data[09]    | Data[10]   | Data[11]   | Data[12]   | Data[13]|
|------------|------------|------------|------------|------------|
| AcYC_H | AcYC_L | AcZC_H | AcZC_L | CRC8  | 

### UART与USB控制包：  
控制包可以对系统输出状态进行设置。控制包共4 Bytes  
- 如果您需要通过UART快速发送大量控制命令，建议首先关闭GY-H1对外输出，写入指令完成后再打开，否则有可能导致GY-H1串口输出异常
>控制包分为直接控制指令和配置指令，直接控制指令的控制符是0x00，调用直接控制指令将会立即生效

>其他控制符的指令是配置指令，调用配置指令前必须进入配置模式，系统才会响应配置信息。退出配置模式后，系统立即重启，配置的设置生效

>GY-H1将在收到有效信息时，重复收到的信息作为应答。对外发送一次6轴校正值例外(30 00 03 BF)，收到这条指令后GY-H1将直接回复6轴校正值

>以0x30开头的，控制符不在下列的数据包，GY-H1仍会回复，但不会影响系统状态，因此可以用于检测GY-H1是否离线
  
| Data[00] | Data[01] | Data[02] | Data[03] | 命令含义 |
|----------|----------|----------|-----------|---------|
|包头 | 控制符   | 命令数据  | CRC8|这是模板|
|0x30 | 0x00   | 0x00  | 0xDE|重启GY-H1|
|0x30 | 0x00   | 0x01  | 0x80|重置旋转四元数|
|0x30 | 0x00   | 0x02  | 0x62|开启对外数据输出|
|0x30 | 0x00   | 0x03  | 0x3C|关闭对外数据输出|
|0x30 | 0x00   | 0x04  | 0xBF|对外发送一次6轴校正值|
||
|0x30 | 0x01   | 0x00  | 0x1A|进入配置模式|
|0x30 | 0x01   | 0x01  | 0x44|退出配置模式|
|0x30 | 0x02   | 0x00  | 0x4F|设置输出为USB-C|
|0x30 | 0x02   | 0x01  | 0x11|设置输出为CAN|
|0x30 | 0x02   | 0x02  | 0xF3|对外输出四元数|
|0x30 | 0x02   | 0x03  | 0xAD|对外输出6轴原始数据|
|0x30 | 0x03   | 0x00  | 0x8B|设置输出速率1KHz|
|0x30 | 0x03   | 0x01  | 0xD5|设置输出速率500Hz|
|0x30 | 0x03   | 0x02  | 0x37|设置输出速率250Hz|
|0x30 | 0x03   | 0x03  | 0x69|设置输出速率125Hz|
|0x30 | 0x04   | 0~0x09 | CRC8|设定ID||
|0x30 | 0x10   | GyX_H  | CRC8|设定校正值GyroX_H|
|0x30 | 0x11   | GyX_L  | CRC8|设定校正值GyroX_L|
|0x30 | 0x12   | GyY_H  | CRC8|设定校正值GyroY_H|
|0x30 | 0x13   | GyY_L  | CRC8|设定校正值GyroY_L|
|0x30 | 0x14   | GyZ_H  | CRC8|设定校正值GyroZ_H|
|0x30 | 0x15   | GyZ_L  | CRC8|设定校正值GyroZ_L|
|0x30 | 0x16   | AcX_H  | CRC8|设定校正值AccelX_H|
|0x30 | 0x17   | AcX_L  | CRC8|设定校正值AccelX_L|
|0x30 | 0x18   | AcY_H  | CRC8|设定校正值AccelY_H|
|0x30 | 0x19   | AcY_L  | CRC8|设定校正值AccelY_L|
|0x30 | 0x1A   | AcZ_H  | CRC8|设定校正值AccelZ_H|
|0x30 | 0x1B   | AcZ_L  | CRC8|设定校正值AccelZ_L| 

### CAN数据包：  
开启CAN输出下，GY-H1将以指定速率向总线广播四元数、原始6轴数据。数据将通过两个CAN标识符发送，它们分别是 **[0x300+(ID<<8)]** 以及 **[0x301+(ID<<8)]**。CAN总线的波特率是1MBit/s，数据长度8 Bytes。 
>GY-H1的CAN功能仅发送数据，不接受数据。其CAN过滤器被配置为过滤所有数据  
>CAN模式下，UART GH1.25接口依然按照用户设置对外输出数据

当ID设置0x01时，旋转四元数将按照下面两种CAN数据包发送 :
|CANID  | Data[00]    | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    |
|---------|------------|------------|------------|------------|------------|------------|------------|------------|
| 0x310    | 四元数A[0] | 四元数A[1] | 四元数A[2] | 四元数A[3] | 四元数B[0] | 四元数B[1] | 四元数B[2] | 四元数B[3] |  

|CANID  | Data[00]    | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    |
|------------|------------|------------|------------|------------|------------|------------|------------|----------|
|0x311 | 四元数C[0] | 四元数C[1] | 四元数C[2] | 四元数C[3] | 四元数D[0] | 四元数D[1] | 四元数D[2] | 四元数D[3] |
>四元数的ABCD四个单精度浮点数被拆分为16个字节发送

当ID设置0x02时，原始6轴数据将按照下面两种CAN数据包发送 :
|CANID  | Data[00]    | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    |
|---------|------------|------------|------------|------------|------------|------------|------------|------------|
| 0x320  | GyXH | GyXL | GyYH | GyYL | GyZH | GyZL | 0x03 | 保留 |  

|CANID  | Data[00]    | Data[01]    | Data[02]    | Data[03]    | Data[04]    | Data[05]    | Data[06]    | Data[07]    |
|------------|------------|------------|------------|------------|------------|------------|------------|----------|
|0x321 | AcXH | AcXL | AcYH | AcYL | AcZH | AcZL | 0x03 | 保留 |
>原始数据的6个半字整型有符号数被拆分为12个字节发送。CAN包的第7字节数据为0x03，表征CAN传输的是原始6轴数据
---

## 用户配置信息存储
- Flash地址:0x803F800
- 总长度:20 Bytes
>如果单片机中没有存储用户配置，或者配置CRC校核失败，则会重新搭建配置信息

|偏移量|+00|+01|+02|+03|+04|+05|+06|+07|
|------------|------------|------------|------------|------------|------------|------------|------------|----------|
|数据信息|OTSEL|ID|ODR|MOD|GyXH|GyXL|GyYH|GyYL|

|偏移量|+08|+09|+0A|+0B|+0C|+0D|+0E|+0F|
|------------|------------|------------|------------|------------|------------|------------|------------|----------|
|数据信息|GyZH|GyZL|AcXH|AcXL|AcYH|AcYL|AcZH|AcZL|

|偏移量|+10|+11|+12|+13|
|------------|------------|------------|------------|------------|
|数据信息|CRC[0]|CRC[1]|CRC[2]|CR[3]|

- OTSEL:  选择输出接口，范围 **[0x00-0x01]**。 0:USB-C 1:CAN。
- ID:     CAN总线ID设置，范围 **[0x00-0x09]**。CAN总线输出使用的两个ID分别为 **[0x300+(ID<<8)]** 和 **[0x301+(ID<<8)]**。
- ODR:  对外输出速率设置，范围 **[0x00-0x03]**。0:1KHz 1:500Hz 2:250Hz 3:125Hz。
- MOD： 对外输出模式设置，范围 **[0x00-0x01]**。0:四元数 1:原始6轴数据
- GyNH\AcNL:  N方向 陀螺仪/加速度计 稳态误差修正值高/低字节。1×int16_t被拆分为2×uint8_t存储。
- CRC32:  多项式0x4C11DB7
- 默认的用户配置信息: 0x00020000U | 0x00000000U | 0x00000000U | 0x00000000U | 0x0AF3CE57U
---

## CRC-8算法
  * Mode:          CRC-8/MAXIM
  * Polynomial:    X8+X5+X4+1(0x31)
  * Init:          0x00
  * XOROUT:        0x00
```C
/**
  * @brief          计算CRC8
  * @param[in]      ptr: 数据指针
  * @param[in]      len: 校验长度
  * @retval         CRC-8值
  */
uint8_t cal_crc8_table(uint8_t *ptr, uint8_t len)
{
	uint8_t uc_index;
	uint8_t ucCRC8 = 0x00;
    while (len--)
    {
        uc_index = ucCRC8^(*ptr++);
        ucCRC8 = CRC8_table[uc_index];
    }
    return(ucCRC8);
}
```
### CRC-8表:
```C
const uint8_t CRC8_table[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
```
---
## IMU信号链路
经测试，RM机器人板材震动频率在120~150Hz,，陀螺仪和角速度计输出频率被配置为1000Hz，经过片内536Hz抗混叠滤波器后，输入片内111Hz低通滤波器。输出数据传入单片机，减去偏移值后，送入MahonyAHRS算法以1KHz更新四元数。