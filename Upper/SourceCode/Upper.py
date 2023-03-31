import threading
import time

import serial
import serial.tools.list_ports

import tkinter
from tkinter import ttk
from tkinter import messagebox

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from collections import deque
import numpy as np
from math import *
from   scipy  import   signal


import crcmod
import DataProcess as DP

import pandas as pd

global GUI

#通讯类 实现串口
class cCOM:
   
    def __init__(self):
        self.COM_handel = serial.Serial()
        self.sel_i = 0
        self.port_list = list()
   
    def write(self,data):
        self.COM_handel.write(data)
   
    def IsGYH1Connected(self):
        try:
            #发送空白信息
            time.sleep(0.01)
            self.COM_handel.flushInput()
            TxBuf = CMDPack(order='FF',data='FF')
            COM.write(TxBuf)
            time.sleep(0.01)
            count=COM.COM_handel.inWaiting()
            #读取有无空白信息
            Rec = COM.COM_handel.read(count)
            if Rec == TxBuf:
                return True
            else:
                return False
        except:
            return False
    
    def connect(self,isRST='none'):
        self.COM_handel.close()
        time.sleep(0.01)
        if isRST == 'none':
            try:
                self.COM_handel = serial.Serial(self.port_list[self.sel_i].name, 864000, timeout = 0.1)
                return True
            except:
                return False
        elif isRST == 'rst':
            try:
                self.COM_handel = serial.Serial(self.port_list[self.sel_i].name, 864000, timeout = 0.1)
                TxBuf = CMDPack(order='00',data='00')
                self.write(TxBuf)
                self.COM_handel.close()
                time.sleep(1)
                self.COM_handel = serial.Serial(self.port_list[self.sel_i].name, 864000, timeout = 0.1)
                #关闭串口对外信息发送
                TxBuf = CMDPack(order='00',data='03')
                COM.write(TxBuf)
                time.sleep(0.01)
                self.COM_handel.flushInput()
                return True
            except:
                return False

#设置类 四个按钮后面加
class cIMU_Set:
    pass

#修正值类 INT List列表
class cIMU_Calib:
    def __init__(self):
        self.GyXc = 0
        self.GyXc = 0
        self.GyYc = 0
        self.GyZc = 0
        self.AcXc = 0
        self.AcYc = 0
        self.AcZc = 0
        self.Cal_Thread_Enable = 0
        self.Cal_ACC_Sem = 0
        self.Position = ''
        self.isAxisCal = [0,0,0,0,0,0]

    def clear(self):
        self.GyXc = 0
        self.GyXc = 0
        self.GyYc = 0
        self.GyZc = 0
        self.AcXc = 0
        self.AcYc = 0
        self.AcZc = 0
        self.Cal_Thread_Enable = 0
        self.Cal_ACC_Sem = 0
        self.Position = ''
        self.isAxisCal = [0,0,0,0,0,0]

#IMU原始数据 INT FIFO队列
class cIMU_Data:
    def __init__(self):
        self.UI_t = deque(maxlen=1500) 
        self.GyXq = deque(maxlen=1500)
        self.GyYq = deque(maxlen=1500)
        self.GyZq = deque(maxlen=1500)
        self.GyZc = deque(maxlen=1500)
        self.AcXq = deque(maxlen=1500)
        self.AcYq = deque(maxlen=1500)
        self.AcZq = deque(maxlen=1500)
        self.Data_Thread_Enable = 0
    def clear(self):
        self.UI_t.clear()
        self.GyXq.clear()
        self.GyYq.clear()
        self.GyZq.clear()
        self.GyZc.clear()
        self.AcXq.clear()
        self.AcYq.clear()
        self.AcZq.clear()
        self.Data_Thread_Enable = 0

COM = cCOM()
IMU_Set = cIMU_Set()
IMU_Calib = cIMU_Calib()
IMU_Data = cIMU_Data()

#输入Byte 返回Byte
def CRC8(bytes):
    crc_class = crcmod.predefined.Crc('crc-8-maxim')
    crc_class.update(bytes)
    return crc_class.crcValue.to_bytes(length=1,byteorder='big')

def portIsUsable(portName):
    try:
       ser = serial.Serial(port=portName)
       return True
    except:
       return False

    
#给出控制符和命令数据，自动补齐包头和CRC8
def CMDPack(order,data):
    buf = bytes.fromhex('30')
    buf += bytes.fromhex(order)
    buf += bytes.fromhex(data)
    buf += CRC8(buf)
    return buf

def COMConnect():
    
    COM.port_list = list(serial.tools.list_ports.comports())
    if len(COM.port_list) < 1:
        print('\n无可用串口,按回车键重新扫描')
        input()
        return False
    else:
        print('可使用串口如下:')
        for COM.sel_i in range(len(COM.port_list)): print('ID',COM.sel_i,':',COM.port_list[COM.sel_i])
        COM.sel_i = input("\n请输入要启用的 GY-H1 串口ID: ")
        while True:
            try:
                COM.sel_i=int(COM.sel_i)
                break
            except:
                COM.sel_i = input("\n请重新输入串口ID: ")

        while COM.sel_i > len(COM.port_list) - 1:
            COM.sel_i = input("\n请重新输入串口ID: ")
            while True:
                try:
                    COM.sel_i=int(COM.sel_i)
                    break
                except:
                    COM.sel_i = input("\n请重新输入串口ID: ")

        if portIsUsable(COM.port_list[COM.sel_i].name) == False:
            print("\n串口已被占用,请重新选择\n")
            return False
        
        print('连接设备中...')
        #连接设备并重启
        COM.connect('rst')
        
        if COM.IsGYH1Connected():
            print('\nGY-H1 已连接\n')
            return True
        else:
            print('\nGY-H1 连接错误\n')
            COM.COM_handel.close()
            return False
        
def SettingApply():

    global GUI

    time.sleep(0.01)
    #重置连接   
    COM.connect('rst')

    time.sleep(0.01)

    if COM.IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1 Disconneted!!')
        print("GY-H1 Setting Failed!!\n")
        GUI.destroy()
        return
    else:
        TxBuf = CMDPack(order='01',data='00')
        COM.write(TxBuf)
        time.sleep(0.1)

        match IMU_Set.OutMode_CB.get():
            case "USB-C":
                TxBuf = CMDPack(order='02',data='00')
                COM.write(TxBuf)
            case "CAN":
                TxBuf = CMDPack(order='02',data='01')
                COM.write(TxBuf)
        time.sleep(0.1)

        match IMU_Set.ODR_CB.get():
            case "1000Hz":
                TxBuf = CMDPack(order='03',data='00')
                COM.write(TxBuf)
            case "500Hz":
                TxBuf = CMDPack(order='03',data='01')
                COM.write(TxBuf)
            case "250Hz":
                TxBuf = CMDPack(order='03',data='02')
                COM.write(TxBuf)
            case "125Hz":
                TxBuf = CMDPack(order='03',data='03')
                COM.write(TxBuf)
        time.sleep(0.1)
        
        match IMU_Set.OutFormat_CB.get():
            case "Quaternion":
                TxBuf = CMDPack(order='02',data='02')
                COM.write(TxBuf)
            case "Raw data":
                TxBuf = CMDPack(order='02',data='03')
                COM.write(TxBuf)
        time.sleep(0.1)

        match IMU_Set.ID_CB.get():
            case "1":
                TxBuf = CMDPack(order='04',data='00')
                COM.write(TxBuf)
            case "2":
                TxBuf = CMDPack(order='04',data='01')
                COM.write(TxBuf)
            case "3":
                TxBuf = CMDPack(order='04',data='02')
                COM.write(TxBuf)
            case "4":
                TxBuf = CMDPack(order='04',data='03')
                COM.write(TxBuf)
            case "5":
                TxBuf = CMDPack(order='04',data='04')
                COM.write(TxBuf)
            case "6":
                TxBuf = CMDPack(order='04',data='05')
                COM.write(TxBuf)
            case "7":
                TxBuf = CMDPack(order='04',data='06')
                COM.write(TxBuf)
            case "8":
                TxBuf = CMDPack(order='04',data='07')
                COM.write(TxBuf)
            case "9":
                TxBuf = CMDPack(order='04',data='08')
                COM.write(TxBuf)
        time.sleep(0.1)
        
        TxBuf = CMDPack(order='01',data='01')
        COM.write(TxBuf)

        tkinter.messagebox.showinfo('Info','GY-H1 Configuration Succeed!!')
        print("GY-H1 Setting Succeed!!\n")
        GUI.destroy()

#GY-H1 Setting
def GYSetFunGUI():

    global GUI

    GUI = tkinter.Tk()
    GUI.title('GY-H1 Setting')
    GUI.geometry('200x350')

    Title1 = tkinter.Label(GUI,anchor='n',text='GY-H1 SETTING',bg='blue',fg="white",font=('Arial', 16),width=40,height=1)
    Title1.pack(side='top',fill='x')
   
    #输出模式设置
    TCB1 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='OTSEL',font=('Arial', 11),width=16,height=1)
    IMU_Set.OutMode_CB = ttk.Combobox(GUI,width='18',justify='left')
    IMU_Set.OutMode_CB["value"]=("USB-C","CAN")
    IMU_Set.OutMode_CB.current(0)

    #输出速率设置
    TCB2 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='ODR',font=('Arial', 11),width=16,height=1)
    IMU_Set.ODR_CB = ttk.Combobox(GUI,width='18',justify='left')
    IMU_Set.ODR_CB["value"]=("1000Hz","500Hz","250Hz","125Hz")
    IMU_Set.ODR_CB.current(2)

    #输出数据设置
    TCB3 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='MOD',font=('Arial', 11),width=16,height=1)
    IMU_Set.OutFormat_CB = ttk.Combobox(GUI,width='18',justify='left')
    IMU_Set.OutFormat_CB["value"]=("Quaternion","Raw data")
    IMU_Set.OutFormat_CB.current(0)

    #ID设置
    TCB4 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='ID',font=('Arial', 11),width=16,height=1)
    IMU_Set.ID_CB = ttk.Combobox(GUI,width='18',justify='left')
    IMU_Set.ID_CB["value"]=("0","1","2","3","4","5","6","7","8","9",)
    IMU_Set.ID_CB.current(0)

    ApplyButton = tkinter.Button(GUI,bg='yellow',relief='groove',fg='red',text='APPLY',font=('Arial', 16),command=SettingApply)
    ApplyButton.pack(side='bottom',fill='x')

    TCB1.pack(pady = (20,0))
    IMU_Set.OutMode_CB.pack(pady = 0)
    TCB2.pack(pady = (20,0))
    IMU_Set.ODR_CB.pack(pady = 0)
    TCB3.pack(pady = (20,0))
    IMU_Set.OutFormat_CB.pack(pady = 0)
    TCB4.pack(pady = (20,0))
    IMU_Set.ID_CB.pack(pady = 0)

    GUI.mainloop()

def ConnecWarning():
    global ConnectButton

    ConnectButton['bg'] = 'yellow'
    ConnectButton['fg'] = 'red'
    ConnectButton['command'] = CalConRec
    ConnectButton['text'] = 'Start Read Data'
    
    tkinter.messagebox.showinfo('Info','此操作将会重置ORD与MOD\n请在结束校准后重新设置!!')


def CalConRec():

    global ConnectButton
    global ACalButton
    global GCalButton
    global GUI
    global MOD_Cal

    ConnectButton['state'] = 'disable'
    MOD_Cal['state'] = 'disable'
    ConnectButton['bg'] = 'white'

    #重置连接
    COM.connect()

    #检验设备连接状态
    if COM.IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1连接异常!!')
        GUI.destroy()
        return False
    
    #初始数据读取模式
    if MOD_Cal.get() == "Check Raw data":
        ConnectButton['text'] = 'Reading Raw Data...'
        ACalButton['bg'] = 'Gold'
        ACalButton['fg'] = 'OrangeRed'
        ACalButton['state'] = 'normal'
        GCalButton['bg'] = 'Gold'
        GCalButton['fg'] = 'OrangeRed'
        GCalButton['state'] = 'normal'
    #后处理数据读取
    else:
        ConnectButton['text'] = 'Reading Processed Data...'
        ACalButton['bg'] = 'white'
        GCalButton['bg'] = 'white'
           
        time.sleep(0.1)
        TxBuf = CMDPack(order='00',data='04')
        COM.COM_handel.flushInput()
        COM.write(TxBuf)
        time.sleep(0.01)
        count=COM.COM_handel.inWaiting()
        time.sleep(0.01)
        Rec = COM.COM_handel.read(count)
        if (count == 14) & (Rec[13:14] == CRC8(Rec[0:13])):
            IMU_Calib.GyXc = float(int.from_bytes(Rec[1:3],byteorder='big',signed=True))/1000
            IMU_Calib.GyYc = float(int.from_bytes(Rec[3:5],byteorder='big',signed=True))/1000
            IMU_Calib.GyZc = float(int.from_bytes(Rec[5:7],byteorder='big',signed=True))/1000
            IMU_Calib.AcXc = float(int.from_bytes(Rec[7:9],byteorder='big',signed=True))/1000
            IMU_Calib.AcYc = float(int.from_bytes(Rec[9:11],byteorder='big',signed=True))/1000
            IMU_Calib.AcZc = float(int.from_bytes(Rec[11:13],byteorder='big',signed=True))/800

            ACalButton['text'] = 'Accel Static Correct Value [X,Y,X] = [' + str(IMU_Calib.AcXc) +', '+ str(IMU_Calib.AcYc) +', '+ str(IMU_Calib.AcZc)+']'
            GCalButton['text'] = 'Gyro Static Correct Value [X,Y,X] =  [' + str(IMU_Calib.GyXc) +', '+ str(IMU_Calib.GyYc) +', '+ str(IMU_Calib.GyZc)+']'

        else:
            tkinter.messagebox.showinfo('Error','读取校准错误')
            GUI.destroy()
            return False
    
    #进入配置模式
    time.sleep(0.5)
    TxBuf = CMDPack(order='01',data='00')
    COM.write(TxBuf)
    time.sleep(0.1)
    #设置输出速率
    TxBuf = CMDPack(order='03',data='01')
    COM.write(TxBuf)
    time.sleep(0.1)
    #原始数据模式
    TxBuf = CMDPack(order='02',data='03')
    COM.write(TxBuf)
    time.sleep(0.1)
    #保存设置
    TxBuf = CMDPack(order='01',data='01')
    COM.write(TxBuf)
    time.sleep(0.5)
    COM.COM_handel.close()
    time.sleep(0.5)
    #连接设备
    COM.connect('rst')
    #重新连接设备
    if COM.IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1连接异常!!')
        GUI.destroy()
        print('GY-H1连接异常!!')
        return False
    #启动接收进程
    IMU_Data.Data_Thread_Enable = 1
    thread_DataRec = threading.Thread(target=DataRec)
    thread_DataRec.start()   

def DataRec():
    
    #开启数据接收
    Txbuf = CMDPack(order='00',data='02')
    COM.write(Txbuf)
    time.sleep(1)
    tbuf = 0
    if MOD_Cal.get() == "Check Processed data":
        while True:
            RecBuf = COM.COM_handel.read(14)
            #对齐数据
            if COM.COM_handel.inWaiting() != 0 :
                COM.COM_handel.flushInput()

            #CRC校验
            if RecBuf[13:14] == CRC8(RecBuf[0:13]):
                GyroX = float(int.from_bytes(RecBuf[1:3],byteorder='big',signed=True))
                GyroY = float(int.from_bytes(RecBuf[3:5],byteorder='big',signed=True))
                GyroZ = float(int.from_bytes(RecBuf[5:7],byteorder='big',signed=True))
                AccelX = float(int.from_bytes(RecBuf[7:9],byteorder='big',signed=True))
                AccelY = float(int.from_bytes(RecBuf[9:11],byteorder='big',signed=True))
                AccelZ = float(int.from_bytes(RecBuf[11:13],byteorder='big',signed=True))

                
                GyroX += IMU_Calib.GyXc
                GyroY += IMU_Calib.GyYc
                GyroZ += IMU_Calib.GyZc
                AccelX += IMU_Calib.AcXc
                AccelY += IMU_Calib.AcYc
                AccelZ += IMU_Calib.AcZc

                #进入队列
                tbuf += 0.003#加了矫正
                IMU_Data.UI_t.append(tbuf)
                IMU_Data.GyXq.append(GyroX)
                IMU_Data.GyYq.append(GyroY)
                IMU_Data.GyZq.append(GyroZ)
                IMU_Data.AcXq.append(AccelX)
                IMU_Data.AcYq.append(AccelY)
                IMU_Data.AcZq.append(AccelZ)

            #进程退出
            if IMU_Data.Data_Thread_Enable == 0:
                return
    else:
        while True:
            RecBuf = COM.COM_handel.read(14)
            #对齐数据
            if COM.COM_handel.inWaiting() != 0 :
                COM.COM_handel.flushInput()

            #CRC校验
            if RecBuf[13:14] == CRC8(RecBuf[0:13]):
                GyroX = int.from_bytes(RecBuf[1:3],byteorder='big',signed=True)
                GyroY = int.from_bytes(RecBuf[3:5],byteorder='big',signed=True)
                GyroZ = int.from_bytes(RecBuf[5:7],byteorder='big',signed=True)
                AccelX = int.from_bytes(RecBuf[7:9],byteorder='big',signed=True)
                AccelY = int.from_bytes(RecBuf[9:11],byteorder='big',signed=True)
                AccelZ = int.from_bytes(RecBuf[11:13],byteorder='big',signed=True)

                #进入队列
                tbuf += 0.002#加了矫正
                IMU_Data.UI_t.append(tbuf)
                IMU_Data.GyXq.append(GyroX)
                IMU_Data.GyYq.append(GyroY)
                IMU_Data.GyZq.append(GyroZ)
                IMU_Data.AcXq.append(AccelX)
                IMU_Data.AcYq.append(AccelY)
                IMU_Data.AcZq.append(AccelZ)

            #进程退出
            if IMU_Data.Data_Thread_Enable == 0:
                return
def Gyro_Update(i):

    if len(IMU_Data.GyXq)!=0:
        IMU_Calib.aG.set_xlim(IMU_Data.UI_t[0],IMU_Data.UI_t[-1])
        max1 = np.amax(IMU_Data.GyXq)
        max2 = np.amax(IMU_Data.GyYq)
        max3 = np.amax(IMU_Data.GyZq)
        min1 = np.amin(IMU_Data.GyXq)
        min2 = np.amin(IMU_Data.GyYq)
        min3 = np.amin(IMU_Data.GyZq)
       
        if max1 > max2:
            max = max1
        else:
            max = max2
        if max3 > max:
            max = max3

        if min1 < min2:
            min = min1
        else:
            min = min2
        if min3 < min:
            min = min3

        IMU_Calib.aG.set_ylim(min-0.2*np.abs(min),max+0.2*np.abs(max))
        IMU_Calib.UI_Gyro_Line[0].set_data(IMU_Data.UI_t,IMU_Data.GyXq)  # update the data
        IMU_Calib.UI_Gyro_Line[1].set_data(IMU_Data.UI_t,IMU_Data.GyYq)  # update the data
        IMU_Calib.UI_Gyro_Line[2].set_data(IMU_Data.UI_t,IMU_Data.GyZq)  # update the data
    return IMU_Calib.UI_Gyro_Line,

def Accel_Update(i):

    if len(IMU_Data.AcXq)!=0:
        IMU_Calib.aA.set_xlim(IMU_Data.UI_t[0],IMU_Data.UI_t[-1])
        max1 = np.amax(IMU_Data.AcXq)
        max2 = np.amax(IMU_Data.AcYq)
        max3 = np.amax(IMU_Data.AcZq)
        min1 = np.amin(IMU_Data.AcXq)
        min2 = np.amin(IMU_Data.AcYq)
        min3 = np.amin(IMU_Data.AcZq)
    
        if max1 > max2:
            max = max1
        else:
            max = max2
        if max3 > max:
            max = max3

        if min1 < min2:
            min = min1
        else:
            min = min2
        if min3 < min:
            min = min3

        IMU_Calib.aA.set_ylim(min-0.2*np.abs(min),max+0.2*np.abs(max))
        IMU_Calib.UI_Accel_Line[0].set_data(IMU_Data.UI_t,IMU_Data.AcXq)  # update the data
        IMU_Calib.UI_Accel_Line[1].set_data(IMU_Data.UI_t,IMU_Data.AcYq)  # update the data
        IMU_Calib.UI_Accel_Line[2].set_data(IMU_Data.UI_t,IMU_Data.AcZq)  # update the data
    return IMU_Calib.UI_Accel_Line,


def GyroCaliThread():

    print('Gyro Calibrating...\n')
    ACalButton['state'] = 'disable'
    GCalButton['state'] = 'disable'
    ACalButton['text'] = '0%'
    GCalButton['text'] = 'Gyro Calibrating...'
    rate = 0
    bufX = list()
    bufY = list()
    bufZ = list()
    Ticker = list()
    ticker_now = 0

    #等待桌面稳定
    time.sleep(2)


    #50s 采样率200Hz 
    while rate < 100:
        rate += 1
        
        for ii in range(100):
            bufX.append(IMU_Data.GyXq[-1]) 
            bufY.append(IMU_Data.GyYq[-1]) 
            bufZ.append(IMU_Data.GyZq[-1])
            Ticker.append(ticker_now)
            ticker_now += 0.005
            time.sleep(0.005) 

        ACalButton['text'] = str(rate)+' %'
    
        if IMU_Calib.Cal_Thread_Enable == 0:
            return
    
    #去除异常值,采用插值替换方法
    bufX_p = DP.Dataculling(data=bufX,metheod='replace')
    bufY_p = DP.Dataculling(data=bufY,metheod='replace')
    bufZ_p = DP.Dataculling(data=bufZ,metheod='replace')
    
    #1阶低通 200Hz采样率 截至频率1Hz 
    b,a = signal.butter(1,0.01,'lowpass')
    bufX_p = signal.filtfilt(b,a,bufX_p)
    b,a = signal.butter(1,0.01,'lowpass')
    bufY_p = signal.filtfilt(b,a,bufY_p)
    b,a = signal.butter(1,0.01,'lowpass')
    bufZ_p = signal.filtfilt(b,a,bufZ_p)
    
  
    #平均一下,四舍五入,截去前2%的数据
    IMU_Calib.GyXc = -round(1000*np.average(bufX_p[int(len(bufX_p)*0.02):-1]))
    IMU_Calib.GyYc = -round(1000*np.average(bufY_p[int(len(bufY_p)*0.02):-1]))
    IMU_Calib.GyZc = -round(1000*np.average(bufZ_p[int(len(bufZ_p)*0.02):-1]))
    
    try:
        #输出用于校准的序列
        csvbuf = {'Ticker':Ticker,'GyroXRaw':bufX,'GyroYRaw':bufY,'GyroZRaw':bufZ,'GyroXP':bufX_p,'GyroYP':bufY_p,'GyroZP':bufZ_p}
        outbuf = pd.DataFrame(csvbuf)
        outbuf = outbuf.set_index('Ticker')
        t = time.localtime()
        name = 'GyroCalibrationData'+'_'+str(t.tm_hour)+'H'+str(t.tm_min)+'M'+str(t.tm_min)+'S.csv'
        outbuf.to_csv(name)
        print('Gyro校准数据已输出: '+name+'\n')
    except:
        print('Gyro校准数据输出异常!!')

    print('Gyro Corret Value:')
    print('Correct [X,Y,Z] = [',float(IMU_Calib.GyXc)/1000,', ',float(IMU_Calib.GyYc)/1000,', ',float(IMU_Calib.GyZc)/1000,']')
    print('STD [X,Y,Z] = [',np.std(bufX,ddof=0),', ',np.std(bufY,ddof=0),', ',np.std(bufZ,ddof=0),']')
    
    ACalButton['state'] = 'disable'
    GCalButton['state'] = 'normal'
    ACalButton['text'] = 'Gyro Static Correct Value [X,Y,X] = [' + str(float(IMU_Calib.GyXc)/1000) +', '+ str(float(IMU_Calib.GyYc)/1000) +', '+ str(float(IMU_Calib.GyZc)/1000)+']'
    GCalButton['text'] = 'Apply'
    tkinter.messagebox.showinfo('Info','GY-H1 Gyro静态校准值计算完成!!\n按Apply写入校准,关闭窗口丢弃值')
    
    #切换标志位
    IMU_Calib.Cal_Thread_Enable = 2
    return

def AccelAssistThread():
    AccX = 0
    AccY = 0
    AccZ = 0
    while len(IMU_Data.AcXq)<3:
        time.sleep(0.1)
    while True:
        time.sleep(0.1)
        AccX = 0.2*AccX + 0.8*IMU_Data.AcXq[-1]
        AccY = 0.2*AccY + 0.8*IMU_Data.AcYq[-1]
        AccZ = 0.2*AccZ + 0.8*IMU_Data.AcZq[-1]

        if abs(AccX) < abs(AccZ) and abs(AccY) < abs(AccZ):
            if AccZ > 0:
                IMU_Calib.Position = 'Z+'
            else:
                IMU_Calib.Position = 'Z-'
        elif abs(AccX) < abs(AccY) and abs(AccZ) < abs(AccY):
            if AccY > 0:
                IMU_Calib.Position = 'Y+'
            else:
                IMU_Calib.Position = 'Y-'
        else:
            if AccX > 0:
                IMU_Calib.Position = 'X+'
            else:
                IMU_Calib.Position = 'X-'
        
        if IMU_Calib.Cal_Thread_Enable == 0:
            return

def AccelCaliThread(axis,direction):
    global Accel_Rate

    if axis == 'X':
        times = 0
        bufX = list()
        bufX_p = list()

        while times < 20:
            times += 0.005
            bufX.append(IMU_Data.AcXq[-1])
            time.sleep(0.005)
            Accel_Rate += 0.00416
        bufX_p = DP.Dataculling(data=bufX,metheod='replace')
        #1阶低通 200Hz采样率 截至频率1Hz 
        b,a = signal.butter(1,0.01,'lowpass')
        bufX_p = signal.filtfilt(b,a,bufX_p)
         #平均一下,四舍五入,截去前2%的数据
        value = -round(1000*np.average(bufX_p[int(len(bufX_p)*0.02):-1]))
        print(axis+direction+' = '+ str(value/1000) +'\n')
        IMU_Calib.AcXc += value
        #标记某轴已校准
        if direction == '+':
            IMU_Calib.isAxisCal[0] = 1
        else:
            IMU_Calib.isAxisCal[1] = 1
        IMU_Calib.Cal_ACC_Sem = 0

    if axis == 'Y':
        times = 0
        bufY = list()
        bufY_p = list()
        while times < 20:
            times += 0.005
            bufY.append(IMU_Data.AcYq[-1])
            time.sleep(0.005)
            Accel_Rate += 0.00416
        bufY_p = DP.Dataculling(data=bufY,metheod='replace')
        #1阶低通 200Hz采样率 截至频率1Hz 
        b,a = signal.butter(1,0.01,'lowpass')
        bufY_p = signal.filtfilt(b,a,bufY_p)
        #平均一下,四舍五入,截去前2%的数据
        value = -round(1000*np.average(bufY_p[int(len(bufY_p)*0.02):-1]))
        print(axis+direction+' = '+str(value/1000)+'\n')
        IMU_Calib.AcYc += value
        #标记某轴已校准
        if direction == '+':
            IMU_Calib.isAxisCal[2] = 1
        else:
            IMU_Calib.isAxisCal[3] = 1
        IMU_Calib.Cal_ACC_Sem = 0

    if axis == 'Z':
        times = 0
        bufZ = list()
        bufZ_p = list()
        while times < 20:
            times += 0.005
            bufZ.append(IMU_Data.AcZq[-1])
            time.sleep(0.005)
            Accel_Rate += 0.00416
        bufZ_p = DP.Dataculling(data=bufZ,metheod='replace')
        #1阶低通 200Hz采样率 截至频率1Hz 
        b,a = signal.butter(1,0.01,'lowpass')
        bufZ_p = signal.filtfilt(b,a,bufZ_p)
        #平均一下,四舍五入,截去前2%的数据
        value = -round(800*np.average(bufZ_p[int(len(bufZ_p)*0.02):-1]))
        print(axis+direction+' = '+str(value/800)+'\n')
        IMU_Calib.AcZc += value
        #标记某轴已校准
        if direction == '+':
            IMU_Calib.isAxisCal[4] = 1
        else:
            IMU_Calib.isAxisCal[5] = 1
        IMU_Calib.Cal_ACC_Sem = 0

    if sum(IMU_Calib.isAxisCal) != 6:
        tkinter.messagebox.showinfo('Info',axis+direction+'Axis Calibrated OK!!\nPlease Change the Direction of GY-H1!!')

def AccelCali(Position):
    IMU_Calib.Cal_ACC_Sem = 1
    ACalButton['text'] = Position+' Accel Calibrating...' 
    ACalButton['state'] = 'disable'
    ACalButton['bg'] = 'DarkKhaki'
    threading.Thread(target=AccelCaliThread,args=(Position)).start()

def AccelCaliControlThread():

    print('Accel Calibrating...\n')
    ACalButton['state'] = 'disable'
    GCalButton['state'] = 'disable'
    GCalButton['text'] = '0%'
    
    global Accel_Rate
    Accel_Rate = 0

# 循坏内 检测当前方向是否已经矫正
    while sum(IMU_Calib.isAxisCal) < 6:
        time.sleep(0.5)
        #某坐标正在矫正,停止检测方位,开启动态更新进度
        while IMU_Calib.Cal_ACC_Sem:
            GCalButton['text'] = str(round(Accel_Rate)) +' %'
            time.sleep(0.5)
            if IMU_Calib.Cal_Thread_Enable == 0:
                return
        if IMU_Calib.Cal_Thread_Enable == 0:
                return
        #无坐标轴正在矫正,开启检测方位,动态更新按钮绑定函数
        match IMU_Calib.Position:
            case "X+":
                if IMU_Calib.isAxisCal[0]:
                    ACalButton['text'] = 'X+ Already Calibrated. Please Change Direction!'
                    ACalButton['state'] = 'disable'
                    ACalButton['bg'] = 'Silver'
                else:
                    ACalButton['text'] = 'X+ Press to Calibrate'
                    ACalButton['command'] = lambda : AccelCali('X+')
                    ACalButton['state'] = 'normal'
                    ACalButton['bg'] = 'DarkGreen'
            case "X-":
                if IMU_Calib.isAxisCal[1]:
                    ACalButton['text'] = 'X- Already Calibrated. Please Change Direction!'
                    ACalButton['state'] = 'disable'
                    ACalButton['bg'] = 'Silver'
                else:
                    ACalButton['text'] = 'X- Press to Calibrate'
                    ACalButton['command'] = lambda : AccelCali('X-')
                    ACalButton['state'] = 'normal'
                    ACalButton['bg'] = 'DarkGreen'

            case "Y+":
                if IMU_Calib.isAxisCal[2]:
                    ACalButton['text'] = 'Y+ Already Calibrated. Please Change Direction!'
                    ACalButton['state'] = 'disable'
                    ACalButton['bg'] = 'Silver'
                else:
                    ACalButton['text'] = 'Y+ Press to Calibrate'
                    ACalButton['command'] = lambda : AccelCali('Y+')
                    ACalButton['state'] = 'normal'
                    ACalButton['bg'] = 'DarkGreen'
            case "Y-":
                if IMU_Calib.isAxisCal[3]:
                    ACalButton['text'] = 'Y- Already Calibrated. Please Change Direction!'
                    ACalButton['state'] = 'disable'
                    ACalButton['bg'] = 'Silver'
                else:
                    ACalButton['text'] = 'Y- Press to Calibrate'
                    ACalButton['command'] = lambda : AccelCali('Y-')
                    ACalButton['state'] = 'normal'
                    ACalButton['bg'] = 'DarkGreen'
                
            case "Z+":
                if IMU_Calib.isAxisCal[4]:
                    ACalButton['text'] = 'Z+ Already Calibrated. Please Change Direction!'
                    ACalButton['state'] = 'disable'
                    ACalButton['bg'] = 'Silver'
                else:
                    ACalButton['text'] = 'Z+ Press to Calibrate'
                    ACalButton['command'] = lambda : AccelCali('Z+')
                    ACalButton['state'] = 'normal'
                    ACalButton['bg'] = 'DarkGreen'
            case "Z-":
                if IMU_Calib.isAxisCal[5]:
                    ACalButton['text'] = 'Z- Already Calibrated. Please Change Direction!'
                    ACalButton['state'] = 'disable'
                    ACalButton['bg'] = 'Silver'
                else:
                    ACalButton['text'] = 'Z- Press to Calibrate'
                    ACalButton['command'] = lambda : AccelCali('Z-')
                    ACalButton['state'] = 'normal'
                    ACalButton['bg'] = 'DarkGreen'
    
    GCalButton['state'] = 'disable'
    ACalButton['state'] = 'normal'
    GCalButton['text'] = 'Accel Static Correct Value [X,Y,X] = [' + str(float(IMU_Calib.AcXc)/1000) +', '+ str(float(IMU_Calib.AcYc)/1000) +', '+ str(float(IMU_Calib.AcZc)/800)+']'
    ACalButton['text'] = 'Apply'
    ACalButton['command'] = AccelKeyHandel
    ACalButton['bg'] = 'Gold'
    ACalButton['fg'] = 'Red'

    print('Correct [X,Y,Z] = [',float(IMU_Calib.AcXc)/1000,', ',float(IMU_Calib.AcYc)/1000,', ',float(IMU_Calib.AcZc)/800,']')
    tkinter.messagebox.showinfo('Info','GY-H1 Accel静态校准值计算完成!!\n按Apply写入校准,关闭窗口丢弃值')
    #切换标志位
    IMU_Calib.Cal_Thread_Enable = 2
    return

def GyroWrite():

    time.sleep(0.1)
    #重置连接
    COM.connect('rst')
    
    #确认连接状态
    if COM.IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1 Disconneted!!')
        print("GY-H1 Calibration Failed!!\n")
        GUI.destroy()
        return
    else:
        time.sleep(0.1)    
        #进入写入状态
        TxBuf = CMDPack(order='01',data='00')
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(IMU_Calib.GyXc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='10',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='11',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(IMU_Calib.GyYc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='12',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='13',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(IMU_Calib.GyZc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='14',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='15',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)

        #退出写入状态
        if COM.IsGYH1Connected() == False:
            tkinter.messagebox.showinfo('Error','GY-H1 Calibratiion Failed!!')
            print("GY-H1 Gyro Correct Value Write Failed!!\n")
        else:
            tkinter.messagebox.showinfo('Info','GY-H1 Calibratiion Succeed!!')
            print("GY-H1 Gyro Calibrating Succeed!!\n")
        
        TxBuf = CMDPack(order='01',data='01')
        COM.write(TxBuf)
        time.sleep(0.5)
        GUI.destroy()

def AccelWrite():
    time.sleep(0.1)
    #重置连接
    COM.connect('rst')
    
    #确认连接状态
    if COM.IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1 Disconneted!!')
        print("GY-H1 Calibration Failed!!\n")
        GUI.destroy()
        return
    else:
        time.sleep(0.1)    
        #进入写入状态
        TxBuf = CMDPack(order='01',data='00')
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(IMU_Calib.AcXc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='16',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='17',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(IMU_Calib.AcYc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='18',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='19',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(IMU_Calib.AcZc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='1A',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='1B',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)

        #退出写入状态
        if COM.IsGYH1Connected() == False:
            tkinter.messagebox.showinfo('Error','GY-H1 Calibratiion Failed!!')
            print("GY-H1 Accel Correct Value Write Failed!!\n")
        else:
            tkinter.messagebox.showinfo('Info','GY-H1 Calibratiion Succeed!!')
            print("GY-H1 Accel Calibrating Succeed!!\n")
        
        TxBuf = CMDPack(order='01',data='01')
        COM.write(TxBuf)
        time.sleep(0.5)
        GUI.destroy()

def GyroKeyHandel():

    #选定要执行的校准
    if IMU_Calib.Cal_Thread_Enable == 0:
        IMU_Calib.Cal_Thread_Enable = 1
        ACalButton['state'] = 'disable'
        GCalButton['state'] = 'normal'
        ACalButton['bg'] = 'white'
        GCalButton['bg'] = 'DarkGreen'
        GCalButton['fg'] = 'Gold'
        GCalButton['text'] = 'Press to start calibrate Gyro'
        tkinter.messagebox.showinfo('Info','请保证GY-H1静止,校准过程持续90s')
    #开始执行校准
    elif IMU_Calib.Cal_Thread_Enable == 1:
        threading.Thread(target=GyroCaliThread).start()
    #执行写入程序
    elif IMU_Calib.Cal_Thread_Enable == 2:
        #关闭数据获取线程
        IMU_Data.Data_Thread_Enable = 0
        threading.Thread(target=GyroWrite).start()
        GCalButton['state'] = 'disable'

def AccelKeyHandel():

    #选定要执行的校准
    if IMU_Calib.Cal_Thread_Enable == 0:
        IMU_Calib.Cal_Thread_Enable = 1
        ACalButton['state'] = 'normal'
        GCalButton['state'] = 'disable'
        ACalButton['bg'] = 'DarkGreen'
        ACalButton['fg'] = 'Gold'
        GCalButton['bg'] = 'white'
        ACalButton['text'] = 'Press to start calibrate Accel'
        threading.Thread(target=AccelAssistThread).start()
        tkinter.messagebox.showinfo('Info','校准分为6小节,请按照提醒翻转GY-H1')
    #开始执行校准
    elif IMU_Calib.Cal_Thread_Enable == 1:
        threading.Thread(target=AccelCaliControlThread).start()
    #执行写入程序
    elif IMU_Calib.Cal_Thread_Enable == 2:
        #关闭数据获取线程
        IMU_Data.Data_Thread_Enable = 0
        threading.Thread(target=AccelWrite).start()
        ACalButton['state'] = 'disable'
    return
#GY-H1 Calibrate
def GYCalFunGUI():
    global GUI

    global ConnectButton
    global ACalButton
    global GCalButton
    global MOD_Cal

    IMU_Data.clear()
    IMU_Calib.clear()

    GUI = tkinter.Tk()
    GUI.title('GY-H1 Calibration')
    GUI.geometry('650x750')

    figGyro = plt.Figure()
    figGyro.set_size_inches(8,3)
    canvasGyro = FigureCanvasTkAgg(figGyro, master=GUI)
    IMU_Calib.aG = figGyro.add_subplot(111)
    IMU_Calib.aG.set_title('Gyro')
    
    IMU_Calib.UI_Gyro_Line = IMU_Calib.aG.plot([],[],'olive',[],[],'teal',[],[],'orchid')
    figGyro.legend(handles=[IMU_Calib.UI_Gyro_Line[0],IMU_Calib.UI_Gyro_Line[1],IMU_Calib.UI_Gyro_Line[2]],labels=['X','Y','Z'])

    aniG = animation.FuncAnimation(figGyro, Gyro_Update, np.arange(0, 1500), interval=3, blit=False)

    figAccel = plt.Figure()
    figAccel.set_size_inches(8,3)
    canvasAccel = FigureCanvasTkAgg(figAccel, master=GUI)
    IMU_Calib.aA = figAccel.add_subplot(111)
    IMU_Calib.aA.set_title('Accel')

    IMU_Calib.UI_Accel_Line = IMU_Calib.aA.plot([],[],'olive',[],[],'teal',[],[],'orchid')
    figAccel.legend(handles=[IMU_Calib.UI_Accel_Line[0],IMU_Calib.UI_Accel_Line[1],IMU_Calib.UI_Accel_Line[2]],labels=['X','Y','Z'])
    aniA = animation.FuncAnimation(figAccel, Accel_Update, np.arange(0, 1500), interval=3, blit=False)

    #输出数据设置栏
    MOD_Cal = ttk.Combobox(GUI,width='20',justify='center',font=('Arial', 14))
    MOD_Cal["value"]=("Check Raw data","Check Processed data")
    MOD_Cal.current(0)
    MOD_Cal.pack(side='top',fill='x')

    #连接按钮
    ConnectButton = tkinter.Button(GUI,bg='DeepSkyBlue',fg='white',relief='groove',state='normal',text='Ready to Calibrate?',font=('Arial', 16),command=ConnecWarning)
    ConnectButton.pack(side='top',fill='x')
    

    #实时数据显示
    canvasGyro.get_tk_widget().pack()
    canvasAccel.get_tk_widget().pack()

    #加速度校准
    ACalButton = tkinter.Button(GUI,bg='PaleTurquoise',fg='Gold',relief='groove',state='disable',text='Accel Calibrate',font=('Arial', 16),command=AccelKeyHandel)
    ACalButton.pack(side='bottom',fill='x')
    #陀螺仪校准
    GCalButton = tkinter.Button(GUI,bg='PaleTurquoise',fg='Gold',relief='groove',state='disable',text='Gyro Calibrate',font=('Arial', 16),command=GyroKeyHandel)
    GCalButton.pack(side='bottom',fill='x')

    GUI.mainloop()
    
    IMU_Calib.Cal_Thread_Enable = 0
    IMU_Data.Data_Thread_Enable = 0

    print("*"*50)
    print('Reloading....')
    time.sleep(1)

###
### Function Begin Here
###

print("-"*50)
print("\n\n\t\tGY-H1 Upper V1.0\n")
print("\t\tDesigned by qianwan\n")
print("\t\t2023-04-01\n\n")
print("-"*50)


while True:
    #连接芯片
    while COMConnect() == False :
        time.sleep(0.001)
        print('*'*50)
    print('*'*50)
    #展开功能界面
    print('请选择功能:\n\n1.GY-H1 Setting\n2.Calibrate')
    FunSel = input("\n请输入功能ID: ")
    while True:
        try:
            FunSel=int(FunSel)
            if 0<FunSel<3:
                break
            FunSel = input("\n请重新输入功能ID: ")
        except:
            FunSel = input("\n请重新输入功能ID: ")

    match FunSel:
        case 1:
            print('GY-H1 Setting Running...\n')
            GYSetFunGUI()
            time.sleep(1)
        case 2:
            print('Calibrate Running...\n')
            GYCalFunGUI()
            time.sleep(1)

    time.sleep(0.01)
    
    #特殊情况下重启GY-H1
    try:
        TxBuf = CMDPack(order='00',data='00')
        COM.write(TxBuf)
    except:
        time.sleep(0.01)

    time.sleep(0.5)
    COM.COM_handel.close()
    print('*'*50)
    time.sleep(0.5)
