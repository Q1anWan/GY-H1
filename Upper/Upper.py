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

import crcmod
import DataProcess as DP

global COM

global OutMode_CB
global ODR_CB
global OutFormat_CB
global ID_CB

global Data_Thread_Enable
global Cal_Thread_Enable

global UI_Gyro_Line
global UI_Accel_Line

global aG
global aA
#IMU原始数据 INT类 FIFO队列
#使用500Hz
global UI_t
global GyXq
global GyYq
global GyZq
global AcXq
global AcYq
global AcZq

#修正值 INT
global GyXc
global GyYc
global GyZc
global AcXc
global AcYc
global AcZc

global GUI


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
    
def IsGYH1Connected():
    try:
        #发送空白信息
        COM.flushInput()
        TxBuf = CMDPack(order='FF',data='FF')
        COM.write(TxBuf)
        time.sleep(0.001)
        count=COM.inWaiting()
        #读取有无空白信息
        Rec = COM.read(count)
        if Rec == TxBuf:
            return True
        else:
            return False
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
    global COM
    global port_list
    global sel_i 
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) < 1:
        print('\n无可用串口,按回车键重新扫描')
        input()
        return False
    else:
        print('可使用串口如下:')
        for sel_i in range(len(port_list)): print('ID',sel_i,':',port_list[sel_i])
        sel_i = input("\n请输入要启用的 GY-H1 串口ID: ")
        while True:
            try:
                sel_i=int(sel_i)
                break
            except:
                sel_i = input("\n请重新输入串口ID: ")

        while sel_i > len(port_list) - 1:
            sel_i = input("\n请重新输入串口ID: ")
            while True:
                try:
                    sel_i=int(sel_i)
                    break
                except:
                    sel_i = input("\n请重新输入串口ID: ")

        if portIsUsable(port_list[sel_i].name) == False:
            print("\n串口已被占用,请重新选择\n")
            return False
        
        print('连接设备中...')
        COM = serial.Serial(port_list[sel_i].name, 512000, timeout = 0.1)
        #重启GY-H1
        TxBuf = CMDPack(order='00',data='00')
        COM.write(TxBuf)
        time.sleep(1)
        #重新连接设备
        if portIsUsable(port_list[sel_i].name) == True:
            COM = serial.Serial(port_list[sel_i].name, 512000, timeout = 0.1)
        
        #关闭串口对外信息发送
        TxBuf = CMDPack(order='00',data='03')
        COM.write(TxBuf)
        time.sleep(0.01)

        # Rec = binascii.b2a_hex(Rec)
        if IsGYH1Connected():
            print('\nGY-H1 已连接\n')
            return True
        else:
            print('\nGY-H1 连接错误\n')
            COM.close()
            return False
        
def SettingApply():
    global COM
    global port_list
    global sel_i 

    global GUI
    global OutMode_CB
    global ODR_CB
    global OutFormat_CB
    global ID_CB
    time.sleep(0.01)
    
    if IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1 Disconneted!!')
        print("GY-H1 Setting Failed!!\n")
        GUI.destroy()
        return
    else:
        TxBuf = CMDPack(order='01',data='00')
        COM.write(TxBuf)
        time.sleep(0.01)

        match OutMode_CB.get():
            case "USB-C":
                TxBuf = CMDPack(order='02',data='00')
                COM.write(TxBuf)
            case "CAN":
                TxBuf = CMDPack(order='02',data='01')
                COM.write(TxBuf)
        time.sleep(0.01)

        match ODR_CB.get():
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
        time.sleep(0.01)

        match OutFormat_CB.get():
            case "Quaternion":
                TxBuf = CMDPack(order='02',data='02')
                COM.write(TxBuf)
            case "Raw data":
                TxBuf = CMDPack(order='02',data='03')
                COM.write(TxBuf)
            case "Processed data":
                TxBuf = CMDPack(order='02',data='04')
                COM.write(TxBuf)
        time.sleep(0.01)

        match ID_CB.get():
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

        time.sleep(0.01)

        if IsGYH1Connected() == False:
            tkinter.messagebox.showinfo('Error','GY-H1 Config Failed!!')
            print("GY-H1 Setting Failed!!\n")
        else:
            tkinter.messagebox.showinfo('Info','GY-H1 Configuration Succeed!!')
            print("GY-H1 Setting Succeed!!\n")
        
        time.sleep(0.01)
        TxBuf = CMDPack(order='01',data='01')
        COM.write(TxBuf)
        time.sleep(0.01)
        GUI.destroy()

#GY-H1 Setting
def GYSetFunGUI():

    global OutMode_CB
    global ODR_CB
    global OutFormat_CB
    global ID_CB

    global GUI

    GUI = tkinter.Tk()
    GUI.title('GY-H1 Setting')
    GUI.geometry('200x350')

    Title1 = tkinter.Label(GUI,anchor='n',text='GY-H1 SETTING',bg='blue',fg="white",font=('Arial', 16),width=40,height=1)
    Title1.pack(side='top',fill='x')
   
    #输出模式设置
    TCB1 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='OTSEL',font=('Arial', 11),width=16,height=1)
    OutMode_CB = ttk.Combobox(GUI,width='18',justify='left')
    OutMode_CB["value"]=("USB-C","CAN")
    OutMode_CB.current(0)

    #输出速率设置
    TCB2 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='ODR',font=('Arial', 11),width=16,height=1)
    ODR_CB = ttk.Combobox(GUI,width='18',justify='left')
    ODR_CB["value"]=("1000Hz","500Hz","250Hz","125Hz")
    ODR_CB.current(2)

    #输出数据设置
    TCB3 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='MOD',font=('Arial', 11),width=16,height=1)
    OutFormat_CB = ttk.Combobox(GUI,width='18',justify='left')
    OutFormat_CB["value"]=("Quaternion","Raw data","Processed data")
    OutFormat_CB.current(0)

    #ID设置
    TCB4 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='ID',font=('Arial', 11),width=16,height=1)
    ID_CB = ttk.Combobox(GUI,width='18',justify='left')
    ID_CB["value"]=("0","1","2","3","4","5","6","7","8","9",)
    ID_CB.current(0)

    ApplyButton = tkinter.Button(GUI,bg='yellow',relief='groove',fg='red',text='APPLY',font=('Arial', 16),command=SettingApply)
    ApplyButton.pack(side='bottom',fill='x')

    TCB1.pack(pady = (20,0))
    OutMode_CB.pack(pady = 0)
    TCB2.pack(pady = (20,0))
    ODR_CB.pack(pady = 0)
    TCB3.pack(pady = (20,0))
    OutFormat_CB.pack(pady = 0)
    TCB4.pack(pady = (20,0))
    ID_CB.pack(pady = 0)

    GUI.mainloop()

def ConnecWarning():
    global ConnectButton

    ConnectButton['bg'] = 'yellow'
    ConnectButton['fg'] = 'red'
    ConnectButton['command'] = CalConRec
    ConnectButton['text'] = 'Start Read Data'
    
    tkinter.messagebox.showinfo('Info','此操作将会重置ORD与MOD\n请在结束校准后重新设置!!')


def CalConRec():
    global COM
    global port_list
    global sel_i
    global ConnectButton
    global ACalButton
    global GCalButton
    global GUI
    global MOD_Cal
    global Data_Thread_Enable

    ConnectButton['state'] = 'disable'
    MOD_Cal['state'] = 'disable'
    ConnectButton['bg'] = 'white'

    if MOD_Cal.get() == "Check Raw data":
        ConnectButton['text'] = 'Reading Raw Data...'
        ACalButton['bg'] = 'Gold'
        ACalButton['fg'] = 'OrangeRed'
        ACalButton['state'] = 'normal'
        GCalButton['bg'] = 'Gold'
        GCalButton['fg'] = 'OrangeRed'
        GCalButton['state'] = 'normal'
    else:
        ConnectButton['text'] = 'Reading Processed Data...'
        ACalButton['bg'] = 'white'
        GCalButton['bg'] = 'white'

    #检验设备连接状态
    if IsGYH1Connected() == False:
        tkinter.messagebox.showinfo('Error','GY-H1连接异常!!')
        GUI.destroy()
        return False
    
    #进入配置模式
    time.sleep(0.1)
    TxBuf = CMDPack(order='01',data='00')
    COM.write(TxBuf)
    time.sleep(0.1)
    #设置输出速率
    TxBuf = CMDPack(order='03',data='01')
    COM.write(TxBuf)
    time.sleep(0.1)
    #设置输出模式
    if MOD_Cal.get() == "Check Raw data":
        TxBuf = CMDPack(order='02',data='03')
    else:
        TxBuf = CMDPack(order='02',data='04')
    COM.write(TxBuf)
    time.sleep(0.1)
    #保存设置
    TxBuf = CMDPack(order='01',data='01')
    COM.write(TxBuf)
    time.sleep(0.5)
    COM.close()
    time.sleep(0.5)
    
    #重新连接设备
    if portIsUsable(port_list[sel_i].name) == False:
        tkinter.messagebox.showinfo('Error','GY-H1连接异常!!')
        GUI.destroy()
        print('GY-H1连接异常!!')
        return False
    COM = serial.Serial(port_list[sel_i].name, 512000, timeout = 0.1)
    
    #启动接收进程
    Data_Thread_Enable = 1
    thread_RawDataRec = threading.Thread(target=RawDataRec)
    thread_RawDataRec.start()   

def RawDataRec():
    global COM
    global Data_Thread_Enable
    
    global UI_t
    global GyXq
    global GyYq
    global GyZq
    global AcXq
    global AcYq
    global AcZq

    #确保开启数据接收
    Txbuf = CMDPack(order='00',data='02')
    COM.write(Txbuf)

    tbuf = 0
    lostpack = 0
    while True:
        RecBuf = COM.read(14)
        #对齐数据
        if COM.inWaiting() != 0 :
            COM.flushInput()
       
        #CRC校验
        if RecBuf[13:14] == CRC8(RecBuf[0:13]):
            GyroX = int.from_bytes(RecBuf[1:3],byteorder='big',signed=True)
            GyroY = int.from_bytes(RecBuf[3:5],byteorder='big',signed=True)
            GyroZ = int.from_bytes(RecBuf[5:7],byteorder='big',signed=True)
            AccelX = int.from_bytes(RecBuf[7:9],byteorder='big',signed=True)
            AccelY = int.from_bytes(RecBuf[9:11],byteorder='big',signed=True)
            AccelZ = int.from_bytes(RecBuf[11:13],byteorder='big',signed=True)

            #进入队列
            tbuf += 0.003#加了矫正
            UI_t.append(tbuf)
            GyXq.append(GyroX)
            GyYq.append(GyroY)
            GyZq.append(GyroZ)
            AcXq.append(AccelX)
            AcYq.append(AccelY)
            AcZq.append(AccelZ)
        # else:
        #     print('-没CRC\n\n')

        #进程退出
        if Data_Thread_Enable == 0:
            break

def Gyro_Update(i):
    global GyXq
    global GyYq
    global GyZq
    global UI_t
    global UI_Gyro_Line

    if len(GyXq)!=0:
        aG.set_xlim(UI_t[0],UI_t[-1])
        max1 = np.amax(GyXq)
        max2 = np.amax(GyYq)
        max3 = np.amax(GyZq)
        min1 = np.amin(GyXq)
        min2 = np.amin(GyYq)
        min3 = np.amin(GyZq)
       
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

        aG.set_ylim(min-0.2*np.abs(min),max+0.2*np.abs(max))
        UI_Gyro_Line[0].set_data(UI_t,GyXq)  # update the data
        UI_Gyro_Line[1].set_data(UI_t,GyYq)  # update the data
        UI_Gyro_Line[2].set_data(UI_t,GyZq)  # update the data
    return UI_Gyro_Line,

def Accel_Update(i):
    global AcXq
    global AcYq
    global AcZq
    global UI_t
    global UI_Accel_Line

    if len(AcXq)!=0:
        aA.set_xlim(UI_t[0],UI_t[len(UI_t)-1])
        max1 = np.amax(AcXq)
        max2 = np.amax(AcYq)
        max3 = np.amax(AcZq)
        min1 = np.amin(AcXq)
        min2 = np.amin(AcYq)
        min3 = np.amin(AcZq)
    
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

        aA.set_ylim(min-0.2*np.abs(min),max+0.2*np.abs(max))
        UI_Accel_Line[0].set_data(UI_t,AcXq)  # update the data
        UI_Accel_Line[1].set_data(UI_t,AcYq)  # update the data
        UI_Accel_Line[2].set_data(UI_t,AcZq)  # update the data
    return UI_Accel_Line,

def AccelCaliThread():
    global Cal_Thread_Enable
    while True:
        time.sleep(0.1)
        if Cal_Thread_Enable == 0:
            return False
    return

def AccelKeyHandel():
    global Cal_Thread_Enable
    Cal_Thread_Enable = 1
    ACalButton['state'] = 'disable'
    GCalButton['state'] = 'disable'
    ACalButton['bg'] = 'DarkGreen'
    ACalButton['fg'] = 'Gold'
    GCalButton['bg'] = 'white'
    threading.Thread(target=AccelCaliThread).start()

    return

def GyroCaliThread():
    global Cal_Thread_Enable
    global Data_Thread_Enable
    global GyXc
    global GyYc
    global GyZc

    print('Gyro Calibrating...')
    ACalButton['state'] = 'disable'
    GCalButton['state'] = 'disable'
    ACalButton['text'] = '0%'
    GCalButton['text'] = 'Gyro Calibrating...'
    rate = 0
    bufX = []
    bufY = []
    bufZ = []
    
    #100s
    while rate < 100:
        rate += 2
        
        for ii in range(100):
            bufX.append(GyXq[-1]) 
            bufY.append(GyYq[-1]) 
            bufZ.append(GyZq[-1]) 
            time.sleep(0.01) 

        ACalButton['text'] = str(rate)+' %'
    
        if Cal_Thread_Enable == 0:
            return
    #去除异常值
    bufX = DP.Dataculling(bufX)
    bufY = DP.Dataculling(bufY)
    bufZ = DP.Dataculling(bufZ)
    
    #平均一下
    GyXc = -int(np.average(bufX))
    GyYc = -int(np.average(bufY))
    GyZc = -int(np.average(bufZ))
    
    print('Gyro Corret Value:')
    print('X =',GyXc,' Y =',GyYc,' Z =',GyZc)

    ACalButton['state'] = 'disable'
    GCalButton['state'] = 'normal'
    ACalButton['text'] = 'Gyro Static Correct Value [X,Y,X]' + str(GyXc) +' '+ str(GyYc) +' '+ str(GyZc)
    GCalButton['text'] = 'Apply'
    tkinter.messagebox.showinfo('Info','GY-H1 Gyro静态校准值计算完成!!\n按Apply写入校准,关闭窗口丢弃值')
    Cal_Thread_Enable = 2
    return

def GyroWrite():

    time.sleep(0.1)
    #关闭对外数据输出
    TxBuf = CMDPack(order='00',data='03')
    COM.write(TxBuf)
    time.sleep(0.1)

    if IsGYH1Connected() == False:
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
        buf = bytes.hex(GyXc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='10',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='11',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(GyYc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='12',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='13',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)
        #写入三轴数据
        buf = bytes.hex(GyZc.to_bytes(2,byteorder='big',signed=True))
        TxBuf = CMDPack(order='14',data=buf[0:2])
        COM.write(TxBuf)
        time.sleep(0.1)
        TxBuf = CMDPack(order='15',data=buf[2:4])
        COM.write(TxBuf)
        time.sleep(0.1)

        #退出写入状态
        if IsGYH1Connected() == False:
            tkinter.messagebox.showinfo('Error','GY-H1 Calibratiion Failed!!')
            print("GY-H1 Gyro Correct Value Write Failed!!\n")
        else:
            tkinter.messagebox.showinfo('Info','GY-H1 Calibratiion Succeed!!')
            print("GY-H1 Calibrating Succeed!!\n")
        
        time.sleep(0.01)
        TxBuf = CMDPack(order='01',data='01')
        COM.write(TxBuf)
        time.sleep(0.01)
        GUI.destroy()
        time.sleep(1)


def GyroKeyHandel():
    global Cal_Thread_Enable
    global Data_Thread_Enable
    global GyXc
    global GyYc
    global GyZc
    #选定要执行的校准
    if Cal_Thread_Enable == 0:
        Cal_Thread_Enable = 1
        ACalButton['state'] = 'disable'
        GCalButton['state'] = 'normal'
        ACalButton['bg'] = 'white'
        GCalButton['bg'] = 'DarkGreen'
        GCalButton['fg'] = 'Gold'
        GCalButton['text'] = 'Press to start calibrate Gyro'
        tkinter.messagebox.showinfo('Info','请保证GY-H1静止,校准过程持续30s')
    #开始执行校准
    elif Cal_Thread_Enable == 1:
        threading.Thread(target=GyroCaliThread).start()
    #执行写入程序
    elif Cal_Thread_Enable == 2:
        Data_Thread_Enable = 0
        threading.Thread(target=GyroWrite).start()
        GCalButton['state'] = 'disable'

#GY-H1 Calibrate
def GYCalFunGUI():
    global GUI
    global Data_Thread_Enable
    global Cal_Thread_Enable
    global ConnectButton
    global ACalButton
    global GCalButton
    global MOD_Cal
    global UI_t
    global GyXq
    global GyYq
    global GyZq
    global AcXq
    global AcYq
    global AcZq

    global UI_Gyro_Line
    global UI_Accel_Line

    global aG
    global aA

    UI_t = deque(maxlen=1500)
    GyXq = deque(maxlen=1500)
    GyYq = deque(maxlen=1500)
    GyZq = deque(maxlen=1500)
    AcXq = deque(maxlen=1500)
    AcYq = deque(maxlen=1500)
    AcZq = deque(maxlen=1500)


    GUI = tkinter.Tk()
    GUI.title('GY-H1 Calibration')
    GUI.geometry('650x750')

    figGyro = plt.Figure()
    figGyro.set_size_inches(8,3)
    canvasGyro = FigureCanvasTkAgg(figGyro, master=GUI)
    aG = figGyro.add_subplot(111)
    aG.set_title('Gyro')

    UI_Gyro_Line = aG.plot([],[],'olive',[],[],'teal',[],[],'orchid')
    figGyro.legend(handles=[UI_Gyro_Line[0],UI_Gyro_Line[1],UI_Gyro_Line[2]],labels=['X','Y','Z'])

    aniG = animation.FuncAnimation(figGyro, Gyro_Update, np.arange(0, 1500), interval=3, blit=False)

    figAccel = plt.Figure()
    figAccel.set_size_inches(8,3)
    canvasAccel = FigureCanvasTkAgg(figAccel, master=GUI)
    aA = figAccel.add_subplot(111)
    aA.set_title('Accel')
    UI_Accel_Line = aA.plot([],[],'olive',[],[],'teal',[],[],'orchid')
    figAccel.legend(handles=[UI_Accel_Line[0],UI_Accel_Line[1],UI_Accel_Line[2]],labels=['X','Y','Z'])
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

    Cal_Thread_Enable = 0

    GUI.mainloop()
    
    Cal_Thread_Enable = 0
    Data_Thread_Enable = 0

    time.sleep(1)

###
### Function Begin Here
###

print("-"*50)
print("\n\n\t\tGY-H1 Upper V0.7\n")
print("\t\tDesigned by qianwan\n")
print("\t\t2023-03-22\n\n")
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
        case 2:
            print('Calibrate Running...\n')
            GYCalFunGUI()

    time.sleep(0.01)
    
    #特殊情况下重启GY-H1
    try:
        TxBuf = CMDPack(order='00',data='00')
        COM.write(TxBuf)
    except:
        time.sleep(0.01)

    time.sleep(0.5)
    COM.close()
    print('*'*50)
    time.sleep(0.5)

while(True):
    time.sleep(1)
