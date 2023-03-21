import threading
import time

import serial
import serial.tools.list_ports

import tkinter
from tkinter import ttk
from tkinter import messagebox

import crcmod

global COM
global FunNow
global OutMode_CB
global ODR_CB
global OutFormat_CB
global ID_CB
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
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) < 1:
        print('\n无可用串口,按回车键重新扫描')
        input()
        return False
    else:
        print('可使用串口如下:')
        for i in range(len(port_list)): print('ID',i,':',port_list[i])
        i = input("\n请输入要启用的 GY-H1 串口ID: ")
        while True:
            try:
                i=int(i)
                break
            except:
                i = input("\n请重新输入串口ID: ")

        while i > len(port_list) - 1:
            i = input("\n请重新输入串口ID: ")
            while True:
                try:
                    i=int(i)
                    break
                except:
                    i = input("\n请重新输入串口ID: ")

        if portIsUsable(port_list[i].name) == False:
            print("\n串口已被占用,请重新选择\n")
            return False
        
        print('连接设备中...')
        COM = serial.Serial(port_list[i].name, 512000, timeout = 0.2)
        #重启GY-H1
        TxBuf = CMDPack(order='00',data='00')
        COM.write(TxBuf)
        time.sleep(1)
        #重新连接设备
        if portIsUsable(port_list[i].name) == True:
            COM = serial.Serial(port_list[i].name, 512000, timeout = 0.2)
        
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
    global FunNow
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
    TCB1 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='OutMode',font=('Arial', 11),width=16,height=1)
    OutMode_CB = ttk.Combobox(GUI,width='18',justify='left')
    OutMode_CB["value"]=("USB-C","CAN")
    OutMode_CB.current(0)

    #输出速率设置
    TCB2 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='ODR',font=('Arial', 11),width=16,height=1)
    ODR_CB = ttk.Combobox(GUI,width='18',justify='left')
    ODR_CB["value"]=("1000Hz","500Hz","250Hz","125Hz")
    ODR_CB.current(2)

    #输出数据设置
    TCB3 = tkinter.Label(GUI,anchor='n',bg='deepskyblue',fg='white',text='OutFormat',font=('Arial', 11),width=16,height=1)
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

def RawDataRec():
    global COM
    #开启数据接收
    Txbuf = CMDPack(order='00',data='02')
    COM.write(Txbuf)
    while True:
        RecBuf = COM.read(14)
        #对齐数据
        if COM.inWaiting() != 0 :
            COM.flushInput()
            print('-\n\n'*10)
            continue
        print(list(RecBuf))

            

###
### Function Begin Here
###
print("-"*50)
print("\n\n\t\tGY-H1 Upper V0.6\n")
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
            time.sleep(0.01)
            RawDataRec()
            
    
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

    

# SerialReadThread = threading.Thread(target=SerialRead)
# SerialWriteThread = threading.Thread(target=SerialWrite)
# SerialReadThread.start()
# SerialWriteThread.start()

while(True):
    time.sleep(1)

#com4.close()
# #设置串口号COM3，波特率115200，连接超时0.05s
# print(com3)
# print(com3.name) #设备名称
# print(com3.port) #设备名
# print(com3.baudrate) #波特率
# print(com3.bytesize) #字节大小
# print(com3.parity) #校验位N－无校验，E－偶校验，O－奇校验
# print(com3.stopbits) #停止位
# print(com3.timeout) #读超时设置
# print(com3.writeTimeout) #写超时
# print(com3.xonxoff) #软件流控
# print(com3.rtscts) #硬件流控
# print(com3.dsrdtr) #硬件流控
# print(com3.interCharTimeout) #字符间隔超时
# 输出串口信息