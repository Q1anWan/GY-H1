import threading
import time

import serial
import serial.tools.list_ports

import tkinter
from tkinter import ttk
from tkinter import messagebox

global COM
global FunNow
global OutMode_CB
global ODR_CB
global OutFormat_CB
global ID_CB
global GUI

def portIsUsable(portName):
    try:
       ser = serial.Serial(port=portName)
       return True
    except:
       return False
    
def IsGYH1Connected():
    try:
        #发送空白信息
        Data = '30 FF FF 6A'
        TxBuf = bytes.fromhex(Data)
        COM.flushInput()
        COM.write(TxBuf)
        time.sleep(0.01)
        count=COM.inWaiting()
        Rec = COM.read(count)
        if Rec == TxBuf:
            return True
        else:
            return False
    except:
       return False
    
def COMConnect():
    global COM
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) < 1:
        print('无可用串口,按回车键退出')
        input()
        exit()
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

        while i > len(port_list):
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
        COM = serial.Serial(port_list[i].name, 512000, timeout = 0.1)
        #重启GY-H1
        Data = '30 00 00 DE'
        TxBuf = bytes.fromhex(Data)
        COM.write(TxBuf)
        time.sleep(1)

        #关闭对外数据输出
        if portIsUsable(port_list[i].name) == True:
            COM = serial.Serial(port_list[i].name, 512000, timeout = 0.05)
        
        Data = '30 00 03 3C'
        TxBuf = bytes.fromhex(Data)
        COM.write(TxBuf)
        time.sleep(0.01)

        #关闭串口对外信息发送
        Data = '30 00 03 3C'
        TxBuf = bytes.fromhex(Data)
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
        Data = '30 01 00 1A'
        TxBuf = bytes.fromhex(Data)
        COM.write(TxBuf)
        time.sleep(0.01)

        match OutMode_CB.get():
            case "USB-C":
                Data = '30 02 00 4F'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "CAN":
                Data = '30 02 01 11'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
        time.sleep(0.01)

        match ODR_CB.get():
            case "1000Hz":
                Data = '30 03 00 8B'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "500Hz":
                Data = '30 03 01 D5'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "250Hz":
                Data = '30 03 02 F3'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "125Hz":
                Data = '30 03 03 69'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
        time.sleep(0.01)

        match OutFormat_CB.get():
            case "Quaternion":
                Data = '30 02 02 F3'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "Raw data":
                Data = '30 02 03 AD'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "Processed data":
                Data = '30 02 04 2E'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
        time.sleep(0.01)

        match ID_CB.get():
            case "1":
                Data = '30 04 00 E5'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "2":
                Data = '30 04 01 BB'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "3":
                Data = '30 04 02 59'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "4":
                Data = '30 04 03 07'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "5":
                Data = '30 04 04 84'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "6":
                Data = '30 04 05 DA'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "7":
                Data = '30 04 06 38'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "8":
                Data = '30 04 07 66'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
            case "9":
                Data = '30 04 08 27'
                TxBuf = bytes.fromhex(Data)
                COM.write(TxBuf)
        time.sleep(0.01)

        if IsGYH1Connected() == False:
            tkinter.messagebox.showinfo('Error','GY-H1 Disconneted!!')
            print("GY-H1 Setting Failed!!\n")
            GUI.destroy()
            return

        Data = '30 01 01 44'
        TxBuf = bytes.fromhex(Data)
        COM.write(TxBuf)
        time.sleep(0.01)
        tkinter.messagebox.showinfo('Info','GY-H1 Setting Succeed!!')
        print("GY-H1 Setting Succeed!!\n")
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
    
###
### Function Begin Here
###
print("-"*50)
print("\n\n\t\tGY-H1 Upper V0.5\n")
print("\t\tDesigned by qianwan\n")
print("\t\t2023-03-20\n\n")
print("-"*50)
while True:
    #连接芯片
    while COMConnect() == False :
        time.sleep(0.5)
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
    
    COM.close()
    print('*'*50)


    


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