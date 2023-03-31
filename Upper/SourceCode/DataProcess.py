import time
import numpy as np
from math import *
from   scipy  import   signal
'''
# 3标准差法清除可疑数据
    data : 传入列表
    metheod : rm去除异常值 place替换为线性插值
'''
def Dataculling(data,metheod = 'rm'):
    while True:
        #计算样本标准差
        stdV =  np.std(data,ddof=1)
        meanV = np.mean(data)
        maxV = max(data)
        minV = min(data)
        isOk = 0

        #去除可疑数据
        if maxV > meanV + 3*stdV :
            if metheod == 'rm' :
                data.remove(maxV)
            elif metheod == 'replace':
                dataIndex = data.index(maxV)
                if dataIndex == len(data):
                    data[dataIndex] = data[dataIndex-1]
                elif dataIndex == 0:
                    data[dataIndex] = data[dataIndex+1]
                else:
                    data[dataIndex] = (data[dataIndex-1] + data[dataIndex+1])/2
       
        else:
            isOk += 1

        if minV < meanV - 3*stdV :
            if metheod == 'rm' :
                data.remove(minV)
            elif metheod == 'replace':
                dataIndex = data.index(minV)
                if dataIndex == len(data):
                    data[dataIndex] = data[dataIndex-1]
                elif dataIndex == 0:
                    data[dataIndex] = data[dataIndex+1]
                else:
                    data[dataIndex] = (data[dataIndex-1] + data[dataIndex+1])/2
        else:
            isOk += 1  
        
        if isOk == 2:
            return data

'''  
#  3标准差法清除可疑数据 带时间序列版
    data : 传入列表
    metheod : rm去除异常值 place替换为均值
'''
def DatacullingTicker(data,tick):
    while True:
        #计算样本标准差
        stdV =  np.std(data,ddof=1)
        meanV = np.mean(data)
        maxV = max(data)
        minV = min(data)
        isOk = 0

        #去除可疑数据
        if maxV > meanV + 3*stdV :
            tick.remove(data.index(maxV))
            data.remove(maxV)
        else:
            isOk += 1

        if minV < meanV - 3*stdV :
            tick.remove(data.index(minV))
            data.remove(minV)

        else:
            isOk += 1  
        
        if isOk == 2:
            return data
        

