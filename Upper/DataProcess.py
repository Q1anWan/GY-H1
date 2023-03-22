import time
import numpy as np
from math import *
from   scipy  import   signal
'''
# 3标准差法清除可疑数据
    data : 传入列表
    metheod : rm去除异常值 place替换为均值
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
            elif metheod == 'place':
                data[data.index(maxV)] = meanV        
        else:
            isOk += 1

        if minV < meanV - 3*stdV :
            if metheod == 'rm' :
                data.remove(minV)
            elif metheod == 'place':
                data[data.index(minV)] = meanV
        else:
            isOk += 1  
        
        if isOk == 2:
            return data
        

