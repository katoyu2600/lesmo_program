# -*- coding: utf-8 -*-
"""
Created on Sat Jun 19 10:54:02 2021

@author: 高橋　辰輔
"""

#from matplotlib import pyplot
import smbus2 as smbus
import sys
import time
import math
import numpy

bus_number = 1
bus = smbus.SMBus(bus_number)
Accel_Gyro_sensor_device_addr = 0x68#0x61かもしれない
#加速度センサの値を取得するためにアドレスを入手する・
Magnet_sensor_devidce_addr = 0x0C
#   0x00 0x02 にアクセス

#smbusのblock_data、第三引数にバイト数を指定する可能性あり
def Get_Accel_status():
    addr = 0x68
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    global pro_count
    pro_count = "GAS"
    scale = 2
    Address_map = [[0x3B , 0x3C] , [0x3D , 0x3E] , [0x3F , 0x40]]
    status = [0,0,0]
    k = 0
    for i in Address_map:
        for j in i:
            n =  bus.read_byte_data(addr,j)
            status[k] = (status[k] << 8) | n
        if status[k] & 0x8000:
            status[k] = -1 * ((status[k] ^ 0xFFFF) + 1)
        status[k] /= float(0x8000/scale)
        k += 1
    return status

def Get_Gyro_status():
    addr = 0x68
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    global pro_count
    scale = 250
    pro_count = "GGS"
    Address_map = [[0x43 , 0x44] , [0x45 , 0x46] , [0x47 , 0x48]]
    status = [0,0,0]
    k = 0
    for i in Address_map:
        for j in i:
            n =  bus.read_byte_data(addr,j)
            status[k] = (status[k] << 8) | n
        if status[k] & 0x8000:
            status[k] = -1 * ((status[k] ^ 0xFFFF) + 1)
        status[k] /= float(0x8000/scale)
        k += 1
    return status

def Get_Magnet_status():
    addr = 0x0C
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    global pro_count
    pro_count = "GMS"
    scale = 4912#MAX:4912マイクロテスラ
    Address_map= [[0x04 , 0x03] , [0x06 , 0x05] , [0x08 , 0x07]]
    status = [0,0,0]
    k = 0
    for i in Address_map:
        for j in i:
            n  =  bus.read_byte_data(addr,j)
            status[k] = (int(status[k]) << 8) | n
            status[k] = int(status[k])
            #print(status[k])
        if int(status[k]) & int(0x8000):
            status[k] = -1 * ((status[k] ^ 0xFFFF) + 1)
        status[k] /= float(8000/scale)
        #print(status[k])
        k += 1
    ST2 = 0x09 #ST2レジスタ、データ読み出し完了時に読み出し
    #(読み出し中はデータ破損防止のためST2が読まれるまでデータ更新が停止)
    bus.read_byte_data(addr,ST2)
    return status
    #OVFでHOFL = 1 今後チェック機能を要実装

def Set_Magnet_Cofigdata():
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    addr = 0x0C
    global pro_count
    pro_count = "SMC"
    bus.write_byte_data(0x68,0x37,0x02)
    Address_map= [0x0A]
    config_status = [0b00010110]
    #mode1 0010 mode2 0100 14bit [4] = 0 16bit [4] = 1
    for i in Address_map:
        bus.write_byte_data(addr,i,config_status[Address_map.index(i)])
    return 0

def Set_AccelGyro_Configdata():
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    addr = 0x68
    global pro_count
    pro_count = "SAGC"
#    print(bus.write_byte_data(0x68,107,0b00000000))
    Address_map= [0x1A , 0x1B , 0x1C , 0x1D , 0x68 , 0x6B , 0x37]
    config_status = [0b00000001 , 0b00000000 ,0b00000000 , 0b00000000 , 0b00000111 , 0b00000000 , 0b00000010]
    #mode1 0010 mode2 0100 14bit [4] = 0 16bit [4] = 1
    #0b00000000 , 0b00001011 ,0b00001000 , 0b00000000 , 0b00000111
    for i in Address_map:
        j = 0
        n = config_status[j]
        bus.write_byte_data(addr,i,n)
        j += 1
        time.sleep(0.05)
    return 0

def start_up():
    Set_AccelGyro_Configdata()
    Set_Magnet_Cofigdata()

def cal_lib():
    max_mg = [0 , 0 , 0]
    min_mg = [0 , 0 , 0]
    k = 100
    while(k):
        n = Get_Magnet_status()
        k = k - 1
        time.sleep(0.05)
        for k in range(3):
            if max_mg[k] < n[k]:
                max_mg[k] = n[k]
            if min_mg[k] > n[k]:
                min_mg[k] = n[k]
            offset = [0 , 0 , 0]
        for l in range(3):
            offset[l] = max_mg[l] / 2 + min_mg[l] / 2
        print(offset)

def tan_calc():
    Magnet_status = Get_Magnet_status()
    n = Magnet_status
    offset = [-7.875, -44.25, 28.2]
    for l in range(3):
        Magnet_status[l] = Magnet_status[l] - offset[l]
        #            print(math.sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]))
        for i in range(3):
            for j in range(3):
                if i != j and i < j:
                    RT_rad = numpy.arctan(Magnet_status[i]/(Magnet_status[j] + 0.01))
                    print(i,j,RT_rad)
                    RT_rad = numpy.arctan(Magnet_status[j]/(Magnet_status[i] + 0.01))
                    print(j,i,RT_rad)

def iran():
    hn = math.sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2])
    for bn in range(3):
        Magnet_status[bn] /= (hn+0.0001)
        Magnet_status[bn] *= 100
        Magnet_status[bn] = int(Magnet_status[bn])

def s_atamawarui_hannbetuki(data):
    # N , E , S , W
    hougaku = [0 , 1.57 , 3.14 , 4.71]
    x_sta = [20 , 40 , 20,  -10]
    y_sta = [20 , -10 , -30 , -10]
    z_sta = [15 , 20 , 20 , 15]
    score = [0 , 0 , 0 , 0]
    for k in range(4):
        xc = (data[0] - x_sta[k]) **2
        yc = (data[1] - y_sta[k]) **2
        zc = (data[2] - z_sta[k]) **2
        score[k] = xc + yc + zc
#    print(score)
    out_d = hougaku[score.index(min(score))]
    return out_d


if __name__ == "__main__":
    try:
        Set_AccelGyro_Configdata()
        Set_Magnet_Cofigdata()
        print("SetupComplete")
        h=1000
        while(h):
            h-= 1
            print(Get_Accel_status())
            print(Get_Gyro_status())
            Magnet_status = Get_Magnet_status()
            n = Magnet_status
            offset = [0, 0, 0]
            for l in range(3):
                Magnet_status[l] = Magnet_status[l] - offset[l]
            print(Magnet_status)
            #print(s_atamawarui_hannbetuki(Magnet_status))
            time.sleep(0.3)
            #cal_lib()
    except KeyboardInterrupt:
        sys.exit("KeyboardInterrupt")
