# -*- coding: utf-8 -*-
#メインプログラム
"""
Created on Wed May 24 01:36:00 2022

@author: 加藤　裕也
"""

#モジュールインポート
import time
import datetime
import math
import numpy as np 
import pigpio
import cv2 
import smbus2 as smbus
import serial
import spidev 
import picamera
import sys
import qwiic_titan_gps

#設定値決定

cycle_time = 1.2                  #機体が1回転する時間
preset_altiude = 10                #着地高度設定
preset_altiude2 = 6               #上昇高度設定
preset_goal_distanc = 10          #誘導方法切り替え距離
go_time = 10                       #GPS誘導時直進時間
preset_pres = 882.61              #ゴール地点気圧 
career_time =1.5                  #!電熱線燃焼時間
magnet_calibrate_list = [-40,250] #!地磁気補正データ

LEFT_TOP_M1 = 4 # lt      #モーター・電熱線・通信電源ピンを割り当て
LEFT_TOP_M2 = 17
RIGHT_TOP_M1 = 27 # rt
RIGHT_TOP_M2 = 22
CAREER_CUT = 6
IM_PIN = 24

V_REF = 3.29476           # 光センサinput Voltage
CHN = 0                   # 光センサ接続チャンネル

#BME280
bus_number  = 1
start = time.time()
bus = smbus.SMBus(bus_number)
i2c_address = 0x76

digT = []
digP = []
digH = []

t_fine = 0.0

def writeReg(reg_address, data):
    bus.write_byte_data(i2c_address,reg_address,data)

def get_calib_param():
    calib = []
    for i in range (0x88,0x88+24):
        calib.append(bus.read_byte_data(i2c_address,i))
    calib.append(bus.read_byte_data(i2c_address,0xA1))
    for i in range (0xE1,0xE1+7):
        calib.append(bus.read_byte_data(i2c_address,i))

    digT.append((calib[1] << 8) | calib[0])
    digT.append((calib[3] << 8) | calib[2])
    digT.append((calib[5] << 8) | calib[4])
    digP.append((calib[7] << 8) | calib[6])
    digP.append((calib[9] << 8) | calib[8])
    digP.append((calib[11]<< 8) | calib[10])
    digP.append((calib[13]<< 8) | calib[12])
    digP.append((calib[15]<< 8) | calib[14])
    digP.append((calib[17]<< 8) | calib[16])
    digP.append((calib[19]<< 8) | calib[18])
    digP.append((calib[21]<< 8) | calib[20])
    digP.append((calib[23]<< 8) | calib[22])
    digH.append( calib[24] )
    digH.append((calib[26]<< 8) | calib[25])
    digH.append( calib[27] )
    digH.append((calib[28]<< 4) | (0x0F & calib[29]))
    digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
    digH.append( calib[31] )

    for i in range(1,2):
        if digT[i] & 0x8000:
            digT[i] = (-digT[i] ^ 0xFFFF) + 1

    for i in range(1,8):
        if digP[i] & 0x8000:
            digP[i] = (-digP[i] ^ 0xFFFF) + 1

    for i in range(0,6):
        if digH[i] & 0x8000:
            digH[i] = (-digH[i] ^ 0xFFFF) + 1 

def read_Data():
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8)  |  data[7]

        #compensate_T(temp_raw)
        #compensate_P(pres_raw)
        #compensate_H(hum_raw)
    t = compensate_T(temp_raw)
    p = compensate_P(pres_raw)
    h = compensate_H(hum_raw)
    
    return   p + "," + t +"," + h 

def read_pres() :
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    p = compensate_P(pres_raw)
    #p = 1013.25
    return p

def read_temp() :
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)   
    t = compensate_T(temp_raw)
    #t = 25
    return t

def get_altitude(pf) :
    p0    = 1013.25
    p0fix = pf
    p1    = float(read_pres()) 
    t1    = float(read_temp())
    T     = t1 + 273.15
    P     = p0 / p1
    Pfix  = p0 / p0fix 
    v1    = 1 / 5.257
    v2    = pow(P,v1)
    v3    = pow(Pfix,v1)
    v4    = v2 - v3
    v5    = v4 * T
    h     = v5 / 0.0065
    #h = 0 
    return h

def compensate_P(adc_P):
    global  t_fine
    pressure = 0.0

    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
    v1 = ((32768 + v1) * digP[0]) / 32768

    if v1 == 0:
        return 0
    pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
    if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  

    #print "pressure : %7.2f hPa" % (pressure/100)
    return "%7.2f" % (pressure/100)

def compensate_T(adc_T):
    global t_fine
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
    t_fine = v1 + v2
    temperature = t_fine / 5120.0
    #print "temp : %-6.2f ℃" % (temperature) 
    return "%.2f" % (temperature) 

def compensate_H(adc_H):
    global t_fine
    var_h = t_fine - 76800.0
    if var_h != 0:
        var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
    else:
        return 0
    var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
    if var_h > 100.0:
        var_h = 100.0
    elif var_h < 0.0:
        var_h = 0.0
    #print "hum : %6.2f ％" % (var_h)
    return "%.2f" % (var_h)

def BME_setup():
#        open('/home/pi/row.csv', 'w')
    osrs_t = 1            #Temperature oversampling x 1
    osrs_p = 1            #Pressure oversampling x 1
    osrs_h = 1            #Humidity oversampling x 1
    mode   = 3            #Normal mode
    t_sb   = 5            #Tstandby 1000ms
    filter = 0            #Filter off
    spi3w_en = 0            #3-wire SPI Disable

    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
    config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
    ctrl_hum_reg  = osrs_h

    writeReg(0xF2,ctrl_hum_reg)
    writeReg(0xF4,ctrl_meas_reg)
    writeReg(0xF5,config_reg)

#緑GPS
UART_PATH = '/dev/serial0'
GPS_Realtime_txt_PATH = "gps_realtime.txt"
#Testdata la = 4222.7992 lo = 14101.9047

def read_gps():
    global pro_count
    global GPS_Realtime_txt_PATH
    Main_State = 0
    GPS_raw_Array = i_GPS_UART_getval()
#    print(GPS_raw_Array)
    GPS_Array = r_NMEA_Decorder(GPS_raw_Array)
    if GPS_Array != 0:
        Main_State = t_TXT_Write_module(GPS_Array,GPS_Realtime_txt_PATH)
    return Main_State

def i_GPS_UART_getval():
    global pro_count
    global UART_PATH
    pro_count = "i"
    ser = serial.Serial(UART_PATH,9600,timeout = 0.5)
    Data = ser.readline()
#    Data = 	["'$GPGGA", '204628.000', '4222.8112', 'N', '14102.0097', 'E', '2', '11', '0.94', '61.5', 'M', '34.5', 'M', '', "*5C\\r\\n'"]
    Data = str(Data)
    Data = Data[1:len(Data)]
    time.sleep(0.1)
    GPS_Array = Data.split(',')
    return GPS_Array

def r_NMEA_Decorder(GPS_Array):
    global pro_count
    pro_count = "r"
    DataType = 0
    GGA_DMM_latitude = 2
    GGA_DMM_longitude = 4
    lalo = [0,0]
    if GPS_Array[DataType] == "'$GPGGA":
        lalo[0] = sub_s_DD_DMM_converter_la(GPS_Array[GGA_DMM_latitude])
        lalo[1] = sub_s_DD_DMM_converter_lo(GPS_Array[GGA_DMM_longitude])

#        GPS_Array_a = list(GPS_Array[GGA_DMM_latitude])

#        GPS_Array_a[0] = " "
#        GPS_Array_a[1] = " "
#        GPS_Array_a[len(GPS_Array_a)-1] = " "
#        GPS_Array_a[len(GPS_Array_a)-2] = " "
#        GPS_Array_b = list(GPS_Array[GGA_DMM_longitude])
#        print(GPS_Array_a)
#        GPS_Array_b[0] = " "
#        GPS_Array_b[1] = " "
#        GPS_Array_b[len(GPS_Array_b)-1] = " "
#        GPS_Array_b[len(GPS_Array_b)-2] = " "

#        print(GPS_Array_b)
#        lalo[0] = sub_s_DD_DMM_converter(''.join([str(i) for i in GPS_Array_a]))
#        lalo[1] = sub_s_DD_DMM_converter(''.join([str(i) for i in GPS_Array_b]))

    if lalo[0] != 0 and lalo[1] != 0:
        return lalo    
    return 0

def sub_s_DD_DMM_converter_la(DMM_val):
    global pro_count
    pro_count = "s"

    y_DD = 0
    if DMM_val != "":
        DMM_val = float(DMM_val)/100
        DMM_val = round(DMM_val , 6)
        y_str = str(DMM_val)
#        print(y_str,"a")
        #y_float = float(DMM_val/100)
        y_min = float(y_str[3:len(y_str):1])/60*100 / 1e6
#        print(len(y_str))
        if len(y_str) != 9:
            y_min *= 10
#        print(y_min , "n")
        y_DD  = float(y_str[0:3:1]) + y_min
        y_DD = round(y_DD , 5)
    return y_DD

def sub_s_DD_DMM_converter_lo(DMM_val):
    global pro_count
    pro_count = "s"
    y_DD = 0
    if DMM_val != "":
        DMM_val = float(DMM_val)/100
        DMM_val = round(DMM_val , 7)
        y_str = str(DMM_val)
#        print(y_str,"a")
        #y_float = float(DMM_val/100)
        y_min = float(y_str[4:len(y_str):1])/60*100 / 1e6
#        print(len(y_str))
        if len(y_str) != 10:
            y_min *= 10
#        print(y_min , "n")
        y_DD  = float(y_str[0:3:1]) + y_min
        y_DD = round(y_DD , 5)
    return y_DD

def t_TXT_Write_module(GPS_Array,GPS_txt_PATH):

    if GPS_Array[0] != 0 and GPS_Array[1] != 0:

        GPS_Array = str(GPS_Array)
        with open(GPS_txt_PATH, "w") as lf :
            gps_write =lf.writelines(GPS_Array[1:len(GPS_Array)-1:1])
    return GPS_Array

#赤GPS 
def read_rgps() :
    global GPS_Realtime_txt_PATH 
    qwiicGPS = qwiic_titan_gps.QwiicTitanGps()
    
    if qwiicGPS.connected is False:
        print("Could not connect to to the SparkFun GPS Unit. Double check that\
              it's wired correctly.", file=sys.stderr)
        return

    qwiicGPS.begin()
    
    if qwiicGPS.get_nmea_data() is True:
        GPS_Array = [qwiicGPS.gnss_messages['Latitude'], qwiicGPS.gnss_messages['Longitude']]
        if GPS_Array != 0:
            Main_State = t_TXT_Write_module(GPS_Array,GPS_Realtime_txt_PATH)
            return Main_State

#MPU9250
bus_number = 1
bus = smbus.SMBus(bus_number)
Accel_Gyro_sensor_device_addr = 0x68 #0x61かもしれない
#加速度センサの値を取得するためにアドレスを入手する
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

def MPU_setup():
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
                    RT_rad = np.arctan(Magnet_status[i]/(Magnet_status[j] + 0.01))
                    print(i,j,RT_rad)
                    RT_rad = np.arctan(Magnet_status[j]/(Magnet_status[i] + 0.01))
                    print(j,i,RT_rad)

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

#フォトレジスタ
spi = spidev.SpiDev()
spi.open(0, 0) # 0：SPI0、0：CE0
spi.max_speed_hz = 1000000 # 1MHz SPIのバージョンアップによりこの指定をしないと動かない

def read_voltage():
    dout = spi.xfer2([((0b1000+CHN)>>2)+0b100,((0b1000+CHN)&0b0011)<<6,0]) # Din(RasPi→MCP3208）を指定
    bit12 = ((dout[1]&0b1111) << 8) + dout[2] # Dout（MCP3208→RasPi）から12ビットを取り出す
    volts = round((bit12 * V_REF) / float(4095),4)  # 取得した値を電圧に変換する（12bitなので4095で割る）
    return volts # 電圧を返す

def stop_spi() :
    spi.close()

#カメラ    
def ca_main(camera_count):
    i = camera_count
    pic_name ="redcorn"+str(i)+ ".jpg"
    # カメラ初期化
    camera  = picamera.PiCamera()
    # 解像度の設定
    camera.resolution = (640, 480)
    # 撮影の準備
    camera.start_preview()
    # 準備している間、少し待機する
    time.sleep(0)
    # 撮影して指定したファイル名で保存する
    camera.capture(pic_name)
    camera.stop_preview()
    camera.close()

# GPIOにアクセスするためのインスタンスを作成
pi = pigpio.pi()                             

def moter_setup():
    # GPIO pin を出力設定
    pi.set_mode(LEFT_TOP_M1, pigpio.OUTPUT)     
    pi.set_mode(LEFT_TOP_M2, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M1, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M2, pigpio.OUTPUT)
    pi.set_mode(CAREER_CUT, pigpio.OUTPUT)
    #GPIOの周波数を設定
    pi.set_PWM_frequency(LEFT_TOP_M1, 50)
    pi.set_PWM_frequency(LEFT_TOP_M2, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M1, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M2, 50)
    #GPIOの範囲を設定
    pi.set_PWM_range(LEFT_TOP_M1, 100)
    pi.set_PWM_range(LEFT_TOP_M2, 100)
    pi.set_PWM_range(RIGHT_TOP_M1, 100)
    pi.set_PWM_range(RIGHT_TOP_M2, 100)
    #電熱線設定
    pi.write(CAREER_CUT , 0)

def stop():
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.write(CAREER_CUT , 0)

def gostop(start, max = 100.0, t = 1.0) :
    s = max / t / 10.0
    for duty in drange2(max, 0.0, s):
        pi.set_PWM_dutycycle(LEFT_TOP_M1, duty)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty)
        time.sleep(0.1)

def drange1(begin, end, step):
    n = begin
    while n+step < end:
        yield n
        n += step

def drange2(begin, end, step):
    n = begin
    while n+step > end:
        yield n
        n -= step        

def step(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange1(0.0, max, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def go(start, max = 100.0, t = 1.0):
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    if start:
        s = max / t / 10.0
        for duty in drange1(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M1, duty)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, max)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, max)

def go2(start, rmax =85, lmax=100 , t = 1.0):
    
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M1, lmax)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, rmax)

def back(start, max = 100.0, t = 1.0):
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    if start:
        s = max / t / 10.0
        for duty in drange1(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M2, duty)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, duty)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M2, max)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, max)

def left(start, max = 100.0, t = 1.0, rate = 1.0):
    if rate >= 0:
        r = max
        l = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
    else:
        r = max
        l = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M2, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M2, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)

def right(start, max = 100.0, t = 1.0, rate = 1.0):
    if rate >= 0:
        l = max
        r = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
    else:
        l = max
        r = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M2, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, r)

def escape() :
        left(0,75,rate = -1)
        time.sleep(2.0)
        stop()
        right(0,75,rate = -1)
        time.sleep(2.0)
        stop()
        go2(0)
        time.sleep(1)
        stop() 
        right(0,75,rate = -1)
        time.sleep(0.35)
        stop()
        go2(0)
        time.sleep(10)
        stop() 

def career_cat(t=1.5) :
    career_time = t 
    print("点火")
    pi.write(CAREER_CUT , 1 )
    time.sleep(career_time)
    pi.write(CAREER_CUT , 0 )
    time.sleep(1)
    print("点火")
    pi.write(CAREER_CUT , 1 )
    time.sleep(career_time)
    pi.write(CAREER_CUT , 0 )
    time.sleep(1)

#色認識(産廃)
def red_search(camera_count) :
    i = camera_count
    pic_name = "redcorn"+str(i)+".jpg"
    pic_name_out = "redcorn"+str(i)+"out.jpg"
    mask_pic = "redcorn"+str(i)+"mask.jpg"
    
    #画像読み込み
    img = cv2.imread(pic_name)
    #画像を180°回転
    img = cv2.rotate(img, cv2.ROTATE_180)
    #cv2.imwrite('redcorn777_rotate_180.jpg', img)
    #画像をHSV化
    img_HSV1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    #cv2.imwrite('redcorn_HSV.jpg', img_HSV)
    #画像の平滑化
    img_HSV2 = cv2.GaussianBlur(img_HSV1, (9, 9), 3)
    #cv2.imwrite('redcorn_HSV2.jpg', img_HSV)
    
    #BGRで画像を分割
    img_B, img_G, img_R = cv2.split(img_HSV2)
    #画像確認用
    #cv2.imwrite('redcorn_H.jpg', img_H)
    #cv2.imwrite('redcorn_S.jpg', img_S)
    #cv2.imwrite('redcorn_V.jpg', img_V)
    
    #画像の2値(赤とそれ以外に分割した画像を作成)
    img_thre, img_redcorn_gray = cv2.threshold(img_G, 175, 255, cv2.THRESH_BINARY)
    cv2.imwrite(mask_pic, img_redcorn_gray)
    
    #輪郭を検出
    #labels, contours, hierarchy = cv2.findContours(img_redcorn_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #バージョンによってはこちらを使用する
    contours, hierarchy = cv2.findContours(img_redcorn_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for i in range(0, len(contours)):
        if len(contours[i]) > 0:

            #小さなオブジェクトを切り取る
            if cv2.contourArea(contours[i]) < 300:
                continue

            #画像に境界を線で出力
            cv2.polylines(img, contours[i], True, (0, 0, 0), 5)
    
    #画像を出力
    cv2.imwrite(pic_name_out, img)
    
    #モーメントを取得
    M = cv2.moments(img_redcorn_gray, False)
    #赤色部分の重心の座標を計算
    x,y= int(M["m10"]/(M["m00"]+1)) , int(M["m01"]/(M["m00"]+1))
    #重心座標を円で出力
    cv2.circle(img, (x,y), 20, 100, 2, 4)
    cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
    
    #画像を出力
    cv2.imwrite('redcorn_out2.jpg', img)
    
    #座標を出力
    out = [x , y]
    print(out)

    return out

#色認識（及川式）
def red_search2(camera_count):
    for i in range(1):
        c = camera_count
        x = 0
        y = 0
        pic_name="redcorn"+str(c)+ ".jpg"
        pic_name_out="redcorn"+str(c)+ "out.jpg"
        gray_pic="redcorn"+str(c)+ "gray.jpg"
        #画像読み込み
        img = cv2.imread(pic_name,cv2.IMREAD_COLOR)
        #画像を180°回転
        img = cv2.rotate(img, cv2.ROTATE_180)
        #画像の平滑化
        img = cv2.GaussianBlur(img, (9, 9), 3)
        height, width = img.shape[:2]
        #画像のHSV化
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        img_redcorn=np.zeros((height,width,3),np.uint8)
        #画像の2値化
        img_redcorn[(h <=255) & (h > 190) + (h <30) & (h >= 0) & (s > 80) & (v > 80)] = 255 #! (h <255) & (h > 190)の値を変えることによって微妙な赤色の変化に対応できます！
        #画像を出力
        cv2.imwrite(gray_pic,np.array(img_redcorn))
        img_gray = cv2.imread(gray_pic,cv2.IMREAD_GRAYSCALE)
        #モーメントを取得
        M = cv2.moments(img_gray, False)
        #輪郭抽出
        contours, hierarchy = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #図心を出す
        x,y= int(M["m10"]/(M["m00"]+1)) , int(M["m01"]/(M["m00"]+1))
        #Zerodivision_kaihi
        #図心を画像に記載する
        cv2.circle(img, (x,y), 20, 100, 2, 4)
        cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
        cv2.imwrite(pic_name_out,np.array(img))
        out = [x , y]
        print(out)
        return out

#log取得
def log_setup() :
    logname = ["dt_now", ",", "gps_x", ",", "gps_y" , ",","altitude", ",", "photvolt"
               , "action", ",", "my_angle", ",", "goal_angle", ",", "goal_destance", "\n"]
    f0 = open('logdete.txt', 'a', encoding='UTF-8')
    f0.writelines(logname)
    f0.close()
    print("set_log")
    
    return 0

def write_log(seqence = "a",gps="0,0",altitude = 0 ,photvolt = 0 ,diredtion = 0 ,my_angle = 0,goal_angle = 0,goal_destance = 0) :
    dt_now = str(datetime.datetime.now())
    log_list = [",", str(seqence), ",", str(gps), ",", str(altitude), ",", str(photvolt), ",",
                str(diredtion), ",", str(my_angle), ",", str(goal_angle), ",", str(goal_destance),",", str(seqence)]
    
    with open("logdete.txt" , "a") as lf :
        log_write =lf.writelines(dt_now)
    with open("logdete.txt" , "a") as lf :
        log_write =lf.writelines(log_list[1:len(log_list)-1:1])
    with open("logdete.txt" , "a") as lf :
        log_write =lf.writelines("\n")
    print("write_log")
    return 0


#シーケンス取得
def get_sequence_count() :
    with open("sequence_count.txt" , "r") as sf :
        sequence_count =sf.read()
    print("get_seqence")
    return int(sequence_count)

#シーケンス保存
def write_sequence_count(sequence_count) :
    sc = str(sequence_count)
    with open("sequence_count.txt" , "w") as sf :
        sequence_count =sf.writelines(sc)
        
    return 0

#通信開始用フラグ保存
def write_im920flag(flag) :
    f = str(flag)
    with open("im_flag.txt" , "w") as sf :
        flag =sf.writelines(f)
        
    return 0

def get_preset_pres() :
    with open("preset_pres.txt" , "r") as pf :
        preset_pres =pf.read()
    print("get_preset_pres")
    return float(preset_pres)

def get_preset_magnet() :
    with open("magnet_calibrate.txt" , "r") as mf :
        magnet_calibrate =mf.read()
    print("get_preset_pres")
    return magnet_calibrate

#GPS取得
def get_gps() :
    read_rgps()  #!赤GPSを使用する場合はこちらを使用する
    #read_gps()  #!緑GPSを使用する場合はこちらを使用する
    with open("gps_realtime.txt", "r") as tf:
        gps = tf.read()   
    #gps = [20.425540,136.081114] #ダミーデータ
    return gps

#機体の現在位置を取得
def get_gps_realtime() :
    read_rgps()
    #read_gps()
    with open("gps_realtime.txt", "r") as tf:
        gps_realtime = tf.read().split(',')    
    #gps_realtime = [20.425540,136.081114]
    return gps_realtime

#ゴールの座標を取得
def get_gps_goal() :
    with open("gps_goalpoint.txt", "r") as tf:
        gps_goal = tf.read().split(',')
    open("gps_goalpoint.txt", "r").close()
    #gps_goal =[24.288722,153.979642]
    return gps_goal

#機体の方角を取得(MPU)
def get_myangle() :
    #!データが真円になるように補正する必要あり
    Px = float(magnet_calibrate_list[0])
    Py = float(magnet_calibrate_list[1])
    #Px =-60                            
    #Py =250
    magnetlist = Get_Magnet_status()  
    #magnetlist = [100,0]             #ダミーデータ
    magnet_x  = float(magnetlist[0])
    magnet_y  = float(magnetlist[1])
    
    x = magnet_x - Px
    y = magnet_y - Py
    
    myangle = (0 - math.degrees(math.atan2(y,x))) % 360
    
    return myangle

#機体の方角を取得(GPS)
def get_myangle2(past_gps, now_gps) :
    past_la  = float(past_gps[0])
    past_lo  = float(past_gps[1])
    now_la  = float(now_gps[0])
    now_lo  = float(now_gps[1])
    
    la = now_la - past_la
    lo = now_lo - past_lo
    
    myangle = (90 - math.degrees(math.atan2(la,lo))) % 360
    
    return myangle

#ヒュペニの法則 使用設定値
GPS_A = 6378137.0   
E2 = 0.00669437999019758
A1E2 = 633439.32729246

#ゴールまでの距離を取得
def get_goal_distance(gps, goal):
	dy = math.radians(float(gps[0]) - float(goal[0]))
	dx = math.radians(float(gps[1]) - float(goal[1]))
	uy = math.radians((float(gps[0]) + float(goal[0])) / 2.0)
	W = math.sqrt(1.0 - E2 * math.sin(uy) ** 2.0)
	N = GPS_A / W
	M = A1E2 / W ** 3
	
	gps_distance = math.sqrt((dy * M) ** 2 + (dx * N * math.cos(uy)) ** 2)
	
	return gps_distance

# ゴールの方角を取得
def get_goal_angle(gps, goal):
    gps_la  = float(gps[0])
    gps_lo  = float(gps[1])
    goal_la  = float(goal[0])
    goal_lo  = float(goal[1])
    
    la = goal_la - gps_la
    lo = goal_lo - gps_lo

    goal_angle = (90 - math.degrees(math.atan2(la,lo))) % 360
    
    return goal_angle

#方角テスト
def test_angle() :
    while True :
        gps = get_gps_realtime()
        goal = get_gps_goal()
        #gps =[25,150]
        #goal =[25,140]
        ga = get_goal_angle(gps, goal)
        ma = get_myangle()
        ma2= get_myangle2()
        da = ga - ma
        da2 =ga - ma2
        
        print("ゴール方角" + "  " + str(ga) )
        print("   MPU    |   GPS      ")
        print( "機体の方角" + "  " + str(ma) +" | "+"機体の方角" + "  " + str(ma2))
        print("方角差1" + "      " + str(da) +" | " +"方角差2" + "      " + str(da2))
        
        time.sleep(1.5)
    return 0

#モーター駆動時間設定
def get_moter_time(Angle) :
    x =  cycle_time
    moter_time = (x/360) * math.fabs(Angle) 
    return moter_time

#モーター駆動時間テスト
def test_moter_time(angle = 90) :
    t = get_moter_time(angle)
    print("駆動時間"+"  "+str(t))
    
    return 0

#裏返り検知
def rivers() :
    return 0 #未製作

#メイン処理
def main() :
    #セットアップ
    log_setup()                                   
    moter_setup()                                 
    MPU_setup()                                  
    BME_setup()                                  
    get_calib_param()                            
    goal_point = get_gps_goal()
    preset_pres = get_preset_pres()                  
    sequence_count = 0                           
    sequence_count = get_sequence_count()        
    write_log(sequence_count,diredtion="set_up") 
    print("set_up_OK")                    

    #投下シーケンス
    if sequence_count == 0 :
        
        while True :
            photoresistor_voltage = float(read_voltage()) 
            my_point = get_gps_realtime()
            
            if photoresistor_voltage > 2 :
                sequence_count = 1 
                write_sequence_count(sequence_count) 
                write_im920flag(1) 
                print("release")
                write_log(sequence_count ,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = photoresistor_voltage ,
                          diredtion = "release" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                          goal_destance = get_goal_distance(my_point,goal_point)) 
                break
            
            else :
                write_log(sequence_count ,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = photoresistor_voltage ,
                          diredtion = "no_release" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                          goal_destance = get_goal_distance(my_point,goal_point))
                time.sleep(1)  
    
    #第二投下シーケンス(光センサーの使用が不可の場合に使用)
    if sequence_count == 0.5 :
        altitude_flag = 0 
        while True :
            if get_altitude(preset_pres) > preset_altiude2 :
                print("rise")
                
                if altitude_flag < 3 :
                    altitude_flag = altitude_flag + 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "rise2" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                              goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
                
                elif altitude_flag == 3 :
                    sequence_count = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "rise3" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                              goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
                    break
            
            elif get_altitude < preset_altiude2 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "rise1" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                          goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
            
            time.sleep(0.1)
    
    #降下シーケンス
    if sequence_count == 1 :
        f = 1
        while True :
            my_point = get_gps_realtime()
            
            if get_altitude(preset_pres) < preset_altiude :
                
                time.sleep(3)
                print("landing_1")
                career_cat(career_time)
                sequence_count = 2
                write_sequence_count(sequence_count)
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "ignition_1" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                          goal_destance = get_goal_distance(my_point,goal_point))
                break
            
            elif f > 300 :
                time.sleep(5)
                print("landing_2")
                career_cat(career_time)
                sequence_count = 2
                write_sequence_count(sequence_count)
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0,
                          diredtion = "ignition_2" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                          goal_destance = get_goal_distance(my_point,goal_point))
                break
            
            else :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "descent" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                          goal_destance = get_goal_distance(my_point,goal_point))
                time.sleep(1.0)
                f = f + 1
                print(f)
    
    #脱出シーケンス
    if sequence_count == 2 :  
        #回収モジュールから脱出
        escape()
        career_cat(career_time)
        sequence_count = 3
        my_point  = get_gps_realtime()
        write_sequence_count(sequence_count)
        write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                  diredtion = "escape" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                  goal_destance = get_goal_distance(my_point,goal_point))
        print("脱出")
    
    #遠距離誘導シーケンス
    if sequence_count == 3 : 
        stack_count = 0
        past_point  = get_gps_realtime()
        while True :
            time.sleep(1)
            #機体の位置を取得
            my_point = get_gps_realtime()
            
            if past_point != my_point :
                stack_count = 0 
                #機体の方位を取得
                #my_angle = get_myangle2(past_point, my_point) #!GPS誘導
                my_angle = get_myangle()                       #!地磁気誘導
                print("機体の方角----------|" + str(my_angle) )
                #ゴールまでの方位を計算
                goal_angle = get_goal_angle(my_point,goal_point)
                print("ゴールの方角----------|" + str(goal_angle) )
                #ゴールと機体の角度差を計算
                Angle =   goal_angle - my_angle 
                print("角度差----------|" + str(Angle) )
                #角度に応じてモーター駆動
                if math.fabs(Angle) < 20 or math.fabs(Angle) > 340 :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "go" ,my_angle = get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    go2(0)
                    print("go")
                    time.sleep(10.0)
                    stop()
                    
            
                elif -340 <= Angle <= -180 or 20 <= Angle <= 180 :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "left" ,my_angle = get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    left_time = get_moter_time(Angle)
                    left(0,75,rate = -1)
                    time.sleep(left_time)
                    #time.sleep(0.2)
                    print("left"+str(Angle))
                    stop()
                    my_angle = get_myangle()
                    #go2(0)
                    #time.sleep(5.0)
                    #stop()
                
                elif -180 <= Angle <= -20 or 180 <= Angle <= 340 :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "right" ,my_angle = get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    right_time = get_moter_time(Angle)
                    right(0,75,rate = -1)
                    time.sleep(right_time)
                    #time.sleep(0.2)
                    print("right"+str(Angle))
                    stop()
                    my_angle = get_myangle()
                    #go2(0)
                    #time.sleep(5.0)
                    #stop()
                    
                #ゴールとの距離を取得
                goal_distance = get_goal_distance(my_point,goal_point)
                print("get_goal_distance"+str(goal_distance))
                if goal_distance < preset_goal_distanc :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = 0 ,my_angle = get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    sequence_count = 4
                    write_sequence_count(sequence_count)
                    break
            
                else :
                    #time.sleep(3.0)
                    past_point = my_point
                    #print("onecycle")
            
            elif stack_count == 3 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "stack" ,my_angle = get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                
                print("stack")
                left(0,100,rate = -1)
                time.sleep(get_moter_time(90))
                go2(0)
                time.sleep(1.0)
                right(0,100,rate = -1)
                time.sleep(get_moter_time(90))
                go2(0)
                time.sleep(1.0)
                stop()
                stack_count = 0
            
            else :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0, 
                          diredtion = "stack?" ,my_angle = get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                go2(0)
                time.sleep(5.0)
                stop()
                stack_count = stack_count + 1 
                print("stack?")
    
    #能代室内用補助誘導
    if sequence_count == 3.5 : 
        goal_angle = 60
        ff = 0
        while True :
            my_angle = get_myangle()
            Angle = my_angle - goal_angle
            if math.fabs(Angle) < 30 or math.fabs(Angle) > 330 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "go" ,my_angle = get_myangle2(past_point, my_point),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                go2(0)
                time.sleep(2.0)
                stop()
                ff = ff + 1
            
            elif -330 <= Angle <= -180 or 30 <= Angle <= 180 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "left" ,my_angle = get_myangle2(past_point, my_point),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                left_time = get_moter_time(Angle)
                left(0,75,rate = -1)
                time.sleep(left_time)
                print("left"+str(Angle))
                stop()
                go2(0)
                time.sleep(2.0)
                stop()
                ff = ff + 1
            
            elif -180 <= Angle <= -30 or 180 <= Angle <= 330 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "right" ,my_angle = get_myangle2(past_point, my_point),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                right_time = get_moter_time(Angle)
                right(0,75,rate = -1)
                time.sleep(right_time)
                print("right"+str(Angle))
                stop()
                go2(0)
                time.sleep(2.0)
                stop()
                ff = ff + 1
            
            if ff == 3 :
                sequence_count == 4
                break


    #近距離誘導シーケンス            
    if sequence_count == 4 :
        f1 = 0
        f2 = 0
        camera_count = 0           #カメラカウント（基本0回）
        move_count   = 0           #カメラ誘導時赤色認識回数（基本0回）
        finish_count = 15           #カメラ誘導時直進回数
        threshold =60              #カメラ誘導時赤色認識した時に直進するかしないかどうか（0~320） 
        while True :
            ca_main(camera_count)
            out_re = red_search2(camera_count)
            camera_count = camera_count + 1
            
            if out_re[0] == 0 and out_re[1] == 0 and f1 == 0 :
                print("探索モード")
                right(0,100,rate=-1)
                time.sleep(0.32)
                stop()
                go2(0)
                time.sleep(1.5)
                stop()
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "red_search" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
            if out_re[0] == 0 and out_re[1] == 0 and f2 == 0 and f1 >= 1 :
                print("認識していないr")
                right(0,100,rate=-1)
                time.sleep(0.20)
                stop()
                f2 = 1
                f1 = f1 + 1
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "lost_r" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
            elif out_re[0] == 0 and out_re[1] == 0 and f2 == 1 and f1 >= 1 :
                print("認識していないl")
                left(0,100,rate=-1)
                time.sleep(0.32)
                stop()
                f2 = 0
                f1 = f1 + 1 
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "lost_l" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
            if f1 == 5 :
                f1 = 0

            if out_re[0] != 0 and out_re[1] != 0 and move_count < finish_count :
                print("赤色認識成功")
            
                if out_re[0] < (320 - threshold) :
                    print("左")
                    left(0,100, rate = -1)
                    time.sleep(0.2)
                    stop()
                    f2 = 0
                    f1 = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "left" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)

                elif out_re[0] > (320 + threshold) :
                    print("右")
                    right(0,100, rate = -1)
                    time.sleep(0.21)
                    stop()
                    f2 = 1
                    f1 = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "right" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
                else :
                    print("直進")
                    go2(0)
                    time.sleep(0.6)
                    f1 = 1
                    move_count = move_count + 1 
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "go" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)

            elif move_count == finish_count :
                print('goal')
                sequence_count = 5
                write_sequence_count(sequence_count)
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                          ,diredtion = "goal" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                break  

if __name__ == '__main__' :
    while True : 
        try:
            main()
        finally:
            stop()


