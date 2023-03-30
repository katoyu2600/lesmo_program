#CanSatセットアップ用プログラム
import smbus2 as smbus
import time
import datetime
import sys
import numpy as np
import qwiic_titan_gps
import pigpio
import time 
import picamera
import cv2

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

CAREER_CUT = 6

pi = pigpio.pi()

pi.set_mode(CAREER_CUT, pigpio.OUTPUT)
pi.write(CAREER_CUT , 0)

def career_cat(c_time) :
    print("点火")
    pi.write(CAREER_CUT , 1 )
    time.sleep(c_time)
    pi.write(CAREER_CUT , 0 )
    time.sleep(1)

def run_example():

    print("SparkFun GPS Breakout - XA1110!")
    qwiicGPS = qwiic_titan_gps.QwiicTitanGps()

    if qwiicGPS.connected is False:
        print("Could not connect to to the SparkFun GPS Unit. Double check that\
              it's wired correctly.", file=sys.stderr)
        return

    qwiicGPS.begin()

    while True:
        if qwiicGPS.get_nmea_data() is True:
            print("Latitude: {}, Longitude: {}, Time: {}".format(
                qwiicGPS.gnss_messages['Latitude'],
                qwiicGPS.gnss_messages['Longitude'],
                qwiicGPS.gnss_messages['Time']))

        time.sleep(1)

GPS_Realtime_txt_PATH = "gps_goalpoint.txt"

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
            time.sleep(0.9)
            return Main_State
        
def t_TXT_Write_module(GPS_Array,GPS_txt_PATH):
    if GPS_Array != 0:
        fp = open(GPS_txt_PATH , "w")
        GPS_Array = str(GPS_Array)
        fp.writelines(GPS_Array[1:len(GPS_Array)-1:1])
        fp.close()
    return GPS_Array 

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

def readData():
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

def readpres() :
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    p = compensate_P(pres_raw)
    #p = 1013.25
    return p

def readtemp() :
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)   
    t = compensate_T(temp_raw)
    #t = 25
    return t

def altitude(pf) :
    p0    = 1013.25
    p0fix = float(pf)
    p1    = float(readpres()) 
    t1    = float(readtemp())
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

def setup():
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

setup()
get_calib_param()

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

def MPUlog_main() :
    start_up()
    txt_PATH = 'MPUlog.txt'
    f = open(txt_PATH, 'w', encoding='UTF-8')
    f.close() 
    log_count = 0
    n = 1
    
    while  log_count <= 1000 :
        txt_PATH = 'MPUlog.txt'
        f = open(txt_PATH, 'a', encoding='UTF-8')

        #accel_list  =Get_Accel_status()
        #gyro_list   =Get_Gyro_status()
        magnet_list =Get_Magnet_status()
    
        #accellog   =accel_list[1:-1]
        #gyrolog    =gyro_list[1:-1]
        magnetlog  =magnet_list
    
        f.writelines(str(magnetlog)[1:-1]+"\n")
        f.close() 

        log_count = log_count + 1
        
        time.sleep(0.05)

def magnet_calibrate() :        
        
    txt_PATH = 'MPUlog.txt'
    #MPUlog.txtの最後の３列がmG（ミリガウス）単位の地磁気データx、y、zとなっている
    data=np.loadtxt(txt_PATH, delimiter=',')

    #地磁気データ抽出
    magx=-data[:,0]
    magy=data[:,1]
    magz=-data[:,2]

    #最小二乗法
    #正規方程式の作成
    #1 x2
    _x4   =sum(magx**4)
    _x2y2 =sum(magx**2*magy**2)
    _x2z2 =sum(magx**2*magz**2)
    _2x3y =sum(2*magx**3*magy)
    _2x2yz=sum(2*magx**2*magy*magz)
    _2x3z =sum(2*magx**3*magz)
    _x3   =sum(magx**3)
    _x2y  =sum(magx**2*magy)
    _x2z  =sum(magx**2*magz)
    #2 y2
    _y4   =sum(magy**4)
    _y2z2 =sum(magy**2*magz**2)
    _2xy3 =sum(2*magx*magy**3)
    _2y3z=sum(2*magy**3*magz)
    _2xy2z=sum(2*magx*magy**2*magz)
    _xy2  =sum(magx*magy**2)
    _y3   =sum(magy**3)
    _y2z  =sum(magy**2*magz)
    #3 z2
    _z4   =sum(magz**4)
    _2xyz2=sum(2*magx*magy*magz**2)
    _2yz3=sum(2*magy*magz**3)
    _2xz3 =sum(2*magx*magz**3)
    _xz2  =sum(magx*magz**2)
    _yz2  =sum(magy*magz**2)
    _z3   =sum(magz**3)
    #4 xy
    _2x2y2=sum(2*magx**2*magy**2)
    _2xy2z=sum(2*magx*magy**2*magz)
    _2x2yz=sum(2*magx**2*magy*magz)
    _x2y  =sum(magx**2*magy)
    _xy2  =sum(magx*magy**2)
    _xyz  =sum(magx*magy*magz)
    #5 yz
    _2y2z2=sum(2*magy**2*magz**2)
    _2xyz2=sum(2*magx*magy*magz**2)
    _xyz  =sum(magx*magy*magz)
    _y2z  =sum(magy**2*magz)
    _yz2  =sum(magy*magz**2)
    #6 xz
    _2x2z2=sum(2*magx**2*magz**2)
    _x2z  =sum(magx**2*magz)
    _xyz  =sum(magx*magy*magz)
    _xz2  =sum(magx*magz**2)
    #7 x
    _x2   =sum(magx**2)
    _xy   =sum(magx*magy)
    _xz   =sum(magx*magz)
    #8 y
    _y2   =sum(magy**2)
    _yz   =sum(magy*magz)
    #9 z
    _z2   =sum(magz**2)
    #b
    _x=sum(magx)
    _y=sum(magy)
    _z=sum(magz)

    #正規行列
    M=np.matrix([[     _x4,    _x2y2,    _x2z2,    _2x3y,   _2x2yz,  _2x3z,  _x3, _x2y, _x2z],
                 [   _x2y2,      _y4,    _y2z2,    _2xy3,    _2y3z, _2xy2z, _xy2,  _y3, _y2z],
                 [   _x2z2,    _y2z2,      _z4,   _2xyz2,    _2yz3,  _2xz3, _xz2, _yz2,  _z3],
                 [ _2x3y/2,  _2xy3/2, _2xyz2/2,   _2x2y2,   _2xy2z, _2x2yz, _x2y, _xy2, _xyz],
                 [_2x2yz/2,  _2y3z/2,  _2yz3/2, _2xy2z/2,   _2y2z2, _2xyz2, _xyz, _y2z, _yz2],
                 [ _2x3z/2, _2xy2z/2,  _2xz3/2, _2x2yz/2, _2xyz2/2, _2x2z2, _x2z, _xyz, _xz2],
                 [     _x3,     _xy2,     _xz2,     _x2y,     _xyz,   _x2z,  _x2,  _xy,  _xz],
                 [    _x2y,      _y3,     _yz2,     _xy2,     _y2z,   _xyz,  _xy,  _y2,  _yz],
                 [    _x2z,     _y2z,      _z3,     _xyz,     _yz2,   _xz2,  _xz,  _yz,  _z2]
                ])

    b=np.matrix([_x2, _y2, _z2, _xy, _yz, _xz, _x, _y, _z]).T

    #楕円体パラメータの算出
    x=np.linalg.inv(M)*(-b)

    #print('楕円体パラメータ')
    #print(x)

    a11=x[0,0]
    a22=x[1,0]
    a33=x[2,0]
    a12=x[3,0]
    a23=x[4,0]
    a13=x[5,0]
    b1 =x[6,0]
    b2 =x[7,0]
    b3 =x[8,0]

    #楕円体2次項パラメータ行列
    A=np.matrix([[a11,a12,a13],[a12,a22,a23],[a13,a23,a33]])

    #2次項パラメータ行列の固有値と固有ベクトルを求める
    lbda,v=np.linalg.eig(A)

    #print('固有値・固有ベクトル')
    #print(lbda)
    #print(v)

    #回転・並行移動
    #固有値と基底ベクトル（固有ベクトル）の並べ替え
    vt=v.T
    vx=vt[1].T
    vy=vt[2].T
    vz=vt[0].T
    lbdx=lbda[1]
    lbdy=lbda[2]
    lbdz=lbda[0]

    #1次項のパラメータベクトル
    B=np.matrix([b1,b2,b3])

    #対角化行列(回転行列)
    P=np.hstack([vx,vy,vz])
    #print('回転行列')
    #print(P.T)

    #楕円体の中心座標算出
    x0=(-B*vx/2/lbdx)[0,0]
    y0=(-B*vy/2/lbdy)[0,0]
    z0=(-B*vz/2/lbdz)[0,0]

    print('中心座標')
    print(x0,y0,z0)
    out = [x0,y0,z0]

    #球に変換
    W=(B*vx)[0,0]**2/4/lbdx + (B*vy)[0,0]**2/4/lbdy + (B*vz)[0,0]**2/4/lbdz -1 
    print('W')
    print(W)

    sx=np.sqrt(lbdx/W)
    sy=np.sqrt(lbdy/W)
    sz=np.sqrt(lbdz/W)
    smax=max(sx,sy,sz)
    sx=sx/smax
    sy=sy/smax
    sz=sz/smax

    #print('拡大係数')
    #print(sx, sy, sz)


    #移動計算のためデータ整形
    mag2=np.vstack([magx,magy,magz])

    #回転
    mag2=P.T*mag2

    #並行移動
    magx2=np.array(mag2)[0]-x0
    magy2=np.array(mag2)[1]-y0
    magz2=np.array(mag2)[2]-z0

    #拡大縮小
    magx2=sx*magx2
    magy2=sy*magy2
    magz2=sz*magz2
    
    txt_PATH = 'magnet_calibrate.txt'
    f = open(txt_PATH, 'w', encoding='UTF-8')
    out= str(out)
    f.write(out[1:len(out)-1:1])
    f.close()
    
    return 0

def red_search(camera_count):
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
        img_redcorn[(h <255) & (h > 190) & (s > 95)& (v > 97)] = 255 #! (h <255) & (h > 190)の値を変えることによって微妙な赤色の変化に対応できます！
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

def get_preset_pres() :
    with open("preset_pres.txt" , "r") as pf :
        preset_pres =pf.read()
    print("get_preset_pres")
    return float(preset_pres)

if __name__ == '__main__' :
    set1 =input("地磁気を補正しますか？"+"\n"+"y/n"+"------------")
    while True :
        if set1 == "y" :
            key=input("新しいデータを取得しますか？"+"\n"+"y/n"+"------------")
            a = str(key)

            if a == "y" :
                print("機体を八の字を描く様に回転させてください")
                MPUlog_main()
                time.sleep(0.1)
                magnet_calibrate()

            if a == "n" :
                magnet_calibrate()

            finish_flag1 = input("次の設定に行きますか？"+"\n"+"y/n"+"------------")
            
            if finish_flag1 == "y" :
                break
    
            elif finish_flag1 == "n" :
                pass

        elif set1 == "n" :
            break
    
    print("ゴール地点へ移動してください")
    time.sleep(1.0)
    
    set2 =input("高度設定を再設定しますか？"+"\n"+"y/n"+"------------")
    while True :
        if set2 == "y" :
            i = 0
            p = get_preset_pres()
            while i <= 10 :
                dt_now      =str(datetime.datetime.now())
                tenplog     =str(readtemp())
                preslog     =str(readpres())
                altitudelog =str(altitude(p))
            
                print_list  =["気温",tenplog, "気圧", preslog, "高度", altitudelog]
            
                f = open('preset_pres.txt', 'w', encoding='UTF-8')
                f.writelines(preslog) 
                f.close()
                print(print_list)
                i = i + 1
                time.sleep(0.1)
                
            finish_flag2 = input("次の設定に行きますか？"+"\n"+"y/n"+"------------")
            
            if finish_flag2 == "y" :
                break
    
            elif finish_flag2 == "n" :
                print("再設定を行います")
                pass

        elif set2 == "n" :
            break

    set3 =input("ゴール地点GPSを再設定しますか？"+"\n"+"y/n"+"------------")
    while True :
        if set3 == "y" :
            GPS_Realtime_txt_PATH = "gps_goalpoint.txt"
            i = 0
            while i <= 10:
                print(read_rgps())
                i +=1
                time.sleep(0.1)
            
            finish_flag3 = input("次の設定に行きますか？"+"\n"+"y/n"+"------------")
            
            if finish_flag3 == "y" :
                break
    
            elif finish_flag3 == "n" :
                print("再設定を行います")

        elif set3 == "n" :
            break

    set4 =input("カメラで試し撮りをしますか？"+"\n"+"y/n"+"------------")
    while True :
        if set4 == "y" :
            a=input("ここに数字を入れてください")
            ca_main(int(a))
            red_search(int(a))
            finish_flag4 = input("次の設定に行きますか？"+"\n"+"y/n"+"------------")
            if finish_flag4 == "y" :
                break
            
            elif finish_flag4 == "n" :
                print("もう一度撮影します")
        
        elif set4 == "n" :
            break

    set4 =input("パラ分離試験をしますか？"+"\n"+"y/n"+"------------")
    while True :
        if set4 == "y" :
            b=input("点火時間を設定してください")
            career_cat(float(b))
            
            finish_flag4 = input("次の設定に行きますか？"+"\n"+"y/n"+"------------")
            if finish_flag4 == "y" :
                break
    
            elif finish_flag4 == "n" :
                print("再点火します")

        elif set4 == "n" :
            break
        
        
    print("最後にモーターの試験を行ってください")