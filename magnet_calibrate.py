#coding: utf-8
#地磁気補正用プログラム

import smbus2 as smbus
import time
import numpy as np

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

    print('楕円体パラメータ')
    print(x)

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

    print('固有値・固有ベクトル')
    print(lbda)
    print(v)

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
    print('回転行列')
    print(P.T)

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

    print('拡大係数')
    print(sx, sy, sz)


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

if __name__ == '__main__' :

    key=input("新しいデータを取得しますか？"+"\n"+"y/n"+"------------")
    a = str(key)

    if a == "y" :
        print("機体を八の字を描く様に回転させてください")
        MPUlog_main()
        time.sleep(0.1)
        magnet_calibrate()
    
    if a == "n" :
        magnet_calibrate()
