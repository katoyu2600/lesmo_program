# coding: utf-8
#対応ハードウェア：IM920c 
#取扱説明書：https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920_HW_manual.pdf
#           https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920_SW_manual.pdf
#           https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920c_HW_manual.pdf

#モジュールインポート
import serial
import binascii
import pigpio
import time
import time
import sys
import qwiic_titan_gps

#パス設定
UART_PATH = '/dev/serial0'
IM_PIN = 24
GPS_Realtime_txt_PATH = "gps_realtime.txt"

#インスタンス設定
pi = pigpio.pi()
com = serial.Serial(UART_PATH, 19200)

#GPS確認用プログラム
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

#GPS読み出し
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

#GPSをテキストファイルに書き込み
def t_TXT_Write_module(GPS_Array,GPS_txt_PATH):
    if GPS_Array != 0:
        fp = open(GPS_txt_PATH , "w")
        GPS_Array = str(GPS_Array)
        fp.writelines(GPS_Array[1:len(GPS_Array)-1:1])
        fp.close()
    return GPS_Array    

#固有ID読み出し
def read_ID() :
    com.flushInput()
    com.write(b'RDID' + b'\r\n')
    com.flushOutput()
    print(com.readline().strip())

#テキスト送信
def send(text):
    com.flushInput()
    com.write(b'TXDA' + binascii.b2a_hex(text.encode('utf-8')) + b'\r\n')
    com.flushOutput()
    com.readline()

def close():
    com.close()

#メイン処理
def main():
    pi.set_mode(IM_PIN, pigpio.OUTPUT)
    pi.write(IM_PIN, 1)
    
    read_ID()
    while True :
        read_rgps()
        with open(text_PASS, "r") as tf:
            text = tf.read().split(',')
        text = str(text)
        text =list(text)
        print(str(text[1:len(text)-1:1]))
        for i in text :
            send(i)
            time.sleep(0.333)
        print('send')

if __name__ == '__main__' :
    text_PASS = "gps_realtime.txt"
    try :
        main()
    
    finally :
        close()

