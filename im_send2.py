# coding: utf-8
#対応ハードウェア：IM920c 
#取扱説明書：https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920_HW_manual.pdf
#            https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920_SW_manual.pdf
#            https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920c_HW_manual.pdf

#モジュールインポート
import serial
import binascii
import pigpio
import time

#パス設定
UART_PATH = '/dev/serial0'
IM_PIN = 24

#インスタンス設定
pi = pigpio.pi()
com = serial.Serial(UART_PATH, 19200)

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

#通信用フラグ取得
def get_flag() :
    with open("im_flag.txt" , "r") as sf :
        flag =sf.read()
    print("get_seqence")
    return int(flag)

#メイン処理
def main():
    pi.set_mode(IM_PIN, pigpio.OUTPUT) #この2行はelifの中の方がいいかもしれない
    pi.write(IM_PIN, 1)                
    while True :                      #フラグが立てば通信開始
        flag = get_flag()
        if flag == 0 :
            print("no_signal")
            time.sleep(10)
        
        elif flag == 1 :
            print("電源ON")
            read_ID()
            while True :
                with open(text_PASS, "r") as tf:
                    text = tf.read().split(',')
                text = list(str(text))
                print(str(text))
                for i in text :
                    send(i)
                    time.sleep(0.333)
                print('send')

if __name__ == '__main__' :
    text_PASS = "gps_realtime.txt"
    try :
        main()
        
    finally :
        pi.set_mode(IM_PIN, pigpio.OUTPUT)
        pi.write(IM_PIN, 0)
        close()
