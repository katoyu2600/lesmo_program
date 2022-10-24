# coding: utf-8
#対応ハードウェア IM920c 
#取扱説明書:https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920_HW_manual.pdf
#          https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920_SW_manual.pdf
#          https://www.interplan.co.jp/support/solution/wireless/im920/manual/IM920c_HW_manual.pdf

#モジュールインポート
import pigpio
import serial
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

#チャンネル番号変更
def change1_CHANNEL(channel) :
    com.flushInput()
    com.write(b'STCH '+ str(channel) +b'\r\n')
    com.flushOutput()
    print(com.readline().strip())

#チャンネル番号変更
def change2_CHANNEL() :
    com.flushInput()
    com.write(b'STCH 02' +b'\r\n')
    com.flushOutput()
    print(com.readline().strip())

#チャンネル番号読み出し
def read_CHANNEL() :
    com.flushInput()
    com.write(b'RDCH'+b'\r\n')
    com.flushOutput()
    print(com.readline().strip())

def close():
    com.close()

#メイン処理
def main():
    pi.set_mode(IM_PIN, pigpio.OUTPUT)
    pi.write(IM_PIN, 1)
    print("通信電源ON")
    time.sleep(0.5)
    print("固有ID読み出し")
    read_ID()
    time.sleep(0.5)
    print("現在のチャンネル数読み出し")
    read_CHANNEL()
    time.sleep(0.5)
    f1 = input("チャンネル数を変更しますか？-----------------y/n")
    if f1 == "y" :
        print("-------変更可能チャンネル-------")
        print("チャンネル番号 ---- 周波数")
        print("01 ----920.6MHz  02 ----920.8MHz")
        print("03 ----921.0MHz  04 ----921.2MHz")
        print("05 ----921.4MHz")
        print("07 ----921.8MHz  08 ----922.0MHz")
        print("09 ----922.2MHz  10 ----922.4MHz")
        print("11 ----922.6MHz  12 ----922.8MHz")
        print("13 ----923.0MHz  14 ----923.2MHz")
        print("15 ----923.4MHz")
            
        f2 = input("チャンネル番号を設定してください")
        change2_CHANNEL() 
        
    elif f1 == "n" :
        print("チャンネル継続")
        
    else :
        print("正しく入力してください")            

if __name__ == '__main__' :
    try :
        main()
    
    finally :
        print("現在のチャンネル")
        read_CHANNEL()
        com.close()