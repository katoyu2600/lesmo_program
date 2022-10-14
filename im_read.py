# coding: utf-8
import pigpio
import serial
import time

UART_PATH = '/dev/serial0'
#IM_PIN = 24

#pi = pigpio.pi()
com = serial.Serial(UART_PATH, 19200)

def read_ID() :
    com.flushInput()
    com.write(b'RDID' + b'\r\n')
    com.flushOutput()
    print(com.readline().strip())


def read() :
    com.flushInput()

    text = ""
    cngtext = ""
    #try:
    text = com.readline().decode('utf-8').strip() #受信と空白の削除
    text = text.replace("\r\n","")
    print(text)
    text = text.split(":")[1]
    text = text.split(",")
    #print(text)

    for x in text:
        cngtext += chr(int(x,16))

    #except Exception:
        #pass

    return cngtext

def close():
    com.close()


if __name__ == '__main__' :
    try :
        #pi.set_mode(IM_PIN, pigpio.OUTPUT)
        #pi.write(IM_PIN, 1))
        read_ID()
        
        while True :
            read()

            
    finally :
        close()
        
#[24.288722,153.979642]