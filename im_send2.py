# coding: utf-8
import serial
import binascii
import pigpio
import time

UART_PATH = '/dev/serial0'
IM_PIN = 24

pi = pigpio.pi()
com = serial.Serial(UART_PATH, 19200)

def read_ID() :
    com.flushInput()
    com.write(b'RDID' + b'\r\n')
    com.flushOutput()
    print(com.readline().strip())

def send(text):
    com.flushInput()
    com.write(b'TXDA' + binascii.b2a_hex(text.encode('utf-8')) + b'\r\n')
    com.flushOutput()
    com.readline()

def close():
    com.close()

def get_flag() :
    with open("im_flag.txt" , "r") as sf :
        flag =sf.read()
    print("get_seqence")
    return int(flag)

if __name__ == '__main__' :
    text_PASS = "gps_realtime.txt"
    try :
        pi.set_mode(IM_PIN, pigpio.OUTPUT)
        pi.write(IM_PIN, 1)
        while True :
            flag = get_flag()
            if flag == 0 :
                print("no_signal")
                pass
            
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
            
            time.sleep(10)

    finally :
        pi.set_mode(IM_PIN, pigpio.OUTPUT)
        pi.write(IM_PIN, 0)
        close()
