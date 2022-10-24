#coding: utf-8
#CanSatラジコン操作用プログラム
#操作キー説明
# W:前進 A:左旋回 D:右旋回 S:後進 Q:左折 E:右折
# K:モーターパワー増加 L:モーターパワー減少
# ,:モーター左右差調節（左） .:モーター左右差調節（右）

import pigpio
import time
import fcntl
import termios
import sys
import os

class _Getch:
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

# GPIO番号定数
LEFT_TOP_M1 = 4 # lt
LEFT_TOP_M2 = 17
RIGHT_TOP_M1 = 27 # rt
RIGHT_TOP_M2 = 22

#グローバル変数
pi = pigpio.pi()

def getkey():
    fno = sys.stdin.fileno()
    attr_old = termios.tcgetattr(fno)
    attr = termios.tcgetattr(fno)
    attr[3] = attr[3] & ~termios.ECHO & ~termios.ICANON
    termios.tcsetattr(fno, termios.TCSADRAIN, attr)
    fcntl_old = fcntl.fcntl(fno, fcntl.F_GETFL)
    fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)
    chr = 0
    try:
        c = sys.stdin.read(1)
        if len(c):
            while len(c):
                chr = (chr << 8) + ord(c)
                c = sys.stdin.read(1)
    finally:
        fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old)
        termios.tcsetattr(fno, termios.TCSANOW, attr_old)

    return chr

def moter_init():
    pi.set_mode(LEFT_TOP_M1, pigpio.OUTPUT)
    pi.set_mode(LEFT_TOP_M2, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M1, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M2, pigpio.OUTPUT)

    pi.set_PWM_frequency(LEFT_TOP_M1, 50)
    pi.set_PWM_frequency(LEFT_TOP_M2, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M1, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M2, 50)

    pi.set_PWM_range(LEFT_TOP_M1, 100)
    pi.set_PWM_range(LEFT_TOP_M2, 100)
    pi.set_PWM_range(RIGHT_TOP_M1, 100)
    pi.set_PWM_range(RIGHT_TOP_M2, 100)

def stop():
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)

def drange_up(begin, end, step):
    n = begin
    while n+step < end:
        yield n
        n += step

def drange_down(begin, end, step):
    n = begin
    while n+step > end:
        yield n
        n -= step

def step_up(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange_up(max, 0.0, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def step_down(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange_down(0.0, max, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def calibrate_fix(l_fix,r_fix,rate) :
    
    if rate == 1 :
        l_fix = l_fix - 0.025
        if l_fix <= 0 :
            l_fix = 0
        r_fix = r_fix + 0.025 
        if r_fix >= 1 :
            r_fix = 1
    
    if rate == -1 :
        l_fix = l_fix + 0.025
        if l_fix >= 1 :
            l_fix = 1
        r_fix = r_fix - 0.025
        if r_fix <= 0 :
            r_fix = 0
    
    fix =[l_fix,r_fix]
    
    return fix

def pawer_fix(p_fix,rate) :
    
    if rate == 1 :
        p_fix = p_fix + 5
    if p_fix >= 100 :
        p_fix = 100
    if p_fix <= 0 :
        p_fix = 0
    
    if rate == -1 :
        p_fix = p_fix - 5
    if p_fix >= 100 :
        p_fix = 100
    if p_fix <= 0 :
        p_fix = 0
    
    return p_fix    

def go(start, max = 100.0, t = 1.0, l_fix = 1.0, r_fix = 1.0):
    
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    if start:
        s = max / t / 10.0
        for duty in drange_up(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M1, duty*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty*r_fix)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, max*l_fix)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, max*r_fix)

def back(start, max = 100.0, t = 1.0, l_fix = 1.0, r_fix = 1.0):
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    if start:
        s = max / t / 10.0
        for duty in drange_up(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M2, duty*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, duty*r_fix)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M2, max*l_fix)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, max*r_fix)

def left(start, max = 100.0, t = 1.0, rate = 0.6, l_fix = 1.0, r_fix = 1.0):
    if rate >= 0:
        r = max
        l = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
    else:
        r = max
        l = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M2, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M2, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)

def right(start, max = 100.0, t = 1.0, rate = 0.6, l_fix = 1.0, r_fix = 1.0):
    if rate >= 0:
        l = max
        r = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
    else:
        l = max
        r = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M2, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, r*r_fix)

def get_moter_pawer() :
    with open("moter_pawer.txt" , "r") as mf :
        moter_pawer =mf.read()
    print("get_moter_pawer")
    return int(moter_pawer)

def write_moter_pawer(moter_pawer) :
    mp = str(moter_pawer)
    with open("moter_pawer.txt" , "w") as mf :
        moter_pawer =mf.writelines(mp)
    return 0

def get_moter_fix() :
    with open("moter_fix.txt" , "r") as mf :
        moter_fix =mf.read()
    print("get_moter_fix")
    return moter_fix

def write_moter_fix(moter_fix) :
    mf = str(moter_fix)
    with open("moter_fix.txt" , "w") as mf :
        moter_fix =mf.writelines(mf)
    return 0

#メイン処理
def main():
    moter_init()
    getch = _Getch()
    
    moter_pawer = 80
    fix = [1.0,1.0]
    
    while True:
        key = getch()
        if key == "w":
            go(0, max = moter_pawer, l_fix = fix[0], r_fix = fix[1])
        if key == "q":
            l_pawer = moter_pawer*0.75
            left(0,l_pawer,rate = 0.3, l_fix = fix[0], r_fix = fix[1])
        if key == "e":
            r_pawer = moter_pawer*0.75
            right(0,r_pawer,rate = 0.3,l_fix = fix[0], r_fix = fix[1])
        if key == "a":
            l_pawer = moter_pawer*0.75
            left(0,l_pawer, rate = -1,l_fix = fix[0], r_fix = fix[1])
        if key == "d":
            r_pawer = moter_pawer*0.75
            right(0,r_pawer, rate = -1,l_fix = fix[0], r_fix = fix[1])
        if key == "s":
            b_pawer = moter_pawer*0.50
            back(0,b_pawer,l_fix = fix[0], r_fix = fix[1])
        if key == ",":
            stop()
            fix = calibrate_fix(fix[0],fix[1],-1)
            print(fix)
        if key == ".":
            stop()
            fix = calibrate_fix(fix[0],fix[1],1)
            print(fix)
        if key == "k":
            stop()
            moter_pawer = pawer_fix(moter_pawer,-1)
            print(moter_pawer)
        if key == "l":
            stop()
            moter_pawer = pawer_fix(moter_pawer,1)
            print(moter_pawer)
        elif key == " ":
            stop()
        elif key == "x":
            break
        
if __name__ == '__main__':
    try :
        main()
    finally :
        stop()
