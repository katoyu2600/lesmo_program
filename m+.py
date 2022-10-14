#m.py
#モジュールインポート
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
LEFT_TOP_M1 = 17 # lt
LEFT_TOP_M2 = 4
RIGHT_TOP_M1 = 27 # rt
RIGHT_TOP_M2 = 22

#グローバル変数
pi = pigpio.pi()

#メイン処理
def main():
    moter_init()
    getch = _Getch()
    f=0
    
    while True:
        key = getch()
        
        if key == "w":
            go(0)
            f = 1
        if key == "a":
            left(0,75, rate = -1)
        if key == "z":
            left2(0)
        if key == "d":
            right(0,75, rate = -1)
        if key == "c":
            right2(0)    
        if key == "s":
            back(0,50)
        elif key == " ":
            if f == 1 :
                stop1(0,60)
            stop()
            f = 0
        elif key == "x":
            break

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

def step(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange1(0.0, max, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def stop():
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)

def stop1(start, max = 100.0, t = 1.0) :
    s = max / t / 10.0
    for duty in drange2(max, 0.0, s):
        pi.set_PWM_dutycycle(LEFT_TOP_M1, duty)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty)
        time.sleep(0.1)

def go(start, rmax = 100, lmax= 65 , t = 1.0):
    
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, rmax)
    pi.set_PWM_dutycycle(LEFT_TOP_M1, lmax)


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
        if start:
            s = max / t / 10.0
            for duty in drange1(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
    else:
        r = max
        l = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange1(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M2, l)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M2, l)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)

def right(start, max = 100.0, t = 1.0, rate = 1.0):
    if rate >= 0:
        l = max
        r = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange1(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
    else:
        l = max
        r = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
        if start:
            s = max / t / 10.0
            for duty in drange1(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M2, r)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, r)
            
def right2(start, rmax = 0, lmax= 75 , t = 1.0):
        
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, rmax)
    pi.set_PWM_dutycycle(LEFT_TOP_M1, lmax)
        
def left2(start, rmax = 75, lmax= 0 , t = 1.0):
    
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, rmax)
    pi.set_PWM_dutycycle(LEFT_TOP_M1, lmax)

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

if __name__ == '__main__':
    main()
