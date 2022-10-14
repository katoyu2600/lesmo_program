import pigpio
import time 

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

if __name__ == "__main__":
    career_cat(2)
