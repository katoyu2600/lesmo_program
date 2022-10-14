# SPDX-FileCopyrightText: Copyright (c) 2022 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import datetime
import board
import adafruit_adxl37x
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
accelerometer = adafruit_adxl37x.ADXL375(i2c)

    #[Ax, Ay, Az] =accelerometer.acceleration
    #with open('ADXLlog.txt','a'encoding='UTF-8')as f:
        #writer = csv.writer(f)
        #writer.writerow(['{0:0.1f}'.format(dt_now),'{0:0.1f}'.format(Ax),'{0:0.1f}'.format(Ay),'{0:0.1f}'.format(Az)])

def ADXLlogmain() :
    
    logname = ["dt_now",",","accel_x" ,",","accel_y",",","accel_z" ]
    f0 = open('ADXLlog.txt', 'w', encoding='UTF-8')
    
    f0.writelines(logname)
    f0.close()
    log_count = 0
    
    while  log_count <= 10000 :
        Ax, Ay, Az = accelerometer.acceleration
        accel_log = [Ax, Ay, Az]
        dt_now    = str(datetime.datetime.now())
        f = open('ADXLlog.txt', 'a', encoding='UTF-8')
        f.writelines(dt_now,",",accel_log)
        f.writelines("\n")
        f.close()
        log_count = log_count + 1
        time.sleep(0.05)

if __name__ == '__main__' :
    ADXLlogmain()
