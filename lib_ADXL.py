#ADXL375用3軸加速度取得プログラム

#モジュールインポート
import time
import datetime
import board
import adafruit_adxl37x

#インスタンス設定
i2c = board.I2C()  # uses board.SCL and board.SDA
accelerometer = adafruit_adxl37x.ADXL375(i2c)

#メイン処理
def ADXLlog_main() :
    
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
    ADXLlog_main()