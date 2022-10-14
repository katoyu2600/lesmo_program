
import spidev 
import time
V_REF = 3.29476 # input Voltage
CHN = 0 # 接続チャンネル

spi = spidev.SpiDev()
spi.open(0, 0) # 0：SPI0、0：CE0
spi.max_speed_hz = 1000000 # 1MHz SPIのバージョンアップによりこの指定をしないと動かない

def get_voltage():
    dout = spi.xfer2([((0b1000+CHN)>>2)+0b100,((0b1000+CHN)&0b0011)<<6,0]) # Din(RasPi→MCP3208）を指定
    bit12 = ((dout[1]&0b1111) << 8) + dout[2] # Dout（MCP3208→RasPi）から12ビットを取り出す
    volts = round((bit12 * V_REF) / float(4095),4)  # 取得した値を電圧に変換する（12bitなので4095で割る）
    return volts # 電圧を返す

def stop_spi() :
    spi.close()

if __name__ == '__main__' :
    try :
        count = 0
        while count < 1000 :
            print(get_voltage())
            time.sleep(0.1)
            count = count + 1

    finally :
        stop_spi()

