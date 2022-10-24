#coding: utf-8
#パラシュート分離機構（ニクロム線）動作確認用プログラム
#対応ハード：ニクロム線

#モジュールインポート
import pigpio
import time 

#電熱線ピン番号
CAREER_CUT = 6
#インスタンス設定
pi = pigpio.pi()

#電熱線のピンをOFFにする
pi.set_mode(CAREER_CUT, pigpio.OUTPUT)
pi.write(CAREER_CUT , 0)

#メイン処理
def career_cat(c_time) :
    print("点火")
    pi.write(CAREER_CUT , 1 )
    time.sleep(c_time)
    pi.write(CAREER_CUT , 0 )
    time.sleep(1)

if __name__ == "__main__":
    career_cat(2)
