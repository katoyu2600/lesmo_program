#coding: utf-8
#raspberrypi camera v2動作確認用プログラム
#対応ハード：raspberrypi camera v2
#取扱説明書：https://jp.rs-online.com/web/p/raspberry-pi-cameras/9132664

#モジュールインポート
import time
import picamera #カメラモジュール用

#メイン処理
def ca_main(camera_count):
    i = camera_count
    pic_name ="redcorn"+str(i)+ ".jpg"
    # カメラ初期化
    camera  = picamera.PiCamera()
    # 解像度の設定
    camera.resolution = (640, 480)
    # 撮影の準備
    camera.start_preview()
    # 準備している間、少し待機する
    time.sleep(0)
    # 撮影して指定したファイル名で保存する
    camera.capture(pic_name)
    camera.stop_preview()
    camera.close()

if __name__ == '__main__':
    a=input("ここに数字を入れてください")
    ca_main(int(a))