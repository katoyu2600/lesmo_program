# lesmo_program(CanSat_program)

# 概要

SARD 10期 CanSat LESMO搭載プログラム

ローバータイプCanSat
![CE92EB07-D1E2-4289-B9B9-05D0CFCB7E15](https://user-images.githubusercontent.com/111445830/219361996-b047609e-4d12-4588-9974-cbdfcb6cfe80.jpg)

# 作成者

SARD　10期　開発

加藤裕也

# 出場大会

・2021年度 種子島ロケットコンテスト

　結果：オンライン開催のため投下機会無し

・2022年度 能代宇宙イベント

　結果：8.3ｍ（室内投下）

・2022年度 ARLISS

　結果：1.3km(公式記録：着地後分離成功，ゴールに向かって100ｍ走行後停止)

　　　　10ｍ（非公式記録：着地後再起動）

# 動作環境

・使用OS

たしかrasbianのバスター版

・使用pythonライブラリ


＜標準ライブラリ＞

    time

    math

    sys

  ＜その他ライブラリ＞

    datetime

    numpy

    pigpio

    cv2(Open CV)

    smbus2

    serial(piserial)

    spidev   
    
    qwiic_titan_gps

    picamera ※raspberrypi OS 最新版では使用が不可能な可能性がありlibcamera,picamera2等の代替ライブラリへの乗り換えを推奨する



# CanSat搭載機器

マイコン :　　　　 Raspberry pi ZERO WH

カメラ :　　　　　 Raspberrypi camera v2

気温気圧センサ :　 AE-bme280 (i2c)

9軸センサ :　　　　MPU9250 (i2c)

GPSモジュール :　　GTOPxa1110 (i2c) or AE-GYSFDMAXB (UART) 

照度センサ :  　　　　   GL5528

ADコンバータ :   　MCP3208-CI/P

通信モジュール :　IM920ｃ(UART) 

レギュレータ（5.0V）:　M78AR05-1

レギュレータ（3.3V） :　 OKI-78SR-3.3/1.5-W36-C

その他必要部品あり

![49CC32F6-EEA5-4975-9EEB-610F12B9AE30](https://user-images.githubusercontent.com/111445830/219353980-86019337-b981-4d72-954a-e1868b2688a2.jpg)

# 使用方法

１．ファイルをすべてダウンロードし，raspberry pi のpi配下に置く．

２．必要なpythonライブラリをダウンロード，インストールする．

３．set_up.pyを実行し必要な設定を行う．

４．キャリアにCanSatの収納を確認次第main.pyを実行．

５．競技開始

# ライセンス

特にないけど
ここに連絡してちょ

https://twitter.com/okleabwstlea

# 詳細な説明・動作原理

# 欠陥

ちゃんと開いたファイルは閉じているはずなのに，OS err: too many open file err と出る．

どこかのファイルが開きっぱなしになってるっぽいがよくわからない( ; ; )

これさえ直れば．．．

・3/30 追記

閉じれていないファイルはi2c ，spi のデバイスファイルの可能性があったため，

デバイスファイルを閉じるようにプログラムを変更しました．
