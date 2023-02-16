# lesmo_program

# 概要

SARD 10期 CanSat LESMO搭載プログラム

ローバータイプCanSat

![S__27041804](https://user-images.githubusercontent.com/111445830/219353064-52dc7994-42f5-46bc-9cfc-0b46829aba06.jpg)

# 作成者

SARD　10期　開発

加藤裕也

# 出場大会

2021年度 種子島ロケットコンテスト

　結果：オンライン開催のため投下機会無し

2022年度 能代宇宙イベント

　結果：8.3ｍ（室内投下）

2022年度 ARLISS

　結果：1.3km(公式記録：着地後分離成功，ゴールに向かって100ｍ走行後停止)

　　　　10ｍ（非公式記録：着地後再起動）

# 動作環境

たしかraspberrypiのバスター版

# CanSat搭載機器

マイコン　　　　 Raspberry pi ZERO WH

カメラ　　　　　 Raspberrypi camera v2

気温気圧センサ　 AE-bme280 (i2c)

9軸センサ　　　　MPU9250 (i2c)

GPSモジュール　　xa1110(i2c)

照度センサー

ADコンバータ

通信モジュール　IM920ｃ(UART)

その他もろもろ

![49CC32F6-EEA5-4975-9EEB-610F12B9AE30](https://user-images.githubusercontent.com/111445830/219353980-86019337-b981-4d72-954a-e1868b2688a2.jpg)


# 使用方法

１．ファイルをすべてダウンロードし，raspberry pi のpi配下に置く．

２．必要なpythonライブラリーをダウンロードする．

３．set_up.pyを実行し必要な設定を行う．

４．キャリアにCanSatの収納を確認次第main.pyを実行．

５．競技開始

# ライセンス

特にないけど
ここに連絡してちょ

https://twitter.com/okleabwstlea
