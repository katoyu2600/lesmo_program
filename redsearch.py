#赤色認識プログラム

import cv2
import numpy as np

#メイン処理
def red_search(camera_count):
    for i in range(1):
        c = camera_count
        x = 0
        y = 0
        pic_name="redcorn"+str(c)+ ".jpg"
        pic_name_out="redcorn"+str(c)+ "out.jpg"
        gray_pic="redcorn"+str(c)+ "gray.jpg"
        #画像読み込み
        img = cv2.imread(pic_name,cv2.IMREAD_COLOR)
        #画像を180°回転
        img = cv2.rotate(img, cv2.ROTATE_180)
        #画像の平滑化
        img = cv2.GaussianBlur(img, (9, 9), 3)
        height, width = img.shape[:2]
        #画像のHSV化
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        img_redcorn=np.zeros((height,width,3),np.uint8)
        #画像の2値化
        img_redcorn[(h <255) & (h > 180) & (s > 60)& (v > 60)] = 255 #! (h <255) & (h > 190)の値を変えることによって微妙な赤色の変化に対応できます！
        #画像を出力
        cv2.imwrite(gray_pic,np.array(img_redcorn))
        img_gray = cv2.imread(gray_pic,cv2.IMREAD_GRAYSCALE)
        #モーメントを取得
        M = cv2.moments(img_gray, False)
        #輪郭抽出
        contours, hierarchy = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #図心を出す
        x,y= int(M["m10"]/(M["m00"]+1)) , int(M["m01"]/(M["m00"]+1))
        #Zerodivision_kaihi
        #図心を画像に記載する
        cv2.circle(img, (x,y), 20, 100, 2, 4)
        cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
        cv2.imwrite(pic_name_out,np.array(img))
        out = [x , y]
        print(out)
        return out


if __name__ == '__main__' :
    red_search(777)