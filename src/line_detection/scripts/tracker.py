import cv2
import numpy as np

# 空の画像を作成（100x100ピクセルで各ピクセルが[0, 0, 255]）
img = np.zeros((100, 100, 3), dtype=np.uint8)
img[:] = [0, 0, 255]

def update_hue(val):
    hsv[:, :, 0] = val
    update_image()

def update_saturation(val):
    hsv[:, :, 1] = val
    update_image()

def update_value(val):
    hsv[:, :, 2] = val
    update_image()

def update_image():
    rgb_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    cv2.imshow("Dynamic HSV Adjustment", rgb_img)

# ウィンドウを作成
cv2.namedWindow("Dynamic HSV Adjustment")

# トラックバーを作成
cv2.createTrackbar("Hue", "Dynamic HSV Adjustment", 0, 179, update_hue)
cv2.createTrackbar("Saturation", "Dynamic HSV Adjustment", 0, 255, update_saturation)
cv2.createTrackbar("Value", "Dynamic HSV Adjustment", 0, 255, update_value)

# RGB画像をHSVに変換
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 初期表示
update_image()

# ウィンドウが閉じるまで待機
cv2.waitKey(0)
cv2.destroyAllWindows()
