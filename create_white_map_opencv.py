import cv2
import numpy as np

# 画像のサイズを指定
width = 662
height = 1263
   
# 白い背景の画像を作成
# image = np.full((height, width, 3), (65, 105, 225), dtype=np.uint8)
image = np.full((height, width, 3), (0, 0, 0), dtype=np.uint8)

# 矩形の座標を辞書型で定義
rect_dict = {0: [[246.5, 350.5], [254.0, 350.5], [254.0, 1263.0], [246.5, 1263.0]],
             1: [[107.5, 350.5], [246.5, 350.5], [246.5, 358.0], [107.5, 357.5]],
             2: [[107.5, 242.5], [115.0, 242.5], [115.0, 350.5], [107.5, 350.5]],
             3: [[0.0, 242.5], [107.5, 242.5], [107.5, 250.0], [0.0, 250.0]],
             4: [[0.0, 127.5], [125.0, 127.5], [125.0, 135.0], [0.0, 135.0]],
             5: [[117.5, 0.0], [125.0, 0.0], [125.0, 127.5], [117.5, 127.5]],
             6: [[219.0, 0.0], [226.0, 0.0], [226.0, 122.0], [219.0, 122.0]],
             7: [[226.0, 114.5], [662.0, 114.5], [662.0, 122.0], [226.0, 122.0]],
             8: [[355.5, 253.5], [662.0, 253.5], [662.0, 261.0], [355.5, 261.0]],
             9: [[348.0, 253.5], [355.5, 253.5], [355.5, 1263.0], [348.0, 1263.0]]
             }

# 矩形を白色で塗りつぶし
for key in rect_dict.keys():
    rect = np.array([rect_dict[key]], dtype=np.int32)
    cv2.fillPoly(image, rect, (255, 255, 255))
    # cv2.putText(image, str(key), tuple(map(int, rect_dict[key][0])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

# 画像内で白情報のcv座標を取得
print(image.shape)
h, w,c = image.shape
print(h, w,c)
print(type(image))
print(image[0,0])
# 白色ピクセルの座標を格納するリスト
white_pixels = []

for y in range(height):
    for x in range(width):
        b, g, r = image[y,x]
        if b == 255 and g == 255 and r == 255:
            white_pixels.append((x,y))

 
print(image[1262,355])
# for coodinate in white_pixels:
#     print(coodinate)

# 画像を表示
cv2.imshow("SB_WhiteLaneMap", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("sb_white_lane_map.png", image)
