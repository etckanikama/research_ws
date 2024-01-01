import cv2
import numpy as np

# 画像のサイズを指定
width = 662
height = 1263
# height = 6100
# 白い背景の画像を作成
image = np.full((height, width, 3), (65, 105, 225), dtype=np.uint8)
# image = np.full((height, width, 3), (0, 0, 0), dtype=np.uint8)

# 矩形の座標を辞書型で定義
rect_dict = {   0: [[246.5, 350.5], [254.0, 350.5], [254.0, 1263.0], [246.5, 1263.0]],
                1: [[107.5, 350.5], [246.5, 350.5], [246.5, 358.0], [107.5, 357.5]],
                2: [[107.5, 242.5], [115.0, 242.5], [115.0, 350.5], [107.5, 350.5]],
                3: [[0.0, 242.5], [107.5, 242.5], [107.5, 250.0], [0.0, 250.0]],
                4: [[0.0, 127.5], [125.0, 127.5], [125.0, 135.0], [0.0, 135.0]],
                5: [[117.5, 0.0], [125.0, 0.0], [125.0, 127.5], [117.5, 127.5]],
                6: [[219.0, 0.0], [226.0, 0.0], [226.0, 122.0], [219.0, 122.0]],
                7: [[226.0, 114.5], [662.0, 114.5], [662.0, 122.0], [226.0, 122.0]],
                8: [[355.5, 253.5], [662.0, 253.5], [662.0, 261.0], [355.5, 261.0]],
                9: [[348.0, 253.5], [355.5, 253.5], [355.5, 1263.0], [348.0, 1263.0]],
                10: [[447.5, 1208.5], [662.0, 1208.5], [662.0, 1216.0], [447.5, 1216.0]], #間隔80cmにした：厳密には違うかも
                11: [[447.5, 1128.5], [455.0, 1128.5], [455.0, 1208.5], [447.5, 1208.5]],
                # 10: [[447.5, 1163.5], [662.0, 1163.5], [662.0, 1171.0], [447.5, 1171.0]], 厳密には違うかもしれない
                # 11: [[447.5, 1128.5], [455.0, 1128.5], [455.0, 1163.5], [447.5, 1163.5]],
                12: [[447.5, 1121.0], [662.0, 1121.0], [662.0, 1128.5], [447.5, 1128.5]],
                13: [[447.5, 1030.5], [662.0, 1030.5], [662.0, 1038.0], [447.5, 1038.0]],
                14: [[447.5, 937.0], [455.0, 937.0], [455.0, 1030.5], [447.5, 1030.5]],
                15: [[447.5, 929.5], [662.0, 929.5], [662.0, 937.0], [447.5, 937.0]],
16: [[447.5, 840.5], [662.0, 840.5], [662.0, 848.0], [447.5, 848.0]],
17: [[447.5, 745.5], [455.0, 745.5], [455.0, 840.5], [447.5, 840.5]],
18: [[447.5, 738.0], [662.0, 738.0], [662.0, 745.5], [447.5, 745.5]],
19: [[447.5, 650.5], [662.0, 650.5], [662.0, 658.0], [447.5, 658.0]],
20: [[447.5, 554.5], [455.0, 554.5], [455.0, 650.5], [447.5, 650.5]],
21: [[447.5, 547.0], [662.0, 547.0], [662.0, 555.0], [447.5, 555.0]],
22: [[447.5, 459.5], [662.0, 459.5], [662.0, 467.0], [447.5, 467.0]],
23: [[447.5, 332.5], [455.0, 332.5], [455.0, 457.5], [447.5, 457.5]],
24: [[447.5, 325.0], [662.0, 325.0], [662.0, 332.5], [447.5, 332.5]],
# 25: [[221.5, 1063.0], [271.5, 1063.0], [271.5, 1070.5], [221.5, 1070.5]]#片側に50cmの白線を配置してみる
# 25: [[171.5, 1063.0], [271.5, 1063.0], [271.5, 1070.5], [171.5, 1070.5]]#片側に100cmの白線を配置してみる
# 25: [[171.5, 1063.0], [430.5, 1063.0], [430.5, 1070.5], [171.5, 1070.5]],#めちゃくちゃ極端
# route1 
# 25: [[220.25, 386.8598901098902], [280.25, 386.8598901098902], [280.25, 394.3598901098901], [220.25, 394.3598901098901]],
# 26: [[220.25, 537.271978021978], [280.25, 537.271978021978], [280.25, 544.7719780219779], [220.25, 544.7719780219779]],
# 27: [[220.25, 938.3708791208792], [280.25, 938.3708791208792], [280.25, 945.8708791208792], [220.25, 945.8708791208792]],
# 28: [[321.75, 683.835], [381.75, 683.835], [381.75, 691.3349999999999], [321.75, 691.3349999999999]],
# 29: [[321.75, 825.1650000000001], [381.75, 825.1650000000001], [381.75, 832.665], [321.75, 832.665]],
# 30: [[321.75, 1027.065], [381.75, 1027.065], [381.75, 1034.565], [321.75, 1034.565]],
# 31: [[282.94767441860466, 88.25], [290.44767441860466, 88.25], [290.44767441860466, 148.25], [282.94767441860466, 148.25]],
# 32: [[545.2333333333333, 227.24999999999977], [552.7333333333333, 227.24999999999977], [552.7333333333333, 287.25], [545.2333333333333, 287.25]],
# 33: [[657.25, 227.24999999999977], [664.75, 227.24999999999977], [664.75, 287.25], [657.25, 287.25]],
# # route1逆走
# 25: [[220.25, 416.94230769230774], [280.25, 416.94230769230774], [280.25, 424.4423076923076], [220.25, 424.4423076923076]],
# 26: [[220.25, 547.2994505494506], [280.25, 547.2994505494506], [280.25, 554.7994505494505], [220.25, 554.7994505494505]],
# 27: [[220.25, 687.6840659340659], [280.25, 687.6840659340659], [280.25, 695.1840659340659], [220.25, 695.1840659340659]],
# 28: [[220.25, 818.0412087912089], [280.25, 818.0412087912089], [280.25, 825.5412087912086], [220.25, 825.5412087912086]],
# 29: [[220.25, 1118.8653846153845], [280.25, 1118.8653846153845], [280.25, 1126.3653846153845], [220.25, 1126.3653846153845]],
# 30: [[220.25, 1259.25], [280.25, 1259.25], [280.25, 1266.75], [220.25, 1266.75]],
# 31: [[81.25000000000003, 314.35], [141.25, 314.35], [141.25, 321.85], [81.25000000000003, 321.85]],
# 32: [[321.75, 976.5899999999999], [381.75, 976.5899999999999], [381.75, 984.09], [321.75, 984.09]],
# 33: [[293.0639534883721, 88.25], [300.5639534883721, 88.25], [300.5639534883721, 148.25], [293.0639534883721, 148.25]],
# 34: [[394.2267441860465, 88.25], [401.7267441860465, 88.25], [401.7267441860465, 148.25], [394.2267441860465, 148.25]],

# # route1 and roote1逆走の統合版
# 25: [[220.25, 838.0961538461538], [280.25, 838.0961538461538], [280.25, 845.5961538461538], [220.25, 845.5961538461538]],
# 26: [[220.25, 1128.892857142857], [280.25, 1128.892857142857], [280.25, 1136.392857142857], [220.25, 1136.392857142857]],
# 27: [[220.25, 1259.25], [280.25, 1259.25], [280.25, 1266.75], [220.25, 1266.75]],
# 28: [[81.25000000000003, 314.35], [141.25, 314.35], [141.25, 321.85], [81.25000000000003, 321.85]],
# 29: [[321.75, 401.17500000000007], [381.75, 401.17500000000007], [381.75, 408.67499999999995], [321.75, 408.67499999999995]],
# 30: [[321.75, 532.4100000000001], [381.75, 532.4100000000001], [381.75, 539.91], [321.75, 539.91]],
# 31: [[321.75, 683.835], [381.75, 683.835], [381.75, 691.3349999999999], [321.75, 691.3349999999999]],
# 32: [[321.75, 986.685], [381.75, 986.685], [381.75, 994.185], [321.75, 994.185]],
# 33: [[313.29651162790697, 88.25], [320.79651162790697, 88.25], [320.79651162790697, 148.25], [313.29651162790697, 148.25]],
# 34: [[566.203488372093, 88.25], [573.703488372093, 88.25], [573.703488372093, 148.25], [566.203488372093, 148.25]],
# 35: [[443.4, 227.24999999999977], [450.9, 227.24999999999977], [450.9, 287.25], [443.4, 287.25]],
# 36: [[657.25, 227.24999999999977], [664.75, 227.24999999999977], [664.75, 287.25], [657.25, 287.25]],

# 俺が作った1.5mの間隔
25: [[651.75, 227.25], [659.25, 227.25], [659.25, 287.2500000000001], [651.75, 287.2500000000001]],
26: [[220.25, 959.25], [280.25, 959.25], [280.25, 966.75], [220.25, 966.75]],
27: [[321.75, 659.25], [381.75, 659.25], [381.75, 666.75], [321.75, 666.75]],
28: [[321.75, 959.25], [381.75, 959.25], [381.75, 966.75], [321.75, 966.75]],
29: [[321.75, 509.25], [381.75, 509.25], [381.75, 516.75], [321.75, 516.75]],
30: [[220.25, 1109.25], [280.25, 1109.25], [280.25, 1116.75], [220.25, 1116.75]],
31: [[321.75, 1109.25], [381.75, 1109.25], [381.75, 1116.75], [321.75, 1116.75]],
32: [[372.25, 85.5], [379.75, 85.5], [379.75, 145.5], [372.25, 145.5]],
33: [[522.25, 85.5], [529.75, 85.5], [529.75, 145.5], [522.25, 145.5]],
34: [[501.75, 227.25], [509.25, 227.25], [509.25, 287.2500000000001], [501.75, 287.2500000000001]],
35: [[321.75, 359.25], [381.75, 359.25], [381.75, 366.75], [321.75, 366.75]],
36: [[220.25, 659.25], [280.25, 659.25], [280.25, 666.75], [220.25, 666.75]],
37: [[220.25, 509.25], [280.25, 509.25], [280.25, 516.75], [220.25, 516.75]],
38: [[220.25, 809.25], [280.25, 809.25], [280.25, 816.75], [220.25, 816.75]],
39: [[321.75, 809.25], [381.75, 809.25], [381.75, 816.75], [321.75, 816.75]],


# 1.5m間隔で適当に配置したポリゴン（被覆問題ではない）
# 25: [[220.25, 574.875], [280.25, 574.875], [280.25, 582.375], [220.25, 582.375]],
# 26: [[220.25, 803.0], [280.25, 803.0], [280.25, 810.5], [220.25, 810.5]],
# 27: [[220.25, 1031.125], [280.25, 1031.125], [280.25, 1038.625], [220.25, 1038.625]],
# 28: [[220.25, 1259.25], [280.25, 1259.25], [280.25, 1266.75], [220.25, 1266.75]],
# 29: [[91.25000000000003, 123.75], [151.25000000000003, 123.75], [151.25000000000003, 131.25], [91.25000000000003, 131.25]],
# 30: [[192.25, 118.25], [252.25, 118.25], [252.25, 125.75], [192.25, 125.75]],
# 31: [[439.75, 88.25], [447.25, 88.25], [447.25, 148.25], [439.75, 148.25]],
# 32: [[657.25, 88.25], [664.75, 88.25], [664.75, 148.25], [657.25, 148.25]],
# 33: [[657.25, 227.24999999999977], [664.75, 227.24999999999977], [664.75, 287.25], [657.25, 287.25]],
# 34: [[321.75, 451.65], [381.75, 451.65], [381.75, 459.14999999999986], [321.75, 459.14999999999986]],
# 35: [[321.75, 653.55], [381.75, 653.55], [381.75, 661.05], [321.75, 661.05]],
# 36: [[321.75, 855.45], [381.75, 855.45], [381.75, 862.9499999999999], [321.75, 862.9499999999999]],
# 37: [[321.75, 1057.35], [381.75, 1057.35], [381.75, 1064.85], [321.75, 1064.85]],
# 38: [[321.75, 1259.25], [381.75, 1259.25], [381.75, 1266.75], [321.75, 1266.75]],
# 39: [[658.25, 1182.25], [665.75, 1182.25], [665.75, 1242.25], [658.25, 1242.25]],
# 40: [[658.25, 1094.75], [665.75, 1094.75], [665.75, 1154.75], [658.25, 1154.75]],
# 41: [[658.25, 1004.25], [665.75, 1004.25], [665.75, 1064.25], [658.25, 1064.25]],
# 42: [[658.25, 903.25], [665.75, 903.25], [665.75, 963.25], [658.25, 963.25]],
# 43: [[658.25, 814.25], [665.75, 814.25], [665.75, 874.25], [658.25, 874.25]],
# 44: [[658.25, 711.75], [665.75, 711.75], [665.75, 771.75], [658.25, 771.75]],
# 45: [[658.25, 624.25], [665.75, 624.25], [665.75, 684.25], [658.25, 684.25]],
# 46: [[658.25, 521.0], [665.75, 521.0], [665.75, 581.0], [658.25, 581.0]],
# 47: [[658.25, 433.2499999999999], [665.75, 433.2499999999999], [665.75, 493.2499999999999], [658.25, 493.2499999999999]],
# 48: [[658.25, 298.7499999999998], [665.75, 298.7499999999998], [665.75, 358.75], [658.25, 358.75]],


             }
# rect_dict = {0: [[246.5, 100.0], [254.0, 100.0], [254.0, 6100.0], [246.5, 6100.0]],
#              1: [[0.0, 100.0], [246.5, 100.0], [246.5, 107.5], [0.0, 107.5]],
#              2: [[0.0, 0.0], [662.0, 0.0], [662.0, 7.5], [0.0, 7.5]],
#              3: [[355.5, 100.0], [662.0, 100.0], [662.0, 107.5], [355.5, 107.5]],
#              4: [[348.0, 100.0], [355.5, 100.0], [355.5, 6100.0], [348.0, 6100.0]]
# }
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


# # 矩形の切り抜き
# start_x = 100
# start_y = 500
# camera_range_height = 200
# camera_range_width = 300
# cropped_image = image[start_y:start_y+camera_range_height, start_x:start_x+camera_range_width]
# cv2.imshow("cropped_image",cropped_image)
# print(image[1262,355])

# for coodinate in white_pixels:
#     print(coodinate)

# 画像を表示
# cv2.imshow("SB_WhiteLaneMap", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# cv2.imwrite("after_set_covering_route1_and_route1_reverse.png", image)
cv2.imwrite("white_map_oregatukutta_1.5.png",image)