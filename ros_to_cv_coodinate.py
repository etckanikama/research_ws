import cv2
import numpy as np

# 画像のサイズを指定
width  = 662
height = 1263 
# Max_Y_cv = 1263
# Max_X_cv = 662

# 画像を生成して茶色っぽい色で塗りつぶす
image = np.full((height, width, 3), (65, 105, 225), dtype=np.uint8)


# cvから見たros座標原点(x0,y0)
x0_cv = 301
y0_cv = 1263
# y0_cv = 6100


corner_num = 4
# polygon_num = 10
polygon_num = 26
# polygon_num = 5

# sb_mapの入力のros座標系([x_ros,y_ros])単位はセンチ
polygon_ros = { 0:[[912.5,54.5],[912.5,47.0],[0.0,47.0],[0.0,54.5]],
                1:[[912.5,193.5],[912.5,54.5],[905.0,54.5],[905.5, 193.5]],
                2:[[1020.5,193.5],[1020.5,186.0],[912.5,186.0],[912.5,193.5]],
                3:[[1020.5,301.0],[1020.5,193.5],[1013.0,193.5],[1013.0,301.0]],
                4:[[1135.5,301.0],[1135.5,176.0],[1128.0,176.0],[1128.0,301.0]],
                5:[[1263.0,183.5],[1263.0,176.0],[1135.5,176.0],[1135.5,183.5]],
                6:[[1263.0,82.0],[1263.0,75.0],[1141.0,75.0],[1141.0,82.0]],
                7:[[1148.5,75.0],[1148.5,-361.0],[1141.0,-361.0],[1141.0, 75.0]],
                8:[[1009.5,-54.5],[1009.5,-361.0],[1002.0,-361.0],[1002.0,-54.5]],
                9:[[1009.5,-47.0],[1009.5,-54.5],[0.0,-54.5],[0.0,-47.0]],
                # 10:[[99.5,-146.5],[99.5,-361.0],[92.0,-361.0],[92.0,-146.5]],
                # 11:[[134.5,-146.5],[134.5,-154.0],[99.5,-154.0],[99.5,-146.5]],
                10:[[54.5,-146.5],[54.5,-361.0],[47.0,-361.0],[47.0,-146.5]],
                11:[[134.5,-146.5],[134.5,-154.0],[54.5,-154.0],[54.5,-146.5]],
                12:[[142.0,-146.5],[142.0,-361.0],[134.5,-361.0],[134.5,-146.5]],
                13:[[232.5,-146.5],[232.5,-361.0],[225.0,-361.0],[225.0,-146.5]],
                14:[[326.0,-146.5],[326.0,-154.0],[232.5,-154.0],[232.5,-146.5]],
                15:[[333.5,-146.5],[333.5,-361.0],[326.0,-361.0],[326.0,-146.5]],
                16:[[422.5,-146.5],[422.5,-361.0],[415.0,-361.0],[415.0,-146.5]],
                17:[[517.5,-146.5],[517.5,-154.0],[422.5,-154.0],[422.5,-146.5]],
                18:[[525.0,-146.5],[525.0,-361.0],[517.5,-361.0],[517.5,-146.5]],
                19:[[612.5,-146.5],[612.5,-361.0],[605.0,-361.0],[605.0,-146.5]],
                20:[[708.5,-146.5],[708.5,-154.0],[612.5,-154.0],[612.5,-146.5]],
                21:[[716.0,-146.5],[716.0,-361.0],[708.0,-361.0],[708.0,-146.5]],
                22:[[803.5,-146.5],[803.5,-361.0],[796.0,-361.0],[796.0,-146.5]],
                23:[[930.5,-146.5],[930.5,-154.0],[805.5,-154.0],[805.5,-146.5]],
                24:[[938.0,-146.5],[938.0,-361.0],[930.5,-361.0],[930.5,-146.5]],
                25:[[200.0,129.5],[200.0,-129.5],[192.5,-129.5],[192.5,129.5]] #極端な白線を追加してみる
                # 25:[[200.0,129.5],[200.0,29.5],[192.5,29.5],[192.5,129.5]] #片側に100cmの白線を配置してみる
                # 25:[[200.0,79.5],[200.0,29.5],[192.5,29.5],[192.5,79.5]] #片側に50cmの白線を配置してみる
                }
# polygon_ros = {0:[[6000.0,54.5],[6000.0,47.0],[0.0,47.0],[0.0,54.5]],
#                1:[[6000.0,301.0],[6000.0,54.5],[5992.5,54.5],[5992.5, 301.0]],
#                2:[[6100.0,301.0],[6100.0,-361.0],[6092.5,-361.0],[6092.5,301.00]],
#                3:[[6000.0,-54.5],[6000.0,-361.0],[5992.5,-361.0],[5992.5, -54.5]],
#                4:[[6000.0,-47.0],[6000.0,-54.5],[0.0,-54.5],[0.0,-47.0]],}


# 出力先のcv座標系([x_cv,y_cv])
polygon_cv = {}


# polygon_ros[key][何個目の座標か][xかyか]
for i in range(polygon_num): #ポリゴンが10個あるから
    coodinate = []
    for j in range(corner_num): # 4つ角だから 
        x_cv = x0_cv - polygon_ros[i][j][1] #x_cv = x0_cv - y_ros
        y_cv = y0_cv - polygon_ros[i][j][0] #y_cv = y0_cv - x_ros
        coodinate.append([x_cv,y_cv])
    polygon_cv[i] = coodinate

# 左上から右上、右下そして左下の座標の順番
for i in range(polygon_num):
    print("{}: {}".format(i, polygon_cv[i]))
