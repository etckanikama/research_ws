# 1.5m間隔の座標を適当に配置
# orega_tukutta_hige = {(1.5,0.5075,'fx'),(3.0,0.5075,'fx'),(4.5,0.5075,'fx'),(6.0,0.5075,'fx'),(7.5,0.5075,'fx'),(9.0,0.5075,'fx'),
#      (1.5,-0.5075,'fx'),(3.0,-0.5075,'fx'),(4.5,-0.5075,'fx'),(6.0,-0.5075,'fx'),(7.5,-0.5075,'fx'),(9.0,-0.5075,'fx'),
#      (11.475,0.75,'fy'),(11.475,-0.75,'fy'),(11.475,-2.25,'fy'),
#      (10.0575,-0.545,'fy'),(10.0575,-2.045,'fy'),(10.0575,-3.545,'fy')}
orega_tukutta_hige = {(1.5,0.5075,'fx'),(3.0,0.5075,'fx'),(4.5,0.5075,'fx'),(6.0,0.5075,'fx'),(7.5,0.5075,'fx'),
     (1.5,-0.5075,'fx'),(3.0,-0.5075,'fx'),(4.5,-0.5075,'fx'),(6.0,-0.5075,'fx'),(7.5,-0.5075,'fx'),(9.0,-0.5075,'fx'),
     (11.475,-0.75,'fy'),(11.475,-2.25,'fy'),
     (10.0575,-2.045,'fy'),(10.0575,-3.545,'fy')}

# ここから、ポリゴンの形に整形する

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
                24:[[938.0,-146.5],[938.0,-361.0],[930.5,-361.0],[930.5,-146.5]],}

# 候補地の座標から白線ポリゴンの形にしてくれる
# ros_to_cv.py用のフォーマット
hige_length = 0.3
hige_thickness = 0.075/2

max_key = max(polygon_ros.keys())

for x, y, state in orega_tukutta_hige:
    max_key += 1
    if state == 'fx':
        y_max = y + hige_length
        y_min = y - hige_length
        x_max = x + hige_thickness
        x_min = x - hige_thickness
    elif state == 'fy':
        x_max = x + hige_length
        x_min = x - hige_length
        y_max = y + hige_thickness
        y_min = y - hige_thickness

    # 新しいポリゴンの座標を計算し、辞書に追加
    polygon_ros[max_key] = [[x_max * 100, y_max * 100], [x_max * 100, y_min * 100], [x_min * 100, y_min * 100], [x_min * 100, y_max * 100]]

# 結果を表示
print("ros_to_cv.py用のフォーマット")
for key, value in polygon_ros.items():
    print(f"{key}: {value},")