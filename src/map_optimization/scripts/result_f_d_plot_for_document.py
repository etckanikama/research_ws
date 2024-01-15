import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import ast 
import math

"""
fx,fyを分割した地図と、dx,dy,dxdyを同時にプロットするプログラム
内容:fx,fyは固定

dは: csvファイルを書き換えて実行する
①地図1.5

 ⇛route1, route2
 ⇛file_pathで地図/
"""
# fx,fyは固定パス-----------------------------------------------------------------------------------------------
Fx_file_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/Fx.text"
Fy_file_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/Fy.text"
# Fx と Fy の辞書を読み込む
with open(Fx_file_path, "r") as file:
    data = file.read()
    Fx = ast.literal_eval(data[data.find('{'):])

with open(Fy_file_path, "r") as file:
    data = file.read()
    Fy = ast.literal_eval(data[data.find('{'):])
# ---------------------------------------------------------------------------------------------------------------


# 元の地図：reoute1:path1
file_path = "/home/hirayama-d/research_ws/src/sim/20240101_original_route1/path_route1/csv/output_eigen_path_route1.csv"
# 元の地図：reoute1_reverse:path2
# file_path = "/home/hirayama-d/research_ws/src/sim/20240101_original_route1/path_route1_reverse/csv/output_eigen_path_route1_reverse.csv"

thresh = 0.28
# 辞書型配列の作成
data = pd.read_csv(file_path)
# 分散の値にルートを適用
data['sqrt_variance_x'] = np.sqrt(data['variance_x'])
data['sqrt_variance_y'] = np.sqrt(data['variance_y'])
# 条件に基づいて色を割り当て
conditions = [
    (data['sqrt_variance_x'] > thresh) & (data['sqrt_variance_y'] <= thresh),  # 青色条件:xの標準偏差"だけ"0.27より大きい⇛xの候補地
    (data['sqrt_variance_y'] > thresh) & (data['sqrt_variance_x'] <= thresh),  # 赤色条件:yの標準偏差"だけ"0.27より大きい⇛yの候補地
    (data['sqrt_variance_x'] <= thresh) & (data['sqrt_variance_y'] <= thresh), # 緑色条件:x,yの標準偏差両方とも0.27より小さい⇛候補地ではない
    (data['sqrt_variance_x'] > thresh) & (data['sqrt_variance_y'] > thresh)    # その他（黒色）
]
choices = ['blue', 'red', 'green', 'black']
data['color'] = np.select(conditions, choices, default='black')

# 辞書型配列の作成
d = {}
for index, row in data.iterrows():
    label = ""
    if row['color'] == 'blue':
        label = 'x-long'
    elif row['color'] == 'red':
        label = 'y-long'
    elif row['color'] == 'green':
        label = 'xy-short'
    else:  # black またはその他の色
        label = 'other'
    d[f'd{index}'] = (row['x'], row['y'],row['yaw'], label)

# # 結果の表示
# for key, value in d.items():
#     print(f"{key}: {value}")


########################### プロット###################################################
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)

# 全ての Fx と Fy の点をプロット------------------------------------------------------
for key, value in Fx.items():
    plt.plot(value[1], value[0], 'yo')
for key, value in Fy.items():
    plt.plot(value[1], value[0], 'mo')  # 'ro' means red color, circle marker
# ------------------------------------------------------------------------------------

# 
# for color in choices:
#     subset = data[data['color'] == color]
#     # plt.plot(subset['x'], subset['y'], 'o', color=color, label=f'{color} points')
#     plt.plot(subset['y'], subset['x'], 'o', color=color)

# # 矢印の長さ
arrow_length = 0.01

for color in choices:
    subset = data[data['color'] == color]
    for _, row in subset.iterrows():
        dx = arrow_length * math.cos(row['yaw'])
        dy = arrow_length * math.sin(row['yaw'])
        plt.arrow(row['y'], row['x'], dy, dx, color=color, head_width=0.1, head_length=0.1)


plt.yticks(range(0,15,1))
plt.xticks(range(-4,4,1))
ax.set_aspect('equal')
ax.invert_xaxis()
plt.title(f'{file_path}')
plt.xlabel("y [m]")
plt.ylabel("x [m]")
plt.grid(linestyle='dotted')
plt.show()


