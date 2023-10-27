import numpy as np
import matplotlib.pyplot as plt
import sys
import csv

if len(sys.argv) < 2:
    print("引数が指定されてないです")
    sys.exit(1)

try:
    print(sys.argv[1][1])

    param_value = float(sys.argv[1])
except ValueError:
    print("引数は数値である必要があリマス")
    sys.exit(1)


# CSVファイルのパス
# csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/particle_coodinate_distribution_time_stamp_gazebo_down_sample_distance{param_value}.csv"
csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/20230826_boxel_nasi_{param_value}_0.0_0.0.csv"

# /home/hirayama-d/research_ws/src/particlefilter_simulation_basic/particle_coodinate_distribution_time_stamp_gazebo_down_sample_8.1.csv
# CSVファイルを読み込んでリストに格納
data = []
with open(csv_file_path, "r") as f:
    reader = csv.reader(f)
    for row in reader:
        data.append(row)

# 最初の400行を取得してnumpy配列に変換
data_np = np.array(data[:400],dtype=float)

x = data_np[:,1]
y = data_np[:,2]
value = data_np[:,4]*400
print(value)
print(type(x[0]),type(y),type(value))

# 3Dグラフの作成
# fig = plt.figure()
fig, ax = plt.subplots()

# x, y, valueのデータを散布図としてプロット
scatter = ax.scatter(y, x, c=value ,s=100,vmin=0.0, vmax=0.6,cmap='viridis')  # cにvalueを指定して、カラーマップをviridisに設定
# vmax=0.0015,
# カラーバーの追加
cbar = plt.colorbar(scatter)

# 軸ラベルの設定
ax.set_xlabel('Y')
ax.set_ylabel('X')
ax.set_title(f'downsampled_{param_value}')
ax.grid(linestyle='dotted')# ax.set_zlabel('Value')

# グラフの表示
plt.show()