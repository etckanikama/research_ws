import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import csv
from scipy.interpolate import griddata

if len(sys.argv) < 4:
    print("引数が指定されてないです")
    sys.exit(1)

try:
    # print(sys.argv[1][1])

    param_value1 = float(sys.argv[1])
    param_value2 = float(sys.argv[2])
    param_value3 = float(sys.argv[3])
    if param_value3 == 0.0:
        param_value3 = '0.00'
except ValueError:
    print("引数は数値である必要があリマス")
    sys.exit(1)

# CSVファイルのパス
# csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/particle_coodinate_distribution_time_stamp_gazebo_nashi_distance{param_value1}_{param_value2}_{param_value3}.csv"
# csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/20230826_boxel_nasi_{param_value1}_{param_value2}_{param_value3}.csv"
# csv_file_path = f"/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230826_boxel_nasi_2.5ikanomi_{param_value1}_{param_value2}_{param_value3}.csv"
# csv_file_path    = f"/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230905_boxel_nasi_2.0ikanomi_{param_value1}_{param_value2}_{param_value3}.csv"
# csv_file_path  = f"/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230930_boxel_nasi_640x360_2.0ikanomi_rsj{param_value1}_{param_value2}_{param_value3}.csv"
# csv_file_path = f"/home/hirayama-d/research_ws/src/sim/20231212_csv/20231212_boxel_nasi_640x360_2.0ikanomi_kairyou{param_value1}_{param_value2}_{param_value3}.csv"

csv_file_path = f"/home/hirayama-d/research_ws/src/sim/20231220_result/csv/20231220_boxel_nasi_640x360_2.0ikanomi_kairyou{param_value1}_{param_value2}_{param_value3}.csv"

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
value = data_np[:,4]
# print(type(x[0]),type(y),type(value))

# グリッドデータの作成
grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]  # 100j means 100 points
grid_value = griddata((x, y), value, (grid_x, grid_y), method='cubic')

# 3Dグラフの作成
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# サーフェスプロットの作成
surf = ax.plot_surface(grid_x, grid_y, grid_value, cmap='viridis')

# カラーバーの追加
fig.colorbar(surf)
ax.view_init(90, -90) #初期の起動場所を固定
# 軸ラベルの設定
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Value')
ax.set_title(f'x:{param_value1}, y:{param_value2}, yaw{param_value3}')
# plt.savefig(f"/home/hirayama-d/research_ws/src/sim/20231212_csv/output/likelyhood/likelyhood-Coodinate_x{param_value1}y{param_value2}theta{param_value3}.png")
plt.savefig(f"/home/hirayama-d/research_ws/src/sim/20231220_result/output/likelyhood/Coodinate_x{param_value1}y{param_value2}theta{param_value3}.png")

# グラフの表示
# plt.show()
