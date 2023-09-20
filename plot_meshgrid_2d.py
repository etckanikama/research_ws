import numpy as np
import matplotlib.pyplot as plt
import sys
import csv
from scipy.interpolate import griddata

if len(sys.argv) < 4:
    print("引数が指定されてないです")
    sys.exit(1)

try:
    param_value1 = float(sys.argv[1])
    param_value2 = float(sys.argv[2])
    param_value3 = float(sys.argv[3])
except ValueError:
    print("引数は数値である必要があリマス")
    sys.exit(1)

# CSVファイルのパス
csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/particle_coodinate_distribution_time_stamp_gazebo_nashi_distance{param_value1}_{param_value2}_{param_value3}.csv"

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

# グリッドデータの作成
grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]
grid_value = griddata((x, y), value, (grid_x, grid_y), method='cubic')

# 2Dヒートマップの作成
fig, ax = plt.subplots()
c = ax.imshow(grid_value, extent=(min(x), max(x), min(y), max(y)), origin='lower', aspect='auto', cmap='viridis')

# カラーバーの追加
fig.colorbar(c, ax=ax)

# 軸ラベルの設定
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title(f'downsampled_{param_value1}_{param_value2}_{param_value3}')

# グラフの表示
plt.show()
