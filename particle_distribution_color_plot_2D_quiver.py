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

csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/20230826_boxel_nasi_{param_value}_0.0_0.0.csv"

data = []
with open(csv_file_path, "r") as f:
    reader = csv.reader(f)
    for row in reader:
        data.append(row)

data_np = np.array(data[:400],dtype=float)

x = data_np[:,1]
y = data_np[:,2]
value = data_np[:,4]*400

fig, ax = plt.subplots()

# quiverを使用して矢印をプロット。矢印の方向や大きさは単なる表示用であり、データに依存しない。
U = np.zeros_like(x)  # 矢印のx方向 (固定)
V = np.ones_like(y)  # 矢印のy方向 (固定)
norm = plt.Normalize(value.min(), value.max())
quiver = ax.quiver(y, x, U, V, angles='xy', scale_units='xy', scale=10.5, color=plt.cm.jet(norm(value)),width=0.05,headwidth=0.8, headlength=3)

# カラーバーの追加
cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap='jet'), ax=ax)

ax.set_xlabel('Y')
ax.set_ylabel('X')
ax.invert_xaxis()
ax.set_title(f'downsampled_{param_value}')
ax.grid(linestyle='dotted')

plt.show()
