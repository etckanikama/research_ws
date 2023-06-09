import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d   
import csv

# CSVファイルのパス
csv_file_path = "/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/particle_coodinate_distribution_time_stamp_gazebo_down_sample_7.0.csv"
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
value = data_np[:,4]
print(type(x[0]),type(y),type(value))


fig = plt.figure()
ax = fig.gca(projection='3d')    # <- axes3dをインポートしていないとエラーになる。
scatter = ax.scatter3D(x,y,value,
                       s=100,                    # マーカーのサイズ
                       c=value,  
                       vmin=0.0,
                       vmax=0.0015,                  # 色分けに使う数値（任意の数値を指定可）
                       cmap=plt.cm.viridis)    # 色のパターン
ax.set_xlabel('Y')
ax.set_ylabel('X')
ax.set_zlabel('data')
plt.colorbar(scatter)                          # カラーバーを表示（省略可）
plt.show()
