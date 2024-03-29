import numpy as np
import matplotlib.pyplot as plt
import csv

# CSVファイルのパス
csv_file_path = "/home/hirayama-d/research_ws/particle_coodinate_distribution_time_stamp_gazebo_10.csv"

# CSVファイルを読み込んでリストに格納
data = []
with open(csv_file_path, "r") as f:
    reader = csv.reader(f)
    for row in reader:
        data.append(row)

# 最初の400行を取得してnumpy配列に変換
data_np = np.array(data[:400])
x = data_np[:,1]
y = data_np[:,2]
values = data_np[:,4]

# print(x.max(),y.max())

print(type(x),type(y),type(values))
# 任意の値を持つ二次元配列を作成する
data = np.zeros((20,20),dtype=float)



data_np[y,x] = values
# ヒートマップを作成する
plt.imshow(data, cmap='YlGnBu', origin='lower', extent=[x.min(), x.max(), y.min(), y.max()])

# グラフにタイトルやラベルを追加する
plt.title("Heatmap Example")
plt.xlabel("X Label")
plt.ylabel("Y Label")

# カラーバーを追加する
plt.colorbar()
# ヒートマップに対応した数値を表示する
for i in range(data.shape[0]):
    for j in range(data.shape[1]):
        plt.text(j, i, data[i, j], ha='center', va='center', color='black')
# plt.xticks(range(-3,3,1))
# plt.yticks(range(0,10,1))
# plt.xlim(x.min(), x.max())
# plt.ylim(y.min(), y.max())
# ヒートマップを表示する
plt.show()
