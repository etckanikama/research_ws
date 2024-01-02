import pandas as pd
import os
import numpy as np
import sys
import matplotlib.pyplot as plt

np.set_printoptions(threshold=np.inf)

if len(sys.argv) < 4:
    print("引数が指定されてないです")
    sys.exit(1)

try:
    param_value1 = float(sys.argv[1])
    param_value2 = float(sys.argv[2])
    param_value3 = float(sys.argv[3])
    if param_value3 == 0.0:
        param_value3 = '0.00'
except ValueError:
    print("引数は数値である必要があリマス")
    sys.exit(1)



# CSVファイルのパス(input)
file_path = f"/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/path_route1_{param_value1}_{param_value2}_{param_value3}.csv"

# file_path = f"/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/path_route1{param_value1}_{param_value2}_{param_value3}.csv"
# file_path = f"/home/hirayama-d/research_ws/src/sim/20231227_re_route1/csv/re_route1{param_value1}_{param_value2}_{param_value3}.csv"

# file_path = f"/home/hirayama-d/research_ws/src/sim/20231226_hige/add_hige_x200length50_csv/add_hige_x200length_kyokutan{param_value1}_{param_value2}_{param_value3}.csv"

# file_path = f"/home/hirayama-d/research_ws/src/sim/20231221_route2/csv/20231221_boxel_nasi_640x360_2.0ikanomi_kairyou{param_value1}_{param_value2}_{param_value3}.csv"
# 計算結果確認用csv
output_csv_path = f"/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/output_eigen_path_route1.csv"

# output_csv_path = f"/home/hirayama-d/research_ws/src/sim/20231227_re_route1/csv/output_eigen_re_route1.csv"

# output_csv_path = f"/home/hirayama-d/research_ws/src/sim/20231221_route2/csv/output_eigen.csv"
# file_path = f"/home/hirayama-d/research_ws/src/sim/20231220_result/csv/20231220_boxel_nasi_640x360_2.0ikanomi_kairyou{param_value1}_{param_value2}_{param_value3}.csv"
# output_csv_path = f"/home/hirayama-d/research_ws/src/sim/20231221_route2/csv/output_eigen_route2.csv"
# file_path = f"/home/hirayama-d/research_ws/src/sim/20231212_csv/20231212_boxel_nasi_640x360_2.0ikanomi_kairyou{param_value1}_{param_value2}_{param_value3}.csv"
# file_path = f"/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230826_boxel_nasi_2.5ikanomi_{param_value1}_{param_value2}_{param_value3}.csv"
# file_path = f"/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230930_boxel_nasi_640x360_2.0ikanomi_kairyou{param_value1}_{param_value2}_{param_value3}.csv"

# CSVファイルのパス
# file_path = '/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230930_boxel_nasi_640x360_2.0ikanomi_rsj8.0_0.0_0.0.csv'
# file_path = '/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230826_boxel_nasi_5.0_0.0_0.0.csv'
# CSVファイルからデータを読み込む
data = pd.read_csv(file_path, header=None, names=['t', 'x', 'y', 'theta', 'w'])

# 最初の400行を選択し、x, y, v のカラムを抽出
data_subset = data.iloc[:400][['x', 'y', 'w']]
# print(data_subset)
# 行列Uのデータを取得
U = data_subset.values

# print("元の行列",U)
# print(type(U))


# xとyの平均を計算
x_mean = U[:, 0].mean()
y_mean = U[:, 1].mean()
print("xの平均",x_mean)
print("yの平均",y_mean)

# for x,y,w in U:
#     print(x,y, w)

# # 分散と共分散の分子と分母を計算
# numerator_x = 0 
# numerator_y = 0
# numerator_cov = 0
# denominator = 0
# for x,y,w in U:
#     numerator_x += w * (x - x_mean)**2
#     numerator_y += w * (y - y_mean)**2
#     denominator += w
print("wのmax",U[:, 2].max())
numerator_x = sum(w * (x - x_mean)**2 for x, y, w in U)
numerator_y = sum(w * (y - y_mean)**2 for x, y, w in U)
numerator_cov = sum(w * (x - x_mean) * (y - y_mean) for x, y, w in U)
denominator = sum(w for x, y, w in U)

print("xの分散の分子",numerator_x)
print("yの分散の分子",numerator_y)
print("xyの共分散の分子",numerator_cov)
print("wの和",denominator)

# 分散と共分散を計算
variance_x = numerator_x / denominator
variance_y = numerator_y / denominator
covariance_xy = numerator_cov / denominator

print("xの分散",variance_x)
print("yの分散",variance_y)
print("xyの共分散",covariance_xy)

# 共分散行列を作成
covariance_matrix = [
    [variance_x, covariance_xy],
    [covariance_xy, variance_y]
]

# 共分散行列を出力
print("共分散行列",covariance_matrix)

eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
print("固有値",eigenvalues)
print("固有ベクトル",eigenvectors)
print("localizability（e=sqrt(λ1*λ2）",eigenvalues[0]*eigenvalues[1])

# 固有値とその他の必要なデータを変数に保存
eigenvalue1 = eigenvalues[0]
eigenvalue2 = eigenvalues[1]

# 新しいデータ行を作成
# new_row = pd.DataFrame({'x': [param_value1], 'y': [param_value2], 'yaw': [param_value3], 'eigenvalue1': [eigenvalue1], 'eigenvalue2': [eigenvalue2]})
new_row = pd.DataFrame({'x': [param_value1], 'y': [param_value2], 'yaw': [param_value3], 'variance_x':[variance_x],'variance_y': [variance_y] ,'eigenvalue1': [eigenvalue1], 'eigenvalue2': [eigenvalue2]})

# CSVファイルが存在するかどうかをチェック
if os.path.exists(output_csv_path):
    # 既存のデータを読み込む
    existing_data = pd.read_csv(output_csv_path)
    # 新しいデータ行を追加
    updated_data = pd.concat([existing_data, new_row], ignore_index=True)
else:
    # 新しいデータフレームを作成
    updated_data = new_row

# データフレームをCSVファイルに保存
updated_data.to_csv(output_csv_path, index=False)

# Plot the eigenvectors for the new data
fig, ax = plt.subplots(figsize=(11, 11))
eigenvector_scale = 0.1
for i in range(len(eigenvalues)):
    vec_new = eigenvectors[:, i] * eigenvalues[i] * eigenvector_scale
    plt.quiver(x_mean, y_mean, vec_new[0], vec_new[1], angles='xy', scale_units='xy', scale=1, label=f'Eigenvector {i+1}')



plt.legend()
plt.ylim(-0.01+param_value2, 0.01+param_value2)
plt.xlim(-0.01+param_value1, 0.01+param_value1)

ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.title(f'Coodinate_x:{param_value1}y:{param_value2}theta{param_value3}\n variance_x: {variance_x} variance_y: {variance_y} \n Eigenvalues: {eigenvalues}\nEigenvectors: {eigenvectors}\nlocalizability(e=sqrt(λ1*λ2)):{eigenvalues[0]*eigenvalues[1]}')
plt.grid(True)
# plt.savefig(f"/home/hirayama-d/research_ws/src/sim/20231220_result/output/eigenvalue/Coodinate_x{param_value1}y{param_value2}theta{param_value3}.png")
# plt.savefig(f"/home/hirayama-d/research_ws/src/sim/20231221_route2/output/eigenvalue/Coodinate_x{param_value1}y{param_value2}theta{param_value3}.png")
# plt.savefig(f"/home/hirayama-d/research_ws/src/sim/20231227_re_route1/output/eigenvalue/Coodinate_x{param_value1}y{param_value2}theta{param_value3}.png")
plt.savefig(f"/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/output/eigenvalue/Coodinate_x{param_value1}y{param_value2}theta{param_value3}.png")

# plt.show()
