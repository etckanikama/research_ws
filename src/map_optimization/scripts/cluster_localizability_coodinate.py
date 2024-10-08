import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


"""
x,yと各座標における分散から自己位置推定精度が低そうなところを分析し地図上に色分けしてプロットするプログラム
"""
def write_dict_to_file(dictionary, file_path):
    """
    Write the entire contents of a dictionary to a text file as a single string.

    :param dictionary: The dictionary to write.
    :param file_path: The path of the file to write to.
    """
    with open(file_path, "w") as file:
        # 辞書全体を文字列として書き込む
        dict_str = str(dictionary)
        file.write(dict_str)

# CSVファイルの読み込み
# file_path = '/home/hirayama-d/research_ws/src/sim/20231221_route/csv/output_eigen_route2.csv'  # CSVファイルのパスを指定してください
# file_path = '/home/hirayama-d/research_ws/src/sim/20231221_route1/csv/output_eigen_route1.csv'  # CSVファイルのパスを指定してください
# file_path = '/home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/csv/output_eigen_set_covering_route1_reverse.csv'
file_path = '/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/output_eigen_path_route1.csv'
# file_path = '/home/hirayama-d/research_ws/src/sim/20231229_after_set_covering_route1/csv/output_eigen_set_covering_route1.csv'  # CSVファイルのパスを指定してください

data = pd.read_csv(file_path)
# 辞書型配列の作成


# 分散の値にルートを適用
data['sqrt_variance_x'] = np.sqrt(data['variance_x'])
data['sqrt_variance_y'] = np.sqrt(data['variance_y'])

thresh = 0.28

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

# 結果の表示
for key, value in d.items():
    print(f"{key}: {value}")


################# プロット###################################################
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
for color in choices:
    subset = data[data['color'] == color]
    # plt.plot(subset['x'], subset['y'], 'o', color=color, label=f'{color} points')
    plt.plot(subset['y'], subset['x'], 'o', color=color)


plt.yticks(range(0,15,1))
# plt.yticks(np.arange(-1,12,1))
plt.xticks(range(-4,4,1))
ax.set_aspect('equal')
ax.invert_xaxis()
plt.xlabel("y [m]")
plt.ylabel("x [m]")
# plt.title('Colored Plot of X and Y Coordinates Based on Variance-Route1')
plt.grid(True)
# plt.legend()
plt.show()


# ###################### d.txtを書き込む############################
# with open('d.text', 'w') as file:
#     for key, value in d.items():
#         file.write(f"{key}: {value}\n")
write_dict_to_file(d, "d.text")




# クラスタリングされた座標の出力
clustered_data = data[['x', 'y', 'color']]
for color in choices:
    clustered_points = clustered_data[clustered_data['color'] == color][['x', 'y']].to_dict(orient='records')
    print(f"{color.capitalize()} Points:\n", clustered_points, "\n")


# 新しいCSVファイルの作成
# output_file_path = '/home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/csv/output_cluster_path1_reverse.csv'  # 出力CSVファイルのパスを指定
# output_file_path = '/home/hirayama-d/research_ws/src/sim/20240101_original_route1/path_route1/csv/origin_output_cluster_path1.csv' 
output_file_path = '/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/1.5_hige_output_cluster_path1.csv' 
output_data = data[['x', 'y', 'yaw', 'sqrt_variance_x', 'sqrt_variance_y', 'color']].rename(
    columns={'sqrt_variance_x': 'std_dev_x', 'sqrt_variance_y': 'std_dev_y', 'color': 'label'}
)
output_data.to_csv(output_file_path, index=False)

print(output_file_path)