import pandas as pd

# CSVファイルの読み込み
# hihuku_output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/csv/output_cluster_path1_reverse.csv'
# output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/1.5_hige_output_cluster_path1.csv'
path = 'path1' #path1 or path2(reverse)

if path == 'path1':
    print("path1について検討")
    one_point_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1/csv/1.5_hige_output_cluster_path1.csv'
    hihuku_output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1/csv/hihuku_output_cluster_path1.csv'
    output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_original_route1/path_route1/csv/origin_output_cluster_path1.csv' 
else:
    print("path2について検討") #reverse
    one_point_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_oregatukutta_1.5_hige/path_route1_reverse/csv/1.5_hige_output_cluster_path1_reverse.csv'
    hihuku_output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/csv/output_cluster_path1_reverse.csv'
    output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_original_route1/path_route1_reverse/csv/origin_output_cluster_path1_reverse.csv' 



origin_data = pd.read_csv(output_cluster_file)

# ----------------------origin_dataの処理--------------------------
blue_data = origin_data[origin_data['label'] == 'blue']
origin_blue_x = blue_data['x']
origin_blue_y = blue_data['y']
origin_blue_yaw = blue_data['yaw']
origin_blue_count = len(blue_data)
origin_blue_std_dev_x_sum = blue_data['std_dev_x'].sum()
origin_blue_std_dev_x_avg = origin_blue_std_dev_x_sum / origin_blue_count if origin_blue_count else 0
print(f"origin_mapの{path}におけるblueの std_dev_xの平均:",origin_blue_std_dev_x_avg)
# red
red_data = origin_data[origin_data['label'] == 'red']
origin_red_y = red_data['y']
origin_red_count = len(red_data)
origin_red_std_dev_y_sum = red_data['std_dev_y'].sum()
origin_red_std_dev_y_avg = origin_red_std_dev_y_sum / origin_red_count if origin_red_count else 0
print(f"origin_mapの{path}におけるredの std_dev_yの平均:", origin_red_std_dev_y_avg)

# ----------------------------------------------------------------------------------------------------
print("--------------------------------------------------------------------------------")


# hihuku_dataの処理
hihuku_data = pd.read_csv(hihuku_output_cluster_file)

# origin_dataのblueラベルの座標と一致しないhihuku_dataの座標を探す
missing_coordinates_hihuku = []
for x, y, yaw in zip(origin_blue_x, origin_blue_y, origin_blue_yaw):
    if not ((hihuku_data['x'] == x) & (hihuku_data['y'] == y) & (hihuku_data['yaw'] == yaw)).any():
        missing_coordinates_hihuku.append((x, y, yaw))

# 欠けているデータがない場合のみstd_dev_xの合計と平均を計算
if not missing_coordinates_hihuku:
    total_std_dev_x = 0
    for index, row in hihuku_data.iterrows():
        if (row['x'], row['y'], row['yaw']) in zip(origin_blue_x, origin_blue_y, origin_blue_yaw):
            total_std_dev_x += row['std_dev_x']
    average_std_dev_x = total_std_dev_x / origin_blue_count
    print(f"hihuku_mapの{path}におけるblueの std_dev_xの平均:", average_std_dev_x)
else:
    print("欠けている座標があります。平均は計算されません。")
    print("欠けている座標の数:", len(missing_coordinates_hihuku))
    print("欠けている座標のリスト:", missing_coordinates_hihuku)

# hihuku_dataの処理（redのy方向）
# 欠けているデータがないか確認（redのy方向）
missing_coordinates_hihuku_red = []
for x, y, yaw in zip(red_data['x'], origin_red_y, red_data['yaw']):
    if not ((hihuku_data['x'] == x) & (hihuku_data['y'] == y) & (hihuku_data['yaw'] == yaw)).any():
        missing_coordinates_hihuku_red.append((x, y, yaw))

# 欠けているデータがない場合のみstd_dev_yの合計と平均を計算（redのy方向）
if not missing_coordinates_hihuku_red:
    total_std_dev_y = 0
    for index, row in hihuku_data.iterrows():
        if (row['x'], row['y'], row['yaw']) in zip(red_data['x'], origin_red_y, red_data['yaw']):
            total_std_dev_y += row['std_dev_y']
    average_std_dev_y = total_std_dev_y / origin_red_count
    print(f"hihuku_mapの{path}におけるredの std_dev_yの平均:", average_std_dev_y)
else:
    print("hihuku_mapでredの欠けている座標があります。平均は計算されません。")
    print("欠けている座標の数:", len(missing_coordinates_hihuku_red))
    print("欠けている座標のリスト:", missing_coordinates_hihuku_red)

print("--------------------------------------------------------------------------------")

# --------------------------------------------------------------------------------------------------------

# 1.5_dataの処理
one_point_five_data = pd.read_csv(one_point_cluster_file)

# origin_dataのblueラベルの座標と一致しない1.5_dataの座標を探す
missing_coordinates = []
for x, y, yaw in zip(origin_blue_x, origin_blue_y, origin_blue_yaw):
    if not ((one_point_five_data['x'] == x) & (one_point_five_data['y'] == y) & (one_point_five_data['yaw'] == yaw)).any():
        missing_coordinates.append((x, y, yaw))

# 欠けているデータがない場合のみstd_dev_xの合計と平均を計算
if not missing_coordinates:
    total_std_dev_x = 0
    for index, row in one_point_five_data.iterrows():
        if (row['x'], row['y'], row['yaw']) in zip(origin_blue_x, origin_blue_y, origin_blue_yaw):
            total_std_dev_x += row['std_dev_x']
    average_std_dev_x = total_std_dev_x / origin_blue_count
    print(f"1.5_mapの{path}におけるblueの std_dev_xの平均:", average_std_dev_x)
else:
    print("欠けている座標があります。平均は計算されません。")
    print("欠けている座標の数:", len(missing_coordinates))
    print("欠けている座標のリスト:", missing_coordinates)



# 欠けているデータがないか確認（redのy方向）
missing_coordinates_one_point_five_red = []
for x, y, yaw in zip(red_data['x'], origin_red_y, red_data['yaw']):
    if not ((one_point_five_data['x'] == x) & (one_point_five_data['y'] == y) & (one_point_five_data['yaw'] == yaw)).any():
        missing_coordinates_one_point_five_red.append((x, y, yaw))

# 欠けているデータがない場合のみstd_dev_yの合計と平均を計算（redのy方向）
if not missing_coordinates_one_point_five_red:
    total_std_dev_y = 0
    for index, row in one_point_five_data.iterrows():
        if (row['x'], row['y'], row['yaw']) in zip(red_data['x'], origin_red_y, red_data['yaw']):
            total_std_dev_y += row['std_dev_y']
    average_std_dev_y = total_std_dev_y / origin_red_count
    print(f"1.5_mapの{path}におけるredの std_dev_yの平均:", average_std_dev_y)
else:
    print("1.5_mapでredの欠けている座標があります。平均は計算されません。")
    print("欠けている座標の数:", len(missing_coordinates_one_point_five_red))
    print("欠けている座標のリスト:", missing_coordinates_one_point_five_red)

print("--------------------------------------------------------------------------------")
# origin_dataの列数に合わせて他のデータセットの列数を調整
origin_column_count = len(origin_data.columns)
hihuku_data = hihuku_data.iloc[:, :origin_column_count]
one_point_five_data = one_point_five_data.iloc[:, :origin_column_count]

# 各データセットのstd_dev_xとstd_dev_yの平均を計算する関数
def calculate_means(data):
    std_dev_x_means = data['std_dev_x'].mean()
    std_dev_y_means = data['std_dev_y'].mean()
    return std_dev_x_means, std_dev_y_means

# 各データセットの平均を計算
origin_means = calculate_means(origin_data)
hihuku_means = calculate_means(hihuku_data)
one_point_five_means = calculate_means(one_point_five_data)

print(f"{path}の時の全体の平均")
# 結果の出力
print("Origin Data: std_dev_x Mean =", origin_means[0], ", std_dev_y Mean =", origin_means[1])
print("Hihuku Data: std_dev_x Mean =", hihuku_means[0], ", std_dev_y Mean =", hihuku_means[1])
print("1.5 Data: std_dev_x Mean =", one_point_five_means[0], ", std_dev_y Mean =", one_point_five_means[1])
