import pandas as pd

# CSVファイルの読み込み
output_cluster_file = '/home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/csv/output_cluster_path1_reverse.csv'  # CSVファイルのパスを指定
data = pd.read_csv(output_cluster_file)

# ① labelがblueのstd_dev_xの平均とその他の情報
blue_data = data[data['label'] == 'blue']
blue_count = len(blue_data)
blue_std_dev_x_sum = blue_data['std_dev_x'].sum()
blue_std_dev_x_avg = blue_std_dev_x_sum / blue_count if blue_count else 0

# ② labelがredのstd_dev_yの平均とその他の情報
red_data = data[data['label'] == 'red']
red_count = len(red_data)
red_std_dev_y_sum = red_data['std_dev_y'].sum()
red_std_dev_y_avg = red_std_dev_y_sum / red_count if red_count else 0

# ③ すべてのstd_dev_xの平均
total_std_dev_x_avg = data['std_dev_x'].mean()

# ④ すべてのstd_dev_yの平均
total_std_dev_y_avg = data['std_dev_y'].mean()

# 結果の出力
print(f"1. Blue (x-long):\n   Count: {blue_count}, Sum: {blue_std_dev_x_sum}, Average: {blue_std_dev_x_avg}")
print(f"2. Red (y-long):\n   Count: {red_count}, Sum: {red_std_dev_y_sum}, Average: {red_std_dev_y_avg}")
print(f"3. All std_dev_x:\n   Count: {len(data)}, Sum: {data['std_dev_x'].sum()}, Average: {total_std_dev_x_avg}")
print(f"4. All std_dev_y:\n   Count: {len(data)}, Sum: {data['std_dev_y'].sum()}, Average: {total_std_dev_y_avg}")
