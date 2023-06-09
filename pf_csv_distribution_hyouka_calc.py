import numpy as np
import csv

# CSVファイルの読み込み
filename = "/home/hirayama-d/research_ws/particle_coodinate_distribution_time_stamp_gazebo_down_sample_10.csv"



# import csv

# CSVファイルの読み込み
# filename = "data.csv"
with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    data = []
    for row in csvreader:
        data_row = []
        for val in row:
            data_row.append(val)
        data.append(data_row)

# 抽出する行の範囲を指定する
start_row = 0
end_row = 399

# 抽出する列の範囲を指定する
start_col = 4
end_col = 4

# データを抽出する
extracted_data = []
for i in range(start_row, end_row+1):
    extracted_row = []
    for j in range(start_col, end_col+1):
        extracted_row.append(data[i][j])
    extracted_data.append(extracted_row)

# print(extracted_data)
print(type(extracted_data))
data = np.array(extracted_data, dtype=np.float64)
print(np.var(data))






