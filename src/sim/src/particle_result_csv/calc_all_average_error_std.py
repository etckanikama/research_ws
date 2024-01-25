import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import ast 
import math


# マップの選択(1.5,hihuku,origin)
map_name = "1.5"
# CSVファイルを読み込む
df1 = pd.read_csv(f'/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/{map_name}_map/1.csv')
df2 = pd.read_csv(f'/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/{map_name}_map/2.csv')
df3 = pd.read_csv(f'/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/{map_name}_map/3.csv')
df4 = pd.read_csv(f'/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/{map_name}_map/4.csv')
df5 = pd.read_csv(f'/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/{map_name}_map/5.csv')

max_count1 = df1['Count'].max()
max_count2 = df2['Count'].max()
max_count3 = df3['Count'].max()
max_count4 = df4['Count'].max()
max_count5 = df5['Count'].max()

min_count = min(max_count1,max_count2,max_count3,max_count4,max_count5)
min_count = 1800


# 全体バージョン
def process_dataframe(df, min_count, column_name):
    unique_counts = df['Count'].unique()
    estimate_error = []
    for count in unique_counts:
        if count <= min_count:
            first_row = df[df['Count'] == count].head(1)
            error = abs(first_row[column_name].values[0])
            estimate_error.append(error)
    return estimate_error

# 区間指定バージョン
def process_dataframe_by_state(df, min_count, column_name):
    x_long_errors = []
    rolling_errors = []
    y_long_errors = []
    
    for count in range(min_count + 1):
        if count <= 1380:
            # x-long 状態
            error = abs(df[df['Count'] == count][column_name].head(1).values[0])
            x_long_errors.append(error)
        elif 1381 <= count <= 1560:
            # rolling 状態
            error = abs(df[df['Count'] == count][column_name].head(1).values[0])
            rolling_errors.append(error)
        elif 1561 <= count <= min_count:
            # y-long 状態
            error = abs(df[df['Count'] == count][column_name].head(1).values[0])
            y_long_errors.append(error)
            
    return x_long_errors, rolling_errors, y_long_errors


# X, Y, Yaw のエラーの総合計と平均を計算
def calculate_average_error(errors_list):
    sum_error = 0
    for errors in errors_list:
        sum_error += sum(errors)
    return sum_error / (5 * len(errors_list[0]))

# X, Y, Yaw のエラーの標準偏差を計算
def calculate_standard_deviation(errors_list, average_error):
    sum_squared_differences = 0
    n = len(errors_list[0]) * 5  # 5回の試行で各試行のデータポイント数

    for errors in errors_list:
        for error in errors:
            sum_squared_differences += (error - average_error) ** 2

    variance = sum_squared_differences / n
    standard_deviation = math.sqrt(variance)
    return standard_deviation

print(f"地図：{map_name}_map")
# --------------------start:全体バージョン：データの抽出-------------------------------------------
estimate_x_error1 = process_dataframe(df1, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error2 = process_dataframe(df2, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error3 = process_dataframe(df3, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error4 = process_dataframe(df4, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error5 = process_dataframe(df5, min_count, 'EstimateOdomX-BeegoX')

estimate_y_error1 = process_dataframe(df1, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error2 = process_dataframe(df2, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error3 = process_dataframe(df3, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error4 = process_dataframe(df4, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error5 = process_dataframe(df5, min_count, 'EstimateOdomY-BeegoY')

estimate_yaw_error1 = process_dataframe(df1, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error2 = process_dataframe(df2, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error3 = process_dataframe(df3, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error4 = process_dataframe(df4, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error5 = process_dataframe(df5, min_count, 'EstimateOdomYaw-BeegoYaw')
# -----------------------------------------------------------------------------------------
print("全体バージョン")
# 各エラーの総合平均を計算
average_x_error = calculate_average_error([estimate_x_error1, estimate_x_error2, estimate_x_error3, estimate_x_error4, estimate_x_error5])
average_y_error = calculate_average_error([estimate_y_error1, estimate_y_error2, estimate_y_error3, estimate_y_error4, estimate_y_error5])
average_yaw_error = calculate_average_error([estimate_yaw_error1, estimate_yaw_error2, estimate_yaw_error3, estimate_yaw_error4, estimate_yaw_error5])

print("Average X Error:", average_x_error)
print("Average Y Error:", average_y_error)
print("Average Yaw Error:", average_yaw_error)


# X, Y, Yaw の標準偏差を計算
standard_deviation_x = calculate_standard_deviation([estimate_x_error1, estimate_x_error2, estimate_x_error3, estimate_x_error4, estimate_x_error5], average_x_error)
standard_deviation_y = calculate_standard_deviation([estimate_y_error1, estimate_y_error2, estimate_y_error3, estimate_y_error4, estimate_y_error5], average_y_error)
standard_deviation_yaw = calculate_standard_deviation([estimate_yaw_error1, estimate_yaw_error2, estimate_yaw_error3, estimate_yaw_error4, estimate_yaw_error5], average_yaw_error)

print("Standard Deviation of X Error:", standard_deviation_x)
print("Standard Deviation of Y Error:", standard_deviation_y)
print("Standard Deviation of Yaw Error:", standard_deviation_yaw)


# -------------end:全体バージョン---------------

print("----------------------------------------------------------------")

# ------------start: 区間指定バージョン:データの抽出------------------------
estimate_x_error1_x_long, estimate_x_error1_rolling, estimate_x_error1_y_long = process_dataframe_by_state(df1, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error2_x_long, estimate_x_error2_rolling, estimate_x_error2_y_long = process_dataframe_by_state(df2, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error3_x_long, estimate_x_error3_rolling, estimate_x_error3_y_long = process_dataframe_by_state(df3, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error4_x_long, estimate_x_error4_rolling, estimate_x_error4_y_long = process_dataframe_by_state(df4, min_count, 'EstimateOdomX-BeegoX')
estimate_x_error5_x_long, estimate_x_error5_rolling, estimate_x_error5_y_long = process_dataframe_by_state(df5, min_count, 'EstimateOdomX-BeegoX')

estimate_y_error1_x_long, estimate_y_error1_rolling, estimate_y_error1_y_long = process_dataframe_by_state(df1, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error2_x_long, estimate_y_error2_rolling, estimate_y_error2_y_long = process_dataframe_by_state(df2, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error3_x_long, estimate_y_error3_rolling, estimate_y_error3_y_long = process_dataframe_by_state(df3, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error4_x_long, estimate_y_error4_rolling, estimate_y_error4_y_long = process_dataframe_by_state(df4, min_count, 'EstimateOdomY-BeegoY')
estimate_y_error5_x_long, estimate_y_error5_rolling, estimate_y_error5_y_long = process_dataframe_by_state(df5, min_count, 'EstimateOdomY-BeegoY')

estimate_yaw_error1_x_long, estimate_yaw_error1_rolling, estimate_yaw_error1_y_long = process_dataframe_by_state(df1, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error2_x_long, estimate_yaw_error2_rolling, estimate_yaw_error2_y_long = process_dataframe_by_state(df2, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error3_x_long, estimate_yaw_error3_rolling, estimate_yaw_error3_y_long = process_dataframe_by_state(df3, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error4_x_long, estimate_yaw_error4_rolling, estimate_yaw_error4_y_long = process_dataframe_by_state(df4, min_count, 'EstimateOdomYaw-BeegoYaw')
estimate_yaw_error5_x_long, estimate_yaw_error5_rolling, estimate_yaw_error5_y_long = process_dataframe_by_state(df5, min_count, 'EstimateOdomYaw-BeegoYaw')

# X, Y, Yaw の総合平均を計算（x-long 状態）
average_x_error_x_long = calculate_average_error([estimate_x_error1_x_long, estimate_x_error2_x_long, estimate_x_error3_x_long, estimate_x_error4_x_long, estimate_x_error5_x_long])
average_y_error_x_long = calculate_average_error([estimate_y_error1_x_long, estimate_y_error2_x_long, estimate_y_error3_x_long, estimate_y_error4_x_long, estimate_y_error5_x_long])
average_yaw_error_x_long = calculate_average_error([estimate_yaw_error1_x_long, estimate_yaw_error2_x_long, estimate_yaw_error3_x_long, estimate_yaw_error4_x_long, estimate_yaw_error5_x_long])

# X, Y, Yaw の総合平均を計算（rolling 状態）
average_x_error_rolling = calculate_average_error([estimate_x_error1_rolling, estimate_x_error2_rolling, estimate_x_error3_rolling, estimate_x_error4_rolling, estimate_x_error5_rolling])
average_y_error_rolling = calculate_average_error([estimate_y_error1_rolling, estimate_y_error2_rolling, estimate_y_error3_rolling, estimate_y_error4_rolling, estimate_y_error5_rolling])
average_yaw_error_rolling = calculate_average_error([estimate_yaw_error1_rolling, estimate_yaw_error2_rolling, estimate_yaw_error3_rolling, estimate_yaw_error4_rolling, estimate_yaw_error5_rolling])

# X, Y, Yaw の総合平均を計算（y-long 状態）
average_x_error_y_long = calculate_average_error([estimate_x_error1_y_long, estimate_x_error2_y_long, estimate_x_error3_y_long, estimate_x_error4_y_long, estimate_x_error5_y_long])
average_y_error_y_long = calculate_average_error([estimate_y_error1_y_long, estimate_y_error2_y_long, estimate_y_error3_y_long, estimate_y_error4_y_long, estimate_y_error5_y_long])
average_yaw_error_y_long = calculate_average_error([estimate_yaw_error1_y_long, estimate_yaw_error2_y_long, estimate_yaw_error3_y_long, estimate_yaw_error4_y_long, estimate_yaw_error5_y_long])

print("各区間バージョン")
# 結果の出力
print("Average X Error (X-Long):", average_x_error_x_long)
print("Average Y Error (X-Long):", average_y_error_x_long)
print("Average Yaw Error (X-Long):", average_yaw_error_x_long)

print("Average X Error (Rolling):", average_x_error_rolling)
print("Average Y Error (Rolling):", average_y_error_rolling)
print("Average Yaw Error (Rolling):", average_yaw_error_rolling)

print("Average X Error (Y-Long):", average_x_error_y_long)
print("Average Y Error (Y-Long):", average_y_error_y_long)
print("Average Yaw Error (Y-Long):", average_yaw_error_y_long)

# X, Y, Yaw の標準偏差を計算（x-long 状態）
std_dev_x_x_long = calculate_standard_deviation([estimate_x_error1_x_long, estimate_x_error2_x_long, estimate_x_error3_x_long, estimate_x_error4_x_long, estimate_x_error5_x_long], average_x_error_x_long)
std_dev_y_x_long = calculate_standard_deviation([estimate_y_error1_x_long, estimate_y_error2_x_long, estimate_y_error3_x_long, estimate_y_error4_x_long, estimate_y_error5_x_long], average_y_error_x_long)
std_dev_yaw_x_long = calculate_standard_deviation([estimate_yaw_error1_x_long, estimate_yaw_error2_x_long, estimate_yaw_error3_x_long, estimate_yaw_error4_x_long, estimate_yaw_error5_x_long], average_yaw_error_x_long)

# X, Y, Yaw の標準偏差を計算（rolling 状態）
std_dev_x_rolling = calculate_standard_deviation([estimate_x_error1_rolling, estimate_x_error2_rolling, estimate_x_error3_rolling, estimate_x_error4_rolling, estimate_x_error5_rolling], average_x_error_rolling)
std_dev_y_rolling = calculate_standard_deviation([estimate_y_error1_rolling, estimate_y_error2_rolling, estimate_y_error3_rolling, estimate_y_error4_rolling, estimate_y_error5_rolling], average_y_error_rolling)
std_dev_yaw_rolling = calculate_standard_deviation([estimate_yaw_error1_rolling, estimate_yaw_error2_rolling, estimate_yaw_error3_rolling, estimate_yaw_error4_rolling, estimate_yaw_error5_rolling], average_yaw_error_rolling)

# X, Y, Yaw の標準偏差を計算（y-long 状態）
std_dev_x_y_long = calculate_standard_deviation([estimate_x_error1_y_long, estimate_x_error2_y_long, estimate_x_error3_y_long, estimate_x_error4_y_long, estimate_x_error5_y_long], average_x_error_y_long)
std_dev_y_y_long = calculate_standard_deviation([estimate_y_error1_y_long, estimate_y_error2_y_long, estimate_y_error3_y_long, estimate_y_error4_y_long, estimate_y_error5_y_long], average_y_error_y_long)
std_dev_yaw_y_long = calculate_standard_deviation([estimate_yaw_error1_y_long, estimate_yaw_error2_y_long, estimate_yaw_error3_y_long, estimate_yaw_error4_y_long, estimate_yaw_error5_y_long], average_yaw_error_y_long)

# 結果の出力
print("Standard Deviation of X Error (X-Long):", std_dev_x_x_long)
print("Standard Deviation of Y Error (X-Long):", std_dev_y_x_long)
print("Standard Deviation of Yaw Error (X-Long):", std_dev_yaw_x_long)

print("Standard Deviation of X Error (Rolling):", std_dev_x_rolling)
print("Standard Deviation of Y Error (Rolling):", std_dev_y_rolling)
print("Standard Deviation of Yaw Error (Rolling):", std_dev_yaw_rolling)

print("Standard Deviation of X Error (Y-Long):", std_dev_x_y_long)
print("Standard Deviation of Y Error (Y-Long):", std_dev_y_y_long)
print("Standard Deviation of Yaw Error (Y-Long):", std_dev_yaw_y_long)