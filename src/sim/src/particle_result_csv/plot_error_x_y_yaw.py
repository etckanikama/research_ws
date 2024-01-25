import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import ast 
import math


# CSVファイルを読み込む
df1 = pd.read_csv('/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/hihuku_map/1.csv')
df2 = pd.read_csv('/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/1.5_map/1.csv')
df3 = pd.read_csv('/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/origin_map/1.csv')

df1_estimate_x_beego_x = []
df1_estimate_y_beego_y = []
df1_estimate_yaw_beego_yaw = []

df2_estimate_x_beego_x = []
df2_estimate_y_beego_y = []
df2_estimate_yaw_beego_yaw = []

df3_estimate_x_beego_x = []
df3_estimate_y_beego_y = []
df3_estimate_yaw_beego_yaw = []
# odom_x_beego_x = []
# odom_y_beego_y = []
# odom_yaw_beego_yaw = []
time_stamp = []

# Countの最大値を取得
# max_count = df1['Count'].max()
# 1600
select_count = 1800 #曲がってすぐとか
#データ抽出：最大値まで ---------------------
for count in range(select_count + 1):

    filtered_df1 = df1[df1['Count'] == count] # 特定のCount値でデータをフィルタリング
    filtered_df2 = df2[df2['Count'] == count] # 特定のCount値でデータをフィルタリング
    filtered_df3 = df3[df3['Count'] == count] # 特定のCount値でデータをフィルタリング
    
    time_stamp.append(filtered_df1['TimeStamp'])
    # odom_x_beego_x.append(filtered_df1['RobotX-BeegoX'])
    # odom_y_beego_y.append(filtered_df1['RobotY-BeegoY'])
    # odom_yaw_beego_yaw.append(filtered_df1['RobotYaw-BeegoYaw'])

    df1_estimate_x_beego_x.append(filtered_df1['EstimateOdomX-BeegoX'])
    df1_estimate_y_beego_y.append(filtered_df1['EstimateOdomY-BeegoY'])
    df1_estimate_yaw_beego_yaw.append(filtered_df1['EstimateOdomYaw-BeegoYaw'])

    df2_estimate_x_beego_x.append(filtered_df2['EstimateOdomX-BeegoX'])
    df2_estimate_y_beego_y.append(filtered_df2['EstimateOdomY-BeegoY'])
    df2_estimate_yaw_beego_yaw.append(filtered_df2['EstimateOdomYaw-BeegoYaw'])
    
    df3_estimate_x_beego_x.append(filtered_df3['EstimateOdomX-BeegoX'])
    df3_estimate_y_beego_y.append(filtered_df3['EstimateOdomY-BeegoY'])
    df3_estimate_yaw_beego_yaw.append(filtered_df3['EstimateOdomYaw-BeegoYaw'])

#---------------------------------------------------- 

########################### プロット###################################################
# プロットの設定
fig, axs = plt.subplots(3, 1, figsize=(10, 15))

# X軸の誤差のプロット
# axs[0].plot(time_stamp, odom_x_beego_x, color='blue', linewidth=1)
axs[0].plot(time_stamp, df1_estimate_x_beego_x, color='green', linewidth=1)
axs[0].plot(time_stamp, df2_estimate_x_beego_x, color='blue', linewidth=1)
axs[0].plot(time_stamp, df3_estimate_x_beego_x, color='red', linewidth=1)
axs[0].set_title('X Axis Error')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Error')
axs[0].grid(linestyle='dotted')
# axs[0].legend(['Odometry-ground truth', 'Estimate'])

# Y軸の誤差のプロット
# axs[1].plot(time_stamp, odom_y_beego_y, color='blue', linewidth=1)
axs[1].plot(time_stamp, df1_estimate_y_beego_y, color='green', linewidth=1)
axs[1].plot(time_stamp, df2_estimate_y_beego_y, color='blue', linewidth=1)
axs[1].plot(time_stamp, df3_estimate_y_beego_y, color='red', linewidth=1)
axs[1].set_title('Y Axis Error')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Error')
axs[1].grid(linestyle='dotted')
# axs[1].legend(['Odometry-ground truth', 'Estimate'])

# Yawの誤差のプロット
# axs[2].plot(time_stamp, odom_yaw_beego_yaw, color='blue', linewidth=1)
axs[2].plot(time_stamp, df1_estimate_yaw_beego_yaw, color='green', linewidth=1)
axs[2].plot(time_stamp, df2_estimate_yaw_beego_yaw, color='blue', linewidth=1)
axs[2].plot(time_stamp, df3_estimate_yaw_beego_yaw, color='red', linewidth=1)
axs[2].set_title('Yaw Error')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Error')
axs[2].grid(linestyle='dotted')
# axs[2].legend(['Odometry-ground truth', 'Estimate'])

# 全体のタイトルを追加
plt.suptitle('red:origin , blue: 1.5 ,green:hihuku')

# レイアウトを整える
plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # 全体のタイトルが切れないように調整

# プロットを表示
plt.show()