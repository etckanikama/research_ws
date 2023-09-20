# 1~700000をめちゃくちゃ削る(間隔的には7000個にひとつ削る)
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import math

fig = plt.figure()
ax = fig.add_subplot(111)

# odom_csv = '/home/hirayama-d/research_ws/2023-03-29_v1.0w0.525_odom.csv'
# estimate_csv = '/home/hirayama-d/research_ws/2023-03-29_v1.0w0.525_estimate.csv'
# amcl_pose_csv = '/home/hirayama-d/research_ws/2023-03-29_v1.0w0.525_amcl.csv'
# time_csv = '/home/hirayama-d/research_ws/2023-03-29_v0.5w1.0472_time_stamp_hyouka.csv'
# font_path = '/usr/share/fonts/TakaoFonts_00303.01/TakaoGothic.ttf'
# font_prop = FontProperties(fname=font_path)
# plt.rcParams['font.family'] = font_prop.get_name()
time_csv = '/home/hirayama-d/research_ws/メディアンなし&ボクセルなし2.5以下.csv'

rows = []
with open(time_csv) as f:   
    reader = csv.reader(f)
    rows = [row for row in reader]

time_data = np.float_(np.array(rows).T)


# 各変数にデータを代入
time_stamp = time_data[0]
robot_x = time_data[1]
robot_y = time_data[2]
amcl_pose_x = time_data[3]
amcl_pose_y = time_data[4]
estimate_odom_x = time_data[5]
estimate_odom_y = time_data[6]

# plt.title('v0.5 & w1.0472')
plt.xticks(range(-5,4,1))
plt.yticks(range(0,15,1))
ax.set_aspect('equal')
ax.invert_xaxis()
plt.grid(linestyle='dotted')
plt.xlabel("y [m]")
plt.ylabel("x [m]")


plt.scatter(robot_y,robot_x,label="odometry", s=0.5,c='blue')

plt.scatter(amcl_pose_y,amcl_pose_x,label="AMCL", s=0.5,c='darkorange')

plt.scatter(estimate_odom_y,estimate_odom_x,label="propose", s=0.5,c='green')


plt.hlines(y=10.095, xmin=-0.47, xmax=-5.0, color='red', linestyles='solid')
plt.hlines(y=9.125, xmin=0.47, xmax=1.935, color='red', linestyles='solid')
plt.hlines(y=11.28, xmin=1.76, xmax=3.01, color='red', linestyles='solid')
plt.hlines(y=10.205, xmin=1.935, xmax=3.01, color='red', linestyles='solid')
plt.hlines(y=11.41, xmin=0.75, xmax=-5.0, color='red', linestyles='solid')



plt.vlines(x=0.47,ymin=0, ymax=9.125,colors='red',linestyles='solid')
plt.vlines(x=-0.47,ymin=0, ymax=10.095,colors='red',linestyles='solid')
plt.vlines(x=1.935,ymin=9.125, ymax=10.205,colors='red',linestyles='solid')
plt.vlines(x=1.76,ymin=11.28, ymax=13.5,colors='red',linestyles='solid')
plt.vlines(x=0.75,ymin=11.41, ymax=13.5,colors='red',linestyles='solid')

# plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0,fontsize=7)
# plt.legend([p,s],['footprint','whilte_lane'])

plt.savefig("not_median_idoukiseki.png")
plt.show()