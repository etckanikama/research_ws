# 1~700000をめちゃくちゃ削る(間隔的には7000個にひとつ削る)
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import math

fig = plt.figure()
ax = fig.add_subplot(111)

odom_csv = '/home/hirayama-d/research_ws/2023-2-28_v_1.0_odom_zure_tyokusen.csv'
estimate_csv = '/home/hirayama-d/research_ws/2_28_v1.0_estimate_odom_zure_tyokusen_xy.csv'
# font_path = '/usr/share/fonts/TakaoFonts_00303.01/TakaoGothic.ttf'
# font_prop = FontProperties(fname=font_path)
# plt.rcParams['font.family'] = font_prop.get_name()
rows = []
with open(odom_csv) as f:   
    reader = csv.reader(f)
    rows = [row for row in reader]

odom_data = np.float_(np.array(rows).T)

rows2 = []
with open(estimate_csv) as f:   
    reader = csv.reader(f)
    rows2 = [row for row in reader]

estimate_data = np.float_(np.array(rows2).T)

plt.xticks(range(-3,4,1))
plt.yticks(range(0,15,1))
ax.set_aspect('equal')
ax.invert_xaxis()
plt.grid(linestyle='dotted')
plt.xlabel("y")
plt.ylabel("x")
# s: marker_size
plt.scatter(odom_data[1],odom_data[0],label="odometry", s=1)
plt.scatter(estimate_data[1],estimate_data[0],label="estimate_value", s=1)
# cnt = 0
# # plt.quiver(data[1], data[0], vy, vx)
# for x, y in zip(data[0], data[1]):
#     cnt += 1
#     if cnt == 1000:
#         # 始点(ｙ，ｘ)、終点（ｙ, x)
#         # p = plt.arrow(y,x, 0.2*math.sin(th), 0.2*math.cos(th),width=0.07, head_length=0.15,head_width=0.2)
#         plt.plot(y,x)
#         cnt = 0

# plt.hlines(y=3.6, xmin=-2, xmax=-0.5, color='red', linestyles='solid')
# plt.hlines(y=3.6, xmin=0.5, xmax=2, color='red', linestyles='solid')
# plt.hlines(y=4.6, xmin=-2, xmax=-0.5, color='red', linestyles='solid')
# plt.hlines(y=4.6, xmin=0.5, xmax=2, color='red', linestyles='solid')
# plt.vlines(x=-0.5,ymin=0, ymax=3.6,colors='red',linestyles='solid')
# plt.vlines(x=0.5,ymin=0, ymax=3.6,colors='red',linestyles='solid')
# plt.vlines(x=-0.5,ymin=4.6, ymax=10,colors='red',linestyles='solid')
# plt.vlines(x=0.5,ymin=4.6, ymax=10,colors='red',linestyles='solid')

plt.hlines(y=5.47, xmin=-4, xmax=-0.5, color='red', linestyles='solid')
plt.hlines(y=5.47, xmin=0.5, xmax=2, color='red', linestyles='solid')
plt.hlines(y=6.47, xmin=-4, xmax=-0.5, color='red', linestyles='solid')
plt.hlines(y=6.47, xmin=0.5, xmax=2, color='red', linestyles='solid')
plt.vlines(x=-0.5,ymin=0, ymax=5.47,colors='red',linestyles='solid')
plt.vlines(x=0.5,ymin=0, ymax=5.47,colors='red',linestyles='solid')
plt.vlines(x=-0.5,ymin=6.47, ymax=13,colors='red',linestyles='solid')
plt.vlines(x=0.5,ymin=6.47, ymax=13,colors='red',linestyles='solid')
plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0)
# plt.legend([p,s],['footprint','whilte_lane'])

# plt.savefig("sasetu_xy_scatter.png")
plt.show()