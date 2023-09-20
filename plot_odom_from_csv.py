import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import math

fig = plt.figure()
ax = fig.add_subplot(111)

time_stamp_odom_csv = '/home/hirayama-d/research_ws/メディアンなし&ボクセルなし2.0以下点群ifの外にリサンプリングとか出してみた - メディアンなし&ボクセルなし2.0以下点群ifの外にリサンプリングとか出してみた.csv'

rows = []
with open(time_stamp_odom_csv) as f:   
    reader = csv.reader(f)
    rows = [row for row in reader]

odom_data = np.float_(np.array(rows).T)

# plt.xticks(range(0,100,10))
# plt.yticks(range(0.3,0.3,0.1))
# plt.yticks(range(-4,1,1))
# plt.title("Y-AMCL-Error")
plt.grid(linestyle='dotted')
plt.xlabel("t [s]")
plt.ylabel("x [m]")


# 10個ずつ間引いたデータでプロットしてみる

time = odom_data[0]
odom_x = odom_data[1]
odom_y = odom_data[2]
amcl_x = odom_data[3]
amcl_y = odom_data[4]
propose_x = odom_data[5]
propose_y = odom_data[6]

odom_amcl_x = odom_data[7]
propose_amcl_x = odom_data[8]
odom_amcl_y = odom_data[9]
propose_amcl_y = odom_data[10]

step = 1
plt.plot(time[::step],odom_amcl_x[::step],label='Odometry-AMCL',linewidth=1)
plt.plot(time[::step],propose_amcl_x[::step],label='Propose-AMCL',linewidth=1,c='orange')
plt.ylim(-0.25,0.3) ##スケールをあわせる
# plt.ylim(-0.25,0.1) ##スケールをあわせる

# plt.yticks(np.arange(-1,12,1))
# plt.plot(time[::step],odom_y[::step],linewidth=3)
# plt.plot(time[::step],amcl_y[::step],linewidth=3,c='green')
# plt.plot(time[::step],propose_y[::step],linewidth=3,c='orange')
# plt.legend(fontsize=15)
# plt.savefig("amcl_sabun_scale_y.png")
plt.show()