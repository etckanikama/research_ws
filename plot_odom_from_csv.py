import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import math

fig = plt.figure()
ax = fig.add_subplot(111)

# time_stamp_odom_csv = '/home/hirayama-d/research_ws/メディアンなし&ボクセルなし2.0以下点群ifの外にリサンプリングとか出してみた - メディアンなし&ボクセルなし2.0以下点群ifの外にリサンプリングとか出してみた.csv'


# time_stamp_odom_csv = '/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_ICMRE/2029-09-28_odom_amcl_rsj_kairyou_sabun.csv'
# time_stamp_odom_csv ='/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_ICMRE/2023-09-30_rsj_icmre_v4.csv'
# time_stamp_odom_csv = '/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_ICMRE/2023-11-01_rsj_icmre - 2023-11-01_rsj.csv'
time_stamp_odom_csv = '/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_ICMRE_modify/2023-11-01_rsj_icmre_modify - 2023-11-01_rsj_v1.csv'
rows = []
with open(time_stamp_odom_csv) as f:   
    reader = csv.reader(f)
    rows = [row for row in reader]
    print(rows)

odom_data = np.float_(np.array(rows).T)

# plt.xticks(range(0,100,10))
# plt.yticks(range(0.3,0.3,0.1))
# plt.yticks(range(-4,1,1))
# plt.title("Y-AMCL-Error")
plt.grid(linestyle='dotted')
plt.xlabel("t [s]")
plt.ylabel("y [m]")


# 10個ずつ間引いたデータでプロットしてみる

time = odom_data[0]
odom_x = odom_data[1]
odom_y = odom_data[2]
amcl_x = odom_data[3]
amcl_y = odom_data[4]
rsj_x = odom_data[5]
rsj_y = odom_data[6]
kairyou_x = odom_data[7]
kairyou_y = odom_data[8]

odom_amcl_x = odom_data[9]
rsj_amcl_x = odom_data[10]
kairyou_amcl_x = odom_data[11]

odom_amcl_y = odom_data[12]
rsj_amcl_y = odom_data[13]
kairyou_amcl_y = odom_data[14]
likelihood_state = odom_data[15]
step = 1
plt.plot(time[::step],odom_amcl_y[::step],label='Odometry-AMCL',linewidth=1,c='blue')
plt.plot(time[::step],rsj_amcl_y[::step],label='rsj-AMCL',linewidth=1,c='green')
plt.plot(time[::step],kairyou_amcl_y[::step],label='kairyou-AMCL',linewidth=1,c='magenta')
plt.ylim(-0.25,0.3) ##スケールをあわせる
# plt.ylim(-0.25,0.1) ##スケールをあわせる
# likelihood_stateが1である位置にテキストをプロット
for t, state in zip(time[::step], likelihood_state[::step]):
    if state == 1:
        ax.plot(t, -0.23, 'o', color='purple', markersize=3)
    else:
         # 赤みが強いオレンジ色の点をプロット
        ax.plot(t, -0.23, 'o', color=(1, 0.5, 0), markersize=3)



# plt.yticks(np.arange(-1,12,1))
# plt.plot(time[::step],odom_y[::step],linewidth=3)
# plt.plot(time[::step],amcl_y[::step],linewidth=3,c='green')
# plt.plot(time[::step],propose_y[::step],linewidth=3,c='orange')
# plt.legend(fontsize=15)
# plt.savefig("amcl_sabun_kairyou_y.png")
plt.show()