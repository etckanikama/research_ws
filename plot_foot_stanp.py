import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import math

fig = plt.figure()
ax = fig.add_subplot(111)

csv_path = '/home/hirayama-d/research_ws/estimate_straight.csv'
# font_path = '/usr/share/fonts/TakaoFonts_00303.01/TakaoGothic.ttf'
# font_prop = FontProperties(fname=font_path)
# plt.rcParams['font.family'] = font_prop.get_name()
rows = []
with open(csv_path) as f:   
    reader = csv.reader(f)
    rows = [row for row in reader]


data = np.float_(np.array(rows).T)



# vx = []
# vy = []

# for x,y,th in zip(data[0], data[1], data[2]):
#     vx.append(x + x*math.cos(th))
#     vy.append(y + y*math.sin(th))


plt.xticks(range(-3,3,1))
plt.yticks(range(0,10,1))
ax.set_aspect('equal')
ax.invert_xaxis()
plt.grid(linestyle='dotted')
plt.xlabel("y")
plt.ylabel("x")

cnt = 0
# plt.quiver(data[1], data[0], vy, vx)
for x, y, th in zip(data[0], data[1], data[2]):
    cnt += 1
    if cnt == 15:
        # 始点(ｙ，ｘ)、終点（ｙ, x)
        p = plt.arrow(y,x, 0.2*math.sin(th), 0.2*math.cos(th),width=0.07, head_length=0.15,head_width=0.2)
        cnt = 0

s = plt.hlines(y=3.6, xmin=-2, xmax=-0.5, color='red', linestyles='solid')
plt.hlines(y=3.6, xmin=0.5, xmax=2, color='red', linestyles='solid')
plt.hlines(y=4.6, xmin=-2, xmax=-0.5, color='red', linestyles='solid')
plt.hlines(y=4.6, xmin=0.5, xmax=2, color='red', linestyles='solid')
plt.vlines(x=-0.5,ymin=0, ymax=3.6,colors='red',linestyles='solid')
plt.vlines(x=0.5,ymin=0, ymax=3.6,colors='red',linestyles='solid')
plt.vlines(x=-0.5,ymin=4.6, ymax=10,colors='red',linestyles='solid')
plt.vlines(x=0.5,ymin=4.6, ymax=10,colors='red',linestyles='solid')
plt.legend([p,s],['footprint','whilte_lane'])

# plt.legend()
# h,l = plt.gca().get_legend_handles_labels()
# h.append(p)
# l.append('Lokale Orientierung')
# plt.legend(h,l)
plt.show()
# plt.savefig('senkai.png')
# plt.savefig('tyokusen.png')

# plt.plot(data[1], data[0], linestyle='solid', marker='o')
# plt.savefig("test.png")