import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random, math, time
import pulp

# マーカーを置く候補の座標を10cm間隔で配置
def make_data():
    # random.seed(5)
    # X = [random.randint(8, upper) for i in range(n)]
    X = np.arange(8, 11, 0.1)
    Y = np.zeros(30)
    # print(X.__len__())
    # Y = [random.uniform(-1, 1) for i in range(n)]
    # print(Y)
    return [Y, X]

# 利用者(p1)と施設(p2)の距離を測る
def distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)

# 需要変数(二値変数)行列を生成
def make_matrix(ps, r):
    def check(a):
        if a <= r: # 半径がr以下であれば要素を1に、そうでなければ要素を0
            return 1
        else:
            return 0

    unit_list = [] # plotできるようにn次正方行列に整形
    for n,m in zip(ps[0], ps[1]):
        unit = (n,m)
        unit_list.append(unit)

    return [[check(distance(p1, p2)) for p2 in unit_list] for p1 in unit_list]

# グラフ上に描画する
def draw(ps, fs, r):
    fig = plt.figure(figsize=(7,7))
    ax = plt.axes()
    plt.scatter(ps[0], ps[1], s=20) # 利用者

    for k, (i,j) in zip(fs, zip(ps[0], ps[1])):
        if k.value() == 1: # マーカーを設置する場合
            plt.scatter(i, j, s=20, color='red')
            c = patches.Circle(xy=(i, j), radius=r, fill=False) # カバー範囲
            ax.add_patch(c)
        else:
            pass

    plt.axis('scaled')
    plt.xticks(range(-5,5,1))
    plt.yticks(range(6,15,1))
    ax.set_aspect('equal') # 縦横比を調整
    plt.grid(linestyle='dotted')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    ax.hlines(y=10.095, xmin=-0.47, xmax=-5.0, color='red', linestyles='solid')
    ax.hlines(y=9.125, xmin=0.47, xmax=1.935, color='red', linestyles='solid')
    ax.hlines(y=11.28, xmin=1.76, xmax=3.01, color='red', linestyles='solid')
    ax.hlines(y=10.205, xmin=1.935, xmax=3.01, color='red', linestyles='solid')
    ax.hlines(y=11.41, xmin=0.75, xmax=-5.0, color='red', linestyles='solid')



    ax.vlines(x=0.47,ymin=0, ymax=9.125,colors='red',linestyles='solid')
    ax.vlines(x=-0.47,ymin=0, ymax=10.095,colors='red',linestyles='solid')
    ax.vlines(x=1.935,ymin=9.125, ymax=10.205,colors='red',linestyles='solid')
    ax.vlines(x=1.76,ymin=11.28, ymax=13.5,colors='red',linestyles='solid')
    ax.vlines(x=0.75,ymin=11.41, ymax=13.5,colors='red',linestyles='solid')

    ax.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0)
    plt.show()

# 解く
def solver_set(ps, r):
    size = len(ps[0])
    cs = make_matrix(ps, r) # 入力データ生成
    prob = pulp.LpProblem('set-cover')
    fs = [pulp.LpVariable('f{}'.format(i), cat='Binary') for i in range(size)] # 施設設置変数
    prob += pulp.lpSum(fs) # 目的関数
    for c in cs:
        prob += pulp.lpDot(c, fs) >= 1 # 制約条件

    s = time.time() # 時間計測
    status = prob.solve() # 問題が解けたのか、状態確認
    print('Status', pulp.LpStatus[status])
    print('z マーカー数：', prob.objective.value())
    print('処理時間：', time.time() - s)
    draw(ps, fs, r)

# n = 20 # 利用者・施設を合わせた地点数
r = 0.2 # 半径
# upper = 12 # 乱数の上限値
solver_set(make_data(), r)
# make_data(n,upper)
# 結果
# Status Optimal
# z 施設数： 9.0
# 処理時間： 0.05631732940673828

# この半径rをどう決めるかが大事
# →カメラが見える範囲4m？