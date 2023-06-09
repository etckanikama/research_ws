import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random, math, time
import pulp

# 入力データ作成、利用者・施設設置可能点生成
def make_data(n, upper):
    random.seed(5)
    X = [random.randint(0, upper) for i in range(n)]
    Y = [random.randint(0, upper) for i in range(n)]
    return [X, Y]

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
    x = np.arange(0, 80, 0.1)
    plt.scatter(ps[0], ps[1], s=20) # 利用者

    for k, (i,j) in zip(fs, zip(ps[0], ps[1])):
        if k.value() == 1: # 施設を設置する場合
            plt.scatter(i, j, s=20, color='red')
            c = patches.Circle(xy=(i, j), radius=r, fill=False) # カバー範囲
            ax.add_patch(c)
        else:
            pass

    plt.axis('scaled')
    ax.set_aspect('equal') # 縦横比を調整
    plt.grid()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_xticks(np.linspace(0, 80, 9))
    ax.set_yticks(np.linspace(0, 80, 9))
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
    print('z 施設数：', prob.objective.value())
    print('処理時間：', time.time() - s)
    draw(ps, fs, r)

n = 25 # 利用者・施設を合わせた地点数
r = 20 # 半径
upper = 80 # 乱数の上限値
solver_set(make_data(n, upper), r)

# 結果
# Status Optimal
# z 施設数： 9.0
# 処理時間： 0.05631732940673828