

# 白線からひげの候補地Fの集合を作る関数
"""
Input:
    ・白線の始点と終点の座標を持つ白線ポリゴン集合（m個）
    ・一つの白線ポリゴンを分裂させたい個数nを与える（f1,f2,...fn)
    →個数ではなくて分割したい長さ0.1m間隔(10cm)で区切る
Output:
    ・ポリゴン数m * 各ポリゴン内に分裂されたn個のひげ候補地を持つリストF(m*n個)
    ・ポリゴンWm(m=0~9でループ:合計10個)の各ポリゴンをn=10個に分裂するとき
        ・Wm_F = {fm*n+1:[xm*n+1,ym*n+1],fm*n+2:[xm*n+2,ym*n+2],....fm*n+n:[xm*n+n,ym*n+n]}が作れる
    
"""

import matplotlib.pyplot as plt

# ----以下パラメータなので適切なものに修正が必要----
white_lane = [[(0,1),(10,1)],[(0,-1),(10,-1)]]# 白線ポリゴンの中心座標を結ぶ始点と終点の座標：[w1_A(x1,y1),w1_B(x2,y2)]
robot_position = [(0,0,0),(1,0,0)] #robotoの自己位置リスト(x,y,θ) 
cv_edge_coodinate   = [(3.35,1.97),(3.35,-1.97),(0.55,-0.23),(0.55,0.24)] #cvの画角の端4点をグローバルな座標系に投影したときの座標


print(len(white_lane))

def divide_line_segment(A, B, n):
    # 始点Aと終点Bの座標
    x1, y1 = A
    x2, y2 = B

    # 分割点の座標を格納するリスト
    points = []

    # 分割点の計算
    for i in range(1, n+1):
        # 分割点のx座標
        x = x1 + (x2 - x1) * i / (n + 1)
        # 分割点のy座標
        y = y1 + (y2 - y1) * i / (n + 1)
        points.append((x, y))

    return points




F=[] #ひげ候補地Fのリスト
divide_n = 10 #1つのポリゴンを何個に分割するか：始点と終点は含まれない
for i in range(len(white_lane)): #白線ポリゴンのループ
    candidate_coodinate = divide_line_segment(white_lane[i][0],white_lane[i][1],divide_n)
    F.append(candidate_coodinate) 
print(F)

"""
    内外判定用の範囲の4点を求める関数
Input:
    ・robotの自己位置x,y,θ
Output:
    ・受け取ったロボットの自己位置に対応したカメラが投影された４つの座標を返す
"""

"""
Input:
    ・robotの自己位置x,y,θ

Output:
    ・X-Fのboolean配列

"""



# # 始点Aと終点Bの座標
# A = (1, 0)
# B = (1, 10)

# C = (-1, 0)
# D = (-1, 10)

# # 分割数n
# n = 10

# # 線分ABをn個に分割した座標の計算
# divided_points_AB = divide_line_segment(A, B, n)
# divided_points_CD = divide_line_segment(C, D, n)
# # プロット
# plt.plot(0,0,marker='x',color='black',label='robot')
# plt.plot([A[0], B[0]], [A[1], B[1]], marker='o',color='blue', label='Line AB')
# plt.plot([C[0], D[0]], [C[1], D[1]], marker='o',color='blue', label='Line CD')

# for point in divided_points_AB:
#     plt.plot(point[0], point[1], marker='o', color='red')

# for point in divided_points_CD:
#     plt.plot(point[0], point[1], marker='o', color='red')

# # 軸ラベル
# plt.xlabel('X')
# plt.ylabel('Y')

# # 凡例表示
# plt.legend()

# # プロット表示
# plt.show()
