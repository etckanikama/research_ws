import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# 初期ポリゴン座標
A = np.array([(0.5,0.9),(2.5,0.9),(2.5,-0.9),(0.5,-0.9),(0.5,0.9)])



polygon, = plt.plot(*A.T, 'b-')  # 初期ポリゴンの描画

# ポリゴンを指定された角度で回転させる関数
def rotate_polygon(polygon, angle):
    theta = np.radians(angle)  # 角度をラジアンに変換

    # 回転行列
    r = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # ポリゴンの各点を回転
    polygon_rotated = np.dot(polygon, r.T)
    return polygon_rotated

# プロット領域の設定
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_aspect('equal', adjustable='box')
ax.invert_yaxis()  # Y軸を反転

polygon, = plt.plot(*A.T, 'b-')  # 初期ポリゴンの描画

# アニメーションのための更新関数
def update_ros(num):
    global A
    A = rotate_polygon(A, -15)  # ポリゴンを-15度回転
    polygon.set_data(A[:, 0], A[:, 1])  # 更新されたポリゴンを描画
    return polygon,

# アニメーションの作成
ani_ros = animation.FuncAnimation(fig, update_ros, frames=6, interval=1000, blit=True)

plt.show()
