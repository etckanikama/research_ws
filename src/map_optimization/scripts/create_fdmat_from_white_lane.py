import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import ast
from sympy import symbols, Eq, solve
import math 
from matplotlib.path import Path

def write_dict_to_file(dictionary, file_path):
    """
    Write the entire contents of a dictionary to a text file as a single string.

    :param dictionary: The dictionary to write.
    :param file_path: The path of the file to write to.
    """
    with open(file_path, "w") as file:
        # 辞書全体を文字列として書き込む
        dict_str = str(dictionary)
        file.write(dict_str)
    
def calculate_new_trapezoid_points(points, x_cut):
    """
    Calculate the new trapezoid points after cutting the original trapezoid at x=x_cut.

    :param points: List of tuples representing the original trapezoid points.
    :param x_cut: The x-coordinate at which to cut the trapezoid.
    :return: List of tuples representing the new trapezoid points.
    """
    # Define the symbols
    x, y = symbols('x y')
    # Extract points
    p1, p2, p3, p4 = points
    # Define the lines of the original trapezoid
    line1 = Eq(y - p4[1], ((p1[1] - p4[1]) / (p1[0] - p4[0])) * (x - p4[0]))
    line2 = Eq(y - p3[1], ((p2[1] - p3[1]) / (p2[0] - p3[0])) * (x - p3[0]))
    # Calculate the intersection points with x=x_cut
    intersection_point1 = solve([line1, Eq(x, x_cut)], (x, y))
    intersection_point2 = solve([line2, Eq(x, x_cut)], (x, y))
    # Return the new trapezoid points
    return [
        (x_cut, intersection_point1[y]),
        (x_cut, intersection_point2[y]),
        p3,
        p4
    ]

# dの状態に基づいて色を決定する関数
def get_color_based_on_d_state(d_state):
    if d_state == 'x-long':
        return 'blue'
    elif d_state == 'y-long':
        return 'red'
    elif d_state == 'xy-short':
        return 'green'
    else:
        return 'black'

def is_inside_polygon(point, polygon):
    """
    Check if a point is inside a given polygon.

    :param point: A tuple (x, y) representing the point.
    :param polygon: A list of tuples [(x1, y1), (x2, y2), ...] representing the polygon vertices.
    :return: True if the point is inside the polygon, False otherwise.
    """
    path = Path(polygon)
    return path.contains_point(point)

############################ Fの辞書型配列の用意とFx,Fyの振り分け ##################################
# 辞書 F と d を読み込む関数
def load_dicts(f_path, d_path):
    with open(f_path, "r") as file:
        data = file.read()
        F = ast.literal_eval(data[data.find('{'):])

    # d = {}
    # with open(d_path, 'r') as file:
    #     for line in file:
    #         key, value_str = line.strip().split(': ')
    #         value = eval(value_str)
    #         d[key] = value

    with open(d_path, "r") as file:
        data = file.read()
        d = ast.literal_eval(data[data.find('{'):])
        
    return F, d

# F と d をそれぞれの辞書に分類する関数
def classify_dicts(F, d):
    Fx, Fy, dx, dy = {}, {}, {}, {}

    for key, value in F.items():
        if value[2] == 'x-long':
            Fx[key] = value
        elif value[2] == 'y-long':
            Fy[key] = value

    for key, (x, y, yaw, dstate) in d.items():
        if dstate == 'x-long':
            dx[key] = (x, y, yaw, dstate)
        elif dstate == 'y-long':
            dy[key] = (x, y, yaw, dstate)

    return Fx, Fy, dx, dy

# 内包判定と結果のファイル出力を行う関数
def generate_binary_matrices_and_write_to_file(F, d, Fx, Fy, dx, dy, origin_camera_range, file_prefix):
    for set_name, F_set, d_set in [("F_d", F, d), ("Fx_dx", Fx, dx), ("Fy_dy", Fy, dy)]:
        binary_matrix = [[0 for _ in range(len(F_set))] for _ in range(len(d_set))]
        for d_index, (d_key, (d_x, d_y, d_yaw, d_state)) in enumerate(d_set.items()):
            new_camera_range = calculate_new_trapezoid_points(origin_camera_range, x_cut=2.5) #カメラの範囲を切る
            cm_x, cm_y = zip(*new_camera_range)
            gm_x = tuple(cm_x[i] * math.cos(d_yaw) - cm_y[i] * math.sin(d_yaw) + d_x for i in range(len(cm_x)))
            gm_y = tuple(cm_x[i] * math.sin(d_yaw) + cm_y[i] * math.cos(d_yaw) + d_y for i in range(len(cm_x)))
            gm_polygon = [(gm_x[i], gm_y[i]) for i in range(len(gm_x))] + [(gm_x[0], gm_y[0])]

            for f_index, f_key in enumerate(F_set):
                if is_inside_polygon(F_set[f_key], gm_polygon):
                    binary_matrix[d_index][f_index] = 1

        with open(f"{file_prefix}_{set_name}_mat.text", "w") as file:
            for row in binary_matrix:
                file.write(','.join(map(str, row)) + '\n')

# 使用例
F_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/F.text"
d_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/d.text"
origin_camera_range = [(10.3, 6.5), (10.3, -6.5), (0.52, -0.21),(0.52, 0.2)]

F, d = load_dicts(F_path, d_path)
Fx, Fy, dx, dy = classify_dicts(F, d)
print(dy)
generate_binary_matrices_and_write_to_file(F, d, Fx, Fy, dx, dy, origin_camera_range, "output")
#########################################################################

# Fx と Fy をテキストファイルに書き込む
write_dict_to_file(Fx, "Fx.text")
write_dict_to_file(Fy, "Fy.text")

# dx と dy をテキストファイルに書き込む
write_dict_to_file(dx, "dx.text")
write_dict_to_file(dy, "dy.text")

###########################描画######################################

fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(-1, 15)
ax.set_ylim(-5, 3)
ax.set_aspect('equal', adjustable='box')
ax.grid(True)

# 初期のプロットの設定：Fとdは静的にプロット
camera_range, = ax.plot([], [], 'r-')
highlighted_points_in_F, = ax.plot([], [], 'ro', markersize=5)
highlighted_point_in_d, = ax.plot([], [], 'yo', markersize=5)  

for i, (f_key, f_value) in enumerate(F.items()):
    if i == 0:
        ax.scatter(f_value[0], f_value[1], c='blue', label='F')
    else:
        ax.scatter(f_value[0], f_value[1], c='blue')

for i, (d_key, d_value) in enumerate(d.items()):
    if i == 0:
        ax.scatter(d_value[0], d_value[1], c='green', label='d')
    else:
        ax.scatter(d_value[0], d_value[1], c='green')

ax.legend()



# アニメーションの初期化関数
def init():
    camera_range.set_data([], [])
    return camera_range,

# 初期のハイライト用プロットを設定
highlighted_points, = ax.plot([], [], 'ro', markersize=5)

# アニメーションの更新関数
def update(num):
    dx, dy, dyaw, dstate = d[f'd{num}']
    new_camera_range = calculate_new_trapezoid_points(origin_camera_range, x_cut=2.5)
    cm_x, cm_y = zip(*new_camera_range)
    # 座標変換
    gm_x = tuple(cm_x[i] * math.cos(dyaw) - cm_y[i] * math.sin(dyaw) + dx for i in range(len(cm_x)))
    gm_y = tuple(cm_x[i] * math.sin(dyaw) + cm_y[i] * math.cos(dyaw) + dy for i in range(len(cm_x)))
    # ポリゴンの作成と閉じる（修正部分）
    gm_polygon = [(gm_x[i], gm_y[i]) for i in range(len(gm_x))] + [(gm_x[0], gm_y[0])]
    camera_range.set_data(*zip(*gm_polygon))  # ポリゴンの頂点を結ぶ線を描画
    # ハイライトする点の更新
    highlighted_x = [F[f_key][0] for f_key in F if is_inside_polygon(F[f_key], gm_polygon)]
    highlighted_y = [F[f_key][1] for f_key in F if is_inside_polygon(F[f_key], gm_polygon)]
    highlighted_points.set_data(highlighted_x, highlighted_y)
    return camera_range, highlighted_points,

# アニメーションの作成
ani = animation.FuncAnimation(fig, update, frames=len(d), init_func=init, blit=True, repeat=True, interval=1000)

plt.show()
####################################################################################