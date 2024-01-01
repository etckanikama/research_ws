import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import ast
from sympy import symbols, Eq, solve
import math 
from matplotlib.path import Path


# 辞書 F と d を読み込む関数
def load_dicts(f_path, d_path):
    with open(f_path, "r") as file:
        data = file.read()
        F = ast.literal_eval(data[data.find('{'):])

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

def is_inside_polygon(point, polygon):
    path = Path(polygon)
    return path.contains_point(point)

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


# 内包判定と結果のファイル出力を行う関数
def generate_binary_matrices_and_write_to_file(F, d, Fx, Fy, dx, dy, origin_camera_range, file_prefix):
    for set_name, F_set, d_set in [("F_d", F, d), ("Fx_dx", Fx, dx), ("Fy_dy", Fy, dy)]:
        # print(set_name, F_set,d_set)
        output_data = []
        binary_matrix = [[0 for _ in range(len(F_set))] for _ in range(len(d_set))]
        print(len(binary_matrix), len(binary_matrix[0]))
        for d_index, (d_key, (d_x, d_y, d_yaw, d_state)) in enumerate(d_set.items()):
            new_camera_range = calculate_new_trapezoid_points(origin_camera_range, x_cut=2.5) #カメラの範囲を切る
            cm_x, cm_y = zip(*new_camera_range)
            gm_x = tuple(cm_x[i] * math.cos(d_yaw) - cm_y[i] * math.sin(d_yaw) + d_x for i in range(len(cm_x)))
            gm_y = tuple(cm_x[i] * math.sin(d_yaw) + cm_y[i] * math.cos(d_yaw) + d_y for i in range(len(cm_x)))
            gm_polygon = [(gm_x[i], gm_y[i]) for i in range(len(gm_x))] + [(gm_x[0], gm_y[0])]

            fulfilled_f_keys = [f_key for f_key in F_set if is_inside_polygon(F_set[f_key], gm_polygon)]
            output_data.append((d_key, fulfilled_f_keys))

        # # Write to file
        # with open(f"{file_prefix}_{set_name}_details.text", "w") as file:
        #     for d_key, f_keys in output_data:
        #         file.write(f"{d_key}: {', '.join(f_keys)}\n")



F_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/F.text"
d_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/d.text"
origin_camera_range = [(10.3, 6.5), (10.3, -6.5), (0.52, -0.21),(0.52, 0.2)]


# dy,fyを読み込むdyとfyを地図プロット
# 内包判定をfdmatを作る関数のデバッグ


F, d = load_dicts(F_path, d_path)
Fx, Fy, dx, dy = classify_dicts(F, d)
generate_binary_matrices_and_write_to_file(F, d, Fx, Fy, dx, dy, origin_camera_range, "output")





# Informing the user that the function has been executed
"Function executed. Plots are saved in the specified output folder."
