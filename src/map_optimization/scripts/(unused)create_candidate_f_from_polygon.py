import numpy as np
import matplotlib.pyplot as plt

# 辺の長さを計算する関数
def compute_edge_length(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# 二点間の内分点を計算する関数
def interpolate_points(p1, p2, num_points):
    x_values = np.linspace(p1[0], p2[0], num_points+2)[1:-1]
    y_values = np.linspace(p1[1], p2[1], num_points+2)[1:-1]
    return list(zip(x_values, y_values))

# 四角形の最長辺のインデックスを計算する関数:fに関係
def compute_longest_edge_indices(rectangle):
    longest_edge_length = 0
    longest_edge_indices = (0, 1)
    for i in range(4):
        edge_length = compute_edge_length(rectangle[i], rectangle[(i+1)%4])
        if edge_length > longest_edge_length:
            longest_edge_length = edge_length
            longest_edge_indices = (i, (i+1)%4)
    return longest_edge_indices

# 二点間を含む内分点を計算する関数：dに関係
def interpolate_points_inclusive(p1, p2, interval_length):
    total_length = compute_edge_length(p1, p2)
    num_points = int(total_length / interval_length) + 1
    x_values = np.linspace(p1[0], p2[0], num_points)
    y_values = np.linspace(p1[1], p2[1], num_points)
    return list(zip(x_values, y_values))

# 与えられた四角形のデータ
# Given rectangles data
rectangles = [
    [[9.125, 0.545],[9.125, 0.47],[0.0, 0.47],[0.0, 0.545]],
    [[9.125, 1.935],[9.125, 0.545],[9.05, 0.545],[9.05, 1.935]],
    [[10.205, 1.935],[10.205, 1.86],[9.125, 1.86],[9.125, 1.935]],
    [[10.205,3.01],[10.205,1.935],[10.13,1.935],[10.13,3.01]],
    [[11.355,3.01],[11.355,1.76],[11.28,1.76],[11.28,3.01]],
    [[14.0,1.835],[14.0,1.76],[11.355,1.76],[11.355,1.835]],
    [[14.0,0.825],[14.0,0.75],[11.41,0.75],[11.41,0.825]],
    [[11.485,0.75],[11.485,-3.6],[11.41,-3.6],[11.41,0.75]],
    [[10.095,-0.545],[10.095,-3.6],[10.02,-3.6],[10.02,-0.545]],
    [[10.095, -0.47],[10.095,-0.545],[0,-0.545],[0,-0.47]],
    [[0.545,-1.465],[0.545,-3.610],[0.470,-3.610],[0.470,-1.465]],
    [[1.345,-1.465],[1.345,-1.540],[.545,-1.540],[.545,-1.465]],
    [[1.420,-1.465],[1.420,-3.610],[1.345,-3.610],[1.345,-1.465]],
    [[2.325,-1.465],[2.325,-3.610],[2.250,-3.610],[2.250,-1.465]],
    [[3.260,-1.465],[3.260,-1.540],[2.325,-1.540],[2.325,-1.465]],
    [[3.335,-1.465],[3.335,-3.610],[3.260,-3.610],[3.260,-1.465]],
    [[4.225,-1.465],[4.225,-3.610],[4.150,-3.610],[4.150,-1.465]],
    [[5.175,-1.465],[5.175,-1.540],[4.225,-1.540],[4.225,-1.465]],
    [[5.250,-1.465],[5.250,-3.610],[5.175,-3.610],[5.175,-1.465]],
    [[6.125,-1.465],[6.125,-3.610],[6.050,-3.610],[6.050,-1.465]],
    [[7.085,-1.465],[7.085,-1.540],[6.125,-1.540],[6.125,-1.465]],
    [[7.160,-1.465],[7.160,-3.610],[7.080,-3.610],[7.080,-1.465]],
    [[8.035,-1.465],[8.035,-3.610],[7.960,-3.610],[7.960,-1.465]],
    [[9.305,-1.465],[9.305,-1.540],[8.055,-1.540],[8.055,-1.465]],
    [[9.380,-1.465],[9.380,-3.610],[9.305,-3.610],[9.305,-1.465]]
]
# 分割の間隔
interval = 0.1  # 10cm

# 四角形の最長辺に沿った内分点の計算
interval_based_interpolated_edges = []
edge_types = []
for rectangle in rectangles:
    longest_edge_indices = compute_longest_edge_indices(rectangle)
    print(longest_edge_indices)
    longest_edge = (rectangle[longest_edge_indices[0]], rectangle[longest_edge_indices[1]])
    x_long = longest_edge[0][0] != longest_edge[1][0]  # x方向が長いか判定
    edge_type = "x-long" if x_long else "y-long"
    edge_types.append(edge_type)

    num_points_interval = int(compute_edge_length(*longest_edge) / interval) - 1
    interpolated_edge = interpolate_points(longest_edge[0], longest_edge[1], num_points_interval)
    interval_based_interpolated_edges.append(interpolated_edge)

# 各内分点にキーを割り当て、辞書を作成
interpolated_points_dict = {}
key_counter = 0
for i, edge in enumerate(interval_based_interpolated_edges):
    for point in edge:
        key = f"f{key_counter}"
        interpolated_points_dict[key] = (point[0],point[1], edge_types[i])
        key_counter += 1
print("候補地 F:\n", interpolated_points_dict)

# # ロボットの位置と経路
# initial_position = (0, 0)
# relay_point = (10.7, 0)
# goal_position = (10.7, -4.0)

# # ロボットの経路の内分点の計算
# d = []
# interval_d = 1
# d.extend(interpolate_points_inclusive(initial_position, relay_point, interval_d))
# d.extend(interpolate_points_inclusive(relay_point, goal_position, interval_d)[1:])  # 重複を避ける

# # ロボットの経路の各内分点にキーを割り当て、辞書を作成
# d_points_dict = {}
# key_counter = 0
# for point in d:
#     key = f"d{key_counter}"
#     d_points_dict[key] = point
#     key_counter += 1
# print("ロボットの自己位置 d:\n", d_points_dict)

# [先程のコードの続き]

# 四角形、内分点、ロボットの経路のプロット
plt.figure(figsize=(10, 10))

# 四角形のプロット
for rectangle in rectangles:
    x_coords = [point[0] for point in rectangle]
    y_coords = [point[1] for point in rectangle]
    plt.plot(x_coords, y_coords, '-o', alpha=0.5, color='blue')

# 内分点のプロット（辺のタイプに応じて色分け）
for i, edge in enumerate(interval_based_interpolated_edges):
    color = 'red' if edge_types[i] == "x-long" else 'blue'
    x_coords = [point[0] for point in edge]
    y_coords = [point[1] for point in edge]
    plt.scatter(x_coords, y_coords, color=color, s=15)

# # ロボットの経路 'd' のプロット
# robot_x_coords = [point[0] for point in d]
# robot_y_coords = [point[1] for point in d]
# plt.plot(robot_x_coords, robot_y_coords, '-o', alpha=0.8, color='green', label='Robot Path')

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Interpolation on Longest Edge and Robot Path with 10cm Interval")
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim([-1, 15])
plt.ylim([-6, 4])
plt.legend()
plt.show()