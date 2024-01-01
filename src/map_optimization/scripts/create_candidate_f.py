import numpy as np
import matplotlib.pyplot as plt
import math 

def calculate_distance(p1, p2):
    """二点間の距離を計算する"""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def interpolate_points(p1, p2, interval):
    """二点間を指定された間隔で内分する点のリストを返す"""
    distance = calculate_distance(p1, p2)
    num_points = int(distance // interval)
    return [(p1[0] + (p2[0] - p1[0]) * i / num_points, p1[1] + (p2[1] - p1[1]) * i / num_points) for i in range(1, num_points + 1)]

# rectangles = [
#     [[9.125, 0.545], [9.125, 0.47], [0.0, 0.47], [0.0, 0.545]],
#     [[0.545,-1.465],[0.545,-3.610],[0.470,-3.610],[0.470,-1.465]],
#     [[3.335,-1.465],[3.335,-3.610],[3.260,-3.610],[3.260,-1.465]],
#     [[5.250,-1.465],[5.250,-3.610],[5.175,-3.610],[5.175,-1.465]],
#     [[14.0,1.835],[14.0,1.76],[11.355,1.76],[11.355,1.835]],
# ]
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

# 分割点
# interval = 0.1 # 10cm
interval = 0.75 # 10cm
# 候補地F
F = {}
key_counter = 0
for rectangle in rectangles:
    rectangle_with_closure = rectangle + [rectangle[0]]
    edge_lengths = [calculate_distance(rectangle_with_closure[i], rectangle_with_closure[i+1]) for i in range(len(rectangle))]
    longest_edge_index = edge_lengths.index(max(edge_lengths))
    shortest_edge_index = edge_lengths.index(min(edge_lengths))
    p1_shortest, p2_shortest = rectangle_with_closure[shortest_edge_index], rectangle_with_closure[(shortest_edge_index+1) % len(rectangle)]
    is_x_direction = abs(p1_shortest[0] - p2_shortest[0]) > abs(p1_shortest[1] - p2_shortest[1])
    direction = 'y-long' if is_x_direction else 'x-long'
    if is_x_direction:
        mid_point_shortest = (p1_shortest[0] + p2_shortest[0]) / 2
    else:
        mid_point_shortest = (p1_shortest[1] + p2_shortest[1]) / 2
    
    p1_longest, p2_longest = rectangle_with_closure[longest_edge_index], rectangle_with_closure[(longest_edge_index+1) % len(rectangle)]
    interpolated_points = interpolate_points(p1_longest, p2_longest, interval)
    # 辞書型配列Fの作成
    for point in interpolated_points:
        key = f'f{key_counter}'
        if is_x_direction:
            F[key] = (mid_point_shortest, point[1], direction)
        else:
            F[key] = (point[0], mid_point_shortest, direction)
        key_counter += 1

    # 描画
    for point in interpolated_points:
        if is_x_direction:
            plt.scatter(mid_point_shortest, point[1], color='orange', marker='o', s=50)
        else:
            plt.scatter(point[0], mid_point_shortest, color='orange', marker='o', s=50)
    for i in range(len(rectangle)):
        p1, p2 = rectangle_with_closure[i], rectangle_with_closure[i+1]
        color = 'blue' if i == longest_edge_index else 'red' if i == shortest_edge_index else 'black'
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, linewidth=2)

print(F)

plt.figure(figsize=(10, 10))
plt.xlabel("X")
plt.ylabel("Y")
plt.xlim([-1, 10])
plt.ylim([-4, 2])
plt.title("Polygon Plot with Longest Edge in Blue and Shortest Edge in Red")
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
