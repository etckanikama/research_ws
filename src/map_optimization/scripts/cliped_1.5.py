import numpy as np
import matplotlib.pyplot as plt
import math 

def calculate_distance(p1, p2):
    """二点間の距離を計算する関数"""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def plot_x_on_shortest_edge_center(rectangle):
    """最短辺の中心にバツ印をプロットする関数"""
    rectangle_with_closure = rectangle + [rectangle[0]]
    edge_lengths = [calculate_distance(rectangle_with_closure[i], rectangle_with_closure[i+1]) for i in range(len(rectangle))]

    shortest_edge_index = edge_lengths.index(min(edge_lengths))
    shortest_edge = rectangle_with_closure[shortest_edge_index:shortest_edge_index+2]

    x_center = (shortest_edge[0][0] + shortest_edge[1][0]) / 2
    y_center = (shortest_edge[0][1] + shortest_edge[1][1]) / 2

    plt.plot(x_center, y_center, 'rx', markersize=10)  # 赤色のバツ印でプロット

def plot_points_relative_to_shortest_edge_center(rectangle, interval):
    """最短辺の中心のX座標を基準として、最長辺上に点をプロットする関数"""
    rectangle_with_closure = rectangle + [rectangle[0]]
    edge_lengths = [calculate_distance(rectangle_with_closure[i], rectangle_with_closure[i+1]) for i in range(len(rectangle))]

    longest_edge_index = edge_lengths.index(max(edge_lengths))
    longest_edge = rectangle_with_closure[longest_edge_index:longest_edge_index+2]

    shortest_edge_index = edge_lengths.index(min(edge_lengths))
    shortest_edge = rectangle_with_closure[shortest_edge_index:shortest_edge_index+2]
    x_center_shortest = (shortest_edge[0][0] + shortest_edge[1][0]) / 2

    ymin = min([point[1] for point in rectangle])
    ymax = max([point[1] for point in rectangle])

    if ymin in [longest_edge[0][1], longest_edge[1][1]] or ymax in [longest_edge[0][1], longest_edge[1][1]]:
        x0, y0 = longest_edge[0]
        x1, y1 = longest_edge[1]
        num_points = int(calculate_distance(longest_edge[0], longest_edge[1]) // interval)

        for i in range(1, num_points):
            t = i / num_points
            x = x0 + (x1 - x0) * t
            if x_center_shortest < x < x1 or x0 < x < x_center_shortest:
                y = y0 + (y1 - y0) * t
                plt.plot(x, y, 'go')  # 緑色の点でプロット

# 四角形のデータ
rectangles = [
    [[9.125, 0.545], [9.125, 0.47], [0.0, 0.47], [0.0, 0.545]],
]

plt.figure(figsize=(10, 10))

# 四角形のプロット
for rectangle in rectangles:
    rectangle_with_closure = rectangle + [rectangle[0]]
    edge_lengths = [calculate_distance(rectangle_with_closure[i], rectangle_with_closure[i+1]) for i in range(len(rectangle))]

    longest_edge_index = edge_lengths.index(max(edge_lengths))
    shortest_edge_index = edge_lengths.index(min(edge_lengths))

    for i in range(len(rectangle)):
        p1, p2 = rectangle_with_closure[i], rectangle_with_closure[i+1]
        color = 'blue' if i == longest_edge_index else 'red' if i == shortest_edge_index else 'black'
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, linewidth=2)

    plot_points_relative_to_shortest_edge_center(rectangle, 1.5)
    plot_x_on_shortest_edge_center(rectangle)

plt.xlabel("X")
plt.ylabel("Y")
plt.xlim([-1, 15])
plt.ylim([-6, 4])
plt.title("Polygon Plot with Marks on Edges (Excluding Ends)")
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
