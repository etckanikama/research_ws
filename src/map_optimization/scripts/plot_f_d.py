import matplotlib.pyplot as plt

# 座標データの定義
f1 = [(x, 1) for x in [i * 0.5 for i in range(1, 21)]]
f2 = [(x, -1) for x in [i * 0.5 for i in range(1, 21)]]
d = [(x, 0) for x in range(6)]

# Bounding boxesの基本定義
A1_base = [(0.5,0.75), (0.5,1.5), (2.5,1.5), (2.5,0.75)]
A2_base = [(0.5,-0.75), (0.5,-1.5), (2.5,-1.5), (2.5,-0.75)]

def translate_points(points, dx, dy):
    """Given a list of points, translate each by dx and dy."""
    return [(x+dx, y+dy) for x, y in points]

def plot_data():
    fig, ax = plt.subplots()
    
    # f1とf2のデータをプロット
    if f1:
        x, y = zip(*f1)
        ax.scatter(x, y, c='r', label='f1')

    if f2:
        x, y = zip(*f2)
        ax.scatter(x, y, c='g', label='f2')

    # グラフの範囲を設定して固定
    ax.set_xlim(-1, 11)
    ax.set_ylim(-2, 2)

    for point in d:
        dx, dy = point
        A1 = translate_points(A1_base, dx, dy)
        A2 = translate_points(A2_base, dx, dy)

        # Bounding boxesのデータをプロット
        A1_x, A1_y = zip(*A1)
        A2_x, A2_y = zip(*A2)

        ax.plot(*zip(*A1, A1[0]), linestyle='-', color='blue')  # 最後に初期点を追加して閉じる
        ax.plot(*zip(*A2, A2[0]), linestyle='-', color='purple')  # 最後に初期点を追加して閉じる

        # dの点をプロット
        ax.scatter(dx, dy, c='b', label='d' if 'd' not in [label.get_label() for label in ax.legend().get_lines()] else "")

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.grid(True)
        ax.axis('equal')  # 同じスケールの軸を保持するため
        ax.set_title('f1, f2, d Coordinates in ROS Coordinate System with Bounding Boxes')

        plt.legend()
        plt.draw()
        plt.pause(1)  # 1Hz (1秒ごと)
        
        # Clear the bounding boxes and d points for the next iteration
        lines = list(ax.lines)
        for line in lines[-2:]:  # The last 2 lines are the bounding boxes
            line.remove()

    plt.show()

if __name__ == '__main__':
    plot_data()
