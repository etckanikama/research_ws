import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

class HeatmapAnimation:
    def __init__(self, grid_size=20, grid_width=2.0, grid_height=2.0):
        self.grid_size = grid_size
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.data = np.random.rand(grid_size, grid_size)
        self.cmap = plt.get_cmap('coolwarm')

        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(self.data, cmap=self.cmap, interpolation='nearest')
        plt.colorbar(self.im)

        tick_positions = np.linspace(0, self.grid_size - 1, 6)
        tick_labels = np.linspace(0, self.grid_width, 6)
        plt.xticks(tick_positions, tick_labels)
        plt.yticks(tick_positions, tick_labels[::-1])
        plt.grid()
        plt.xlabel('X-axis (meters)')
        plt.ylabel('Y-axis (meters)')
        plt.title('Heatmap of Data')

    def update_data(self, frame):
        # 規則性のあるデータを生成
        x = np.linspace(0, self.grid_width, self.grid_size)
        y = np.linspace(0, self.grid_height, self.grid_size)
        print(x, y)
        X, Y = np.meshgrid(x, y)
        print("test", X, Y)
        new_data = np.sin(X) * np.cos(Y)  # 例: サインとコサインを使用
        print("last", new_data)
        self.im.set_data(new_data)
        return self.im,

    def create_heatmap_animation(self):
        ani = FuncAnimation(self.fig, self.update_data, frames=200, repeat=False, interval=100)
        plt.show()
        plt.close()

# クラスをインスタンス化してアニメーションを作成
heatmap_animator = HeatmapAnimation(grid_size=20, grid_width=2.0, grid_height=2.0)
heatmap_animator.create_heatmap_animation()
