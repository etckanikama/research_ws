import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class AnimatedScatterWithDynamicFunc:
    def __init__(self, coordinates):
        self.coordinates = coordinates
        self.num_points = len(coordinates)
        self.value_values = np.array([self.calculate_value(x, y, 0) for x, y in coordinates])
        
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter(self.coordinates[:, 0], self.coordinates[:, 1], c=self.value_values, 
                                       s=100, vmin=-1.0, vmax=1.0, cmap='viridis')
        
        # カラーバーの追加
        cbar = plt.colorbar(self.scatter)

        # 軸ラベルの設定
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Animated Scatter Plot with Dynamic Value Calculation')
        self.ax.grid(linestyle='dotted')
        
        # アニメーションの作成
        self.ani = FuncAnimation(self.fig, self.update, frames=100, init_func=self.init, blit=True)
    
    # 初期化関数
    def init(self):
        self.scatter.set_array(np.array([]))
        return (self.scatter,)
    
    # x, yの座標とフレーム番号を受け取ってvalueを計算する関数
    def calculate_value(self, x, y, frame):
        # 偶数フレームと奇数フレームで異なる計算をする
        if frame % 2 == 0:
            return (x + y) / 4.0
        else:
            return (x - y) / 4.0
    
    # アニメーション更新関数
    def update(self, frame):
        self.value_values = np.array([self.calculate_value(x, y, frame) for x, y in self.coordinates])
        self.scatter.set_offsets(self.coordinates)
        self.scatter.set_array(self.value_values)
        return (self.scatter,)
    
    def show(self):
        plt.show()
        
    def save(self, filename):
        self.ani.save(filename, writer='ffmpeg', fps=10)

# -2から2の範囲で20個に分割するx, yの座標を生成
x_coords = np.linspace(-2, 2, 20)
y_coords = np.linspace(-2, 2, 20)

# x, y座標のペアを二次元配列として格納
coordinate_array = np.array([[x, y] for y in y_coords for x in x_coords])

# 使用例
animation_dynamic = AnimatedScatterWithDynamicFunc(coordinate_array)
animation_dynamic.show()
