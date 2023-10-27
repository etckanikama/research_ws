#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

class ROSAnimatedScatter:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('scatter_plot_publisher', anonymous=True)
        
        # Publisherの設定
        self.image_pub = rospy.Publisher("/scatter_image", Image, queue_size=10)
        
        # CvBridgeのインスタンス化
        self.bridge = CvBridge()

        # ロボットの初期座標
        self.robot_coords = np.array([0.0, 0.0])

        # 画像を格納する変数
        self.img = None

        # 座標データの設定
        x_coords = np.linspace(-2, 2, 20) + self.robot_coords[0]
        y_coords = np.linspace(-2, 2, 20) + self.robot_coords[1]
        self.coordinates = np.array([[x, y] for y in y_coords for x in x_coords])
        self.num_points = len(self.coordinates)
        self.value_values = np.array([self.calculate_value(x, y) for x, y in self.coordinates])
        
        # プロットの設定
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter(self.coordinates[:, 0], self.coordinates[:, 1], c=self.value_values, 
                                       s=100, vmin=0.0, vmax=0.6, cmap='viridis')
        # ロボットの位置に×マーカーを追加
        self.robot_marker = self.ax.scatter(self.robot_coords[0], self.robot_coords[1], c='red', marker='x', s=150)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Scatter Plot')
        self.ax.grid(linestyle='dotted')
        
        # カラーバーを追加
        self.fig.colorbar(self.scatter)

    # x, yの座標を受け取ってvalueを計算する関数
    def calculate_value(self, x, y):
        return (x + y) / 4.0

    # ロボット座標の更新
    def update_robot_coords(self, new_coords):
        self.robot_coords = new_coords
        x_coords = np.linspace(-2, 2, 20) + self.robot_coords[0]
        y_coords = np.linspace(-2, 2, 20) + self.robot_coords[1]
        self.coordinates = np.array([[x, y] for y in y_coords for x in x_coords])

    def update_and_publish(self):
        self.value_values = np.array([self.calculate_value(x, y) for x, y in self.coordinates])
        self.scatter.set_offsets(self.coordinates)
        self.scatter.set_array(self.value_values)
        
        # ロボットマーカーの位置を更新
        self.robot_marker.set_offsets(self.robot_coords)
        
        # 画像の更新と変換
        self.fig.canvas.draw()
        img_arr = np.array(self.fig.canvas.renderer.buffer_rgba())
        image_message = self.bridge.cv2_to_imgmsg(img_arr, encoding="bgra8")
        
        # 画像のpublish
        self.image_pub.publish(image_message)
        
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.update_and_publish()
            rate.sleep()

if __name__ == '__main__':
    scatter_plot_node = ROSAnimatedScatter()
    scatter_plot_node.run()
