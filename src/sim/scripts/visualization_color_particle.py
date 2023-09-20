#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import random

rospy.init_node("particle_visualizer")

marker_pub = rospy.Publisher("particle_markers", MarkerArray, queue_size=10)

# サンプルのパーティクルデータ
class Particle:
    def __init__(self, x, y, z, likelihood):
        self.x = x
        self.y = y
        self.z = z
        self.likelihood = likelihood

particles = [
    Particle(random.uniform(0.0, 5.0), random.uniform(0.0, 5.0), 0.0, random.uniform(0.0, 1.0)) for _ in range(10)
]

def visualize_particles():
    marker_array = MarkerArray()

    for idx, particle in enumerate(particles):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = idx
        marker.type = Marker.SPHERE

        marker.pose.position.x = particle.x
        marker.pose.position.y = particle.y
        marker.pose.position.z = particle.z

        # Quaternionの初期化
        marker.pose.orientation.w = 1.0

        # スケールの設定
        marker.scale.x = 0.1  # 例: 0.1mの直径の球
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # 尤度に基づいて色を設定
        max_likelihood = max([p.likelihood for p in particles])
        min_likelihood = min([p.likelihood for p in particles])
        ratio = (particle.likelihood - min_likelihood) / (max_likelihood - min_likelihood)
        marker.color.r = ratio
        marker.color.g = 0
        marker.color.b = 1 - ratio
        marker.color.a = 1.0  # 透明度
        
        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

# 一定の間隔でパーティクルをpublishする
rate = rospy.Rate(10)  # 10Hzで更新
while not rospy.is_shutdown():
    visualize_particles()
    rate.sleep()
