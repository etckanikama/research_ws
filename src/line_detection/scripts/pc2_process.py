#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

def point_cloud_callback(msg):
    # ROSのPointCloud2メッセージをOpen3Dの点群データに変換
    pc = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
    points = []
    for p in pc:
        points.append([p[0], p[1], p[2]])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # ボクセルダウンサンプリングを実行
    voxel_size = 0.03  # ボクセルサイズを設定
    downsampled = pcd.voxel_down_sample(voxel_size)

    # ダウンサンプリング後の点群をPointCloud2メッセージに変換してpublish
    header = msg.header
    downsampled_points = []
    for point in downsampled.points:
        downsampled_points.append((point[0], point[1], point[2]))
    downsampled_msg = pc2.create_cloud_xyz32(header, downsampled_points)
    print(type(downsampled_msg))
    publisher.publish(downsampled_msg)

def main():
    rospy.init_node('point_cloud_processor', anonymous=True)
    rospy.Subscriber('/front_camera/line_points2', PointCloud2, point_cloud_callback)
    global publisher
    publisher = rospy.Publisher('point_cloud_down_sample', PointCloud2, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()