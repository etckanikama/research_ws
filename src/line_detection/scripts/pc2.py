#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import random
import open3d as o3d
import sys



def publish_point_cloud():
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # パブリッシュの周波数を設定
    print("rospyで呼び出しているpython-version")
    rospy.loginfo(sys.version) #3.8.10が呼び出されている

    # ポイントクラウドの座標データを作成
    points = []
    for _ in range(1000):
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = 0.0
        points.append([x, y, z])

    while not rospy.is_shutdown():
        # PointCloud2メッセージのヘッダーを作成
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        # フィールド（座標情報）を作成
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]



        # PointCloud2メッセージを作成
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        cloud_msg.fields = fields

        pub.publish(cloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 呼び出されているpythonのバージョン確認
        
        publish_point_cloud()
    except rospy.ROSInterruptException:
        pass
