#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from std_msgs.msg import Header
import struct
import ctypes

def point_cloud_callback(point_cloud):
    # Header情報の作成
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = point_cloud.header.frame_id

    # PointCloud2メッセージの作成
    point_cloud2 = PointCloud2()
    point_cloud2.header = header
    point_cloud2.height = 1
    point_cloud2.width = len(point_cloud.points)

    # フィールドの設定
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    point_cloud2.fields = fields

    # ポイントデータの設定
    point_cloud2.data = []
    for point in point_cloud.points:
        # 浮動小数点数を符号なし整数に変換してクリップ
        x = max(min(point.x, 255.0), 0.0)
        y = max(min(point.y, 255.0), 0.0)
        z = max(min(point.z, 255.0), 0.0)

        # 変換した整数をデータに追加
        point_cloud2.data.extend([struct.pack('B', int(x)), struct.pack('B', int(y)), struct.pack('B', int(z)), struct.pack('B', 0)])

    point_cloud2.point_step = len(point_cloud2.data) // len(point_cloud.points)
    point_cloud2.row_step = point_cloud2.point_step * point_cloud2.width

    # PointCloud2をパブリッシュ
    point_cloud2_pub.publish(point_cloud2)

# ノードの初期化
rospy.init_node('point_cloud_converter')

# サブスクライバの設定
point_cloud_sub = rospy.Subscriber('/front_camera/line_points', PointCloud, point_cloud_callback)

# パブリッシャの設定
point_cloud2_pub = rospy.Publisher('output_point_cloud2_topic', PointCloud2, queue_size=10)

# ループを実行してノードを稼働させる
rospy.spin()
