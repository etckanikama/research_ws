#!/usr/bin/env python3
import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import csv

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('pointcloud_transformer', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pc_subscriber = rospy.Subscriber('/front_camera/line_points_pc2', PointCloud2, self.pc_callback)
        self.pc_publisher = rospy.Publisher('/transformed_pointcloud', PointCloud2, queue_size=10)
        self.csv_file_path = 'transformed_pointcloud.csv'  # CSV ファイルのパス
        # CSV ファイルを初期化
        with open(self.csv_file_path, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['x', 'y', 'z'])

    def pc_callback(self, cloud_msg):
        try:
            transform = self.tf_buffer.lookup_transform('map', cloud_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_cloud = do_transform_cloud(cloud_msg, transform)
            self.pc_publisher.publish(transformed_cloud)
            # 変換されたポイントクラウドから座標を抽出し、CSV ファイルに書き込む
            self.write_points_to_csv(transformed_cloud)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('TF transform error: %s' % str(e))

    def write_points_to_csv(self, cloud):
        points = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        with open(self.csv_file_path, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            for point in points:
                csvwriter.writerow(point)

if __name__ == '__main__':
    try:
        transformer = PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
