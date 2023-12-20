#!/usr/bin/env python3
import rospy
import math 
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

from nav_msgs.msg import Odometry

class VelodyneMapPaster:
    def __init__(self):
        self.pub = rospy.Publisher('white_line_paster', PointCloud2, queue_size=10)
        # self.final_pub = rospy.Publisher('final_point_cloud_out', PointCloud2, queue_size=10)
        self.accumulated_points = []
        rospy.Subscriber('/front_camera/line_points', PointCloud, self.callback_point_cloud)
        rospy.Subscriber('hdl_odom', Odometry, self.callback_hdl_odom)
        self.robot_pose = None
        self.last_msg_time = rospy.get_time()
        # self.timer = rospy.Timer(rospy.Duration(1.0), self.check_for_inactive_topic)  # Check every second

    # 推定値を使ったロボット座標系への変換
    def transform_point_cloud(self, points, rx, ry, rtheta):
        transformed_points = []
        for px, py, pz in points:
            tx = px * math.cos(rtheta) - py * math.sin(rtheta) + rx
            ty = px * math.sin(rtheta) + py * math.cos(rtheta) + ry
            transformed_points.append((tx, ty, pz))
        return transformed_points
    
    def callback_hdl_odom(self, msg):
        pose = msg.pose.pose
        rx, ry = pose.position.x, pose.position.y
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        rtheta = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        self.robot_pose = (rx, ry, rtheta)

    def callback_point_cloud(self, msg_in):
        self.last_msg_time = rospy.get_time()
        if self.robot_pose is not None:
            rx, ry, rtheta = self.robot_pose
            header = Header(stamp=rospy.Time.now(), frame_id=msg_in.header.frame_id)
            points_as_tuples = [(point.x, point.y, point.z) for point in msg_in.points]
            
            # Transform the point cloud
            transformed_points = self.transform_point_cloud(points_as_tuples, rx, ry, rtheta)
            
            # Accumulate and create a PointCloud2 message
            self.accumulated_points.extend(transformed_points)
            accumulated_cloud = pc2.create_cloud_xyz32(header, self.accumulated_points)
            
            # Publish the accumulated PointCloud2 message
            self.pub.publish(accumulated_cloud)

def main():
    rospy.init_node('velodyne_map_paster')
    paster = VelodyneMapPaster()
    rospy.spin()

if __name__ == '__main__':
    main()
