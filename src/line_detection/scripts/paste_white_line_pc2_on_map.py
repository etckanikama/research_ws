#!/usr/bin/env python3

import rospy
import math
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped

class PointCloudPaster:
    def __init__(self):
        self.pub = rospy.Publisher('point_cloud_out', PointCloud2, queue_size=10)
        self.final_pub = rospy.Publisher('final_point_cloud_out', PointCloud2, queue_size=10)
        self.accumulated_points = []
        rospy.Subscriber('/front_camera/line_points', PointCloud, self.callback_point_cloud)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.callback_amcl_pose)
        self.robot_pose = None
        self.last_msg_time = rospy.get_time()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_for_inactive_topic)  # Check every second

    def transform_point_cloud(self, points, rx, ry, rtheta):
        transformed_points = []
        for px, py, pz in points:
            tx = px * math.cos(rtheta) - py * math.sin(rtheta) + rx
            ty = px * math.sin(rtheta) + py * math.cos(rtheta) + ry
            transformed_points.append((tx, ty, pz))
        return transformed_points
    
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

    def callback_amcl_pose(self, msg):
        pose = msg.pose.pose
        rx, ry = pose.position.x, pose.position.y
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        rtheta = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        self.robot_pose = (rx, ry, rtheta)
    
    def check_for_inactive_topic(self, event):
        if rospy.get_time() - self.last_msg_time > 5.0:  # 5 seconds timeout
            self.publish_final_point_cloud()
            self.timer.shutdown()  # Stop the timer

    def publish_final_point_cloud(self):
        if self.accumulated_points:
            header = Header(stamp=rospy.Time.now(), frame_id="map")  # Assuming world frame
            final_accumulated_cloud = pc2.create_cloud_xyz32(header, self.accumulated_points)
            
            # Convert to Open3D PointCloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(self.accumulated_points))
            
            # Apply voxel grid filter
            down_pcd = pcd.voxel_down_sample(voxel_size=0.05)
            downsampled_points = np.asarray(down_pcd.points)
            
            # Convert back to PointCloud2 and publish
            downsampled_cloud = pc2.create_cloud_xyz32(header, downsampled_points)
            self.final_pub.publish(downsampled_cloud)
            rospy.loginfo("Final accumulated point cloud published!")
            
            # Continuously publish until the node is shut down
            rate = rospy.Rate(1)  # 1 Hz
            while not rospy.is_shutdown():
                self.final_pub.publish(downsampled_cloud)
                rate.sleep()

def main():
    rospy.init_node('point_cloud_paster')
    paster = PointCloudPaster()
    rospy.spin()

if __name__ == '__main__':
    main()
