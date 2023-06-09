#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import random
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState


def normalize_angle(angle):
    """
    -πから+πに正規化する関数
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def beego_initializer():
    """
    beegoの位置を変更する関数
    """
    rospy.init_node('beego_initializer', anonymous=True)
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    robot_name = rospy.get_param('~robot_name', 'beego')  # 初期配置を変更するロボット名
    random_pose = rospy.get_param('~random', True)  # randomに配置するかどうか　True or False
 
    pos_min = tuple(rospy.get_param('~pos_min', [0.0, 0.0]))
    pos_max = tuple(rospy.get_param('~pos_max', [1.0, 0.0]))
    pos = [0.0, 0.0]
    yaw_min = rospy.get_param('~yaw_min', 0.0)
    yaw_max = rospy.get_param('~yaw_max', 3.14)

    if random_pose == True:
        r = random.random()  # 0 <= r < 1.0
        pos[0] = (1-r) * pos_min[0] + r * pos_max[0]
        pos[1] = (1-r) * pos_min[1] + r * pos_max[1]
        yaw = random.uniform(yaw_min, yaw_max)
        yaw = normalize_angle(yaw)
    else:
        pos = rospy.get_param('~initial_pos', [0.0, 0.0])
        yaw = rospy.get_param('~initial_yaw', 0.0)

    rospy.loginfo('initial_x: %s' % str(pos))
    rospy.loginfo('initial_yaw: %s' % yaw)

    model = ModelState()
    model.model_name = robot_name
    model.pose.position.x = pos[0]
    model.pose.position.y = pos[1]
    model.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    model.pose.orientation.x = quat[0]
    model.pose.orientation.y = quat[1]
    model.pose.orientation.z = quat[2]
    model.pose.orientation.w = quat[3]
    model.reference_frame = "world"

    # 何回か"/gazebo/set_model_state"をPublish
    r = rospy.Rate(10)
    for i in range(10):
        model_pub.publish(model)
        r.sleep()


if __name__ == '__main__':
    try:
        beego_initializer()
    except rospy.ROSInterruptException:
        pass
