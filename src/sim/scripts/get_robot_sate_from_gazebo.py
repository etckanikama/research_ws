#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import tf.transformations
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Lock

# gazebo/model_states からのデータ
x_data_gazebo, y_data_gazebo, yaw_data_gazebo = [], [], []
# beego/diff_drive_controller/odom からのデータ
x_data_odom, y_data_odom, yaw_data_odom = [], [], []

data_lock = Lock()

def gazebo_callback(data):
    with data_lock:
        try:
            beego_index = data.name.index('beego')
            position = data.pose[beego_index].position
            quaternion = (
                data.pose[beego_index].orientation.x,
                data.pose[beego_index].orientation.y,
                data.pose[beego_index].orientation.z,
                data.pose[beego_index].orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            x_data_gazebo.append(position.x)
            y_data_gazebo.append(position.y)
            yaw_data_gazebo.append(euler[2])
        except ValueError:
            rospy.loginfo("Beego not found in Model States")

def odom_callback(data):
    with data_lock:
        position = data.pose.pose.position
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        x_data_odom.append(position.x)
        y_data_odom.append(position.y)
        yaw_data_odom.append(euler[2])

def update_plot(frame):
    with data_lock:
        plt.cla()
        if x_data_gazebo and y_data_gazebo:
            plt.plot(x_data_gazebo, y_data_gazebo, label='Gazebo Position', color='blue')
        if x_data_odom and y_data_odom:
            plt.plot(x_data_odom, y_data_odom, label='Odom Position', color='green')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Beego Position Comparison')
        plt.legend()

def listener():
    rospy.init_node('beego_state_listener', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_callback)
    rospy.Subscriber("/beego/diff_drive_controller/odom", Odometry, odom_callback)
    ani = FuncAnimation(plt.gcf(), update_plot, interval=1000, save_count=100)
    plt.show()

if __name__ == '__main__':
    listener()
