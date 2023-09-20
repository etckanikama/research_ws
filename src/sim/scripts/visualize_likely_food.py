#!/usr/bin/env python3

import rospy
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.cm as cm
import matplotlib.colors as colors

def csv_to_markers(csv_path):
    data = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i >= 400:  # Limit to the first 400 rows
                break
            data.append(row)

    data_np = np.array(data, dtype=float)

    # Normalize the 'value' for color mapping
    norm = colors.Normalize(vmin=data_np[:,4].min(), vmax=data_np[:,4].max())
    colormap = cm.get_cmap('viridis')  # You can change 'viridis' to any other colormap

    markers = []
    for i, row in enumerate(data_np):
        color = colormap(norm(row[4]))

        marker = Marker()
        marker.header.frame_id = "beego/odom"  # Assuming the frame is "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "arrows"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = row[1]
        marker.pose.position.y = row[2]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = np.sin(row[3] / 2.0)
        marker.pose.orientation.w = np.cos(row[3] / 2.0)
        marker.scale.x = 0.08  # Arrow length
        marker.scale.y = 0.05  # Arrow width
        marker.scale.z = 0.01  # Arrow height
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        markers.append(marker)

    return markers

def main():
    rospy.init_node('arrow_visualizer')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    csv_path = "/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230905_boxel_nasi_2.0ikanomi_9.0_0.0_0.0.csv"  # Directly specifying the path here
    markers = csv_to_markers(csv_path)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        for marker in markers:
            pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    main()
