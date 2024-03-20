#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import csv
import os

def map_callback(data):
    resolution = data.info.resolution
    width = data.info.width
    height = data.info.height
    origin = data.info.origin.position

    # 現在実行しているディレクトリにCSVファイルを作成
    csv_filename = 'map_data.csv'
    
    with open(csv_filename, mode='w', newline='') as csv_file:
        fieldnames = ['map_x', 'map_y', 'state']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        writer.writeheader()
        for y in range(height):
            for x in range(width):
                index = x + y * width
                value = data.data[index]
                
                if value in [0, 100]:
                    map_x = x * resolution + origin.x
                    map_y = y * resolution + origin.y
                    writer.writerow({'map_x': map_x, 'map_y': map_y, 'state': value})

    print(f"Map data has been written to {csv_filename}")

def listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
