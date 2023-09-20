# #!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import csv
import sys
from scipy.interpolate import griddata


def create_occupancy_grid(grid_value):
    # グリッドのメタデータを設定
    map_meta_data = MapMetaData()
    map_meta_data.resolution = 0.1  # グリッドセルの解像度 (1セルのサイズ)
    map_meta_data.width = grid_value.shape[0]
    map_meta_data.height = grid_value.shape[1]
    
    # グリッドデータを0~100の範囲にスケーリング
    scaled_grid_value = (grid_value / np.max(grid_value)) * 100
    flattened_grid = scaled_grid_value.astype(int).ravel()

    # OccupancyGridメッセージを作成
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"  # tfフレームのID
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.info = map_meta_data
    occupancy_grid.data = list(flattened_grid)

    return occupancy_grid

def main():
    rospy.init_node('grid_data_visualizer')

    # if len(sys.argv) < 4:
    #     rospy.logerr("引数が指定されてないです")
    #     return

    # try:
    #     param_value1 = float(sys.argv[1])
    #     param_value2 = float(sys.argv[2])
    #     param_value3 = float(sys.argv[3])
    # except ValueError:
    #     rospy.logerr("引数は数値である必要があリマス")
    #     return

    param_value1 = 8.0
    param_value2 = 0.0
    param_value3 = 0.0
    csv_file_path = f"/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_distance/particle_coodinate_distribution_time_stamp_gazebo_down_sample_distance{param_value1}_{param_value2}_{param_value3}.csv"

    data = []
    with open(csv_file_path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(row)

    data_np = np.array(data[:400],dtype=float)
    x = data_np[:,1]
    y = data_np[:,2]
    value = data_np[:,4]*400
    grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]
    grid_value = griddata((x, y), value, (grid_x, grid_y), method='cubic')

    grid_pub = rospy.Publisher('grid_data', OccupancyGrid, queue_size=1)

    rate = rospy.Rate(1)  # 1Hzでpublish

    while not rospy.is_shutdown():
        occupancy_grid_msg = create_occupancy_grid(grid_value)
        grid_pub.publish(occupancy_grid_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
