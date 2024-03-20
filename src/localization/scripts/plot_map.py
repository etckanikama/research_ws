import matplotlib.pyplot as plt
import csv

# CSVファイルからデータを読み込む
free_space_x = []
free_space_y = []
obstacles_x = []
obstacles_y = []

# CSVファイル名
csv_filename = 'map_data.csv'

with open(csv_filename, mode='r') as csv_file:
    csv_reader = csv.DictReader(csv_file)
    for row in csv_reader:
        map_x = float(row['map_x'])
        map_y = float(row['map_y'])
        state = int(row['state'])
        
        if state == 0:  # 自由空間
            free_space_x.append(map_x)
            free_space_y.append(map_y)
        elif state == 100:  # 障害物
            obstacles_x.append(map_x)
            obstacles_y.append(map_y)

# プロット
plt.figure(figsize=(10, 6))
plt.scatter(free_space_x, free_space_y, color='blue', s=1, label='Free Space')
plt.scatter(obstacles_x, obstacles_y, color='red', s=1, label='Obstacles')
plt.xlabel('X in ROS Coordinate System')
plt.ylabel('Y in ROS Coordinate System')
plt.title('Map Visualization in ROS Coordinate System')
plt.axis('equal')  # x軸とy軸のスケールを等しくする
plt.legend()
plt.show()
