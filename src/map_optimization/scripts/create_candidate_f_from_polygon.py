import numpy as np
import matplotlib.pyplot as plt

# Function to compute the length between two points
def compute_edge_length(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# Function to interpolate between two points
def interpolate_points(p1, p2, num_points):
    x_values = np.linspace(p1[0], p2[0], num_points+2)[1:-1]
    y_values = np.linspace(p1[1], p2[1], num_points+2)[1:-1]
    return list(zip(x_values, y_values))

# Function to compute the indices of the longest edge of a rectangle
def compute_longest_edge_indices(rectangle):
    longest_edge_length = 0
    longest_edge_indices = (0, 1)
    for i in range(4):
        edge_length = compute_edge_length(rectangle[i], rectangle[(i+1)%4])
        if edge_length > longest_edge_length:
            longest_edge_length = edge_length
            longest_edge_indices = (i, (i+1)%4)
    return longest_edge_indices

# Function to interpolate between two points (including start and end points)
def interpolate_points_inclusive(p1, p2, interval_length):
    total_length = compute_edge_length(p1, p2)
    num_points = int(total_length / interval_length) + 1
    x_values = np.linspace(p1[0], p2[0], num_points)
    y_values = np.linspace(p1[1], p2[1], num_points)
    return list(zip(x_values, y_values))

# Given rectangles data
rectangles = [
    [[9.125, 0.545],[9.125, 0.47],[0.0, 0.47],[0.0, 0.545]],
    [[9.125, 1.935],[9.125, 0.545],[9.05, 0.545],[9.05, 1.935]],
    [[10.205, 1.935],[10.205, 1.86],[9.125, 1.86],[9.125, 1.935]],
    [[10.205,3.01],[10.205,1.935],[10.13,1.935],[10.13,3.01]],
    [[11.355,3.01],[11.355,1.76],[11.28,1.76],[11.28,3.01]],
    [[14.0,1.835],[14.0,1.76],[11.355,1.76],[11.355,1.835]],
    [[14.0,0.825],[14.0,0.75],[11.41,0.75],[11.41,0.825]],
    [[11.485,0.75],[11.485,-5.0],[11.41,-5.0],[11.41,0.75]],
    [[10.095,-0.545],[10.095,-5.0],[10.02,-5.0],[10.02,-0.545]],
    [[10.095, -0.47],[10.095,-0.545],[0,-0.545],[0,-0.47]]
]

# Interval for division
interval = 0.1  # 10cm

# Compute the interpolated points only for the longest edge of each rectangle
interval_based_interpolated_edges = []
for rectangle in rectangles:
    longest_edge_indices = compute_longest_edge_indices(rectangle)
    longest_edge_length = compute_edge_length(rectangle[longest_edge_indices[0]], rectangle[longest_edge_indices[1]])
    num_points_interval = int(longest_edge_length / interval) - 1
    interpolated_edge = interpolate_points(rectangle[longest_edge_indices[0]], rectangle[longest_edge_indices[1]], num_points_interval)
    interval_based_interpolated_edges.append(interpolated_edge)

# Assigning a key for each interpolated point and creating a dictionary
interpolated_points_dict = {}
key_counter = 0
for edge in interval_based_interpolated_edges:
    for point in edge:
        key = f"f{key_counter}"
        interpolated_points_dict[key] = point
        key_counter += 1
print("候補地 F:\n",interpolated_points_dict)

# Given robot's positions and path
initial_position = (0, 0)
relay_point = (10.7, 0)
goal_position = (10.7, -4.0)

# Compute the interpolated points for robot's path based on the interval
d = []
interval_d = 1
d.extend(interpolate_points_inclusive(initial_position, relay_point, interval_d))
d.extend(interpolate_points_inclusive(relay_point, goal_position, interval_d)[1:])  # avoid duplicating the relay point

# Assigning a key for each interpolated point of the robot's path and creating a dictionary
d_points_dict = {}
key_counter = 0

for point in d:
    key = f"d{key_counter}"
    d_points_dict[key] = point
    key_counter += 1

print("ロボットの自己位置 d:\n",d_points_dict)

# Plotting the rectangles, interpolated points, and robot's path
plt.figure(figsize=(10, 10))

# Plot original rectangles
for rectangle in rectangles:
    x_coords = [point[0] for point in rectangle]
    y_coords = [point[1] for point in rectangle]
    plt.plot(x_coords, y_coords, '-o', alpha=0.5, color='blue')

# Plot interpolated points for rectangles
for edge in interval_based_interpolated_edges:
    x_coords = [point[0] for point in edge]
    y_coords = [point[1] for point in edge]
    plt.scatter(x_coords, y_coords, color='red', s=15)

# Plot robot's path 'd'
robot_x_coords = [point[0] for point in d]
robot_y_coords = [point[1] for point in d]
plt.plot(robot_x_coords, robot_y_coords, '-o', alpha=0.8, color='green', label='Robot Path')

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Interpolation on Longest Edge and Robot Path with 10cm Interval")
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim([-1, 15])
plt.ylim([-6, 4])
plt.legend()
plt.show()
