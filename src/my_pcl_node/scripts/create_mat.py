import numpy as np

# -2から2の範囲で10個に分割するx, yの座標を生成
x_coords = np.linspace(-2, 2, 20)
y_coords = np.linspace(-2, 2, 20)

print(len(x_coords))
# x, y座標のペアを二次元配列として格納
coordinate_array = np.array([[x, y] for y in y_coords for x in x_coords])
# print(len(coordinate_array))
# print(coordinate_array)

def pot(x,y):
    value_values = np.random.uniform(0, 0.6)
    return value_values

for x, y in zip(coordinate_array[:, 0], coordinate_array[:, 1]):
    print(x,y)



