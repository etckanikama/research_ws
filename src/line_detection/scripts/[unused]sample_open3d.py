import numpy as np
import open3d as o3d

# 仮想の点群データを生成
num_points = 1000  # 点の数
points = np.random.rand(num_points, 3)  # 3次元の座標をランダムに生成

# Open3DのPointCloudオブジェクトに変換
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# 点群の表示
o3d.visualization.draw_geometries([point_cloud])
