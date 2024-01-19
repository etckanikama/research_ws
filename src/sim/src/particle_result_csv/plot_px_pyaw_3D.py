import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

# ３Dプロット

# コマンドライン引数の解析
parser = argparse.ArgumentParser(description='3D Scatter Plot for specified Count value')
parser.add_argument('count', type=int, help='Count value to plot')
args = parser.parse_args()

# CSVファイルを読み込む
df = pd.read_csv('/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/result.csv')# Replace with your CSV file path

# 特定のCount値を選択
# コマンドラインから指定されたCount値を取得
selected_count = args.count
filtered_df = df[df['Count'] == selected_count]

# データ抽出
beego_x_x = filtered_df['Px-BeegoX']
beego_theta_theta = filtered_df['Pyaw-BeegoYaw']
likelihood = filtered_df['ParticleLikelihood_']

# 3Dグラフを描画
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 3D散布図
ax.scatter(beego_x_x, beego_theta_theta, likelihood)

# 軸のラベル
ax.set_xlabel('Px - Beego X')
ax.set_ylabel('Pyaw - Beego Theta')
ax.set_zlabel('Particle Likelihood')

# タイトル
ax.set_title(f'3D Scatter Plot for Count {selected_count}')

plt.show()
