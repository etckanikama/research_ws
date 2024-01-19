import pandas as pd
import matplotlib.pyplot as plt
import os

# CSVファイルを読み込む
df = pd.read_csv('/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/result.csv')


# Countの最大値を取得
max_count = df['Count'].max()

# 保存するディレクトリのパス (適宜変更してください)
save_dir = '/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/output'  # Replace with your desired save directory path
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Countが0から最大値までのプロットを作成して保存
for count in range(max_count + 1):
    # 特定のCount値でデータをフィルタリング
    filtered_df = df[df['Count'] == count]

    # データ抽出
    beego_x_x = filtered_df['Px-BeegoX']
    beego_theta_theta = filtered_df['Pyaw-BeegoYaw']
    likelihood = filtered_df['ParticleLikelihood_']

    # グラフを描画
    plt.figure(figsize=(12, 6))

    # X軸に関するサブプロット
    plt.subplot(1, 2, 1)
    plt.scatter(beego_x_x, likelihood)
    plt.xlabel('Px - Beego X')
    plt.ylabel('Particle Likelihood')
    plt.title(f'Count {count}: Likelihood vs Px-Beego X')
    plt.grid(True)

    # Y軸に関するサブプロット
    plt.subplot(1, 2, 2)
    plt.scatter(beego_theta_theta, likelihood)
    plt.xlabel('Pyaw - Beego Theta')
    plt.ylabel('Particle Likelihood')
    plt.title(f'Count {count}: Likelihood vs Pyaw-Beego Theta')
    plt.grid(True)

    # プロットの保存
    plt.savefig(os.path.join(save_dir, f'plot_count_{count}.png'))
    plt.close()

# # 特定のCount値を選択 (例: selected_count = 0)
# selected_count = 182 # 278
# filtered_df = df[df['Count'] == selected_count]

# beego_x_x = filtered_df['BeegoX-Px']
# beego_theta_theta = filtered_df['BeegoYaw-Pyaw']
# likelihood = filtered_df['ParticleLikelihood_']

# # グラフを描画
# plt.figure(figsize=(12, 6))

# # X軸に関するサブプロット
# plt.subplot(1, 2, 1)
# plt.scatter(beego_x_x, likelihood)
# plt.xlabel('Beego X - Px')
# plt.ylabel('Particle Likelihood')
# plt.title('Likelihood vs Beego X-Px')
# plt.grid(True)

# # Y軸に関するサブプロット
# plt.subplot(1, 2, 2)
# plt.scatter(beego_theta_theta, likelihood)
# plt.xlabel('Beego Theta - Pyaw')
# plt.ylabel('Particle Likelihood')
# plt.title('Likelihood vs Beego Theta-Pyaw')
# plt.grid(True)

# plt.show()


# beego_x_x = filtered_df['Px-BeegoX']
# beego_theta_theta = filtered_df['Pyaw-BeegoYaw']