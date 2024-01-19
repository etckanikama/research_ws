import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルを読み込む
df = pd.read_csv('/home/hirayama-d/research_ws/src/sim/src/particle_result_csv/result.csv')

# 特定のCount値を選択 (例: selected_count = 0)
selected_count = 300
filtered_df = df[df['Count'] == selected_count]
# ParticleLikelihood_ でソートして上位2つの値を取得
top_likelihoods = filtered_df['ParticleLikelihood_'].nlargest(2)
max_likelihood = top_likelihoods.iloc[0]
second_max_likelihood = top_likelihoods.iloc[1]
# プロットを準備する
plt.figure(figsize=(10, 10))

# 各パーティクルの位置と姿勢をプロットする
for _, row in filtered_df.iterrows():
    x = row['ParticlePosX_']
    y = row['ParticlePosY_']
    yaw = row['ParticlePosYAW_']
    likelihood = row['ParticleLikelihood_']
    
    dx = np.cos(yaw) * 0.05  # x方向の変化（非常に小さい）
    dy = np.sin(yaw) * 0.05  # y方向の変化（非常に小さい）


    # 色と透明度を設定
    if likelihood == max_likelihood:
        color = 'red'
        label = 'Max ParticleLikelihood'
        alpha = 1  # 半透明

    elif likelihood == second_max_likelihood:
        color = 'green'
        label = 'Second Large ParticleLikelihood'
        alpha = 1  
    else:
        color = 'blue'
        label = 'Other ParticleLikelihood'
        alpha = 0.1  # 半透明

    plt.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color=color, alpha=alpha, headwidth=10, label=label)

# 凡例の重複を除去
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys())

# グラフの表示範囲を調整する
plt.xlim(df['ParticlePosX_'].min() - 1, df['ParticlePosX_'].max() + 1)
plt.ylim(df['ParticlePosY_'].min() - 1, df['ParticlePosY_'].max() + 1)

# グラフの表示
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()