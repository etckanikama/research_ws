#!/bin/sh

# cd ~/research_ws
# sleep 3
# # アクティブなウィンドウのみスクリーンショットするコマンド
# scrot '%Y-%m-%d_%H-%M-%S.png' -u -e 'mv $f ~/Pictures'
# # initial_xを1から10まで0.1刻みで変化させる
for initial_y in $(seq -3.6 0.1 -1.0)
do
    # ローカライザビリティの計算ループ
    python3 calculate_Localizability.py 9.7 $initial_y 1.57
    python3 particle_distribution_color_plot_3D.py 9.7 $initial_y 1.57
    sleep 1
done

for initial_x in $(seq 9.7 -0.1 0.0)
do
    python3 calculate_Localizability.py $initial_x -1.0 -3.14
    python3 particle_distribution_color_plot_3D.py $initial_x -1.0 -3.14
    
    sleep 1
done
