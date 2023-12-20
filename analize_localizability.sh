#!/bin/sh

# cd ~/research_ws
# sleep 3
# # アクティブなウィンドウのみスクリーンショットするコマンド
# scrot '%Y-%m-%d_%H-%M-%S.png' -u -e 'mv $f ~/Pictures'
# # initial_xを1から10まで0.1刻みで変化させる
for initial_x in $(seq 0 0.1 10.7)
do
    # ローカライザビリティの計算ループ
    python3 calculate_Localizability.py $initial_x 0.0 0.0
    python3 particle_distribution_color_plot_3D.py $initial_x 0.0 0.0
    sleep 1
done

for initial_y in $(seq 0 -0.1 -3)
do
    python3 calculate_Localizability.py 10.7 $initial_y -1.57
    python3 particle_distribution_color_plot_3D.py 10.7 $initial_y -1.57 
    
    sleep 1
done
