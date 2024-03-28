<!-- # map_optimization内の主に重要なコード(後に説明)
- calculate_Localizability.py
- cluster_Localizability_coodinate.py
- calculate_std_label.py
- plot_cluster_d_by_f_mono_color.py -->

# 概要
- 任意の走路にてlocalizabilityの計測  
- localizabilityの分類によるロボットの自己位置集合dの作成  
- 白線地図上を分割した配置候補地集合Fの作成  
- 候補地集合とロボットの自己位置集合をx,y方向に分割＆各方向のF-dバイナリ配列の作成
- 集合被覆問題を解く


# 事前準備
- 以下の記事の通りにlinkを固定しロボットが動かないようにする
[タイヤのリンクを固定&摩擦を０にする](gazebo_link_fixed.md)
- sim/line_detection_opencv5_homography_time_aisin_for_rsj.py内のheader.frame_idを"fixed_paritcle_posi"にしておく
  
# localizabilityの計測
処理の流れ  
① localizabilityを計測したい走路を指定分割した場所をfor文で順次移動させ、その地点でのデータを取得するようなシェルスクリプトを起動  
```
sh white_lane.sh
```

計測したい走路とは
- white_lane.sh  
地図原点(0,0,0)から交差点(0,10.7,0)までx方向に0.1cm間隔で直進後⇛交差点地点から回転を与えた初期値(10.7,0,-1.57)から(10.7,-3.6,-1.57)までy方向に-0.1mずつ移動するときの経路を示す

以下２つはスタート位置とゴールが違う走路を移動するシェルスクリプト
- white_lane_route1_reverse.sh  
- white_lane_route2.sh


※実行するshellに合わせて保存したいcsvの場所になるようにプログラム内（sim/likelyfood_distribution_for_gazebo.cpp）で書き換える  

シェル内で起動される内容
- ある静止地点を指定し、その地点（真値）の周りに等間隔に評価用パーティクルをばらまいたときの尤度計算を行い、その地点（真値）の各パーティクルの尤度をcsvに吐き出すノード（sim/likelyfood_distribution_for_gazebo.cpp）
- 白線認識をするノード(line_detection_opencv5_homography_timer_for_gazebo.py)
- その地点で観測される点群の様子をパス記載場所にスクリーンショットとして保存する機能


②①の処理が全て終わったら、①で取得した各位置のcsv全てのデータに対してlocalizabiltiyの計算とその時の固有値・固有ベクトルを可視化するためのシェルを起動
※シェルを実行する前にcsvやパスの名前を確認
※計測した走路ごとにcsvや可視化画像の保存先を変えているため、プログラム内のcsvの名前を変更

```
sh analize_localizability.sh
```
シェルで起動される内容
- calculate_Localizability.py  
ある位置・姿勢における尤度(w)のデータから、その位置におけるx方向の分散、y方向の分散を計算
（その他の情報も計算しているが、のちに使用するのは分散のみ）
- particle_distribution_color_plot_3D.py  
ある位置での固有値・固有ベクトルを可視化して図として想定する走路を示すディレクトリに保存する

# Next
[集合被覆問題のための候補地集合の作成と今まで抽出したロボットの自己位置の集合を分類して、集合被覆問題に必要なバイナリ配列を作成するまでを行う](create_varialble_set_text.md)


<!-- # localizabilityの分類
処理の流れ
③calculate_Localizability.pyで作成した各位置全てのx方向y方向の分散を計算したcsvファイルを入力とし、その時のx,y方向の標準偏差の大きさが一定閾値（今回は0.28）以上の条件などから各位置のlocalizabilityの分類を行う
```
cd src/map_optimization/scripts
python3 cluster_localizability_coodinate.py
```
全ての位置候補地を標準偏差の閾値からクラスタリングした結果を出力 -->





