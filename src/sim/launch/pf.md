# pfの回し方
```
cd reserch_ws
souce devel/setup.bush (全ターミナルで通す)
```
- ターミナル1
```
roscore
```
- ターミナル2(launchの起動)

パラメータあり（試したい地図用のtopicになるようにlauchファイルを書き換える）
```
rosparam set /use_sim_time true
roslaunch sim execute_pf_for_gazebo.launch
```

- ターミナル3(pfプログラム本体)
 rosparam set /use_sim_time true
起動時の実行方法が試したい地図とbagごとに違う（保存したいcsvごとに違う）


- ターミナル4(bag再生用)
rosparam set /use_sim_time true
rosbag play -r 0.5 ~.bag (0.5倍速で再生)