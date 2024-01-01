#!/bin/sh

source ~/research_ws/devel/setup.bash 
# 
# rosrun particlefilter_simulation_basic rectangle_marker_node
# initial_xを1から10まで0.1刻みで変化させる

for initial_x in $(seq 0 0.1 10.7)
do
    # launchファイルを実行
    roslaunch sim white_lane.launch initial_x:=$initial_x initial_y:=0.0 initial_yaw:=0.0 &
    # プロセスIDを取得
    pid_launch=$!
    sleep 2

    # # gazebo用の地図publisher
    rosrun sim sample_marker_publisher.cpp &
    pid_rosrun_map=$!
    sleep 1

    # # rosrunでノードを起動
    rosrun sim likelyfood_distribution_for_gazebo.cpp $initial_x 0.0 0.0 &
    # プロセスIDを取得
    pid_rosrun=$!
    sleep 1

    rosrun sim line_detection_opencv5_homography_timer_for_gazebo.py &
    sleep 1
    pid_rosrun2=$!

    # # # 数秒待つ（例：15秒）
    sleep 10
    scrot "matching-${initial_x}-0.0-0.0.png" -u -e 'mv $f /home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/output/matching'

    # プロセスを終了
    kill $pid_launch
    kill $pid_rosrun_map
    kill $pid_rosrun
    kill -9 $pid_rosrun2

    # 更に数秒待つ（例：25秒）
    sleep 25
done

for initial_y in $(seq 0 -0.1 -3.6)
do
    # launchファイルを実行
    roslaunch sim white_lane.launch initial_x:=10.7 initial_y:=$initial_y initial_yaw:=-1.57 &
    # プロセスIDを取得
    pid_launch=$!
    sleep 2
    # gazebo用の地図publisher
    rosrun sim sample_marker_publisher.cpp &
    pid_rosrun_map=$!
    sleep 1

    # rosrunでノードを起動
    rosrun sim likelyfood_distribution_for_gazebo.cpp 10.7 $initial_y -1.57 &
    # プロセスIDを取得
    pid_rosrun=$!
    sleep 1

    rosrun sim line_detection_opencv5_homography_timer_for_gazebo.py &
    sleep 1
    pid_rosrun2=$!

    # 数秒待つ（例：15秒）
    sleep 10
    scrot "matching-10.7-${initial_y}--1.57.png" -u -e 'mv $f /home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/output/matching'


    # プロセスを終了
    kill $pid_launch
    kill $pid_rosrun_map
    kill $pid_rosrun
    kill -9 $pid_rosrun2

    # 更に数秒待つ（例：25秒）
    sleep 25
done


