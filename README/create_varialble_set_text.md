# localizabilityの分類
calculate_Localizability.pyで作成した各位置全てのx方向y方向の分散を計算したcsvファイルを入力とし、その時のx,y方向の標準偏差の大きさが一定閾値（今回は0.28）以上の条件などから各位置のlocalizabilityの分類を行う
```
cd src/map_optimization/scripts
python3 cluster_localizability_coodinate.py
```
全ての位置候補地を標準偏差の閾値からクラスタリングしたロボットの自己位置集合dを作成(d.text)

# 候補地集合Fの作成
intervalの間隔(今回は0.1m)で白線ポリゴンを分割した候補地点集合F（F.text）の作成する
```
cd src/map_optimization/scripts
python3 create_candidate_f_from_polygon.py
```
# x,y方向への各集合の分類とF-dバイナリ配列を作成
F.textとd.textを入力として、Fx,Fy,dx,dyに分類(Fx.text,Fy.text,dx.text,dy.textの作成)  
dxのときのFxやdyのときのFyを満たすようなバイナリ配列を作成(output_Fx_dx_mat.text,output_Fy_dy_mat.text)

```
cd src/map_optimization/scripts
python3 create_fdmat_from_white_lane.py
```
# Next
[集合被覆問題を解く](set_covering_problem.md)
