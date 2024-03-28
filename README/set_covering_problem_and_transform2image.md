# 集合被覆問題を解く
```
cd src/map_optimization/scripts
python3 set_covering_problem.py
```
higeの候補地座標の決定(hige_coodinate)

# ランドマークのポリゴン化と全体ポリゴン情報の整理
上記できたhigeの候補地座標(hige_coodinate)の内容をコピー  
画像化するためのフォーマットに変換
```
python3 recmmend_hige_polygon.py
```


# ポリゴン情報をros座標系からcv座標系に変換
```
python3 create_white_map_opencv.py
```

# 実際に画像化
```
python3 create_white_map_opencv.py
```
ここで作成した画像地図をgazebo上に後にセットすることでgazebo環境での検証が可能

