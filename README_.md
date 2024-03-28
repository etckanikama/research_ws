# 概要
このREADMEを最初に読んでください

主な内容
- lego-loam&hdl_localizationの推定値に白線を貼り合わせた地図を作成し、その地図で自己位置推定を行う話
    - [グリッドマップの作成と作成した地図での自己位置推定](README/Create_WhiteLane_Gridmap_and_localization.md)
- gazebo環境内でlocalizabilityを測定し、抽出したlocalizabilityの低い位置をカバーするような地図を改良する話
    - [localizabilityの抽出の話](README/extract_localizability.md)
    - [候補地集合やロボットの自己位置集合の作成とF-dバイナリ配列の作成](README/create_varialble_set_text.md)
    - [集合被覆問題でランドマーク配置場所の決定と地図の画像化](README/set_covering_problem.md)
    - [gazebo上での改良地図を使った自己位置推定をする話](README/localization_for_gazebo.md)
- Reference
    - [gazebo環境構築の話](README/SetUp_gazebo_environment.md)
    - [gazebo内でタイヤのリンク固定を切り替える話](README/gazebo_link_fixed.md)


    