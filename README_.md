# 2024.03.29
*このREADMEを最初に読んでください*
# 概要
  
2024年卒業のDHAの研究引き継ぎ資料です

# 構成
- README  
- bag  
各種rosbagが含まれています
- src  
各種パッケージが含まれています
- yp-robot-params  
ロボットパラメータ(M1,さきがけ)のパラメータが含まれています  
ypspur-coordinaterのときのパスをここになるようにしてください  
- その他プログラム  
頭に(unused)と書かれたプログラムは基本的には使わないプログラムです

# 主な内容
<!-- - M1での床面ライン地図に基づく自己位置推定
    particlefilter_simulation_basicパッケージ説明が必要
    icmre時の説明


     -->
- lego-loam&hdl_localizationの推定値に白線を貼り合わせた地図を作成し、その地図で自己位置推定を行う話（2024年最終報告時の内容）
    - [グリッドマップの作成と作成した地図での自己位置推定](README/Create_WhiteLane_Gridmap_and_localization.md)
- gazebo環境内でlocalizabilityを測定し、抽出したlocalizabilityの低い位置をカバーするような地図を改良する話（修論）
    - [localizabilityの抽出の話](README/extract_localizability.md)
    - [候補地集合やロボットの自己位置集合の作成とF-dバイナリ配列の作成](README/create_varialble_set_text.md)
    - [集合被覆問題でランドマーク配置場所の決定と地図の画像化](README/set_covering_problem_and_transform2image.md)
    - [gazebo上での改良地図を使った自己位置推定をする話](README/localization_for_gazebo.md)
- Reference
    - [gazebo環境構築の話](README/SetUp_gazebo_environment.md)
    - [gazebo内でタイヤのリンク固定を切り替える話](README/gazebo_link_fixed.md)
    - [可視化プログラムの解説](README/Explain_visualize_program.md)
    <!-- - 
    [さきがけのロボットパラメータ調整

    line_testとか言うパッケージの説明をする必要がある
    ] -->
    


    