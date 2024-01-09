import pulp
import matplotlib.pyplot as plt
import ast 
import numpy as np
def solve_set_cover_problem(F_file_path, Fd_Mat_file_path):
    # Fx を読み込む
    with open(F_file_path, "r") as file:
        data = file.read()
        F = ast.literal_eval(data[data.find('{'):])

    # Fxdx_Mat を読み込む
    Fd_Mat = load_fd_matrix(Fd_Mat_file_path)

    # 線形計画問題を定義
    prob = pulp.LpProblem('set-cover')
    fs = [pulp.LpVariable(f'f{i}', cat='Binary') for i in range(len(Fd_Mat[0]))]

    # 目的関数
    prob += pulp.lpSum(fs)

    # 制約条件（全て0の行は無視する）
    for w in Fd_Mat:
        if sum(w) > 0:  # この行が少なくとも1つの1を含む場合のみ制約を追加
            prob += pulp.lpDot(w, fs) >= 1

    # 問題を解く
    status = prob.solve(pulp.PULP_CBC_CMD(msg=0))
    decide_f = [f.value() for f in fs]

    # 選択されたインデックスを見つける
    selected_indices = [i for i, value in enumerate(decide_f) if value == 1.0]
    print("selected:",selected_indices)

    # 選択されたインデックスに対応する Fx のキーを取得
    origin_F_key = [get_nth_key(F, i) for i in selected_indices]

    return origin_F_key

def get_nth_key(dictionary, n):
    # 辞書のキーをリストに変換し、n番目の要素を取得
    if n < len(dictionary):
        return list(dictionary)[n]
    else:
        return None

def load_F_from_file(file_path):
    """
    Load data from a text file and store it in a dictionary.

    :param file_path: The path of the file to load data from.
    :return: A dictionary containing the loaded data.
    """
    F = {}
    with open(file_path, "r") as file:
        for line in file:
            parts = line.strip().split(': ', 1)  # 最初のコロンでのみ分割
            if len(parts) == 2:
                key, value_str = parts
                # Converting the string representation of the tuple to an actual tuple
                value = eval(value_str)
                F[key] = value
            else:
                print(f"Invalid line format: {line}")
    return F

def load_fd_matrix(file_path):
    """
    指定されたファイルから二次元配列を読み込む

    :param file_path: ファイルのパス
    :return: 二次元配列
    """
    Fd_Mat = []
    with open(file_path, "r") as file:
        for line in file:
            # 各行をカンマで分割し、整数に変換してリストに追加
            row = [int(value) for value in line.strip().split(',')]
            Fd_Mat.append(row)
    return Fd_Mat

########################################################################################################################################
# 関数を使用して問題を解く例
Fx_file_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/Fx.text"
Fxdx_Mat_file_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/output_Fx_dx_mat.text"
origin_Fx_key = solve_set_cover_problem(Fx_file_path, Fxdx_Mat_file_path)
print(origin_Fx_key)

Fy_file_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/Fy.text"
Fydy_Mat_file_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/output_Fy_dy_mat.text"
origin_Fy_key = solve_set_cover_problem(Fy_file_path, Fydy_Mat_file_path)
print(origin_Fy_key)

origin_F_key = origin_Fx_key + origin_Fy_key
# origin_F_key = origin_Fy_key
print(origin_F_key)


with open("/home/hirayama-d/research_ws/src/map_optimization/scripts/dx.text", "r") as file:
    data = file.read()
    dx = ast.literal_eval(data[data.find('{'):])  # 辞書部分を抽出して変換

with open("/home/hirayama-d/research_ws/src/map_optimization/scripts/dy.text", "r") as file:
    data = file.read()
    dy = ast.literal_eval(data[data.find('{'):])  # 辞書部分を抽出して変換

# with open("/home/hirayama-d/research_ws/src/map_optimization/scripts/dxy.text", "r") as file:
#     data = file.read()
#     dxy = ast.literal_eval(data[data.find('{'):])  # 辞書部分を抽出して変換

with open("/home/hirayama-d/research_ws/src/map_optimization/scripts/F.text", "r") as file:
    data = file.read()
    origin_F = ast.literal_eval(data[data.find('{'):])

# Fx と Fy の辞書を読み込む
with open(Fx_file_path, "r") as file:
    data = file.read()
    Fx = ast.literal_eval(data[data.find('{'):])

with open(Fy_file_path, "r") as file:
    data = file.read()
    Fy = ast.literal_eval(data[data.find('{'):])


###################ひげの座標の出力################
hige_candidate = []

for key in origin_F_key:
    x = origin_F[key][0]
    y = origin_F[key][1]
    state = ''

    if key in origin_Fx_key:
        state = 'fx'
    elif key in origin_Fy_key:
        state = 'fy'

    hige_candidate.append((x, y, state))
    # keyがorigin_fxに入ってたらfx, origin_fyに入ってたらfyをstatatusに加えて白線ひげ地点として出力する
    # print(origin_F[key][0], origin_F[key][1])
print(hige_candidate)
fig = plt.figure()
ax = fig.add_subplot(111)
#################描画############################
# 全ての Fx と Fy の点をプロット
for key, value in Fx.items():
    # plt.plot(value[0], value[1], 'yo')  # 'bo' means blue color, circle marker
    plt.plot(value[1], value[0], 'yo')

for key, value in Fy.items():
    # plt.plot(value[0], value[1], 'mo') 
    plt.plot(value[1], value[0], 'mo')  # 'ro' means red color, circle marker

# # Plot all d points
# for key, value in dx.items():
#     plt.plot(value[0], value[1], 'bo')  # 'gx' means green color, x marker
# # Plot all d points
# for key, value in dy.items():
#     plt.plot(value[0], value[1], 'ro')  # 'gx' means green color, x marker

# # Plot the selected F points:配置場所のプロット
for index in origin_F_key:
    key = str(index)
    # plt.plot(origin_F[key][0], origin_F[key][1], 'cx', markersize=10)  # 'ro' means red color, circle marker
    # plt.plot(origin_F[key][1],origin_F[key][0], 'cx', markersize=10)


# plt.xticks(range(-4,4,1))
plt.yticks(range(0,15,1))
# plt.yticks(np.arange(-1,12,1))
plt.xticks(range(-4,4,1))
ax.set_aspect('equal')
ax.invert_xaxis()
plt.xlabel("y [m]")
plt.ylabel("x [m]")
# plt.title('')
plt.grid(linestyle='dotted')
plt.show()