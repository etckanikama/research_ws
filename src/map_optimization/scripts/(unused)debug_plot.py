import matplotlib.pyplot as plt
import ast
import os

# F と d のデータを読み込む関数
def load_dicts(f_path, d_path):
    with open(f_path, "r") as file:
        data = file.read()
        F = ast.literal_eval(data[data.find('{'):])

    with open(d_path, "r") as file:
        data = file.read()
        d = ast.literal_eval(data[data.find('{'):])
    return F, d

# output_detailed ファイルから d_f ペアを読み込む関数
def read_output_detailed(file_path):
    d_f_pairs = []
    with open(file_path, "r") as file:
        for line in file:
            parts = line.strip().split(": ")
            d_key = parts[0]
            f_keys = parts[1].split(", ") if len(parts) > 1 else []
            d_f_pairs.append((d_key, f_keys))
    return d_f_pairs

# F の全ての点をプロットし、d キーに対応する f キーをハイライトする関数
def plot_and_highlight_Fy_dy(F, d_f_pairs_Fy_dy, output_folder):
    for d_key, f_keys in d_f_pairs_Fy_dy:
        d_key = d_key.rstrip(":")
        fig, ax = plt.subplots()
        # F の全ての点を青色でプロット
        for f_key, (x, y, _) in F.items():
            ax.plot(x, y, 'bo')
        # d キーに対応する f キーを赤色でハイライト
        for f_key in f_keys:
            if f_key in F:
                x, y, _ = F[f_key]
                ax.plot(x, y, 'ro')
        ax.set_title(f"Highlights for {d_key}")
        plt.savefig(f"{output_folder}/{d_key}_highlighted.png")
        plt.close()

# ファイルパスを指定
F_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/F.text"
d_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/d.text"
output_detailed_Fy_dy_path = "/home/hirayama-d/research_ws/src/map_optimization/scripts/output_Fy_dy_details.text"

# データを読み込む
F, d = load_dicts(F_path, d_path)
d_f_pairs_Fy_dy = read_output_detailed(output_detailed_Fy_dy_path)

# 出力フォルダを指定（存在しない場合は作成）
output_folder = "/home/hirayama-d/research_ws/src/map_optimization/output_images"
os.makedirs(output_folder, exist_ok=True)

# Fy_dy の各 d キーに対応する画像をプロットして保存
plot_and_highlight_Fy_dy(F, d_f_pairs_Fy_dy, output_folder)
