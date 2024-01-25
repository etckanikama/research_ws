
import matplotlib.pyplot as plt
import numpy as np

# データセットの定義
maps = ["origin_map", "1.5_map", "hihuku_map"]
index = np.arange(len(maps))
bar_width = 0.2
y_lim_min = -0.1
y_lim_max = 0.5

# origin_map
average_x_errors_all_origin = 0.19789470047707952
average_y_errors_all_origin = 0.047151192405863435
average_yaw_errors_all_origin = 0.013601192315214877
std_x_errors_all_origin = 0.23615420762086473
std_y_errors_all_origin = 0.06433139885839752
std_yaw_errors_all_origin = 0.009785806175403235

# 1.5_map
average_x_errors_all_1_5 = 0.022109781077861823
average_y_errors_all_1_5 = 0.02171949870558911
average_yaw_errors_all_1_5 = 0.023027448805953345
std_x_errors_all_1_5 = 0.034542160843862704
std_y_errors_all_1_5 = 0.021819236181452352
std_yaw_errors_all_1_5 = 0.025102033912648727

# hihuku_map
average_x_errors_all_hihuku = 0.013005961030749472
average_y_errors_all_hihuku = 0.027798196535098293
average_yaw_errors_all_hihuku = 0.016941519659540254
std_x_errors_all_hihuku = 0.014049216491198849
std_y_errors_all_hihuku = 0.039440301725174196
std_yaw_errors_all_hihuku = 0.01645185909787021

# 配列に格納
average_x_errors_all = [average_x_errors_all_origin, average_x_errors_all_1_5, average_x_errors_all_hihuku]
average_y_errors_all = [average_y_errors_all_origin, average_y_errors_all_1_5, average_y_errors_all_hihuku]
average_yaw_errors_all = [average_yaw_errors_all_origin, average_yaw_errors_all_1_5, average_yaw_errors_all_hihuku]
std_x_errors_all = [std_x_errors_all_origin, std_x_errors_all_1_5, std_x_errors_all_hihuku]
std_y_errors_all = [std_y_errors_all_origin, std_y_errors_all_1_5, std_y_errors_all_hihuku]
std_yaw_errors_all = [std_yaw_errors_all_origin, std_yaw_errors_all_1_5, std_yaw_errors_all_hihuku]

# 同様に、直進区間のデータを配列に格納
average_x_errors_straight = [0.25349220388280946, 0.00694841108356927, 0.007377181199970892]
average_y_errors_straight = [0.01879642099997103, 0.018518569541336714, 0.018668637671745116]
average_yaw_errors_straight = [0.012712854434618398, 0.012437946833701653, 0.012675131170783492]
std_x_errors_straight = [0.2438251771660384, 0.00484079700357102, 0.006130204352661028]
std_y_errors_straight = [0.013574789671418197, 0.014133242051371474, 0.01352915106656051]
std_yaw_errors_straight = [0.009602964563351711, 0.00935302979032218, 0.01010274751008277]

# 同様に、右折区間のデータを配列に格納
average_x_errors_right_turn = [0.02246534009254438, 0.0685168865635503, 0.03619322507692307]
average_y_errors_right_turn = [0.15143631627218931, 0.027866432024615386, 0.03302181620305326]
average_yaw_errors_right_turn = [0.045708117397674546, 0.11202313899585802, 0.07391859658106509]
std_x_errors_right_turn = [0.021250480738733927, 0.037251774113085806, 0.018858702669943432]
std_y_errors_right_turn = [0.062320018320992716, 0.035342832102169666, 0.035893015674825]
std_yaw_errors_right_turn = [0.06608779636882264, 0.1165975733081628, 0.09167221447379362]

# # 全体のデータ
# average_x_errors_all = [0.1896292906618748, 0.024029610600162787, 0.014167841596829807]
# average_y_errors_all = [0.05186328584674041, 0.024140969292672985, 0.02964901137846868]
# average_yaw_errors_all = [0.018806025744972088, 0.034887505655356484, 0.025416455253729334]
# std_x_errors_all = [0.2328027923985642, 0.035951105327118524, 0.01584575844536955]
# std_y_errors_all = [0.06641644760294362, 0.025260897562424854, 0.03983684303868454]
# std_yaw_errors_all = [0.031795733991316785, 0.06386979515980024, 0.04704683097364507]

# # 直進区間のデータ
# average_x_errors_straight = [0.25349220388280946, 0.00694841108356927, 0.007377181199970892]
# average_y_errors_straight = [0.01879642099997103, 0.018518569541336714, 0.018668637671745116]
# average_yaw_errors_straight = [0.012712854434618398, 0.012437946833701653, 0.012675131170783492]
# std_x_errors_straight = [0.2438251771660384, 0.00484079700357102, 0.006130204352661028]
# std_y_errors_straight = [0.013574789671418197, 0.014133242051371474, 0.01352915106656051]
# std_yaw_errors_straight = [0.009602964563351711, 0.00935302979032218, 0.01010274751008277]

# # 右折区間のデータ
# average_x_errors_right_turn = [0.02246534009254438, 0.0685168865635503, 0.03619322507692307]
# average_y_errors_right_turn = [0.15143631627218931, 0.027866432024615386, 0.03302181620305326]
# average_yaw_errors_right_turn = [0.045708117397674546, 0.11202313899585802, 0.07391859658106509]
# std_x_errors_right_turn = [0.021250480738733927, 0.037251774113085806, 0.018858702669943432]
# std_y_errors_right_turn = [0.062320018320992716, 0.035342832102169666, 0.035893015674825]
# std_yaw_errors_right_turn = [0.06608779636882264, 0.1165975733081628, 0.09167221447379362]

# 淡い色の設定
light_colors = ['#ADD8E6', '#90EE90', '#FFB6C1']  # 淡い青、淡い緑、淡いピンク

x_size =10
# 全体
plt.figure(figsize=(x_size, 6))
plt.ylim(y_lim_min, y_lim_max)
# X, Y, Yawの棒グラフとエラーバーを描画
for i, err_type in enumerate(["X", "Y", "Yaw"]):
    avg_errors = [average_x_errors_all, average_y_errors_all, average_yaw_errors_all][i]
    std_errors = [std_x_errors_all, std_y_errors_all, std_yaw_errors_all][i]
    plt.bar(index + i * bar_width, avg_errors, bar_width, color=light_colors[i], label=f'Average {err_type} Error')
    plt.errorbar(index + i * bar_width, avg_errors, yerr=std_errors, fmt='none', ecolor='black', capsize=5, capthick=2, elinewidth=2, markeredgewidth=2)

# グラフの設定
plt.xlabel('Maps')
plt.ylabel('Errors')
plt.title('Errors and Standard Deviations by Map')
plt.xticks(index + bar_width, maps)
plt.legend()
plt.grid(linestyle='dotted')
# グラフを表示
plt.show()


# 直進区間のグラフ作成
plt.figure(figsize=(x_size, 6))
plt.ylim(y_lim_min, y_lim_max)
for i, err_type in enumerate(["X", "Y", "Yaw"]):
    avg_errors = [average_x_errors_straight, average_y_errors_straight, average_yaw_errors_straight][i]
    std_errors = [std_x_errors_straight, std_y_errors_straight, std_yaw_errors_straight][i]
    plt.bar(index + i * bar_width, avg_errors, bar_width, color=light_colors[i], label=f'Average {err_type} Error in Straight Section')
    plt.errorbar(index + i * bar_width, avg_errors, yerr=std_errors, fmt='none', ecolor='black', capsize=5, capthick=2, elinewidth=2, markeredgewidth=2)
plt.xlabel('Maps')
plt.ylabel('Errors')
plt.title('Straight Section Errors and Standard Deviations by Map')
plt.xticks(index + bar_width, maps)
plt.legend()
plt.grid(linestyle='dotted')
plt.show()

# 右折区間のグラフ作成
plt.figure(figsize=(x_size, 6))
plt.ylim(y_lim_min, y_lim_max)
for i, err_type in enumerate(["X", "Y", "Yaw"]):
    avg_errors = [average_x_errors_right_turn, average_y_errors_right_turn, average_yaw_errors_right_turn][i]
    std_errors = [std_x_errors_right_turn, std_y_errors_right_turn, std_yaw_errors_right_turn][i]
    plt.bar(index + i * bar_width, avg_errors, bar_width, color=light_colors[i], label=f'Average {err_type} Error in Right Turn Section')
    plt.errorbar(index + i * bar_width, avg_errors, yerr=std_errors, fmt='none', ecolor='black', capsize=5, capthick=2, elinewidth=2, markeredgewidth=2)
plt.xlabel('Maps')
plt.ylabel('Errors')
plt.title('Right Turn Section Errors and Standard Deviations by Map')
plt.xticks(index + bar_width, maps)
plt.legend()
plt.grid(linestyle='dotted')
plt.show()
