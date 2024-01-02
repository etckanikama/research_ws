# 元のデータ：route1
# sample = {
#     25: [[876.1401098901098, 80.75000000000001], [876.1401098901098, 20.750000000000007], [868.6401098901099, 20.750000000000007], [868.6401098901099, 80.75000000000001]],
#     26: [[725.728021978022, 80.75000000000001], [725.728021978022, 20.750000000000007], [718.2280219780221, 20.750000000000007], [718.2280219780221, 80.75000000000001]],
#     27: [[324.6291208791209, 80.75000000000001], [324.6291208791209, 20.750000000000007], [317.1291208791209, 20.750000000000007], [317.1291208791209, 80.75000000000001]],
#     28: [[579.165, -20.750000000000007], [579.165, -80.75000000000001], [571.6650000000001, -80.75000000000001], [571.6650000000001, -20.750000000000007]],
#     29: [[437.8349999999999, -20.750000000000007], [437.8349999999999, -80.75000000000001], [430.335, -80.75000000000001], [430.335, -20.750000000000007]],
#     30: [[235.93500000000006, -20.750000000000007], [235.93500000000006, -80.75000000000001], [228.43500000000003, -80.75000000000001], [228.43500000000003, -20.750000000000007]],
#     31: [[1174.75, 18.05232558139536], [1174.75, 10.552325581395358], [1114.75, 10.552325581395358], [1114.75, 18.05232558139536]],
#     32: [[1035.7500000000002, -244.23333333333335], [1035.7500000000002, -251.73333333333338], [975.75, -251.73333333333338], [975.75, -244.23333333333335]],
#     33: [[1035.7500000000002, -356.25], [1035.7500000000002, -363.75], [975.75, -363.75], [975.75, -356.25]],
# }


# route1:逆走
# sample = {
# 25: [[846.0576923076923, 80.75000000000001], [846.0576923076923, 20.750000000000007], [838.5576923076924, 20.750000000000007], [838.5576923076924, 80.75000000000001]],
# 26: [[715.7005494505494, 80.75000000000001], [715.7005494505494, 20.750000000000007], [708.2005494505495, 20.750000000000007], [708.2005494505495, 80.75000000000001]],
# 27: [[575.3159340659341, 80.75000000000001], [575.3159340659341, 20.750000000000007], [567.8159340659341, 20.750000000000007], [567.8159340659341, 80.75000000000001]],
# 28: [[444.9587912087912, 80.75000000000001], [444.9587912087912, 20.750000000000007], [437.4587912087913, 20.750000000000007], [437.4587912087913, 80.75000000000001]],
# 29: [[144.13461538461542, 80.75000000000001], [144.13461538461542, 20.750000000000007], [136.63461538461542, 20.750000000000007], [136.63461538461542, 80.75000000000001]],
# 30: [[3.75, 80.75000000000001], [3.75, 20.750000000000007], [-3.75, 20.750000000000007], [-3.75, 80.75000000000001]],
# 31: [[948.65, 219.74999999999997], [948.65, 159.75], [941.15, 159.75], [941.15, 219.74999999999997]],
# 32: [[286.41, -20.750000000000007], [286.41, -80.75000000000001], [278.90999999999997, -80.75000000000001], [278.90999999999997, -20.750000000000007]],
# 33: [[1174.75, 7.936046511627917], [1174.75, 0.43604651162791674], [1114.75, 0.43604651162791674], [1114.75, 7.936046511627917]],
# 34: [[1174.75, -93.22674418604649], [1174.75, -100.7267441860465], [1114.75, -100.7267441860465], [1114.75, -93.22674418604649]],

# }
# route1：統合
# sample = {
# 25: [[424.90384615384613, 80.75000000000001], [424.90384615384613, 20.750000000000007], [417.4038461538462, 20.750000000000007], [417.4038461538462, 80.75000000000001]],
# 26: [[134.1071428571429, 80.75000000000001], [134.1071428571429, 20.750000000000007], [126.60714285714288, 20.750000000000007], [126.60714285714288, 80.75000000000001]],
# 27: [[3.75, 80.75000000000001], [3.75, 20.750000000000007], [-3.75, 20.750000000000007], [-3.75, 80.75000000000001]],
# 28: [[948.65, 219.74999999999997], [948.65, 159.75], [941.15, 159.75], [941.15, 219.74999999999997]],
# 29: [[861.8249999999999, -20.750000000000007], [861.8249999999999, -80.75000000000001], [854.325, -80.75000000000001], [854.325, -20.750000000000007]],
# 30: [[730.5899999999999, -20.750000000000007], [730.5899999999999, -80.75000000000001], [723.09, -80.75000000000001], [723.09, -20.750000000000007]],
# 31: [[579.165, -20.750000000000007], [579.165, -80.75000000000001], [571.6650000000001, -80.75000000000001], [571.6650000000001, -20.750000000000007]],
# 32: [[276.315, -20.750000000000007], [276.315, -80.75000000000001], [268.815, -80.75000000000001], [268.815, -20.750000000000007]],
# 33: [[1174.75, -12.296511627906979], [1174.75, -19.79651162790698], [1114.75, -19.79651162790698], [1114.75, -12.296511627906979]],
# 34: [[1174.75, -265.203488372093], [1174.75, -272.703488372093], [1114.75, -272.703488372093], [1114.75, -265.203488372093]],
# 35: [[1035.7500000000002, -142.4], [1035.7500000000002, -149.9], [975.75, -149.9], [975.75, -142.4]],
# 36: [[1035.7500000000002, -356.25], [1035.7500000000002, -363.75], [975.75, -363.75], [975.75, -356.25]],

# }

# 俺が作った1.5のひげ
sample = {
25: [[1035.75, -350.75], [1035.75, -358.25], [975.7499999999999, -358.25], [975.7499999999999, -350.75]],
26: [[303.75, 80.74999999999999], [303.75, 20.749999999999996], [296.25, 20.749999999999996], [296.25, 80.74999999999999]],
27: [[603.75, -20.749999999999996], [603.75, -80.74999999999999], [596.25, -80.74999999999999], [596.25, -20.749999999999996]],
28: [[303.75, -20.749999999999996], [303.75, -80.74999999999999], [296.25, -80.74999999999999], [296.25, -20.749999999999996]],
29: [[753.75, -20.749999999999996], [753.75, -80.74999999999999], [746.25, -80.74999999999999], [746.25, -20.749999999999996]],
30: [[153.75, 80.74999999999999], [153.75, 20.749999999999996], [146.25, 20.749999999999996], [146.25, 80.74999999999999]],
31: [[153.75, -20.749999999999996], [153.75, -80.74999999999999], [146.25, -80.74999999999999], [146.25, -20.749999999999996]],
32: [[1177.5, -71.25], [1177.5, -78.75], [1117.5, -78.75], [1117.5, -71.25]],
33: [[1177.5, -221.25], [1177.5, -228.75], [1117.5, -228.75], [1117.5, -221.25]],
34: [[1035.75, -200.74999999999997], [1035.75, -208.25], [975.7499999999999, -208.25], [975.7499999999999, -200.74999999999997]],
35: [[903.75, -20.749999999999996], [903.75, -80.74999999999999], [896.25, -80.74999999999999], [896.25, -20.749999999999996]],
36: [[603.75, 80.74999999999999], [603.75, 20.749999999999996], [596.25, 20.749999999999996], [596.25, 80.74999999999999]],
37: [[753.75, 80.74999999999999], [753.75, 20.749999999999996], [746.25, 20.749999999999996], [746.25, 80.74999999999999]],
38: [[453.74999999999994, 80.74999999999999], [453.74999999999994, 20.749999999999996], [446.25000000000006, 20.749999999999996], [446.25000000000006, 80.74999999999999]],
39: [[453.74999999999994, -20.749999999999996], [453.74999999999994, -80.74999999999999], [446.25000000000006, -80.74999999999999], [446.25000000000006, -20.749999999999996]],

}

# 各ポイントを1/100にスケーリング
scaled_sample = {}
for key, points in sample.items():
    scaled_sample[key] = [[x / 100, y / 100] for x, y in points]

# スケーリング後のデータをフォーマットに従って出力
scaled_output = ""
for key, points in scaled_sample.items():
    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    scaled_output += f"{{{x_min}, {x_max}, {y_min}, {y_max}}}, "

scaled_output = scaled_output.rstrip(', ')  # 末尾のコンマを削除
print("distributionのポリゴン用のフォーマット")
print(scaled_output)


print("rosrun sim sample_marker_publisher用のフォーマット")
# データのスケーリングと新しいフォーマットでの出力
formatted_output_corrected = ""
for key, points in sample.items():
    # 各ポイントを1/100にスケーリング
    scaled_points = [[x / 100, y / 100] for x, y in points]
    # xとyの最小値と最大値を取得
    x_values = [point[0] for point in scaled_points]
    y_values = [point[1] for point in scaled_points]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    # 新しいフォーマットで出力
    # データのみで出力
    formatted_output_corrected += f"{{{{{x_max}, {y_max}}}, {{{x_max}, {y_min}}}, {{{x_min}, {y_min}}}, {{{x_min}, {y_max}}}}}, "

formatted_output_corrected = formatted_output_corrected.rstrip(', ')  # 末尾のコンマを削除

print(formatted_output_corrected)


