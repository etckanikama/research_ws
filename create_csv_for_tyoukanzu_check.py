import csv

# 24degのホモグラフィ行列（キャリブレーション済み）
# H=  [[ 0.00000000e+00 , 6.56249884e-03 , 4.65000010e+00],
#  [-8.59375000e-03 , -5.20833046e-04 , 2.79999995e+00],
#  [-0.00000000e+00 , 3.54166667e-02  ,1.00000000e+00]]
# 23degのホモグラフィ行列（理論値なのでキャリブレーションはまだ）
# H=  [[ 0.00000000e+00 , 1.92402378e-02 , 7.21000004e+00],
#  [-1.48593746e-02 , -5.09759765e-04 , 4.80999994e+00],
#  [-0.00000000e+00 , 6.86186144e-02 , 1.00000000e+00]]
# 22.5deg(キャリブレーション済み)
H=  [[ 0.00000000e+00 , 1.82314781e-02 , 9.97000027e+00],
 [-1.93749994e-02 , -1.32478571e-03 , 6.19999981e+00],
 [-0.00000000e+00 , 8.55413069e-02 , 1.00000000e+00]]

def transform_check(u,v,H):
  x = (H[0][0]*u + H[0][1]*v + H[0][2])/(H[2][0]*u + H[2][1]*v + H[2][2])
  y = (H[1][0]*u + H[1][1]*v + H[1][2])/(H[2][0]*u + H[2][1]*v + H[2][2])
  return x,y

# # 左上
# u1,v1 = 0,0
# x1,y1 = transform_check(u1,v1,H)
# print("x1:{0},y1:{1}".format(x1,y1))

# # 右上
# u2,v2 = 640,0
# x2,y2 = transform_check(u2,v2,H)
# print("x2:{0},y2:{1}".format(x2,y2))

# # 真ん中上
# u5, v5 = 320,0
# x5,y5 = transform_check(u5,v5,H)
# print("x5:{0},y5:{1}".format(x5,y5))
# # 右下
# u3,v3 = 640,360
# x3,y3 = transform_check(u3,v3,H)
# print("x3:{0},y3:{1}".format(x3,y3))


# # 左下
# u4,v4 = 0,360
# x4,y4 = transform_check(u4,v4,H)
# print("x4:{0},y4:{1}".format(x4,y4))


cv_mx = [[320, i] for i in range(361)]
print(cv_mx)
rs_mx = []
for k in range(len(cv_mx)):
  x,y = transform_check(cv_mx[k][0],cv_mx[k][1],H)
  rs_mx.append([x,y])
print(rs_mx)

# nx = [item[0] for item in rs_mx]
# differences = [nx[i+1] - nx[i] for i in range(len(nx)-1)]
# print(differences)

# CSVファイルに出力
with open('22.5deg.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    
    # ヘッダーを出力 (必要であれば)
    # csvwriter.writerow(["cv_num", "rs_num"])

    # データの出力
    for c, r in zip(cv_mx, rs_mx):
        csvwriter.writerow([c[1], r[0]])

print("CSVファイルに出力しました。")