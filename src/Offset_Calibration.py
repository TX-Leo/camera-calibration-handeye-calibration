'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.6.13(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 自动化偏移标定,通过target2base和magnet2base数据,来获取识别到的tag到最终抓取点magnet的偏移magnet2target_xyzrxryrz,这个就是磁铁到tag所需要的六个偏移值(单位是m和度)
            其中target2base_H是识别到的tag的位姿(两个tag取平均就是中心点),单位是m和弧度
            magnet2base是通过用手托动机械臂到最终要抓取的磁铁位置的位姿(直接从机械臂示教界面读取就行),单位是m和弧度
            多采集几组数据生成csv文件之后放到./output/里          
'''

from H_R_t_xyzrxryrz_transform import xyzrxryrz_to_H,H_to_xyzrxryrz,xyz_to_t,EulerAngle_to_R,R_to_EulerAngle,xyz_rxryrz_to_xyzrxryrz
import numpy as np
import math
import csv

# ===============导入数据(单位是m和弧度)===============
with open('./output/target2base_xyzrxryrz.csv', 'r') as file:
    # 使用csv.reader读取每一行数据,并将其转换为列表
    target2base_xyzrxryrz = [list(map(float, row)) for row in csv.reader(file)]
with open('./output/magnet2base_xyzrxryrz.csv', 'r') as file:
    # 使用csv.reader读取每一行数据,并将其转换为列表
    magnet2base_xyzrxryrz = [list(map(float, row)) for row in csv.reader(file)]

# ===============数据处理（弧度转角度）===============
for i in range(len(target2base_xyzrxryrz)):
    for j in range(3):
        # # m转mm
        # target2base_xyzrxryrz[i][j] *= 1000
        # magnet2base_xyzrxryrz[i][j] *= 1000

        # 弧度转角度
        target2base_xyzrxryrz[i][j+3] = math.degrees(target2base_xyzrxryrz[i][j+3])
        magnet2base_xyzrxryrz[i][j+3] = math.degrees(magnet2base_xyzrxryrz[i][j+3])

# ===============最终的数据-偏移矩阵===============
all_magnet2target_xyzrxryrz = []

# ===============数据处理===============
for i in range(len(target2base_xyzrxryrz)):
    # print(f'================================================================')
    # # 将xyzrxryrz 转换为 H
    # target2base_H = xyzrxryrz_to_H(np.array(target2base_xyzrxryrz[i]))
    # # print(f'target2base_H:\n{target2base_H}')
    # magnet2base_H = xyzrxryrz_to_H(np.array(magnet2base_xyzrxryrz[i]))
    # # print(f'magnet2base_H:\n{magnet2base_H}')

    # # 计算magnet2target_H = magnet2base_H * target2base_H^(-1)
    # magnet2target_H = magnet2base_H @ np.linalg.inv(target2base_H)
    # # print(f'magnet2target_H:\n{magnet2target_H}')
    # all_magnet2target_H.append(magnet2target_H)

    target2base_R = EulerAngle_to_R(np.array(target2base_xyzrxryrz[i][3:]))
    target2base_t = xyz_to_t(np.array(target2base_xyzrxryrz[i][:3]))
    magnet2base_R = EulerAngle_to_R(np.array(magnet2base_xyzrxryrz[i][3:]))
    magnet2base_t = xyz_to_t(np.array(magnet2base_xyzrxryrz[i][:3]))
    
    magnet2target_xyz = np.linalg.inv(target2base_R)@np.array(magnet2base_t.reshape(3, 1)-target2base_t.reshape(3, 1)).reshape(3, 1)
    # print(f'magnet2target_xyz:\n{magnet2target_xyz}')

    magnet2target_rxryrz = R_to_EulerAngle(magnet2base_R @ np.linalg.inv(target2base_R))
    # print(f'magnet2target_rxryrz:\n{magnet2target_rxryrz}')

    magnet2target_xyzrxryrz = xyz_rxryrz_to_xyzrxryrz(magnet2target_xyz.squeeze(),magnet2target_rxryrz.squeeze())
    print(f'magnet2target_xyzrxryrz:\n{magnet2target_xyzrxryrz}')

    all_magnet2target_xyzrxryrz.append(magnet2target_xyzrxryrz.tolist())


#====================取平均值================
# 将列表中的所有ndarray沿着第0个维度拼接成一个n*6的数组,然后沿着第0个维度计算平均值
magnet2target_xyzrxryrz = np.mean(np.stack(all_magnet2target_xyzrxryrz, axis=0), axis=0)

# ====================保存magnet2target_xyzrxryrz================
with open('./output/magnet2target_xyzrxryrz.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(magnet2target_xyzrxryrz)
