'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 自动化通过设置线性步长来生成一定空间区域内的点集,顺序是除草机式,尽可能包含这个空间区域内的各个位姿(x,y,z,rx,ry,rz) (xyz单位为mm,rxryrz单位为度)
    1.generate_trajector()
        首先设定边界和步长,线性生成xyz点集,然后把xyz点集的顺序变成除草机式,然后对于每个点xyz增加几组设定的rxryrz
    2.optimize_points_order()
        优化点集的顺序,变成除草机式,使得机械臂尽可能短距离遍历这些点
    2.save_trajectory()
        将采样点的数组保存为txt配置文件
    3.test_load_trajectory()
        测试读取运动点

@Usage: 参看Main()函数
'''

import numpy as np
from scipy.spatial.distance import cdist
import pandas as pd

def generate_trajectory():
    '''
    首先设定边界和步长,线性生成xyz点集,然后把xyz点集的顺序变成除草机式,然后对于每个点xyz增加几组设定的rxryrz
    :return: new_points
    '''

    # ================设定空间轨迹参数================
    x_stride, y_stride, z_stride = 200, 100, 100
    x_min, x_max, y_min, y_max, z_min, z_max = 150, 550, 400, 600, 100, 300

    # ================初始化点集================
    points = []
    
    # ================xyz方向的个数================
    nx = (x_max - x_min) // x_stride + 1
    ny = (y_max - y_min) // y_stride + 1
    nz = (z_max - z_min) // z_stride + 1
    
    # ================xyz================
    x = np.linspace(x_min, x_max, nx)
    y = np.linspace(y_min, y_max, ny)
    z = np.linspace(z_min, z_max, nz)
    
    # ================生成三维网格坐标================
    xx, yy, zz = np.meshgrid(x, y, z)
   
    # ================将三个坐标数组转换为一维数组================
    xx = xx.reshape(-1)
    yy = yy.reshape(-1)
    zz = zz.reshape(-1)
    
    # ================将三个一维数组合并成一个二维数组================
    points = np.column_stack((xx, yy, zz))
    
    # ================直接读取修改顺序的xyz的轨迹了(除草机式),到时候生成要注释掉这些================
    points = []
    with open('./cfg/trajectory_xyz.txt', 'r') as f:
        for line in f:
            point = np.array(line.strip().split(','), dtype=np.float32)
            points.append(point)
    points = np.array(points)
    print(points)

    # ================优化xyz点集,变成除草机式的轨迹,这样能以一个较短的路径遍历所有的点(目前这个函数写的不完善,并不能用,只能手动调整顺序了,~_~)================
    # points = optimize_points_order(points)
    # print(points)
    
    # ================定义每个点的几组姿态================
    # rx1, ry1, rz1 = -179.704,0.64,-45.306 # 中间
    # rx2, ry2, rz2 = -162.413,-6.014,-44.484# 右边
    # rx3, ry3, rz3 = 165.963,-11.618,-46.15  # 上边
    # rx4, ry4, rz4 = 160.389,13.054,-44.22 # 左边
    # rx5, ry5, rz5 = -171.159,8.827,-47.008# 下边
    # rx6, ry6, rz6 = 179.279,3.65,-65.179# 左旋
    # rx7, ry7, rz7 = 178.656,4.186,-92.246# 右旋

    rx1, ry1, rz1 = 142.810, 8.65700, 126.102
    rx2, ry2, rz2 = 142.886, -9.1730, 99.1000
    rx3, ry3, rz3 = 143.429, -7.1640, 85.4380
    rx4, ry4, rz4 = 172.399, 25.4230, 133.743
    rx5, ry5, rz5 = 121.132, 14.4600, 115.035
    rx6, ry6, rz6 = 162.549, -11.975, 132.980
    rx7, ry7, rz7 = 128.944, -16.253, 130.408

    # ================将每个点增加n组姿态,组成点(x,y,z,rx,ry,rz)================
    new_points = []
    for point in points:
        new_points.append(point.tolist() + [rx1, ry1, rz1])
        new_points.append(point.tolist() + [rx2, ry2, rz2])
        new_points.append(point.tolist() + [rx3, ry3, rz3])
        new_points.append(point.tolist() + [rx4, ry4, rz4])
        new_points.append(point.tolist() + [rx5, ry5, rz5])
        new_points.append(point.tolist() + [rx6, ry6, rz6])
        new_points.append(point.tolist() + [rx7, ry7, rz7])
    new_points = np.array(new_points)
    return new_points
    return points

def optimize_points_order(points):
    '''
    优化点集的顺序,变成除草机式,使得机械臂尽可能短距离遍历这些点,
    首要条件就是相邻的两个点之间只有一个方向是不同的,
    比如只有x是不同的。具体来说：先沿着z轴只变化z,
    然后zy不变沿着x轴变化一下x,再次沿着z轴只变化z,
    这个xz平面便利完之后,沿着y轴变化一下y之后同理,
    要求使得机械臂从一个点运动到另一个点的轨迹是沿着x或者y或者z的直线。
    :param points:
    :return:points
    '''

    # 按照z坐标排序
    z_order = np.argsort(points[:, 2])
    points = points[z_order]

    # 对于每个z值,按照y坐标排序
    y_order = np.argsort(points[:, 1], kind='mergesort')  # 使用稳定排序算法
    for i in range(len(z_order)):
        z_idx = z_order[i]
        z_val = points[z_idx, 2]
        if i == len(z_order) - 1 or points[z_order[i + 1], 2] != z_val:
            # 当前z值是最后一个或者下一个z值不同,说明当前z层已经排好序
            continue
        # 当前z值和下一个z值相同,需要对这些点按照y坐标排序
        y_slice = slice(z_idx, z_order[i + 1])
        points[y_slice] = points[y_slice][y_order[y_slice]]

    # 对于每个(z, y)值,按照x坐标排序
    x_order = np.argsort(points[:, 0], kind='mergesort')  # 使用稳定排序算法
    for i in range(len(z_order)):
        z_idx = z_order[i]
        z_val = points[z_idx, 2]
        for j in range(z_idx, z_order[i + 1]):
            y_val = points[j, 1]
            if j == z_order[i + 1] - 1 or points[j + 1, 1] != y_val:
                # 当前(y, z)值是最后一个或者下一个(y, z)值不同,说明当前(y, z)层已经排好序
                continue
            # 当前(y, z)值和下一个(y, z)值相同,需要对这些点按照x坐标排序
            x_slice = slice(j, z_order[i + 1])
            points[x_slice] = points[x_slice][x_order[x_slice]]

    # 按照相邻两个点的坐标差来判断是否需要交换它们的顺序
    for i in range(len(points) - 1):
        diff = points[i + 1] - points[i]
        if np.count_nonzero(diff) == 1:
            # 只有一个维度不同,不能交换顺序
            continue
        elif np.count_nonzero(diff) >= 2:
            # 两个或三个维度不同,可以交换顺序
            points[i], points[i + 1] = points[i + 1], points[i]
    return points

def save_trajectory(points,trajectory_path,num_points):
    '''
    将采样点的数组保存为txt配置文件
    :param points: 点集
    :param trajectory_path: 保存路径
    :param num_points: 点的个数
    :return: None
    '''
    with open(trajectory_path, 'w') as f:
        for i in range(num_points):
            f.write(",".join(str(x) for x in points[i]) + "\n")

def test_load_trajectory(trajectory_path):
    '''
    测试读取运动点
    :param trajectory_path: 点集的路径
    :return:None
    '''
    # 读取txt文件中的点
    points = []
    with open(trajectory_path, 'r') as f:
        for line in f:
            point = np.array(line.strip().split(','), dtype=np.float32)
            points.append(point)
    # 将读取到的点存储到一个numpy数组中
    points = np.array(points)
    # 打印存储的点的形状
    #print(points)
    print('points shape:', points.shape)

def main():
    '''主函数'''
    # =====================生成采样点=====================
    points = generate_trajectory()

    # =====================保存采样点=====================
    num_points = np.shape(points)[0]
    trajectory_path = f'./cfg/trajectory_{num_points}.txt'
    # trajectory_path = './cfg/trajectory_xyz.txt'
    save_trajectory(points, trajectory_path, num_points)

    # =====================测试读取点代码=====================
    test_load_trajectory(trajectory_path)

if __name__ == '__main__':
    main()