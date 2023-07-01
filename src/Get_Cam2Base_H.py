'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 自动化高精度手眼标定(眼在手外),获得【cam2base_H和camera_depth_scale】 (适用于各种机械臂和相机的组合)
    1.handeye_calibration():
        利用机械臂末端位姿arrry:end2base_xyzrxryrz和标定板位姿arrry:board2cam_xyzrxryrz,使用cv2.calibrateHandEye()函数(默认为Tsai方法计算AX=XB)来获得相机到机械臂基底的变换矩阵cam2base_H;
        一共有TSAI,PARK,DANIILIDIS,HORAUD,ANDREFF这些方法,测试结果表明Park方法比较准确!!
    2.get_camera_depth_scale():
        获得深度图的比例因子,表示深度图像素值与实际距离之间的转换关系,通常以米/像素为单位。它是通过手眼标定得到的机械臂末端在相机坐标系下的变换矩阵和相机的畸变系数来计算的。
    3.target2cam_xyzrxryrz_to_target2base_xyzrxryrz():
        实现已知基于cam的位姿,转换到基于base的位姿
    4.target2base_xyzrxryrz_to_end2base_xyzrxryrz():
        基于给定的最终要抓去的地方end相对于识别到的tag的相对位置xyzrxryrz偏移,以及rxryrz的偏移顺序,给定target2base_xyzrxryrz,转换为end2base_xyzrxryrz
    5.get_new_cam2base_H():
        获取基于新基座的cam2base_H(对于基座的位置变化了的情况,需要重新算出来一个cam2base_H)
    6.get_now_base_x():
        获取现在的机械臂x轴的坐标
    7.test_get_all_board2end_H():
        验证是否所有的board2base_H一致
'''

import argparse
import csv
import json
import math
import time
from collections import namedtuple
import cv2
import numpy as np
import transforms3d as tfs
from Base import FR5BASE
from H_R_t_xyzrxryrz_transform import \
    EulerAngle_to_R,\
    R_to_EulerAngle,\
    Rvec_to_R,\
    R_to_Rvec,\
    xyz_to_t,\
    t_to_xyz,\
    xyz_rxryrz_to_xyzrxryrz,\
    Rt_to_H,\
    H_to_Rt,\
    rotate_R

def handeye_calibration(end2base_xyzrxryrz,board2cam_xyzrxryrz):
    """
    说明:
        利用机械臂末端位姿arrry:end2base_xyzrxryrz和
        标定板位姿arrry:board2cam_xyzrxryrz,
        使用cv2.calibrateHandEye()函数(默认为Tsai方法计算AX=XB)
        来获得相机到机械臂基底的变换矩阵cam2base_H

    Args:
        end2base_xyzrxryrz: 机械臂末端位姿arrry
        board2cam_xyzrxryrz: 标定板位姿arrry

    Returns:
        cam2base_H: 相机到机械臂基地的齐次变换矩阵
    """
    end2base_R , end2base_t = [], []
    board2cam_R , board2cam_t = [], []

    # end2base_xyzrxryrz转换为end2base_R和end2base_t
    for xyzrxryrz in end2base_xyzrxryrz: # xyzrxryrz为array类型
        R = EulerAngle_to_R(xyzrxryrz[3:6])
        t = xyz_to_t(xyzrxryrz[0:3])
        end2base_R.append(R)
        end2base_t.append(t)

    # board2cam_xyzrxryrz转换为board2cam_R和board2cam_t
    for xyzrxryrz in board2cam_xyzrxryrz:# xyzrxryrz为array类型
        R = Rvec_to_R(xyzrxryrz[3: 6])
        t = xyz_to_t(xyzrxryrz[0:3])
        board2cam_R.append(R)
        board2cam_t.append(t)

    # end2base转换为base2end
    base2end_R, base2end_t = [], []
    for R, t in zip(end2base_R, end2base_t):
        R_b2e = R.T
        t_b2e = -R_b2e @ t
        base2end_R.append(R_b2e)
        base2end_t.append(t_b2e)

    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_TSAI)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@TSAI@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_DANIILIDIS)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@DANIILIDIS@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_HORAUD)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@HORAUD@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_ANDREFF)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@ANDREFF@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_PARK)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@PARK@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    
    cam2base_H = tfs.affines.compose(np.squeeze(cam2base_t), cam2base_R, [1, 1, 1])

    return cam2base_H

def get_camera_depth_scale(cam2base_H,distortion_coefficients):
    """
    说明:
    获得深度图的比例因子,表示深度图像素值与实际距离之间的转换关系,通常以米/像素为单位。它是通过手眼标定得到的机械臂末端在相机坐标系下的变换矩阵和相机的畸变系数来计算的。

    参数:
    cam2base_H: cam到base的齐次变换矩阵
    distortion_coefficients: 相机畸变矩阵

    返回值:
    camera_depth_scale: 深度图的比例因子

    """
    # 计算camera_depth_scale
        # 从T_cam2base中取出前三行第四列的元素,即位于相机坐标系下机械臂末端的z坐标。然后对这个值求平方并求和,再开根号,得到相机与机械臂末端之间的距离(单位为米)。
        # 从相机的畸变系数dist_coeffs中取出平均值。dist_coeffs是相机标定的结果,用于纠正畸变。这里取平均值是为了让这个比例因子更加准确地反映相机的畸变情况。
        # 将上述两个值相除

    camera_depth_scale = np.sqrt(np.sum(cam2base_H[:3, 3] ** 2)) / np.mean(distortion_coefficients)

    return camera_depth_scale

def target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz):
    '''
    说明:
        实现target已知基于cam的位姿,使用cam2base_H转换到基于base的位姿,
    
    :param cam2base_H:相机到机械臂基地的齐次变换矩阵
    :param target2cam_xyzrxryrz: target到cam的位姿
    :return: target2base_xyzrxryrz: target到base的位姿
    '''

    # print("========使用cam2base_H由target2cam_xyzrxryrz转换为target2base_xyzrxryrz=========")
    # 给定一个target2cam_xyzrxryrz
    target2cam_R = Rvec_to_R(np.array(target2cam_xyzrxryrz[3:]))
    target2cam_t = xyz_to_t(np.array(target2cam_xyzrxryrz[:3]))
    target2cam_H = Rt_to_H(target2cam_R, target2cam_t)
    # print(f'target2cam_R:\n{target2cam_R}')


    # 计算target2base_H = cam2base_H * target2cam_H
    target2base_H = cam2base_H @ target2cam_H
    # print("target2base_H:\n", target2base_H)

    # 提取target2base的旋转矩阵和平移向量
    target2base_R,target2base_t = H_to_Rt(target2base_H)

    # 旋转矩阵 转 欧拉角rxryrz(度)
    target2base_xyz = t_to_xyz(target2base_t)
    target2base_rxryrz = R_to_EulerAngle(target2base_R)

    # 拼接xyz和rxryrz
    target2base_xyzrxryrz = xyz_rxryrz_to_xyzrxryrz(target2base_xyz,target2base_rxryrz)

    # ndarrary转list
    target2base_xyzrxryrz = target2base_xyzrxryrz.tolist()
    # print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')

    return target2base_xyzrxryrz


def target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz,delta_x=0,delta_y=0,delta_z=0,delta_rx=0, delta_ry=0, delta_rz=0, rotate_order='zyx'):
    '''
    说明:
        基于给定的最终要抓去的地方end相对于识别到的tag的相对位置xyzrxryrz偏移,以及rxryrz的偏移顺序,给定target2base_xyzrxryrz,转换为end2base_xyzrxryrz
    
    :param target2base_xyzrxryrz: list类型,xyz(m),rxryrz(度)
    :param delta_x: end相对于target的x偏差,m为单位
    :param delta_y: end相对于target的y偏差,m为单位
    :param delta_z: end相对于target的z偏差,m为单位
    :param delta_rx: end相对于target的rx偏差,度为单位
    :param delta_ry: end相对于target的ry偏差,度为单位
    :param delta_rz: end相对于target的rz偏差,度为单位
    :param delta_rx: end相对于target的rx偏差,度为单位
    :param rotate_order: 旋转的顺序
    :return: end2base_xyzrxryrz
    '''

    # target2cam_xyzrxryrz为list类型
    target2base_xyz = target2base_xyzrxryrz[:3]
    target2base_rxryrz = target2base_xyzrxryrz[3:]

    # 定义end2base_xyzrxryrz
    end2target_xyz = [delta_x,delta_y,delta_z]

    # 欧拉角转旋转矩阵
    target2base_R = EulerAngle_to_R(np.array(target2base_rxryrz))

    # end2base_xyz(1*3)
    end2base_xyz = np.dot(target2base_R, np.array(end2target_xyz).reshape(3, 1)) + np.array(target2base_xyz).reshape(3, 1)
    end2base_xyz = end2base_xyz.squeeze() #展成1*3
    # print(f'end2base_xyz:{end2base_xyz}')

    # end2base_rxryrz(1*3)
    target2base_R = rotate_R(target2base_R, delta_rx, delta_ry, delta_rz, rotate_order)
    end2base_rxryrz = R_to_EulerAngle(target2base_R)
    # print(f'end2base_rxryrz:{end2base_rxryrz}')

    # end2base_xyzrxryrz
    end2base_xyzrxryrz = xyz_rxryrz_to_xyzrxryrz(end2base_xyz,end2base_rxryrz)
    end2base_xyzrxryrz = end2base_xyzrxryrz.tolist() # 转换为list

    return end2base_xyzrxryrz

def test_get_all_board2end_H(cam2base_H,end2base_xyzrxryrz,board2cam_xyzrxryrz):
    '''
    说明:
        验证是否所有的board2base_H一致
    
    :param cam2base_H:
    :param end2base_xyzrxryrz:
    :param board2cam_xyzrxryrz:
    :return:
    '''
    print("========使用cam2base_H所有的获得board2base_H=========")
    # 结果验证,原则上来说,每次结果相差较小
    end2base_R , end2base_t =  [], []
    board2cam_R , board2cam_t =  [], []

    # end2base_xyzrxryrz转换为end2base_R和end2base_t
    for xyzrxryrz in end2base_xyzrxryrz:
        R = EulerAngle_to_R(xyzrxryrz[3:6])
        t = xyz_to_t(xyzrxryrz[0:3])
        end2base_R.append(R)
        end2base_t.append(t)

    # board2cam_xyzrxryrz转换为board2cam_R和board2cam_t
    for xyzrxryrz in board2cam_xyzrxryrz:
        R = Rvec_to_R(xyzrxryrz[3: 6])
        t = xyz_to_t(xyzrxryrz[0:3])
        board2cam_R.append(R)
        board2cam_t.append(t)

    for i in range(len(end2base_R)):
        end2base_H = Rt_to_H(end2base_R[i],end2base_t[i])
        # print(end2base_H)
        board2cam_H = Rt_to_H(board2cam_R[i],board2cam_t[i])
        # print(board2cam_H)
        board2end_H = np.linalg.inv(end2base_H) @ board2cam_H @ cam2base_H
        print(f'第{i}次board2end_H:\n{board2end_H}')

def get_new_cam2base_H(cam2base_H,base_x = 1.200,base_y = 0,base_z = 0,now_base_x = 1.200,now_base_y = 0,now_base_z = 0):
    '''
    说明:
        获取基于新基座的cam2base_H(对于基座的位置变化了的情况,需要重新算出来一个cam2base_H)
    
    :param cam2base_H: 基于原来手眼标定基座时的cam2base_H
    :param base_x: 手眼标定时的基座x
    :param base_y: 手眼标定时的基座y
    :param base_z: 手眼标定时的基座z
    :param now_base_x: 新基座x
    :param now_base_y: 新基座y
    :param now_base_z: 新基座z
    :return: 新基座的cam2base_H
    '''

    # 计算两个坐标系的变化
    change_x = base_x - now_base_x
    change_y = base_y - now_base_y
    change_z = base_z - now_base_z

    # 计算新坐标系下的cam2base_H
    cam2base_H[0,3] += change_x
    cam2base_H[1,3] += change_y
    cam2base_H[2,3] += change_z

    return cam2base_H

def get_now_base_x(base):
    '''
    说明:
        获取现在的机械臂x轴的坐标

    :param base: 机器人基座类的示例化
    :return:基座现在的位置now_base_x
    '''

    # 先读几遍来避免一开始数据的错误
    for i in range(5):
        _, _, _, now_base_x = base._get_motor_state()
        time.sleep(0.2)
    # print(f'now_base_x:{now_base_x}')

    # 避免数据的错误
    if now_base_x <= 0 or now_base_x > 1220:
        raise ValueError("value out of range")

    # m转mm
    now_base_x = now_base_x / 1000

    return now_base_x


def main():
    '''主函数'''
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='HandEye-Calibration')
    parser.add_argument('--camera_params_path', type=str, default='./cfg/camera_params.json',help='the path of camera prams json')
    args = parser.parse_args()
    camera_params_path = args.camera_params_path

    # =====================创建一个camera params结构体实例,设置相机内参矩阵和畸变矩阵=====================
    print(f'===============读取camera params中.....==============')
    with open(camera_params_path, 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)
    # print(f'===============camera_matrix{camera_matrix},distortion_coefficients{distortion_coefficients}')
    print(f'===============读取camera params成功==============')

    print("========读取end2base_xyzrxryrz和board2cam_xyzrxryrz测试手眼标定获得cam2base_H=========")
    with open('./output/end2base_xyzrxryrz.csv', newline='') as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')
        end2base_xyzrxryrz = [list(map(float, row)) for row in data_reader]
    end2base_xyzrxryrz = np.array(end2base_xyzrxryrz)

    with open('./output/board2cam_xyzrxryrz.csv', newline='') as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')
        board2cam_xyzrxryrz = [list(map(float, row)) for row in data_reader]
    board2cam_xyzrxryrz = np.array(board2cam_xyzrxryrz)

    # =====================计算相机到机械臂基地的变换矩阵cam2base_H=====================
    print(f'===============正在计算cam2base_H......===============')
    cam2base_H = handeye_calibration(end2base_xyzrxryrz, board2cam_xyzrxryrz)
    print(f'===============计算cam2base_H成功===============')

    # =====================获得camera_depth_scale=====================
    print(f'===============正在计算camera_depth_scale......===============')
    camera_depth_scale = get_camera_depth_scale(cam2base_H,camera_params.distortion_coefficients)
    print(f'===============计算camera_depth_scale成功===============')

    # ====================保存cam2base_H和camera_depth_scale================
    print(f'===============正在保存cam2base_H和camera_depth_scale......===============')
    with open('./output/cam2base_H.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(cam2base_H)
    with open("./output/camera_depth_scale.txt", "w") as f:
        f.write(str(camera_depth_scale))
        f.close()
    print(f'===============保存成功============')

    # =====================最后结果输出=====================
    print(f'@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 最后结果 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
    print("camera_depth_scale:\n{}".format(camera_depth_scale))
    print("cam2base_H:\n{}".format(cam2base_H))

    # ====================测试target2cam_xyzrxryrz_to_target2base_xyzrxryrz====================
    # target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H, target2cam_xyzrxryrz = board2cam_xyzrxryrz[0])
    # 测试是否所有的board2end_H都差不多
    # test_get_all_board2end_H(cam2base_H, end2base_xyzrxryrz, board2cam_xyzrxryrz)

    # =====================移动基座的情况================
    # # 创建base
    # base = FR5BASE()
    # base.create_receive_threading()
    # # 获取现在的base_x
    # now_base_x = get_now_base_x(base)
    # # 计算新的cam2base_H
    # new_cam2base_H = get_new_cam2base_H(cam2base_H, base_x=1.220, base_y=0, base_z=0, now_base_x=now_base_x, now_base_y=0, now_base_z=0)
    # print("new cam2base_H:\n{}".format(new_cam2base_H))


if __name__ == '__main__':
    main()