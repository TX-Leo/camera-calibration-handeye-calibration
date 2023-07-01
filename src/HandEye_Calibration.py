'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 自动化高精度手眼标定(眼在手外),获得【cam2base_H和camera_depth_scale】 (适用于各种机械臂和相机的组合)
                直接调用Get_End2Base_xyzrxryrz.py和Get_Board2Cam_xyzrxryrz.py和Get_Cam2Base_H.py里的函数,融合三步成一个函数
'''

import cv2
import apriltag
import numpy as np
import os
import json
import time
import math
from collections import namedtuple
import argparse
import asyncio
import glob
import transforms3d as tfs
import csv
from Robot import FR5ROBOT
from Get_Cam2Base_H import handeye_calibration,get_camera_depth_scale,target2cam_xyzrxryrz_to_target2base_xyzrxryrz,test_get_all_board2end_H,get_new_cam2base_H,get_now_base_x
from Get_End2Base_xyzrxryrz import get_end2base_xyzrxryrz
from Get_Board2Cam_xyzrxryrz import get_board2cam_xyzrxryrz,delete_unused_end2base_xyzrxryrz
from Base import FR5BASE

async def main():
    """
        主函数
            初始化内容：
            # image_dir: 采集的图片保存路径
            # robot: Robot对象
            # trajectory: 机械臂运动轨迹
            # camera_params: 相机参数(内参矩阵和畸变矩阵)
            # apriltag_board: apriltag板参数(tag大小)
            # grid_board: 网格板参数(内角点数量和每个格子的尺寸)
    """

    # =====================设置parser=====================
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='HandEye-Calibration')
    parser.add_argument('--image_dir', type=str, default="./image/handeye_calibration_images",
                        help='the dir of image')  # 采集的图片保存路径
    parser.add_argument('--host', type=str, default="192.168.50.2", help='robot tcp host')  # tcp host
    parser.add_argument('--port', type=int, default=8080, help='robot tcp port')  # tcp port
    parser.add_argument('--trajectory_path', type=str, default='./cfg/trajectory_315.txt', help='the path of trajectory')
    parser.add_argument('--camera_params_path', type=str, default='./cfg/camera_params.json',
                        help='the path of camera prams json')
    parser.add_argument('--APRILTAG', action='store_false', help='Enable Apriltag Board Calib mode')
    parser.add_argument('--tag_size', type=float, default=0.055, help='the size of tag(m)')
    parser.add_argument('--GRID', action='store_true', help='Enable Grid Board Calib mode')
    parser.add_argument('--board_size', type=list, default=[11, 9], help='the size of tag(8*6)')
    parser.add_argument('--square_size', type=float, default=0.015, help='the size of square(m)')

    # =====================初始化参数=====================
    args = parser.parse_args()
    image_dir = args.image_dir
    host = args.host
    port = args.port
    trajectory_path = args.trajectory_path
    camera_params_path = args.camera_params_path
    APRILTAG = args.APRILTAG
    tag_size = args.tag_size
    GRID = args.GRID
    board_size = args.board_size
    square_size = args.square_size

    # =====================删除采集的图片保存路径所有已存的图片================================================
    file_names = os.listdir(image_dir)
    for file_name in file_names:
        os.remove(os.path.join(image_dir, file_name))

    # =====================初始化Robot对象=====================
    print(f'===============初始化robot并连接中.....==============')
    robot = FR5ROBOT(host, port)
    await robot.connect()
    # pose = await robot.get_pose()
    # print(f'===============robot现在的位姿:{pose}==============')
    print(f'===============初始化robot并连接成功==============')

    # =====================设置机械臂运动轨迹=====================
    print(f'===============读取机械臂运动轨迹trajectory中.....==============')
    points = []
    with open(trajectory_path, 'r') as f:
        for line in f:
            point = np.array(line.strip().split(','), dtype=np.float32)
            points.append(point)
    trajectory = np.array(points)
    trajectory = trajectory.tolist()
    num_images = len(trajectory)
    # print(f'===============trajectory:\n{trajectory========================}')
    print(f'===============读取机械臂运动轨迹trajectory成功==============')

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

    # =====================创建Board对象=====================
    if APRILTAG:
        print(f'===============创建Apriltag Board中.....==============')
        # 创建一个apriltag board结构体实例
        Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
        apriltag_board = Apriltag_Board(tag_size)
        print(f'===============创建Apriltag Board成功,tag_size: {tag_size}==============')
    elif GRID:
        # 创建一个grid board结构体实例
        print(f'===============创建Grid Board中.....==============')
        Grid_Board = namedtuple('Grid_Board', ['board_size', 'square_size'])  # 标定板内角点数量和标定板上每个格子的尺寸,mm
        grid_board = Grid_Board(board_size, square_size)
        print(f'===============创建Grid Board成功,board_size: {board_size},square_size:{square_size}==============')

    # =====================生成机械臂运动轨迹,拍摄标定板图像并存储,同时存储机械臂位姿end2base_xyzrxryrz=====================
    print(f'===============开始沿轨迹运动来采集数据获得end2base_xyzrxryrz==============')
    end2base_xyzrxryrz = await get_end2base_xyzrxryrz(robot, trajectory, num_images, image_dir)
    print(f'==============成功获得end2base_xyzrxryrz==============')

    # =====================机械臂断开连接=====================
    await robot.disconnect()

    # =====================使用glob模块获取所有照片的数量=====================
    num_images = len(glob.glob(os.path.join(image_dir, '*.jpg')))

    # =====================计算标定板位姿board2cam_xyzrxryrz=====================
    print(f'===============开始处理照片获得board2cam_xyzrxryrz==============')
    if APRILTAG:
        board2cam_xyzrxryrz, delete_idx = get_board2cam_xyzrxryrz(num_images, image_dir, camera_params,
                                                                  apriltag_board=apriltag_board,
                                                                  board_mode='Apriltag_Board')
    elif GRID:
        board2cam_xyzrxryrz, delete_idx = get_board2cam_xyzrxryrz(num_images, image_dir, camera_params,
                                                                  grid_board=grid_board, board_mode='Grid_Board')
    print(f'===============成功获得board2cam_xyzrxryrz==============')

    # =====================删除不能用的end2base_xyzrxryrz=====================
    print(f'===============正在删除在delete_idx:{delete_idx}的位姿......=========')
    end2base_xyzrxryrz = delete_unused_end2base_xyzrxryrz(end2base_xyzrxryrz, delete_idx)
    print(f'===============删除在delete_idx的位姿成功===============')

    # =====================计算相机到机械臂基地的变换矩阵cam2base_H=====================
    print(f'===============正在计算cam2base_H......===============')
    cam2base_H = handeye_calibration(end2base_xyzrxryrz, board2cam_xyzrxryrz)
    print(f'===============计算cam2base_H成功===============')

    # =====================获得camera_depth_scale=====================
    print(f'===============正在计算camera_depth_scale......===============')
    camera_depth_scale = get_camera_depth_scale(cam2base_H, camera_params.distortion_coefficients)
    print(f'===============计算camera_depth_scale成功===============')

    # ====================保存board2cam_xyzrxryrz和end2base_xyzrxryrz和cam2base_H和camera_depth_scale================
    print(f'===============正在保存board2cam_xyzrxryrz和end2base_xyzrxryrz和cam2base_H和camera_depth_scale......===============')
    with open('./output/board2cam_xyzrxryrz.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(board2cam_xyzrxryrz)
    with open('./output/end2base_xyzrxryrz.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(end2base_xyzrxryrz)
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

    # =====================测试target2cam_xyzrxryrz_to_target2base_xyzrxryrz=====================
    #target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H, target2cam_xyzrxryrz = board2cam_xyzrxryrz[0])
    # =====================测试是否所有的board2end_H都差不多=====================
    # test_get_all_board2end_H(cam2base_H, end2base_xyzrxryrz, board2cam_xyzrxryrz)

    # =====================移动基座的情况================
    # # 创建base
    # base = FR5BASE()
    # base.create_receive_threading()
    # # 获取现在的base_x
    # now_base_x = get_now_base_x(base)
    # # 计算新的cam2base_H
    # new_cam2base_H = get_new_cam2base_H(cam2base_H, base_x=1.219, base_y=0, base_z=0, now_base_x=now_base_x, now_base_y=0, now_base_z=0)
    # print("new cam2base_H:\n{}".format(new_cam2base_H))

if __name__ == '__main__':
    asyncio.run(main())
