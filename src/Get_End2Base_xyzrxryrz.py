'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 生成机械臂运动轨迹,拍摄标定板图像并存储,同时存储机械臂位姿end2base_xyzrxryrz(xyz以m为单位,rxryrz是欧拉角以度为单位)生成机械臂运动轨迹,拍摄标定板图像并存储,同时存储机械臂位姿end2base_xyzrxryrz(xyz以m为单位,rxryrz是欧拉角以度为单位)
    1.get_end2base_xyzrxryrz():
        生成机械臂运动轨迹,拍摄标定板图像并存储,同时存储机械臂位姿end2base_xyzrxryrz(xyz以m为单位,rxryrz是欧拉角以度为单位)
    2.manual_add_end2base_xyzrxryrz(robot,amount_already,num_images, image_dir):
        手动继续添加轨迹点,在给定的时间内拖动机械臂到下一个位姿，拍摄标定板图像并存储
'''

import csv
import cv2
import numpy as np
import os
import time
from Robot import FR5ROBOT
import argparse
import asyncio

async def get_end2base_xyzrxryrz(robot, trajectory, num_images, image_dir):
    """
    说明:
        读取机械臂运动轨迹,拍摄标定板图像并存储,
        同时存储机械臂位姿arrry:end2base_xyzrxryrz
        (xyz以m为单位,rxryrz是欧拉角以度为单位)

    Args:
        robot: Robot对象,用于控制机械臂运动
        trajectory: 机械臂运动轨迹
        num_images: 采集的图片数量
        image_dir: 采集的图片保存路径

    Returns:
        end2base_xyzrxryrz 机械臂末端位姿(array)
    """

    # 初始化机械臂末端位姿列表
    end2base_xyzrxryrz = []

    # 将机械臂移动到初始位置
    print(f'===============机械臂运动到第0个位置===============')
    ret = await robot.move(pose=trajectory[0],vel=70,ovl=70)
    print(f'ret = {ret}')
    time.sleep(2)

    print(f'打开摄像头中.....')
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1280像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    time.sleep(6)
    print(f'摄像头打开完成')

    # 循环拍摄标定板图像
    i = 0
    while True:
        # 拍摄标定板图像并保存
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        # cv2.waitKey(200)
        filename = os.path.join(image_dir, f"image_{i}.jpg")
        cv2.imwrite(filename, frame)
        print(f'拍摄image_{i}并已保存')

        # 获取机械臂末端位姿
        xyzrxryrz = await robot.get_pose()# [x,y,z,rx,ry,rz]形式（xyz单位为mm,rxryrz单位为度）
        xyzrxryrz = [x / 1000 if i < 3 else x for i, x in enumerate(xyzrxryrz)] #将xyz单位从mm转换为m
        # print(f'===============此时机械臂末端位姿:{xyzrxryrz}==============')

        # 存储机械臂末端位姿(转array)
        end2base_xyzrxryrz.append(np.array(xyzrxryrz))
        print(f'已存储第{i}张照片的end2base_xyzrxryrz')

        # 移动机械臂到下一个位置
        i += 1
        print(f'===============机械臂运动到第{i}个位置==============')
        ret = await robot.move(pose=trajectory[(i) % num_images],vel=70,ovl=70)
        print(f'ret = {ret}')
        time.sleep(1.5)

        # if cv2.waitKey(8000) & 0xFF == ord('a'):
        #     pass

        if i == num_images:
            # 退出
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f'摄像头关闭')

    return np.array(end2base_xyzrxryrz)  # 转array

async def manual_add_end2base_xyzrxryrz(robot,amount_already,num_images, image_dir):
    """
    说明:
        手动继续添加轨迹点,在给定的时间内拖动机械臂到下一个位姿，拍摄标定板图像并存储
        (xyz以m为单位,rxryrz是欧拉角以度为单位)

    Args:
        robot: Robot对象,用于控制机械臂运动
        amount_already: 已经采集到的图片数量
        num_images: 手动采集的图片数量
        image_dir: 采集的图片保存路径

    Returns:
        end2base_xyzrxryrz 机械臂末端位姿(array)
    """
    
    # 打开已有的end2base_xyzrxryrz
    print("========读取end2base_xyzrxryrz和board2cam_xyzrxryrz测试手眼标定获得cam2base_H=========")
    with open('./output/end2base_xyzrxryrz.csv', newline='') as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')
        end2base_xyzrxryrz = [list(map(float, row)) for row in data_reader]
    # end2base_xyzrxryrz = np.array(end2base_xyzrxryrz)

    print(f'打开摄像头中.....')
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1280像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    time.sleep(6)
    print(f'摄像头打开完成')

    i = amount_already
    while True:
        # 拍摄标定板图像并保存
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        # cv2.waitKey(200)
        filename = os.path.join(image_dir, f"image_{i}.jpg")
        cv2.imwrite(filename, frame)
        print(f'拍摄image_{i}并已保存')

        # 获取机械臂末端位姿
        xyzrxryrz = await robot.get_pose()# [x,y,z,rx,ry,rz]形式（xyz单位为mm,rxryrz单位为度）
        xyzrxryrz = [x / 1000 if i < 3 else x for i, x in enumerate(xyzrxryrz)] #将xyz单位从mm转换为m
        # print(f'===============此时机械臂末端位姿:{xyzrxryrz}==============')

        # 存储机械臂末端位姿(转array)
        end2base_xyzrxryrz.append(np.array(xyzrxryrz))
        print(f'已存储第{i}张照片的end2base_xyzrxryrz')
        # 到了指定的数目
        if i == amount_already+num_images:
            break
        # 等待7s拖动一次
        print(f'等待拖动中.....')
        time.sleep(7)
        i = i+1

    # list转array
    end2base_xyzrxryrz = np.array(end2base_xyzrxryrz)
    return end2base_xyzrxryrz

async def main():
    # =====================设置parser=====================
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='HandEye-Calibration')
    parser.add_argument('--image_dir', type=str, default="./image/handeye_calibration_images", help='the dir of image')  # 采集的图片保存路径
    parser.add_argument('--host', type=str, default="192.168.50.2", help='robot tcp host')  # tcp host   192.168.50.2  xcp:50.12
    parser.add_argument('--port', type=int, default=8080, help='robot tcp port')  # tcp port
    parser.add_argument('--trajectory_path', type=str, default='./cfg/trajectory_189.txt', help='the path of trajectory')

    # =====================初始化参数=====================
    args = parser.parse_args()
    image_dir = args.image_dir
    host = args.host
    port = args.port
    trajectory_path = args.trajectory_path

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
    trajectory =  trajectory.tolist()
    num_images = len(trajectory)
    # print(f'===============trajectory:\n{trajectory========================}')
    print(f'===============读取机械臂运动轨迹trajectory成功==============')

    # =====================生成机械臂运动轨迹,拍摄标定板图像并存储,同时存储机械臂位姿end2base_xyzrxryrz=====================
    print(f'===============开始沿轨迹运动来采集数据获得end2base_xyzrxryrz==============')
    end2base_xyzrxryrz = await get_end2base_xyzrxryrz(robot, trajectory, num_images, image_dir)
    print(f'==============成功获得end2base_xyzrxryrz==============')

    # =====================手动添加一些end2base_xyzrxryrz==========================================
    # print(f'===============开始手动添加数据获得end2base_xyzrxryrz==============')
    #end2base_xyzrxryrz = await manual_add_end2base_xyzrxryrz(robot, amount_already=num_images, num_images=20, image_dir=image_dir)
    # print(f'==============成功手动添加end2base_xyzrxryrz==============')

    # =====================机械臂断开连接=====================
    await robot.disconnect()

    # ====================保存end2base_xyzrxryrz================
    print(f'===============正在保存end2base_xyzrxryrz===============')
    with open('./output/end2base_xyzrxryrz.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(end2base_xyzrxryrz)


if __name__ == '__main__':
    asyncio.run(main())