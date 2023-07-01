'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 
    主要能实现几个任务:
        1.实现机械臂精准到达相机识别到的apriltag的位置 
        2.实现机械臂精准遍历到达相机识别到的所有apriltag的位置
        3.实现机械臂精准跟随相机识别到的apriltag
        4.实现机械臂精准吸取磁铁(磁铁和apriltag在一个刚体上,相机识别到的apriltag的位置和磁铁的位置偏移是固定的)
        5.【最终任务1】抓取炸篮(炸篮上有磁铁和tag),对应grasp_basket_magnet_by_two_tag()
        6.【最终任务2】放回炸篮到锅里(炸锅上有tag),对应putback_basket_to_pot_by_two_tag()
        7.【最终任务3】放回炸篮到悬挂架上(炸锅上有tag),对应putback_basket_to_bracket_by_two_tag()
    所有函数说明:
        1.open_camera_and_capture():
            打开指定的摄像头并拍摄
        2.get_target2cam_xyzrxryrz():
            获取拍摄到的apriltag图片的位姿,并转换到base坐标系下
        3.move_to_target():
            机械臂运动到apriltag的位置
        4.keep_moving_to_target():
            直接打开摄像头,对视频中的apriltag实时处理,然后运动到apriltag的点
        5.move_to_all_targets():
            直接打开摄像头,对视频中的apriltag实时处理,然后运动到所有的apriltag的点
        6.grasp_basket_magnet(cap):
            通过识别一个放在篮子上的tag来抓取篮子上的磁铁
        7.grasp_basket_magnet_by_two_tag(cap):
            通过识别两个放在篮子上的tag来抓取篮子上的磁铁,两个tag的中点是中心位置,所以先拍三次照片,
            求得三个中心点位姿后取平均,最后进行磁铁相对于中心tag的偏移,然后抓取磁铁
        8.putback_basket_to_pot(cap):
            通过识别一个放在炸锅上的tag来将篮子放回炸锅里
        9.putback_basket_to_pot_by_two_tag(cap):
            通过识别两个放在炸锅上的tag来将篮子放回炸锅里,两个tag的中点是中心位置,所以先拍三次照片,
            求得三个中心点位姿后取平均,最后进行磁铁相对于中心tag的偏移,然后抓取磁铁
        10.putback_basket_to_bracket(cap):
            通过识别一个放在炸锅上的tag来将篮子放到炸锅前方的悬挂架上
        11.putback_basket_to_bracket_by_two_tag(cap):
            通过识别两个个放在炸锅上的tag来将篮子放到炸锅前方的悬挂架上,两个tag的中点是中心位置,所以先拍三次照片,
            求得三个中心点位姿后取平均,最后进行磁铁相对于中心tag的偏移,然后抓取磁铁
        12.test_grasp_basket_magnet():
            测试抓取篮子
    
@Usage: 参看Main函数!!!

'''
import csv
import json
from collections import namedtuple
import apriltag
import numpy as np
from Get_Cam2Base_H import target2cam_xyzrxryrz_to_target2base_xyzrxryrz,get_new_cam2base_H,get_now_base_x,target2base_xyzrxryrz_to_end2base_xyzrxryrz
from Robot import FR5ROBOT
import time
import asyncio
import cv2
from Get_Board2Cam_xyzrxryrz import get_apriltag_board2cam_xyzrxryrz,get_all_apriltag_board2cam_xyzrxryrz
from Base import FR5BASE
import keyboard


def open_camera_and_capture():
    '''
    说明:
        打开指定的摄像头并拍摄
    '''
    
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1080像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    time.sleep(6)
    # 拍照
    ret, frame = cap.read()
    # 保存照片
    cv2.imwrite('./apriltag.jpg', frame)
    # 释放摄像头
    cap.release()

def get_target2cam_xyzrxryrz():
    '''
    说明:
        获取拍摄到的apriltag图片的位姿,并转换到base坐标系下
    '''
    
    '''设置图片路径'''
    img_path = './apriltag.jpg'

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''初始化基座'''
    base = FR5BASE()
    base.create_receive_threading()

    # 读取cam2base_H
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)
    print("cam2base_H:\n{}".format(cam2base_H))

    # 检测到target2cam_xyzrxryrz
    target2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(img_path, apriltag_board, camera_params)
    print(f'target2cam_xyzrxryrz:\n{target2cam_xyzrxryrz}')

    # 获取新的base_x
    now_base_x = get_now_base_x(base)

    # 计算新的cam2base_H
    new_cam2base_H = get_new_cam2base_H(cam2base_H, base_x=1.220, base_y=0, base_z=0, now_base_x=now_base_x,now_base_y=0, now_base_z=0)
    print("new cam2base_H:\n{}".format(new_cam2base_H))

    # 转换为target2base_xyzrxryrz
    target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(new_cam2base_H,target2cam_xyzrxryrz)

    # xyz转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000

    # z加上150（为了安全）
    target2base_xyzrxryrz[2] += 110
    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')

    return target2base_xyzrxryrz

async def move_to_target(target2base_xyzrxryrz):
    '''机械臂运动到apriltag的位置'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()
    ret = await robot.move(pose=target2base_xyzrxryrz)
    time.sleep(3)
    await robot.disconnect()

async def keep_moving_to_target():
    '''
    说明:
        直接打开摄像头,对视频中的apriltag实时处理,然后运动到apriltag的点
    
    :return: None
    '''
    
    '''设置相机'''
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1080像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    '''初始化基座'''
    base = FR5BASE()
    base.create_receive_threading()

    while True:
        print('====================')
        # 读取摄像头的一帧图像
        ret, frame = cap.read()

        # 获取新的base_x
        now_base_x = get_now_base_x(base)

        # 计算新的cam2base_H
        new_cam2base_H = get_new_cam2base_H(cam2base_H, base_x=1.219, base_y=0, base_z=0, now_base_x=now_base_x,now_base_y=0, now_base_z=0)
        print("new cam2base_H:\n{}".format(new_cam2base_H))

        # 获取target2cam_xyzrxryrz
        target2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)

        # 检测apriltag,转换为target2base_xyzrxryrz
        if target2cam_xyzrxryrz:
            # 如果检测到apriltag,转换为target2base_xyzrxryrz
            target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(new_cam2base_H, target2cam_xyzrxryrz)
        else:
            # 如果没有检测到就再次检测
            continue

        # 显示
        # cv2.imshow('target',frame)

        # xyz的单位由m转换为mm
        for i in range(3):
            target2base_xyzrxryrz[i] *= 1000

        # 处理z
        target2base_xyzrxryrz[2] += 25 # 探针1cm+1.5cm = 25mm
        target2base_xyzrxryrz[2] += 75 # 为了安全
        print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')

        # 运动到apriltag的位置
        ret = await robot.move(pose=target2base_xyzrxryrz)
        print(f'ret:{ret}')

        # 每3s刷新一下apriltag的位置
        time.sleep(3)

        # 检测是否按下空格键来控制基座的移动（1s内若有空格按下则运动到指定位置)
        i = 0
        while 1:
            if i >=100:
                break
            # 检测是否按下空格键
            if keyboard.is_pressed('space'):
                base._ser_x_move(position=800, velocity=100)
                time.sleep(5)
                break
            else:
                # 等待0.1秒钟,避免CPU占用过高
                time.sleep(0.01)
                i += 1

    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()

    await robot.disconnect()


async def move_to_all_targets():
    '''
    说明:
        直接打开摄像头,对视频中的apriltag实时处理,然后运动到所有的apriltag的点
    
    :return: None
    '''
    
    '''设置相机'''
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1080像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    '''初始化基座'''
    base = FR5BASE()
    base.create_receive_threading()

    '''获取新的base_x'''
    now_base_x = get_now_base_x(base)

    '''计算新的cam2base_H'''
    new_cam2base_H = get_new_cam2base_H(cam2base_H, base_x=1.219, base_y=0, base_z=0, now_base_x=now_base_x,
                                        now_base_y=0, now_base_z=0)
    print("new cam2base_H:\n{}".format(new_cam2base_H))

    '''到达home状态'''
    home_pose = [-131.608,-268.481,211.671,-177.557,-0.462,-41.956]
    ret = await robot.move(pose=home_pose)
    print(f'ret:{ret}')
    time.sleep(5)

    # 读取摄像头的一帧图像
    ret, frame = cap.read()

    # 获取所有的target2cam_xyzrxryrz
    all_target2cam_xyzrxryrz = get_all_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)

    # 如果没有检测到
    if all_target2cam_xyzrxryrz == None:
        print('没有检测到任何一个apriltag')

    # 如果检测到apriltag,转换为target2base_xyzrxryrz,并运动到其位置
    for target2cam_xyzrxryrz in all_target2cam_xyzrxryrz:
        target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(new_cam2base_H, target2cam_xyzrxryrz)
        # xyz的单位由mm转换为m
        for i in range(3):
            target2base_xyzrxryrz[i] *= 1000
        # z加上150（为了安全）
        target2base_xyzrxryrz[2] += 100
        print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
        # 运动到apriltag的位置
        ret = await robot.move(pose=target2base_xyzrxryrz)
        print(f'ret:{ret}')
        time.sleep(5)

    # 控制基座的移动（1s内若有空格按下则运动到指定位置)
    base._ser_x_move(position=800, velocity=100)
    time.sleep(8)

    # ===重复上述操作===
    '''获取新的base_x'''
    now_base_x = get_now_base_x(base)

    '''计算新的cam2base_H'''
    new_cam2base_H = get_new_cam2base_H(cam2base_H, base_x=1.219, base_y=0, base_z=0, now_base_x=now_base_x,
                                        now_base_y=0, now_base_z=0)
    print("new cam2base_H:\n{}".format(new_cam2base_H))

    '''到达home状态'''
    home_pose = [-131.608,-268.481,211.671,-177.557,-0.462,-41.956]
    ret = await robot.move(pose=home_pose)
    print(f'ret:{ret}')
    time.sleep(5)

    # 读取摄像头的一帧图像
    ret, frame = cap.read()

    # 获取所有的target2cam_xyzrxryrz
    all_target2cam_xyzrxryrz = get_all_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)

    # 如果没有检测到
    if all_target2cam_xyzrxryrz == None:
        print('没有检测到任何一个apriltag')

    # 如果检测到apriltag,转换为target2base_xyzrxryrz,并运动到其位置
    for target2cam_xyzrxryrz in all_target2cam_xyzrxryrz:
        target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(new_cam2base_H, target2cam_xyzrxryrz)
        # xyz的单位由mm转换为m
        for i in range(3):
            target2base_xyzrxryrz[i] *= 1000
        # z加上150（为了安全）
        target2base_xyzrxryrz[2] += 100
        print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
        # 运动到apriltag的位置
        ret = await robot.move(pose=target2base_xyzrxryrz)
        print(f'ret:{ret}')
        time.sleep(5)


    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()

    await robot.disconnect()

async def grasp_basket_magnet(cap):
    '''
    说明：
        通过识别一个放在篮子上的tag来抓取篮子上的磁铁

    :return: None
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    print('====================')

    # 读取摄像头的一帧图像
    ret, frame = cap.read()
    cv2.imwrite('./apriltag.jpg', frame)
    # frame = cv2.imread('./apriltag-right.jpg')

    # 获取target2cam_xyzrxryrz
    target2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz)

        # target2base_xyzrxryrz的位姿顺时针旋转45度
        # target2base_xyzrxryrz = rotate_xyzrxryrz(target2base_xyzrxryrz, rotate_angle=45)

        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        basket_delta_x = -0.052 #增大导致x减小y增大
        basket_delta_y = -0.060 #增大导致xy都增大
        basket_delta_z = 0.025
        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz,delta_x = basket_delta_x,delta_y = basket_delta_y,delta_z = basket_delta_z)

    else:
        # 如果没有检测到就再次检测
        print("没有检测到")
        time.sleep(5)
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

    # # # 运动到apriltag的上方位置
    # # ret = await robot.move(pose=[target2base_xyzrxryrz[0],target2base_xyzrxryrz[1],target2base_xyzrxryrz[2]+100,target2base_xyzrxryrz[3],target2base_xyzrxryrz[4],target2base_xyzrxryrz[5]])
    # # print(f'ret:{ret}')
    # # time.sleep(2)

    # 关闭磁铁
    ret = await robot.switch_electromagnet(bopen=0)
    print(f'关闭磁铁 ret:{ret}')

    # 运动到磁铁的上方位置8cm
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2]+80,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],vel=30, ovl=30)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)

    # 运动到磁铁位置
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2],end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],vel=15, ovl=15)
    print(f'运动到磁铁 ret:{ret}')
    time.sleep(5)
    #
    # 拿起来磁铁
    ret = await robot.switch_electromagnet(bopen=1)
    print(f'开启磁铁 ret:{ret}')
    time.sleep(2)

    # 运动到磁铁上方位置
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2]+250,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],vel=40, ovl=40)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)
    #
    # 运动到篮子上方
    pose1 = [-164.155,-203.246,408.39, end2base_xyzrxryrz[3], end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]]
    ret = await robot.move(pose=pose1, vel=30, ovl=30)
    print(f'运动到篮子上方 ret:{ret}')
    time.sleep(3)

    # # 运动到篮子上方
    # pose1 = [-709.599, -339.111, 296.266,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]]
    # ret = await robot.move(pose=pose1, vel=30, ovl=30)
    # print(f'运动到篮子上方 ret:{ret}')
    # time.sleep(3)

    # # 运动到篮子里
    # pose2 = [-709.599, -339.111, 150,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]]
    # ret = await robot.move(pose=pose2, vel=30, ovl=30)
    # print(f'运动到篮子里 ret:{ret}')
    # time.sleep(3)
    #
    # # 放下
    # ret = await robot.switch_electromagnet(bopen=0)
    # print(f'关闭磁铁ret:{ret}')
    # time.sleep(2)

    # 断开机器人
    await robot.disconnect()

async def grasp_basket_magnet_by_two_tag(cap):
    '''
    说明：
        通过识别两个放在篮子上的tag来抓取篮子上的磁铁,两个tag的中点是中心位置,所以先拍三次照片,
        求得三个中心点位姿后取平均,最后进行磁铁相对于中心tag的偏移,然后抓取磁铁
    
    :return: None
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.040)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    print('====================')

    # 读取摄像头的一帧图像
    ret, frame = cap.read()
    cv2.imwrite('./apriltag1-basket.jpg', frame)
    # frame = cv2.imread('./apriltag1-basket-left.jpg')
    # frame = cv2.imread('./apriltag1-basket-left-mid.jpg')
    # frame = cv2.imread('./apriltag1-basket-mid.jpg')
    # frame = cv2.imread('./apriltag1-basket-right.jpg')
    # frame = cv2.imread('./apriltag1-basket-right-mid.jpg')

    # 获取target2cam_xyzrxryrz（第一张）
    target2cam_xyzrxryrz_left_1 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params,tag_id=4)
    target2cam_xyzrxryrz_right_1 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params,tag_id=6)

    time.sleep(0.3)
    ret, frame = cap.read()
    cv2.imwrite('./apriltag2-basket.jpg', frame)
    # frame = cv2.imread('./apriltag2-basket-left.jpg')
    # frame = cv2.imread('./apriltag2-basket-left-mid.jpg')
    # frame = cv2.imread('./apriltag2-basket-mid.jpg')
    # frame = cv2.imread('./apriltag2-basket-right.jpg')
    # frame = cv2.imread('./apriltag2-basket-right-mid.jpg')

    # 获取target2cam_xyzrxryrz（第二张）
    target2cam_xyzrxryrz_left_2 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=4)
    target2cam_xyzrxryrz_right_2 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=6)

    time.sleep(0.3)
    ret, frame = cap.read()
    cv2.imwrite('./apriltag3-basket.jpg', frame)
    # frame = cv2.imread('./apriltag3-basket-left.jpg')
    # frame = cv2.imread('./apriltag3-basket-left-mid.jpg')
    # frame = cv2.imread('./apriltag3-basket-mid.jpg')
    # frame = cv2.imread('./apriltag3-basket-right.jpg')
    # frame = cv2.imread('./apriltag3-basket-right-mid.jpg')

    # 获取target2cam_xyzrxryrz（第三张）
    target2cam_xyzrxryrz_left_3 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=4)# 11 # 5
    target2cam_xyzrxryrz_right_3 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=6)# 14 # 3

    # 三张取平均
    target2cam_xyzrxryrz_left = [(a + b + c) / 3 for a, b, c in zip(target2cam_xyzrxryrz_left_1 , target2cam_xyzrxryrz_left_2 , target2cam_xyzrxryrz_left_3)]
    target2cam_xyzrxryrz_right = [(a + b + c) / 3 for a, b, c in zip(target2cam_xyzrxryrz_right_1 , target2cam_xyzrxryrz_right_2 , target2cam_xyzrxryrz_right_3)]

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz_left and target2cam_xyzrxryrz_right:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        target2base_xyzrxryrz_left = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz_left)
        target2base_xyzrxryrz_right = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz_right)

        # 融合一下left和right,来减小误差
        target2base_xyzrxryrz = [(target2base_xyzrxryrz_left[0] + target2base_xyzrxryrz_right[0]) / 2,
                                 (target2base_xyzrxryrz_left[1] + target2base_xyzrxryrz_right[1]) / 2,
                                 (target2base_xyzrxryrz_left[2] + target2base_xyzrxryrz_right[2]) / 2,
                                 (target2base_xyzrxryrz_left[3] + target2base_xyzrxryrz_right[3]) / 2,
                                 (target2base_xyzrxryrz_left[4] + target2base_xyzrxryrz_right[4]) / 2,
                                 (target2base_xyzrxryrz_left[5] + target2base_xyzrxryrz_right[5]) / 2]

        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        basket_delta_x = 0.0       # 0.0           0
        basket_delta_y = -0.036      # -0.037        0
        basket_delta_z = 0.030      # 0.030         0
        basket_delta_rx = -25       # -25           -18
        basket_delta_ry = -23       # -23           -16
        basket_delta_rz = -38.5       # -38           -42
        rotate_order = 'zyx'

        # 平移+旋转
        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz, delta_x=basket_delta_x,delta_y=basket_delta_y,delta_z=basket_delta_z,delta_rx=basket_delta_rx,delta_ry=basket_delta_ry, delta_rz=basket_delta_rz, rotate_order=rotate_order)

    else:
        # 如果没有检测到就再次检测
        print("没有检测到所有的tag")
        time.sleep(5)
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    print(f'target2cam_xyzrxryrz_left_1:\n{target2cam_xyzrxryrz_left_1}')
    print(f'target2cam_xyzrxryrz_right_1:\n{target2cam_xyzrxryrz_right_1}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left_2:\n{target2cam_xyzrxryrz_left_2}')
    print(f'target2cam_xyzrxryrz_right_2:\n{target2cam_xyzrxryrz_right_2}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left_3:\n{target2cam_xyzrxryrz_left_3}')
    print(f'target2cam_xyzrxryrz_right_3:\n{target2cam_xyzrxryrz_right_3}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left:\n{target2cam_xyzrxryrz_left}')
    print(f'target2cam_xyzrxryrz_right:\n{target2cam_xyzrxryrz_right}')
    print(f'\n')
    print(f'target2base_xyzrxryrz_left:\n{target2base_xyzrxryrz_left}')
    print(f'target2base_xyzrxryrz_right:\n{target2base_xyzrxryrz_right}')
    print(f'\n')
    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'\n')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

    # # 运动到apriltag的上方位置
    # ret = await robot.move(pose=[target2base_xyzrxryrz[0],target2base_xyzrxryrz[1],target2base_xyzrxryrz[2]+20,target2base_xyzrxryrz[3],target2base_xyzrxryrz[4],target2base_xyzrxryrz[5]],tool=1,vel=30, ovl=30)
    # print(f'ret:{ret}')
    # time.sleep(3)

    # 关闭磁铁
    ret = await robot.switch_electromagnet(bopen=0)
    print(f'关闭磁铁 ret:{ret}')

    # 运动到磁铁的上方位置8cm
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2]+30,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],tool=1,vel=30, ovl=30)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)

    # 运动到磁铁位置
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2],end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],tool=1,vel=10, ovl=10)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)

    # 拿起来磁铁
    ret = await robot.switch_electromagnet(bopen=1)
    print(f'开启磁铁 ret:{ret}')
    time.sleep(1)

    # 运动到磁铁上方位置
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2]+250,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],tool=1,vel=30, ovl=30)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)

    # # 运动到篮子上方
    # pose1 = [-164.155,-203.246,408.39, end2base_xyzrxryrz[3], end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]]
    # ret = await robot.move(pose=pose1, vel=30, ovl=30)
    # print(f'运动到篮子上方 ret:{ret}')
    # time.sleep(3)

    # 断开机器人
    await robot.disconnect()


async def putback_basket_to_pot(cap):
    '''
    说明：
        通过识别一个放在炸锅上的tag来将篮子放回炸锅里
    
    :return: None
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    print('====================')

    # 读取摄像头的一帧图像
    ret, frame = cap.read()
    cv2.imwrite('./apriltag.jpg', frame)
    # frame = cv2.imread('./apriltag.jpg')

    # 获取target2cam_xyzrxryrz
    target2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H, target2cam_xyzrxryrz)

        # target2base_xyzrxryrz的位姿顺时针旋转45度
        #target2base_xyzrxryrz = rotate_xyzrxryrz(target2base_xyzrxryrz, rotate_angle=45)

        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        pot_delta_x = 0.36 # -420
        pot_delta_y = 0.21# -300
        pot_delta_z = 0.08 # 60

        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz, delta_x=pot_delta_x,
                                                                         delta_y=pot_delta_y, delta_z=pot_delta_z)

    else:
        # 如果没有检测到就再次检测
        print("没有检测到")
        time.sleep(5)
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    end2base_xyzrxryrz[3:] = 167.8203511345337, -13.089473882341899, 133.90943923157533

    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

    # 运动到磁铁的上方位置8cm
    ret = await robot.move(
        pose=[end2base_xyzrxryrz[0], end2base_xyzrxryrz[1], end2base_xyzrxryrz[2] + 80, end2base_xyzrxryrz[3],
              end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]], vel=30, ovl=30)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)

    # 运动到磁铁位置
    ret = await robot.move(
        pose=[end2base_xyzrxryrz[0], end2base_xyzrxryrz[1], end2base_xyzrxryrz[2], end2base_xyzrxryrz[3],
              end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]], vel=15, ovl=15)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(5)

    # 放下
    ret = await robot.switch_electromagnet(bopen=0)
    print(f'关闭磁铁ret:{ret}')
    time.sleep(2)

    # 断开机器人
    await robot.disconnect()

async def putback_basket_to_pot_by_two_tag(cap):
    '''    
    说明：
        通过识别两个放在炸锅上的tag来将篮子放回炸锅里,两个tag的中点是中心位置,所以先拍三次照片,
        求得三个中心点位姿后取平均,最后进行磁铁相对于中心tag的偏移,然后抓取磁铁
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.040)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    print('====================')

    # 读取摄像头的一帧图像
    # ret, frame = cap.read()
    # cv2.imwrite('./apriltag1-pot-left-4cm.jpg', frame)
    frame = cv2.imread('./apriltag1-pot-left-4cm.jpg')
    # frame = cv2.imread('./apriltag1-basket-left.jpg')

    # 获取target2cam_xyzrxryrz（第一张）
    target2cam_xyzrxryrz_left_1 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=0)
    target2cam_xyzrxryrz_right_1 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=1)

    time.sleep(0.3)
    # ret, frame = cap.read()
    # cv2.imwrite('./apriltag2-pot-left-4cm.jpg', frame)
    frame = cv2.imread('./apriltag2-pot-left-4cm.jpg')
    # frame = cv2.imread('./apriltag2-basket-left.jpg')

    # 获取target2cam_xyzrxryrz（第二张）
    target2cam_xyzrxryrz_left_2 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=0)
    target2cam_xyzrxryrz_right_2 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=1)

    time.sleep(0.3)
    # ret, frame = cap.read()
    # cv2.imwrite('./apriltag3-pot-left-4cm.jpg', frame)
    frame = cv2.imread('./apriltag3-pot-left-4cm.jpg')
    # frame = cv2.imread('./apriltag3-basket-left.jpg')

    # 获取target2cam_xyzrxryrz（第三张）
    target2cam_xyzrxryrz_left_3 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params,tag_id=0)  # 11 # 5  # 1
    target2cam_xyzrxryrz_right_3 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params,tag_id=1)  # 14 # 3 # 3

    # 三张取平均
    target2cam_xyzrxryrz_left = [(a + b + c) / 3 for a, b, c in
                                 zip(target2cam_xyzrxryrz_left_1, target2cam_xyzrxryrz_left_2,
                                     target2cam_xyzrxryrz_left_3)]
    target2cam_xyzrxryrz_right = [(a + b + c) / 3 for a, b, c in
                                  zip(target2cam_xyzrxryrz_right_1, target2cam_xyzrxryrz_right_2,
                                      target2cam_xyzrxryrz_right_3)]

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz_left and target2cam_xyzrxryrz_right:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        target2base_xyzrxryrz_left = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz_left)
        target2base_xyzrxryrz_right = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz_right)

        # 融合一下left和right,来减小误差
        target2base_xyzrxryrz = [(target2base_xyzrxryrz_left[0] + target2base_xyzrxryrz_right[0]) / 2,
                                 (target2base_xyzrxryrz_left[1] + target2base_xyzrxryrz_right[1]) / 2,
                                 (target2base_xyzrxryrz_left[2] + target2base_xyzrxryrz_right[2]) / 2,
                                 (target2base_xyzrxryrz_left[3] + target2base_xyzrxryrz_right[3]) / 2,
                                 (target2base_xyzrxryrz_left[4] + target2base_xyzrxryrz_right[4]) / 2,
                                 (target2base_xyzrxryrz_left[5] + target2base_xyzrxryrz_right[5]) / 2]

        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        pot_delta_x  = -0.004       # -0.003                # -0.004
        pot_delta_y  = -0.115       # 0.0015                # -0.115
        pot_delta_z  = -0.01       # -0.028                 # -0.01
        pot_delta_rx = -23           # 0  值越大右边翘        # -23
        pot_delta_ry = -22           # 0  值越大左边翘        # -22
        pot_delta_rz = -40          # -44                   # -40
        rotate_order = 'zyx'

        # 平移+旋转
        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz, delta_x=pot_delta_x,delta_y=pot_delta_y,delta_z=pot_delta_z,delta_rx=pot_delta_rx,delta_ry=pot_delta_ry, delta_rz=pot_delta_rz, rotate_order=rotate_order)

    else:
        # 如果没有检测到就再次检测
        print("没有检测到所有的tag")
        time.sleep(5)
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    print(f'target2cam_xyzrxryrz_left_1:\n{target2cam_xyzrxryrz_left_1}')
    print(f'target2cam_xyzrxryrz_right_1:\n{target2cam_xyzrxryrz_right_1}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left_2:\n{target2cam_xyzrxryrz_left_2}')
    print(f'target2cam_xyzrxryrz_right_2:\n{target2cam_xyzrxryrz_right_2}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left_3:\n{target2cam_xyzrxryrz_left_3}')
    print(f'target2cam_xyzrxryrz_right_3:\n{target2cam_xyzrxryrz_right_3}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left:\n{target2cam_xyzrxryrz_left}')
    print(f'target2cam_xyzrxryrz_right:\n{target2cam_xyzrxryrz_right}')
    print(f'\n')
    print(f'target2base_xyzrxryrz_left:\n{target2base_xyzrxryrz_left}')
    print(f'target2base_xyzrxryrz_right:\n{target2base_xyzrxryrz_right}')
    print(f'\n')
    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'\n')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

    # # 运动到apriltag的上方位置
    # ret = await robot.move(pose=[target2base_xyzrxryrz[0],target2base_xyzrxryrz[1],target2base_xyzrxryrz[2]+100,target2base_xyzrxryrz[3],target2base_xyzrxryrz[4],target2base_xyzrxryrz[5]],tool=1,vel=30, ovl=30)
    # print(f'ret:{ret}')
    # time.sleep(3)

    # 运动到磁铁的上方位置8cm
    ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2]+0,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],tool=1,vel=30, ovl=30)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)
    #
    # # 运动到磁铁位置
    # ret = await robot.move(pose=[end2base_xyzrxryrz[0],end2base_xyzrxryrz[1],end2base_xyzrxryrz[2]+0,end2base_xyzrxryrz[3],end2base_xyzrxryrz[4],end2base_xyzrxryrz[5]],tool=1,vel=15, ovl=15)
    # print(f'运动到磁铁 ret:{ret}')
    # time.sleep(5)
    #
    # # 放下
    # ret = await robot.switch_electromagnet(bopen=0)
    # print(f'开启磁铁 ret:{ret}')
    # time.sleep(2)

    # 断开机器人
    await robot.disconnect()

async def putback_basket_to_bracket(cap):
    '''
    说明：
        通过识别一个放在炸锅上的tag来将篮子放到炸锅前方的悬挂架上

    :return: None
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    print('====================')

    # 读取摄像头的一帧图像
    ret, frame = cap.read()
    cv2.imwrite('./apriltag.jpg', frame)
    # frame = cv2.imread('./apriltag.jpg')


    # 获取target2cam_xyzrxryrz
    target2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H, target2cam_xyzrxryrz)

        # target2base_xyzrxryrz的位姿顺时针旋转45度
        #target2base_xyzrxryrz = rotate_xyzrxryrz(target2base_xyzrxryrz, rotate_angle=45)

        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        bracket_delta_x = 0.323 # -423
        bracket_delta_y = 0.145 # -381
        bracket_delta_z = -0.09 # 230

        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz, delta_x=bracket_delta_x,
                                                                         delta_y=bracket_delta_y, delta_z=bracket_delta_z)

    else:
        # 如果没有检测到就再次检测
        print("没有检测到")
        time.sleep(5)
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    end2base_xyzrxryrz[3:] = 145.571,-24.828,142.287
    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

    # 运动到磁铁的上方位置8cm
    ret = await robot.move(
        pose=[end2base_xyzrxryrz[0], end2base_xyzrxryrz[1], end2base_xyzrxryrz[2] + 80, end2base_xyzrxryrz[3],
              end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]], vel=30, ovl=30)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(3)
    #
    # 运动到磁铁位置
    ret = await robot.move(
        pose=[end2base_xyzrxryrz[0], end2base_xyzrxryrz[1], end2base_xyzrxryrz[2], end2base_xyzrxryrz[3],
              end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]], vel=15, ovl=15)
    print(f'运动到磁铁上方 ret:{ret}')
    time.sleep(5)

    # 放下
    ret = await robot.switch_electromagnet(bopen=0)
    print(f'关闭磁铁ret:{ret}')
    time.sleep(2)

    # 断开机器人
    await robot.disconnect()

async def putback_basket_to_bracket_by_two_tag(cap):
    '''
    说明：
        通过识别两个个放在炸锅上的tag来将篮子放到炸锅前方的悬挂架上,两个tag的中点是中心位置,所以先拍三次照片,
        求得三个中心点位姿后取平均,最后进行磁铁相对于中心tag的偏移,然后抓取磁铁
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.040)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    '''初始化机械臂'''
    robot = FR5ROBOT('192.168.50.2', 8080)
    await robot.connect()

    print('====================')

    # 读取摄像头的一帧图像
    ret, frame = cap.read()
    cv2.imwrite('./apriltag1-bracket-left-4cm.jpg', frame)
    # frame = cv2.imread('./apriltag1-bracket-left-4cm.jpg')
    # frame = cv2.imread('./apriltag1-basket-left.jpg')

    # 获取target2cam_xyzrxryrz（第一张）
    target2cam_xyzrxryrz_left_1 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=10)
    target2cam_xyzrxryrz_right_1 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=11)

    time.sleep(0.3)
    ret, frame = cap.read()
    cv2.imwrite('./apriltag2-bracket-left-4cm.jpg', frame)
    # frame = cv2.imread('./apriltag2-bracket-left-4cm.jpg')
    # frame = cv2.imread('./apriltag2-basket-left.jpg')

    # 获取target2cam_xyzrxryrz（第二张）
    target2cam_xyzrxryrz_left_2 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=10)
    target2cam_xyzrxryrz_right_2 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params, tag_id=11)

    time.sleep(0.3)
    ret, frame = cap.read()
    cv2.imwrite('./apriltag3-bracket-left-4cm.jpg', frame)
    # frame = cv2.imread('./apriltag3-bracket-left-4cm.jpg')
    # frame = cv2.imread('./apriltag3-basket-left.jpg')

    # 获取target2cam_xyzrxryrz（第三张）
    target2cam_xyzrxryrz_left_3 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params,tag_id=10)  # 11 # 5
    target2cam_xyzrxryrz_right_3 = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params,tag_id=11)  # 14 # 3

    # 三张取平均
    target2cam_xyzrxryrz_left = [(a + b + c) / 3 for a, b, c in
                                 zip(target2cam_xyzrxryrz_left_1, target2cam_xyzrxryrz_left_2,
                                     target2cam_xyzrxryrz_left_3)]
    target2cam_xyzrxryrz_right = [(a + b + c) / 3 for a, b, c in
                                  zip(target2cam_xyzrxryrz_right_1, target2cam_xyzrxryrz_right_2,
                                      target2cam_xyzrxryrz_right_3)]

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz_left and target2cam_xyzrxryrz_right:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        target2base_xyzrxryrz_left = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz_left)
        target2base_xyzrxryrz_right = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz_right)

        # 融合一下left和right,来减小误差
        target2base_xyzrxryrz = [(target2base_xyzrxryrz_left[0] + target2base_xyzrxryrz_right[0]) / 2,
                                 (target2base_xyzrxryrz_left[1] + target2base_xyzrxryrz_right[1]) / 2,
                                 (target2base_xyzrxryrz_left[2] + target2base_xyzrxryrz_right[2]) / 2,
                                 (target2base_xyzrxryrz_left[3] + target2base_xyzrxryrz_right[3]) / 2,
                                 (target2base_xyzrxryrz_left[4] + target2base_xyzrxryrz_right[4]) / 2,
                                 (target2base_xyzrxryrz_left[5] + target2base_xyzrxryrz_right[5]) / 2]

        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        bracket_delta_x  = 0.003
        bracket_delta_y  = -0.267
        bracket_delta_z  = -0.27
        bracket_delta_rx = -30          # 0  值越大右边翘
        bracket_delta_ry = -28          # 0  值越大左边翘
        bracket_delta_rz = -38
        rotate_order = 'zyx'

        # 平移+旋转
        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz, delta_x=bracket_delta_x,delta_y=bracket_delta_y,delta_z=bracket_delta_z,delta_rx=bracket_delta_rx,delta_ry=bracket_delta_ry, delta_rz=bracket_delta_rz, rotate_order=rotate_order)

    else:
        # 如果没有检测到就再次检测
        print("没有检测到所有的tag")
        time.sleep(5)
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    print(f'target2cam_xyzrxryrz_left_1:\n{target2cam_xyzrxryrz_left_1}')
    print(f'target2cam_xyzrxryrz_right_1:\n{target2cam_xyzrxryrz_right_1}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left_2:\n{target2cam_xyzrxryrz_left_2}')
    print(f'target2cam_xyzrxryrz_right_2:\n{target2cam_xyzrxryrz_right_2}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left_3:\n{target2cam_xyzrxryrz_left_3}')
    print(f'target2cam_xyzrxryrz_right_3:\n{target2cam_xyzrxryrz_right_3}')
    print(f'\n')
    print(f'target2cam_xyzrxryrz_left:\n{target2cam_xyzrxryrz_left}')
    print(f'target2cam_xyzrxryrz_right:\n{target2cam_xyzrxryrz_right}')
    print(f'\n')
    print(f'target2base_xyzrxryrz_left:\n{target2base_xyzrxryrz_left}')
    print(f'target2base_xyzrxryrz_right:\n{target2base_xyzrxryrz_right}')
    print(f'\n')
    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'\n')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

    # # 运动到apriltag的上方位置
    # ret = await robot.move(pose=[target2base_xyzrxryrz[0],target2base_xyzrxryrz[1],target2base_xyzrxryrz[2]+10,target2base_xyzrxryrz[3],target2base_xyzrxryrz[4],target2base_xyzrxryrz[5]],tool=1,vel=30, ovl=30)
    # print(f'ret:{ret}')
    # time.sleep(3)

    # # 运动到磁铁的上方位置8cm
    # ret = await robot.move(
    #     pose=[end2base_xyzrxryrz[0], end2base_xyzrxryrz[1], end2base_xyzrxryrz[2] + 50, end2base_xyzrxryrz[3],
    #           end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]], tool=1, vel=10, ovl=30)
    # print(f'运动到磁铁上方 ret:{ret}')
    # time.sleep(3)
    #
    # # 运动到磁铁位置
    # ret = await robot.move(
    #     pose=[end2base_xyzrxryrz[0], end2base_xyzrxryrz[1], end2base_xyzrxryrz[2] + 0, end2base_xyzrxryrz[3],
    #           end2base_xyzrxryrz[4], end2base_xyzrxryrz[5]], tool=1, vel=10, ovl=30)
    # print(f'运动到磁铁 ret:{ret}')
    # time.sleep(3)
    #
    # # 放下
    # ret = await robot.switch_electromagnet(bopen=0)
    # print(f'放下磁铁 ret:{ret}')
    # time.sleep(2)

    # 断开机器人
    await robot.disconnect()

async def test_grasp_basket_magnet():
    '''
    测试抓取篮子

    :return: None
    '''

    '''设置apriltag板子'''
    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(tag_size=0.055)

    '''设置相机参数'''
    with open('./cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    '''读取cam2base_H'''
    with open('./output/cam2base_H.csv', newline='') as csvfile:
        # 创建CSV读取器
        reader = csv.reader(csvfile, delimiter=',')
        # 读取CSV文件中的数据
        data = []
        for row in reader:
            data.append(row)
    cam2base_H = np.array(data, dtype=np.float32)

    print('====================')

    # 读取摄像头的一帧图像
    frame = cv2.imread('./apriltag.jpeg')

    # 获取target2cam_xyzrxryrz
    print('====================target2cam=====================')
    target2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(frame, apriltag_board, camera_params)
    print(f'target2cam_xyzrxryrz:\n{target2cam_xyzrxryrz}')

    # 检测apriltag,转换为target2base_xyzrxryrz
    if target2cam_xyzrxryrz:
        # target2cam_xyzrxryrz转换为target2base_xyzrxryrz
        print('====================target2base=====================')
        target2base_xyzrxryrz = target2cam_xyzrxryrz_to_target2base_xyzrxryrz(cam2base_H,target2cam_xyzrxryrz)


        # target2base_xyzrxryrz转换为end2base_xyzrxryrz
        basket_delta_x = -0.052  # 这个部分增大导致x减小y增大
        basket_delta_y = -0.060  # 这个部分增大导致xy都增大
        # basket_delta_y = -0.063639610306789277196075992589436  # 0.09/sqrt(2)+(0.055/2)*sqrt(2)--(0.055/2)*sqrt(2)
        basket_delta_z = 0.025
        # 右边篮子：-0.056 -0.061 0.022
            # -425.42165305428557, -335.86335092078053, 99.73120015479829
        # 左边篮子：
            # -136.5,343.7

        print('====================end2base=====================')
        end2base_xyzrxryrz = target2base_xyzrxryrz_to_end2base_xyzrxryrz(target2base_xyzrxryrz,delta_x = basket_delta_x,delta_y = basket_delta_y,delta_z = basket_delta_z)


    else:
        # 如果没有检测到就再次检测
        print("没有检测到")
        return

    # xyz的单位由mm转换为m
    for i in range(3):
        target2base_xyzrxryrz[i] *= 1000
        end2base_xyzrxryrz[i] *= 1000

    print(f'target2base_xyzrxryrz:\n{target2base_xyzrxryrz}')
    print(f'end2base_xyzrxryrz:\n{end2base_xyzrxryrz}')

async def main():
    # =============先拍摄照片后检测,然后运动到apriltag的点=============
    # open_camera_and_capture()
    # target2base_xyzrxryrz = get_target2cam_xyzrxryrz()
    # await move_to_target(target2base_xyzrxryrz)

    # =============直接打开摄像头,对视频中的apriltag实时处理,每3s刷新一次,然后运动到apriltag的点=============
    # await keep_moving_to_target()

    # =============先运动到home位置,检测视野中的所有apriltag位置,然后开始一个一个到达,base再运动到另一个位置,再检测然后遍历到达================
    # await move_to_all_targets()

    # =============抓取磁铁=============

    '''设置相机'''
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1080像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    # 等待初始化
    time.sleep(6)

    # 抓篮子并放到一边
    # await grasp_basket_magnet(cap)
    # await grasp_basket_magnet_by_two_tag(cap)

    # 放篮子到炸锅里
    # await putback_basket_to_pot(cap)
    await putback_basket_to_pot_by_two_tag(cap)


    # 抓篮子并放到一边
    # await grasp_basket_magnet(cap)
    # await grasp_basket_magnet_by_two_tag(cap)


    # # 放篮子到悬挂架上
    # await putback_basket_to_bracket(cap)
    # await putback_basket_to_bracket_by_two_tag(cap)


    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    asyncio.run(main())
    # 为了测试
    # asyncio.run(test_grasp_basket_magnet())