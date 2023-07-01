'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 获取每个照片的标定板相对摄像头的位姿board2cam_xyzrxryrz
    1.get_board2cam_xyzrxryrz():
        计算每个照片的标定板相对摄像头的位姿board2cam_xyzrxryrz
        1.get_apriltag_board2cam_xyzrxryrz():
            计算一张照片的标定板(apriltag板)相对摄像头的位姿board2cam_xyzrxryrz
        2.get_grid_board2cam_xyzrxryrz():
            计算一张照片的标定板(网格板)相对摄像头的位姿board2cam_xyzrxryrz
    2.delete_unused_end2base_xyzrxryrz():
        删除end2base_xyzrxryrz中在delete_idx的位姿
'''

import argparse
import csv
import glob
import json
from collections import namedtuple
import cv2
import apriltag
import numpy as np
import os

def get_board2cam_xyzrxryrz(num_images, image_dir, camera_params, grid_board=None, apriltag_board=None, board_mode = 'Apriltag_Board'): # or 'Grid_Board'
    """
    说明：
        计算每个照片的标定板相对摄像头的位姿board2cam_xyzrxryrz

    Args:
        detector: apriltag检测器
        num_images: 采集的图片数量
        image_dir: 采集的图片保存路径

    Returns:
        board2cam_xyzrxryrz: 标定板位姿arrry
    """

    # 初始化标定板位姿列表
    board2cam_xyzrxryrz =[]
    delete_idx = []

    # 循环处理每一张标定板图像
    for i in range(num_images):
        # 读取标定板图像
        print(f'===============开始处理第{i}张照片==============')
        img_path = os.path.join(image_dir, f"image_{i}.jpg")
        if board_mode == 'Grid_Board':
            # 获得的xyzrxryrz是array类型的
            xyzrxryrz = get_grid_board2cam_xyzrxryrz(img_path, grid_board, camera_params)
        elif board_mode == 'Apriltag_Board':
            xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(img_path, apriltag_board, camera_params)
        else:
            print('Mode Error')

        # 记录未识别到板子的情况
        if xyzrxryrz is None:
            delete_idx.append(i)
            continue

        # 存储标定板位姿
        board2cam_xyzrxryrz.append(xyzrxryrz)
        print(f'已存储第{i}张照片的board2cam_xyzrxryrz')

    return np.array(board2cam_xyzrxryrz),delete_idx

def get_apriltag_board2cam_xyzrxryrz(img_path,apriltag_board,camera_params,tag_id=None,tag_families='tag36h11'):
    """
    说明：
        计算一张照片里的一个标定板(apriltag板)相对摄像头的位姿board2cam_xyzrxryrz

    Args:
        image_path: 图片位置
        apriltag_board: apriltag板结构体
        camera_params: 相机参数结构体

    Returns:
        board2cam_xyzrxryrz: 标定板位姿(array)
    """
    # 定义tag大小
    tag_size = apriltag_board.tag_size

    # 定义相机参数
    camera_matrix = camera_params.camera_matrix
    distortion_coefficients = camera_params.distortion_coefficients

    # 读取图片
    if isinstance(img_path, str):
        image = cv2.imread(img_path)
    else:
        image = img_path

    # 定义apriltag检测器
    detector = apriltag.Detector()

    # 对图像进行apriltag检测
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    result = detector.detect(gray)

    # 如果检测到apriltag,则计算其位姿
    if len(result) > 0:
        print('找到了apriltag')
        # 画出四个坐标点
        for tag in result:
            # cv2.circle(image, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
            # cv2.circle(image, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
            # cv2.circle(image, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
            # cv2.circle(image, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom
            # print(f'tag_id:{tag.tag_id}')

            # 如果不指定tag_id或者等于指定的tag_id
            if tag_id == None or tag.tag_id == tag_id:
                # 获取apriltag的3D点和2D点
                object_points = np.array([[-tag_size / 2, -tag_size / 2, 0], [tag_size / 2, -tag_size / 2, 0],
                                          [tag_size / 2, tag_size / 2, 0], [-tag_size / 2, tag_size / 2, 0]])
                # print(f'object_points:\n{object_points}')

                image_points = np.array([tag.corners[0], tag.corners[1],
                                        tag.corners[2], tag.corners[3]])
                # print(f'image_points:\n{image_points}')

                # 计算相机与apriltag的位姿
                success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distortion_coefficients)
                # print(f'target2cam_rvec:\n{rvec}')
                # print(f'target2cam_tvec:\n{tvec}')

                # 由旋转向量rvec和平移向量tvec获取xyzrxryrz
                xyzrxryrz = np.concatenate((tvec, rvec), axis=0).reshape((1, 6)).tolist()[0]
                # print(f'标定板相对于相机的位姿xyzrxryrz：{xyzrxryrz}')

                return xyzrxryrz

    else:
        print('未找到apriltag')
        return None


def get_all_apriltag_board2cam_xyzrxryrz(img_path,apriltag_board,camera_params):
    """
    说明：
        计算一张照片的所有标定板(apriltag板)相对摄像头的位姿all_board2cam_xyzrxryrz

    Args:
        image_path: 图片位置
        apriltag_board: apriltag板结构体
        camera_params: 相机参数结构体

    Returns:
        all_board2cam_xyzrxryrz: 标定板位姿(array)
    """
    # 定义tag大小
    tag_size = apriltag_board.tag_size

    # 定义相机参数
    camera_matrix = camera_params.camera_matrix
    distortion_coefficients = camera_params.distortion_coefficients

    # 读取图片
    if isinstance(img_path, str):
        image = cv2.imread(img_path)
    else:
        image = img_path

    # 定义apriltag检测器
    detector = apriltag.Detector()

    # 对图像进行apriltag检测
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    result = detector.detect(gray)

    #
    all_board2cam_xyzrxryrz = []

    # 如果检测到apriltag,则计算其位姿
    if len(result) > 0:
        print('找到了apriltag')
        # 画出四个坐标点
        for tag in result:
            # cv2.circle(image, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
            # cv2.circle(image, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
            # cv2.circle(image, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
            # cv2.circle(image, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom

            # 获取apriltag的3D点和2D点
            object_points = np.array([[-tag_size / 2, -tag_size / 2, 0], [tag_size / 2, -tag_size / 2, 0],
                                      [tag_size / 2, tag_size / 2, 0], [-tag_size / 2, tag_size / 2, 0]])
            image_points = np.array([tag.corners[0], tag.corners[1],
                                     tag.corners[2], tag.corners[3]])

            # 计算相机与apriltag的位姿
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distortion_coefficients)

            # 由旋转向量rvec和平移向量tvec获取xyzrxryrz
            xyzrxryrz = np.concatenate((tvec, rvec), axis=0).reshape((1, 6)).tolist()[0]
            # print(f'标定板相对于相机的位姿xyzrxryrz：{xyzrxryrz}')

            # 添加到列表
            all_board2cam_xyzrxryrz.append(xyzrxryrz)

        return all_board2cam_xyzrxryrz

    else:
        print('未找到apriltag')
        return None



def get_grid_board2cam_xyzrxryrz(img_path,grid_board,camera_params):
    """
    说明：
        计算一张照片的标定板(网格板)相对摄像头的位姿board2cam_xyzrxryrz

    Args:
        image_path: 图片位置
        grid_board: 网格板结构体
        camera_params: 相机参数结构体

    Returns:
        board2cam_xyzrxryrz: 标定板位姿(array)
    """
    # 定义标定板大小和格子尺寸,单位为mm
    board_size = grid_board.board_size  # 标定板内角点数量
    square_size = grid_board.square_size  # 标定板上每个格子的尺寸

    # 定义相机参数
    camera_matrix = camera_params.camera_matrix
    distortion_coefficients = camera_params.distortion_coefficients

    # 生成标定板的三维坐标点
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size

    # 读取标定板图像
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 寻找标定板角点
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)

    if ret:
        print('找到了标定板角点')
        # 计算标定板相对于相机的位姿
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, camera_matrix, distortion_coefficients)

        # 由旋转向量rvec和平移向量tvec获取xyzrxryrz
        xyzrxryrz = np.concatenate((tvecs, rvecs), axis=0).reshape((1, 6)).tolist()[0]

        # print(f'标定板相对于相机的位姿xyzrxryrz:{xyzrxryrz}')
        return xyzrxryrz

    else:
        print('未找到标定板角点')
        return None

def delete_unused_end2base_xyzrxryrz(end2base_xyzrxryrz,delete_idx):
    """
    说明：
    删除end2base_xyzrxryrz中在delete_idx的位姿

    参数：
    end2base_xyzrxryrz: 机械臂末端位姿arrry
    delete_idx: 需要删除的位姿索引列表

    返回值：
    end2base_xyzrxryrz: 机械臂末端位姿arrry
    """

    end2base_xyzrxryrz = [xyzrxryrz for i, xyzrxryrz in enumerate(end2base_xyzrxryrz) if i not in delete_idx]
    return np.array(end2base_xyzrxryrz)

def main():
    '''主函数'''
    # =====================设置parser=====================
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='HandEye-Calibration')
    parser.add_argument('--image_dir', type=str, default="./image/handeye_calibration_images", help='the dir of image')  # 采集的图片保存路径
    parser.add_argument('--camera_params_path', type=str, default='./cfg/camera_params.json',help='the path of camera prams json')
    parser.add_argument('--APRILTAG', action='store_false', help='Enable Apriltag Board Calib mode')
    parser.add_argument('--tag_size', type=float, default=0.055, help='the size of tag(m)')
    parser.add_argument('--GRID', action='store_true', help='Enable Grid Board Calib mode')
    parser.add_argument('--board_size', type=list, default=[11, 9], help='the size of tag(8*6)')
    parser.add_argument('--square_size', type=float, default=0.015, help='the size of square(m)')

    # =====================初始化参数=====================
    args = parser.parse_args()
    image_dir = args.image_dir
    camera_params_path = args.camera_params_path
    APRILTAG = args.APRILTAG
    tag_size = args.tag_size
    GRID = args.GRID
    board_size = args.board_size
    square_size = args.square_size

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

        # =====================使用glob模块获取所有照片的数量=====================
    num_images = len(glob.glob(os.path.join(image_dir, '*.jpg')))

    # =====================计算标定板位姿board2cam_xyzrxryrz=====================
    print(f'===============开始处理照片获得board2cam_xyzrxryrz==============')
    if APRILTAG:
        board2cam_xyzrxryrz,delete_idx = get_board2cam_xyzrxryrz(num_images, image_dir, camera_params, apriltag_board = apriltag_board, board_mode='Apriltag_Board')
    elif GRID:
        board2cam_xyzrxryrz,delete_idx = get_board2cam_xyzrxryrz(num_images, image_dir, camera_params, grid_board = grid_board, board_mode='Grid_Board')
    print(f'===============成功获得board2cam_xyzrxryrz==============')

    # =====================读取end2base_xyzrxryrz=====================
    with open('./output/end2base_xyzrxryrz.csv', newline='') as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')
        end2base_xyzrxryrz = [list(map(float, row)) for row in data_reader]
    end2base_xyzrxryrz = np.array(end2base_xyzrxryrz)

    # =====================删除不能用的end2base_xyzrxryrz=====================
    print(f'===============正在删除在delete_idx:{delete_idx}的位姿......=========')
    end2base_xyzrxryrz = delete_unused_end2base_xyzrxryrz(end2base_xyzrxryrz, delete_idx)
    print(f'===============删除在delete_idx的位姿成功===============')

    # ====================保存board2cam_xyzrxryrz和end2base_xyzrxryrz================
    print(f'===============正在保存board2cam_xyzrxryrz===============')
    with open('./output/board2cam_xyzrxryrz.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(board2cam_xyzrxryrz)
    print(f'===============正在保存end2base_xyzrxryrz===============')
    with open('./output/end2base_xyzrxryrz.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(end2base_xyzrxryrz)

if __name__ == '__main__':
    main()