'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.04.27(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 检测grid图片,获得gird的位姿xyzrxryrz
    1.get_grid_board2cam_xyzrxryrz():
        计算一张照片的标定板(grid板)相对摄像头的位姿board2cam_xyzrxryrz(rxryrz为旋转向量)
@Usage: 参看Main()函数
'''

import cv2
import numpy as np
from collections import namedtuple
import json
import transforms3d as tfs

def get_grid_board2cam_xyzrxryrz(img_path,grid_board,camera_params):
    """
    说明：
        计算一张照片的标定板(网格板)相对摄像头的位姿board2cam_xyzrxryrz(rxryrz为旋转矩阵）

    Args:
        image_path: 图片位置
        grid_board: 网格板结构体
        camera_params: 相机参数结构体

    Returns:
        board2cam_xyzrxryrz: 标定板位姿xyzrxryrz
    """
    # 定义标定板大小和格子尺寸，单位为mm
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

        #print(f'标定板相对于相机的位姿xyzrxryrz:{xyzrxryrz}')
        return xyzrxryrz

    else:
        print('未找到标定板角点')
        return None

def Rvec_to_R(rvec):
    '''旋转向量转旋转矩阵
    rvec为array类型
    '''
    R, _ = cv2.Rodrigues(rvec)
    return R

def xyz_to_t(xyz):
    '''xyz转平移向量t'''
    t = xyz.reshape(3, 1)
    return t

def Rt_to_H(R,t):
    '''旋转矩阵R和平移向量t结合成齐次矩阵H'''
    H = tfs.affines.compose(np.squeeze(t), R, [1, 1, 1])
    return H

def main():
    img_path = 'grid.jpg'

    with open('../src/cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    Grid_Board = namedtuple('Grid_Board', ['board_size', 'square_size'])  # 标定板内角点数量和标定板上每个格子的尺寸,mm
    grid_board = Grid_Board((11,9), 0.015)

    board2cam_xyzrxryrz = get_grid_board2cam_xyzrxryrz(img_path, grid_board, camera_params)
    board2cam_R = Rvec_to_R(board2cam_xyzrxryrz[3:])
    board2cam_t = xyz_to_t(board2cam_xyzrxryrz[:3])
    board2cam_H = Rt_to_H(board2cam_R,board2cam_t)

    print(f'---board2cam_xyzrxryrz---:\n {board2cam_xyzrxryrz}')
    print(f'---board2cam_R---:\n {board2cam_R}')
    print(f'---board2cam_t---:\n {board2cam_t}')
    print(f'---board2cam_H---:\n {board2cam_H}')


if __name__ == '__main__':
    main()