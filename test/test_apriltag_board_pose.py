'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.04.27(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 检测apriltag图片,获得apriltag的位姿xyzrxryrz
    1.get_apriltag_board2cam_xyzrxryrz():
        计算一张照片的标定板(apriltag板)相对摄像头的位姿board2cam_xyzrxryrz(rxryrz为旋转向量)
@Usage: 参看Main()函数
'''

import cv2
import numpy as np
from collections import namedtuple
import json
import apriltag
import transforms3d as tfs

def get_apriltag_board2cam_xyzrxryrz(img_path,apriltag_board,camera_params):
    """
    说明：
        计算一张照片的标定板(apriltag板)相对摄像头的位姿board2cam_xyzrxryrz(rxryrz为旋转向量)

    Args:
        image_path: 图片位置
        apriltag_board: apriltag板结构体
        camera_params: 相机参数结构体

    Returns:
        board2cam_xyzrxryrz: 标定板位姿
    """
    # 定义tag大小
    tag_size = apriltag_board.tag_size

    # 定义相机参数
    camera_matrix = camera_params.camera_matrix
    distortion_coefficients = camera_params.distortion_coefficients

    # 读取图片
    image = cv2.imread(img_path)

    # 定义apriltag检测器
    detector = apriltag.Detector()

    # 对图像进行apriltag检测
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    result = detector.detect(gray)

    # 如果检测到apriltag，则计算其位姿
    if len(result) > 0:
        print('=============找到了apriltag=========')
        # 遍历检测结果
        for tag in result:
            # 获取 AprilTag ID 和位姿
            id = tag.tag_id
            pose = tag.pose_t
            # 打印结果
            print(f"AprilTag ID: {id}")
            print(f"AprilTag Pose:\n{pose}")

        # 获取apriltag的3D点和2D点
        object_points = np.array([[-tag_size / 2, -tag_size / 2, 0], [tag_size / 2, -tag_size / 2, 0],
                                  [tag_size / 2, tag_size / 2, 0], [-tag_size / 2, tag_size / 2, 0]])
        image_points = np.array([result[0].corners[0], result[0].corners[1],
                                 result[0].corners[2], result[0].corners[3]])

        # 计算相机与apriltag的位姿
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distortion_coefficients)

        # 由旋转向量rvec和平移向量tvec获取xyzrxryrz
        xyzrxryrz = np.concatenate((tvec, rvec), axis=0).reshape((1, 6)).tolist()[0]
        # print(f'标定板相对于相机的位姿xyzrxryrz：{xyzrxryrz}')

        return xyzrxryrz

    else:
        print('=============未找到apriltag=========')
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
    img_path = './apriltag.jpg'

    with open('../src/cfg/camera_params.json', 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)

    Apriltag_Board = namedtuple('Apriltag_Board', ['tag_size'])
    apriltag_board = Apriltag_Board(0.0545)

    board2cam_xyzrxryrz = get_apriltag_board2cam_xyzrxryrz(img_path, apriltag_board, camera_params)
    board2cam_R = Rvec_to_R(board2cam_xyzrxryrz[3:])
    board2cam_t = xyz_to_t(board2cam_xyzrxryrz[:3])
    board2cam_H = Rt_to_H(board2cam_R,board2cam_t)

    print(f'---board2cam_xyzrxryrz---:\n {board2cam_xyzrxryrz}')
    print(f'---board2cam_R---:\n {board2cam_R}')
    print(f'---board2cam_t---:\n {board2cam_t}')
    print(f'---board2cam_H---:\n {board2cam_H}')

if __name__ == '__main__':
    main()