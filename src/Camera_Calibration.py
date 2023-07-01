'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 自动化高精度相机标定【内参矩阵和畸变矩阵】 (适用于各种相机)
    1.capture_images()
        拍摄指定张照片,并保存(按键s保存)
    2.calibrate_camera_SELET_cornerSubPix_or_flags_or_getOptimal_or_projectPoints()
        相机参数标定
            优化方式:默认值开启第一个optimize_cornerSubPix
            【1.角点亚像素级别的精度优化】
            【2.设置标志位flags的精度优化】
            【3.开启利用畸变校正优化】
            【4.基于重投影误差的精度优化】
    3.save_camera_params()
        将相机参数保存到json文件
    4.undistort_img_with_camera_matrix_and_distortion_coefficients()
        使用相机标定的参数来去除图像的畸变

@Usage:参看Main()函数

@命令行使用方法:

【四个优化方式全带】:
python3 Camera-Calibration.py                                   \
--nums_images 50                                                \
--images_path               ./camera_calibration_images         \
--pattern_size              (11,9)                              \
--square_size               0.015                                \
--camera_params_json_path   ./camera_params.json                \
--test_image_path           ./camera_calibration_images/image_0.jpg   \
--optimize_cornerSubPix                                         \
--optimize_flags                                                \
--optimize_getOptimal                                           \
--optimize_projectPoints                                        \
--max_error                 3

【只带cornerSubPix优化方式都不带】
python3 Camera-Calibration.py                                   \
--nums_images 50                                                \
--images_path              ./camera_calibration_images          \
--pattern_size              (11,9)                              \
--square_size               0.015                               \
--camera_params_json_path  ./camera_params.json                 \
--test_image_path          ./camera_calibration_images/image_0.jpg \
--optimize_cornerSubPix                                         \
'''

import cv2
import numpy as np
import json
import argparse
import glob

def capture_images(num_images, save_path):
    '''
    拍摄指定张照片,并保存(按键s保存)
    :param num_images:照片数量
    :param save_path:照片保存路径
    :return: None
    '''
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1280像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为720像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    i = 0
    while True:
        ret, frame = cap.read()
        # frame = cv2.flip(frame, 1)  将图像左右调换
        cv2.imshow('frame', frame)

        # Press 's' to capture an image
        if cv2.waitKey(1) & 0xFF == ord('s'):
            filename = f"{save_path}/image_{i}.jpg"
            cv2.imwrite(filename, frame)
            i += 1

        # Stop capturing images if the desired number has been reached
        if i == num_images:
            break

    cap.release()
    cv2.destroyAllWindows()


def calibrate_camera_SELET_cornerSubPix_or_flags_or_getOptimal_or_projectPoints(images_path,pattern_size,square_size,optimize_cornerSubPix,optimize_flags,optimize_getOptimal,optimize_projectPoints,max_error):
    '''
    相机参数标定,主要使用cv2.findChessboardCorners和cv2.calibrateCamera。
    优化方式(可以自己任意选择搭配)
        【1.角点亚像素级别的精度优化】
        【2.设置标志位flags的精度优化】
        【3.开启利用畸变校正优化】
        【4.基于重投影误差的精度优化】

        :param images_path: 照片集路径
        :param pattern_size: 网格标定板内角点个数
        :param square_size:网格标定板方格大小(m)
        :param optimize_cornerSubPix: 开启角点亚像素级别的精度优化
        :param optimize_flags: 开启设置标志位flags的精度优化
        :param optimize_getOptimal: 开启利用畸变校正优化
        :param optimize_projectPoints: 开启基于重投影误差的精度优化
        :param max_error: 在开启基于重投影误差的精度优化的条件下,设置最大误差阈值(对于一个10x7的标定板,如果标定板上有70个角点,那么一个合适的max_error值可能在2-5像素之间)
        :return: camera_matrix, distortion_coefficients
    '''
    print('===========开始自动化高精度相机参数标定===========')
    # 准备标定板的角点
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    # 用于存储标定板角点的三维坐标和图像中对应的二维坐标
    object_points = []
    image_points = []

    # 遍历所有照片
    print('===========Loading All Images.................===========')
    images = sorted(glob.glob(images_path + '/*.jpg'))

    if optimize_cornerSubPix:
        print('===========采用优化方式1:进行角点亚像素精度提升===========')
        # =================设置终止准则指定,表示搜索到一定的精度或迭代次数时停止搜索=================
        # cv2.TERM_CRITERIA_EPS: 表示迭代的终止准则是通过算法的迭代误差(每个校正图像中检测到的角点的位置与它们的参考位置之间的距离)是否满足指定的准确度来终止。
        # cv2.TERM_CRITERIA_MAX_ITER: 表示迭代的终止准则是通过算法的迭代次数是否超过指定值来终止。
        # 0.001: 表示算法的迭代误差阈值。如果每个校正图像中检测到的角点的位置与它们的参考位置之间的距离小于此阈值,则认为算法已经收敛并终止。
        # 因此,这个元组的含义是:在迭代误差小于0.001或迭代次数达到30次时,停止迭代。
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # ====================================================

    if optimize_flags:
        print('===========采用优化方式2:设置终止准则提高精度===========')
        # =================设置相机参数的优化标志,控制标定的精度和速度=================
        # 是相机标定时 cv2.calibrateCamera() 函数中的标定标志(flag)参数,用于控制标定过程中的优化模式和条件。
        # cv2.CALIB_RATIONAL_MODEL 表示采用比例畸变模型来进行标定。相机在成像时可能存在畸变,这里采用比例畸变模型对畸变进行校正。
        # cv2.CALIB_FIX_K3 表示在标定过程中固定 k3 系数的值。 k3 是径向畸变的三阶项系数,因为它的值较小,通常情况下可以固定为0,不进行估计,以减少标定误差和提高标定精度。
        flags = cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3
        # ====================================================
    i = 0
    for fname in images:
        img = cv2.imread(fname)
        # 转换为灰度图像
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 查找标定板角点
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        # 如果找到了,添加到角点列表中
        if ret == True:
            if optimize_cornerSubPix:
                # =================进行角点亚像素精度提升=================
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                # ====================================================
            if len(corners) == np.prod(pattern_size):
                # 将世界坐标系下的三维点加入到object_points中
                object_points.append(pattern_points)
                # 将图像坐标系下的二维点加入到image_points中
                image_points.append(corners)
                print(f'===========第{i}张照片识别到apriltag角点并显示===========')
                # 在图像上显示角点
                img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)
                # 显示图像
                cv2.imshow('img', img)
                cv2.waitKey(500)
        else:
            print(f'xxxxxxxxxxxxxx第{i}张照片未识别到apriltag角点xxxxxxxxxxxxxxx')
        i += 1

    if optimize_cornerSubPix and not optimize_flags:
        # 进行相机参数标定:cornerSubPix优化
        ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None,criteria=criteria)
    elif optimize_flags and not optimize_cornerSubPix:
        # 进行相机参数标定:flags优化
        ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None,flags=flags)
    elif optimize_cornerSubPix and optimize_flags:
        # 进行相机参数标定:cornerSubPix优化和flags优化
        ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None,criteria=criteria,flags=flags)
    else: # 不使用cornerSubPix优化和flags优化
        ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)


    # ==========================getOptimalNewCameraMatrix优化(利用畸变校正优化)==================
    if optimize_getOptimal:
        print('===========采用优化方式3:利用畸变校正优化===========')
        img2 = cv2.imread(images_path + f'image_0.jpg')
        h, w = img2.shape[:2]
        new_camera_matrix, new_distortion_coefficients ,ori = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
        # ⚠️上一行代码会报错,说是因为返回值太多了,但是查getOptimalNewCameraMatrix的手册都是这么写的,所以建议先关掉这个优化
        # 获得新参数
        camera_matrix, distortion_coefficients = new_camera_matrix,new_distortion_coefficients

    if optimize_projectPoints:
        print('===========采用优化方式4:基于重投影误差的方法来剔除高重投影误差的点===========')
        # =================基于重投影误差的方法来剔除高重投影误差的点=================
        def calibrate_camera_optimize_based_on_reprojection_error(object_points,image_points,max_error,camera_matrix,distortion_coefficients,rvecs,tvecs):

            # Calculate reprojection errors
            mean_error = 0
            for i in range(len(object_points)):
                image_points2, _ = cv2.projectPoints(
                    object_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coefficients)
                error = cv2.norm(image_points[i], image_points2, cv2.NORM_L2) / len(image_points2)
                mean_error += error
            mean_error /= len(object_points)

            # Discard points with high reprojection errors
            object_points_refined = []
            image_points_refined = []
            tot_error = 0
            for i in range(len(object_points)):
                image_points2, _ = cv2.projectPoints(
                    object_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coefficients)
                error = cv2.norm(image_points[i], image_points2, cv2.NORM_L2) / len(image_points2)
                tot_error += error
                if error < max_error:
                    print(f'error:{error}====更新object_points和image_points====')
                    object_points_refined.append(object_points[i])
                    image_points_refined.append(image_points[i])
            print("total error: ", tot_error / len(object_points))
            return object_points_refined,image_points_refined

        # 获得新点
        object_points,image_points = calibrate_camera_optimize_based_on_reprojection_error(object_points,image_points,max_error,camera_matrix,distortion_coefficients,rvecs,tvecs)
        # 重新标定
        if optimize_cornerSubPix and not optimize_flags:
            # 进行相机参数标定:cornerSubPix优化
            ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points,
                                                                                            gray.shape[::-1], None,
                                                                                            None, criteria=criteria)
        elif optimize_flags and not optimize_projectPoints:
            # 进行相机参数标定:flags优化
            ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points,
                                                                                            gray.shape[::-1], None,
                                                                                            None, flags=flags)
        elif optimize_cornerSubPix and optimize_flags:
            # 进行相机参数标定:cornerSubPix优化和flags优化
            ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points,
                                                                                            gray.shape[::-1], None,
                                                                                            None, criteria=criteria,
                                                                                            flags=flags)
        else: # 不使用cornerSubPix优化和flags优化
            ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points,
                                                                                            gray.shape[::-1], None,None)
    print(f'====================camera_matrix:====================\n{camera_matrix}')
    print(f'====================distortion_coefficients:====================\n{distortion_coefficients}')
    return camera_matrix, distortion_coefficients
def save_camera_params(filename,camera_matrix,distortion_coefficients):
    '''
    将相机参数保存到json文件(内参矩阵和畸变矩阵)
    :param filename: 保存文件路径
    :param camera_matrix:内参矩阵
    :param distortion_coefficients:畸变矩阵
    :return: None
    '''
    print(f'===========正在保存camera_matrix和distortion_coefficients===========')
    data = {'camera_matrix': camera_matrix.tolist(), 'distortion_coefficients': distortion_coefficients.tolist()}
    with open(filename, 'w') as f:
        json.dump(data, f)
    print(f'===========保存成功===========')

def undistort_img_with_camera_matrix_and_distortion_coefficients(img_path):
    '''
    使用相机标定的参数来去除图像的畸变
    :param img_path:输入照片路径
    :return: undistorted_img
    '''
    # 读取相机参数配置文件
    with open('camera_params_cornerSubPix_and_flags.json', 'r') as f:
        params = json.load(f)

    # 读取相机内参矩阵
    camera_matrix = np.array(params['camera_matrix'])

    # 读取相机畸变矩阵
    distortion_coefficients = np.array(params['distortion_coefficients'])
    img = cv2.imread(img_path)

    def undistort(img, camera_matrix, distortion_coefficients):
        '''
        这段代码是用于去除图像畸变的函数。输入参数为原始图像,相机内参矩阵和畸变系数,返回去除畸变后的图像。
        首先使用cv2.getOptimalNewCameraMatrix()函数获得一个最优的相机内参矩阵和图像ROI区域,
        然后使用cv2.initUndistortRectifyMap()函数获得图像坐标映射表,最后使用cv2.remap()函数将原始图像映射到新的图像空间中。
        映射后的图像大小可能会改变,因此使用ROI区域剪裁得到的是去除畸变后的图像。
        :param img:
        :param camera_matrix:
        :param distortion_coefficients:
        :return: undistorted_img
        '''
        h, w = img.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
        mapx, mapy = cv2.initUndistortRectifyMap(
            camera_matrix, distortion_coefficients, None, new_camera_matrix, (w, h), 5)
        undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y + h, x:x + w]
        return undistorted_img

    # 开始去除图像的畸变
    undistorted_img = undistort(img, camera_matrix,distortion_coefficients)
    cv2.imwrite('./camera_calibration_images/image_0_undistored.jpg', undistorted_img)
    cv2.imshow('img', undistorted_img)
    cv2.waitKey(0)

    return undistorted_img

def main():
    '''主函数'''
    # =====================设置parser=====================
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='Camera-Calibration')
    parser.add_argument('--nums_images',             type = int,    default = 50,                                   help = 'number of images')
    parser.add_argument('--images_path',             type = str,    default = './image/camera_calibration_images/',       help = 'path of images')
    parser.add_argument('--pattern_size',            type = tuple,  default = (11,8),                               help = 'number of corner points')#网格标定板内角点个数(角点个数 = (列数-1) x (行数-1))
    parser.add_argument('--square_size',             type = float,  default = 0.015,                                 help = 'number of corner points')# 网格标定板方格大小(m)
    parser.add_argument('--camera_params_json_path', type = str,    default = './cfg/camera_params.json',               help = 'path of camera_params.json')
    parser.add_argument('--test_image_path',         type = str,    default = './image/camera_calibration_images/image_0.jpg',  help = 'path of test image(undistort image)')# 待测试照片(利用相机参数去畸变)
    parser.add_argument('--optimize_cornerSubPix',   action = 'store_false',                                         help = 'Enable cornerSubPix  optimization mode')
    parser.add_argument('--optimize_flags',          action = 'store_true',                                         help = 'Enable setting flags optimization mode')
    parser.add_argument('--optimize_getOptimal',     action = 'store_true',                                         help = 'Enable getOptimal optimization mode')
    parser.add_argument('--optimize_projectPoints',  action = 'store_true',                                         help = 'Enable projectPoints optimization mode')
    parser.add_argument('--max_error',               type = float,  default = 2,                                    help = 'Enable projectPoints optimization mode')# 3 只有开启optimize_projectPoints模式才会有用

    # =====================初始化参数=====================
    args = parser.parse_args()
    nums_images = args.nums_images
    images_path = args.images_path
    pattern_size = args.pattern_size
    square_size = args.square_size
    camera_params_json_path = args.camera_params_json_path
    test_image_path = args.test_image_path
    optimize_cornerSubPix = args.optimize_cornerSubPix
    optimize_flags = args.optimize_flags
    optimize_getOptimal = args.optimize_getOptimal
    optimize_projectPoints = args.optimize_projectPoints
    max_error = args.max_error

    print(f'optimize_cornerSubPix:{optimize_cornerSubPix}\r\noptimize_flags:{optimize_flags}\r\noptimize_getOptimal:{optimize_getOptimal}\r\noptimize_projectPoints:{optimize_projectPoints}\r\n')

    # =====================拍照指定数量的照片=====================
    capture_images(nums_images, images_path)

    # =====================相机标定(任意选择四种优化方式1.亚像素级别的优化2.设置标志位优化3.开启利用畸变校正优化 4.基于重投影误差优化)=====================
    camera_matrix, distortion_coefficients = calibrate_camera_SELET_cornerSubPix_or_flags_or_getOptimal_or_projectPoints(images_path, pattern_size, square_size,optimize_cornerSubPix,optimize_flags,optimize_getOptimal,optimize_projectPoints,max_error)

    # =====================保存相机参数=====================
    save_camera_params(camera_params_json_path,camera_matrix,distortion_coefficients)

    # =====================测试:去除图像的畸变=====================
    undistort_img_with_camera_matrix_and_distortion_coefficients(test_image_path)

if __name__ == '__main__':
    main()
