'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.04.27(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 检测可用的摄像头，打开摄像头，拍照
    1.open_camera():
        打开指定的一个摄像头打开指定的一个摄像头
    2.open_camera_and_capture():
        打开指定的摄像头并拍摄
    3.test_open_all_camera():
        查看有哪些摄像头可用

@Usage: 参看Main()函数
'''

import cv2
def open_camera():
    '''打开指定的一个摄像头'''
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1080像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)  # 摄像头是和人对立的，将图像左右调换回来正常显示。
        cv2.imshow('frame', frame)
        c = cv2.waitKey(50)# 在这里，程序等待50毫秒，如果50毫秒内有键盘事件，则返回按键的 ASCII 码，如果没有，则返回-1。
        if c == 27:# ESC键退出
            break
    cap.release()
    cv2.destroyAllWindows()

def open_camera_and_capture():
    '''打开指定的摄像头并拍摄'''
    # 获取摄像头实例并设置分辨率
    cap = cv2.VideoCapture(1)
    # 设置摄像头的帧宽属性为1920像素
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为1080像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    i = 24
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.flip(frame, 1)  # 摄像头是和人对立的，将图像左右调换回来正常显示。
        cv2.imshow('frame', frame)
        # Press 's' to capture an image
        if cv2.waitKey(1) & 0xFF == ord('s'):
            filename = f"./image_{i}.jpg"
            cv2.imwrite(filename, frame)
            i += 1
        # press ESC to exit
        c = cv2.waitKey(1)# 在这里，程序等待1毫秒，如果1毫秒内有键盘事件，则返回按键的 ASCII 码，如果没有，则返回-1。
        if c == 27:# ESC键退出
            print('exit')
            break
    cap.release()
    cv2.destroyAllWindows()

def test_open_all_camera():
    '''查看有哪些摄像头可用'''
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera {i} is available")
        cap.release()
def main():
    # 打开指定的摄像头
    open_camera()
    # 打开指定的摄像头并拍摄
    # open_camera_and_capture()
    # 查看有哪些摄像头可用
    # test_open_all_camera()

if __name__ == '__main__':
    main()