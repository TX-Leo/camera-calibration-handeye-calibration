'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.04.27(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 测试检测apriltag图片、检测视频里的apriltag
    1.detect_apriltag_image():
        检测图片里的aptiltag
    2.detect_apriltag_video():
        检测视频里的apriltag
@Usage: 参看Main()函数
'''
import time

import apriltag
#import pupil_apriltags as apriltag     # for windows
import cv2

def detect_apriltag_image():
    '''检测图片里的aptiltag'''
    # 读取图片
    img =cv2.imread("./apriltag_5.jpg")

    # 转灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 创建一个apriltag检测器
    at_detector = apriltag.Detector()
    # at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9') ) # for linux
    # at_detector = apriltag.Detector(families='tag36h11 tag25h9')  #for windows

    # 进行apriltag检测，得到检测到的apriltag的列表
    tags = at_detector.detect(gray)
    print("%d apriltags have been detected."%len(tags))

    for tag in tags:
        cv2.circle(img, tuple(tag.corners[0].astype(int)), 4,(255,0,0), 2) # left-top
        cv2.circle(img, tuple(tag.corners[1].astype(int)), 4,(255,0,0), 2) # right-top
        cv2.circle(img, tuple(tag.corners[2].astype(int)), 4,(255,0,0), 2) # right-bottom
        cv2.circle(img, tuple(tag.corners[3].astype(int)), 4,(255,0,0), 2) # left-bottom

    cv2.imshow("apriltag_test",img)
    cv2.waitKey()

def detect_apriltag_video():
    '''检测视频里的apriltag'''
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # 设置摄像头的帧高属性为720像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    at_detector = apriltag.Detector()
    # at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))
    # at_detector = apriltag.Detector(families='tag36h11 tag25h9')  #for windows

    i = 0
    while (1):
        # 获得图像
        ret, frame = cap.read()

        # 检测按键
        k = cv2.waitKey(1)
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite('./apriltag_' + str(i) + '.jpg', frame)
            i += 1
        # 检测apriltag
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray)
        for tag in tags:
            # print(f'tag.corners[0]:{tag.corners[0]}')
            # print(f'tag.corners[1]:{tag.corners[1]}')
            # print(f'tag.corners[2]:{tag.corners[2]}')
            # print(f'tag.corners[3]:{tag.corners[3]}')
            print(f'tag_id:{tag.tag_id}')
            print(f'tag_family:{tag.tag_family}')
            cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
            cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
            cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
            cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom
        # 显示检测结果
        cv2.imshow('capture', frame)
        time.sleep(0.5)

    cap.release()
    cv2.destroyAllWindows()

def main():
    # detect_apriltag_image()
    detect_apriltag_video()

if __name__ == '__main__':
    main()