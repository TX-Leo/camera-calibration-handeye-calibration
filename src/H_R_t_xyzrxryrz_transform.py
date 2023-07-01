'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: H(齐次矩阵),R(旋转矩阵),t(平移向量),xyzrxryrz(位姿),xyz,EulerAngle(欧拉角),Rvec(旋转矩阵)之间的转换,以及R或者xyzrxryrz的指定角度的旋转
    1.EulerAngle_to_R(rxryrz):
        欧拉角(度)转旋转矩阵
    2.R_to_EulerAngle(R):
        旋转矩阵转欧拉角(度)
    3.Rvec_to_R(rvec):
        旋转向量转旋转矩阵
    4.R_to_Rvec(R):
        旋转矩阵转旋转向量
    5.xyz_to_t(xyz):
        xyz转平移向量t
    6.t_to_xyz(t):
        平移向量t转xyz
    7.xyz_rxryrz_to_xyzrxryrz(xyz,rxryrz):
        xyz和rxryrz转xyzrxryrz
    8.Rt_to_H(R,t):
        旋转矩阵R和平移向量t结合成齐次矩阵H
    9.H_to_Rt(H):
        齐次矩阵H转旋转矩阵R和平移向量
    10.xyzrxryrz_to_H(xyzrxryrz):
        xyzrxryrz转齐次矩阵H
    11.H_to_xyzrxryrz(H):
        齐次矩阵H转xyzrxryrz
    12.rotate_xyzrxryrz(xyzrxryrz,delta_rx=0,delta_ry=0,delta_rz=0,rotate_order = 'zyx'):
        rxryrz按顺时针旋转某个角度
    13.rotate_R(R, delta_rx=0, delta_ry=0, delta_rz=0, rotate_order='zyx'):
        R按顺时针按照指定顺序旋转某个角度

@Usage: 参看Main()函数
'''


import numpy as np
import transforms3d as tfs
import cv2
import math

def EulerAngle_to_R(rxryrz):
    '''欧拉角(度)转旋转矩阵
    rxryrz为array类型 '''
    rx,ry,rz = rxryrz
    R = tfs.euler.euler2mat(math.radians(rx), math.radians(ry), math.radians(rz), axes='sxyz')  # 注意欧拉角顺序
    return R

def R_to_EulerAngle(R):
    '''旋转矩阵转欧拉角(度)'''
    rxryrz = np.degrees(tfs.euler.mat2euler(R, 'sxyz'))
    return rxryrz

def Rvec_to_R(rvec):
    '''旋转向量转旋转矩阵
    rvec为array类型
    '''
    R, _ = cv2.Rodrigues(rvec)
    return R

def R_to_Rvec(R):
    '''旋转矩阵转旋转向量'''
    Rvec, _ = cv2.Rodrigues(R)
    return Rvec

def xyz_to_t(xyz):
    '''xyz转平移向量t'''
    t = xyz.reshape(3, 1)
    return t

def t_to_xyz(t):
    '''平移向量t转xyz'''
    xyz = np.squeeze(t.reshape(1,3))
    return xyz

def xyz_rxryrz_to_xyzrxryrz(xyz,rxryrz):
    '''xyz和rxryrz转xyzrxryrz'''
    xyzrxryrz = np.concatenate((xyz, rxryrz), axis=0).flatten()
    return xyzrxryrz

def Rt_to_H(R,t):
    '''旋转矩阵R和平移向量t结合成齐次矩阵H'''
    H = tfs.affines.compose(np.squeeze(t), R, [1, 1, 1])
    return H

def H_to_Rt(H):
    '''齐次矩阵H转旋转矩阵R和平移向量'''
    R = H[0:3, 0:3]
    t = H[0:3, 3]
    return R,t

def xyzrxryrz_to_H(xyzrxryrz):
    '''xyzrxryrz转齐次矩阵H
    xyzrxryrz为array'''
    R = EulerAngle_to_R(xyzrxryrz[3:])
    t = xyz_to_t(xyzrxryrz[:3])
    H = Rt_to_H(R,t)
    return H

def H_to_xyzrxryrz(H):
    '''齐次矩阵H转xyzrxryrz
    xyzrxryrz为array'''
    R,t = H_to_Rt(H)
    xyz = t_to_xyz(t)
    rxryrz = R_to_EulerAngle(R)
    xyzrxryrz = xyz_rxryrz_to_xyzrxryrz(xyz,rxryrz)
    return xyzrxryrz

def rotate_xyzrxryrz(xyzrxryrz,delta_rx=0,delta_ry=0,delta_rz=0,rotate_order = 'zyx'):
    '''rxryrz顺时针旋转某个角度'''
    # 获取之前的旋转矩阵
    # R = Rvec_to_R(np.array(xyzrxryrz[3:]))
    R = EulerAngle_to_R(xyzrxryrz[3:])

    # 角度转弧度
    delta_rx = np.radians(delta_rx)
    delta_ry = np.radians(delta_ry)
    delta_rz = np.radians(delta_rz)
    
    # 旋转一定角度的矩阵
    R_rx = np.array([[1, 0, 0],
                    [0, np.cos(delta_rx), -np.sin(delta_rx)],
                    [0, np.sin(delta_rx), np.cos(delta_rx)]])
    R_ry = np.array([[np.cos(delta_ry), 0, np.sin(delta_ry)],
                    [0, 1, 0],
                    [-np.sin(delta_ry), 0, np.cos(delta_ry)]])
    R_rz = np.array([[np.cos(delta_rz),-np.sin(delta_rz),0],
                    [np.sin(delta_rz),np.cos(delta_rz),0],
                    [0, 0, 1]])

    # 获得新的旋转矩阵
    s = f'np.dot(np.dot(np.dot(R, R_r{rotate_order[0]}),R_r{rotate_order[1]}),R_r{rotate_order[2]})'
    new_R = eval(s)

    # 旋转矩阵转欧拉角
    new_rxryrz = R_to_EulerAngle(new_R)

    # 获取新的xyzrxryrz
    new_xyzrxryrz = xyz_rxryrz_to_xyzrxryrz(np.array(xyzrxryrz[:3]), new_rxryrz).tolist()

    return new_xyzrxryrz


def rotate_R(R, delta_rx=0, delta_ry=0, delta_rz=0, rotate_order='zyx'):
    '''R按顺时针按照指定顺序旋转某个角度'''

    # 角度转弧度
    delta_rx = np.radians(delta_rx)
    delta_ry = np.radians(delta_ry)
    delta_rz = np.radians(delta_rz)

    # 旋转一定角度的矩阵
    R_rx = np.array([[1, 0, 0],
                     [0, np.cos(delta_rx), -np.sin(delta_rx)],
                     [0, np.sin(delta_rx), np.cos(delta_rx)]])
    R_ry = np.array([[np.cos(delta_ry), 0, np.sin(delta_ry)],
                     [0, 1, 0],
                     [-np.sin(delta_ry), 0, np.cos(delta_ry)]])
    R_rz = np.array([[np.cos(delta_rz), -np.sin(delta_rz), 0],
                     [np.sin(delta_rz), np.cos(delta_rz), 0],
                     [0, 0, 1]])

    # 获得新的旋转矩阵
    s = f'np.dot(np.dot(np.dot(R, R_r{rotate_order[0]}),R_r{rotate_order[1]}),R_r{rotate_order[2]})'
    new_R = eval(s)

    # print(f'delta_rx:\n{delta_rx}')
    # print(f'delta_ry:\n{delta_ry}')
    # print(f'delta_rz:\n{delta_rz}')
    # print(f'R:\n{R}')
    # print(f'R_rx: \n{R_rx}')
    # print(f'R_ry: \n{R_ry}')
    # print(f'R_rz: \n{R_rz}')

    return new_R


def main():
    '''测试用的'''

    # xyz+欧拉角(度) 转 R,t,H
    xyzrxryrz = np.array([0.1,0.2,0.3,170,120,60])
    R = EulerAngle_to_R(xyzrxryrz[3:6])
    t = xyz_to_t(xyzrxryrz[0:3])
    H = Rt_to_H(R,t)

    # xyz+旋转向量 转 R,t,H
    xyzrxryrz = np.array([0.1, 0.2, 0.3, 0.5, 1.1, 3.0])
    R = Rvec_to_R(xyzrxryrz[3:6])
    t = xyz_to_t(xyzrxryrz[0:3])
    H = Rt_to_H(R, t)

    # H 转 R,t
    R,t = H_to_Rt(H)

    # R 转 欧拉角rxryrz(度)
    rxryrz = R_to_EulerAngle(R)

    # R 转 旋转向量
    rvec = R_to_Rvec(R)

    # t 转 xyz
    xyz = t_to_xyz(t)

    # xyz和rxryrz转xyzrxryrz
    xyz_rxryrz_to_xyzrxryrz(xyz,rxryrz)

if __name__ == '__main__':
    # main()

    '''测试用的'''
    R = [[-0.06472430155853726,-0.9973381232093603,-0.03357726583553946,],[-0.9973485867817671,0.06353005346138718,0.03549265771403007],[-0.033265015138603936,0.03578547611006781,-0.9988057060647002]]
    R= [[-0.99978544 , 0.01618313,  0.01292909],[ 0.01924667 , 0.95650893  ,0.29106747],[-0.00765641  ,0.29125386 ,-0.95661516]]

    EulerAngle = R_to_EulerAngle(np.array(R))
    for i in EulerAngle:
        print(math.radians(i))
    print(EulerAngle)