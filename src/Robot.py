'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description: 机械臂运动控制,使用asynico控制方式,主要函数有
    1.move
        关节空间运动(PTP),对于有没有joints和pose都可以兼容
        1.输入joints和pose
        2.只输入pose,joints可以逆运动学GetInverseKin解算出来
        3.只输入joints,pose可以正运动学GetForwardKin解算出来
    2.get_pose
        获取当前位姿(x,y,z,rx,ry,rz) xyz都是以mm为单位,rxryrz都是以度位单位
    3.GetInverseKin
        逆运动学解算,pose2joints
    4.GetForwardKin
        正运动学解算joints2pose
    5.switch_electromagnet
        开关电磁铁
    6.plane_grasp
        平面抓取动作流程

@Usage: 参看Main()函数
python3 Robot.py                                            \
--host 192.168.58.2                                         \
--port 8080                                                 \
--joints [-168.847,-93.977,-93.118,-80.262,88.985,11.831]   \
--pose [-558.082,27.343,208.135,-177.205,-0.450,89.288]
'''

import time
import argparse
import asyncio

class FR5ROBOT:
    '''
    FR5机械臂asynico控制
    '''
    def __init__(self, host, port):
        '''
        初始化机械臂,定义ip地址和端口
        '''
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
        self.home_pose = [-131.608,-268.481,211.671,-177.557,-0.462,-41.956] # 设置一个home状态

    async def connect(self):
        '''
        连接机械臂
        '''
        self.reader, self.writer = await asyncio.open_connection(self.host, self.port)


    async def disconnect(self):
        '''
        断开连接机械臂
        '''
        self.writer.close()
        await self.writer.wait_closed()

    async def send_command(self, cmd):
        '''
        发送字符串
        '''
        self.writer.write(cmd.encode())
        await self.writer.drain()

    async def receive_data(self):
        '''
        接收字符串
        '''
        data = await self.reader.read(1024)
        return data.decode()

    async def GetInverseKin(self,pose):
        '''
        逆运动学解算,pose2joints
        :param pose: xyzrxryrz
        :return: joints:j1,j2,j3,j4,j5,j6
        '''
        
        # print('==============开始逆运动学解算=============')
        HEAD = '/f/b'
        END = '/b/f'
        DELIMITOR = 'III'
        CNT = '52'
        CMD_ID = '377'
        DATA = f"GetInverseKin(" \
               f"0," \
               f"{','.join([str(p) for p in pose])}," \
               f"-1)"
        LEN = str(len(DATA))
        cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
        
        # print(f'==============[发送逆运动学解算指令]：{cmd_str}==============')
        # 发送
        await self.send_command(cmd_str)
        # 接收
        receive_str = await self.receive_data()
        
        # print(f'==============[接收逆运动学解算指令]：{receive_str}==============')  # /f/bIII52III377IIIyIIIxIII/b/f 解码获得x
        # 解码receive_str
        last_iii_pos = receive_str.rfind('III')
        second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
        joints = receive_str[second_last_iii_pos + 3:last_iii_pos]
        #print(f'==============[解码获得joints]：{joints}==============')
        return joints

    async def GetForwardKin(self,joints):
        '''
        正运动学解算joints2pose
        :param joints: j1,j2,j3,j4,j5,j6
        :return: pose: x,y,z,rx,ry,rz
        '''
        
        # print('==============开始正运动学解算=============')
        HEAD = '/f/b'
        END = '/b/f'
        DELIMITOR = 'III'
        CNT = '52'
        CMD_ID = '377'
        DATA = f"GetForwardKin(" \
               f"{','.join([str(j) for j in joints])})"
        LEN = str(len(DATA))
        cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
        
        # print(f'==============[发送正运动学解算指令]：{cmd_str}==============')
        # 发送
        await self.send_command(cmd_str)
        # 接收
        receive_str = await self.receive_data()
        
        # print(f'==============[接收正运动学解算指令]：{receive_str}==============')  # /f/bIII52III377IIIyIIIxIII/b/f 解码获得x
        # 解码receive_str
        last_iii_pos = receive_str.rfind('III')
        second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
        pose = receive_str[second_last_iii_pos + 3:last_iii_pos]
        # print(f'==============[解码获得pose]：{pose}==============')
        return pose

    async def move(self, pose = None, joints = None, tool=0, user=0, vel=70, acc=100, ovl=70, exaxis_pos=[0, 0, 0, 0],blendT=0, offset_flag=0, offset_pos=[0.0] * 6):
        ''' 
            关节空间运动(PTP)
                - 可以同时输入joints和pose,注意要对应
                - 可以只输入关节信息joints,位姿信息pose可以正运动学解算出来
                - 可以只输入位姿信息pose,关节信息joints可以逆运动学解算出来
            
            :param pose:目标笛卡尔位姿,单位xyz:[mm]rxryrz:[°]
            :param tool:工具号,[0~14]；默认为0(当改变机械臂末端坐标系时要改变成对应的tool标号)
            :param user:工件号,[0~14]；默认为0
            :param vel:速度百分比,[0~100]；30-50较为合适（最终的速度为vel*ovl,最大是100*100）
            :param acc:加速度百分比,[0~180]; 默认为100; 注意！！：目前设置这个参数没有任何作用
            :param ovl:速度缩放因子,[0~100]； 30-70较为合适
            :param exaxis_pos:外部轴 1 位置~外部轴 4 位置； 默认为[0,0,0,0]
            :param blendT:[-1.0]-运动到位(阻塞),[0~500]-平滑时间(非阻塞),单位[ms]；默认为0
            :param offset_flag:[0]-不偏移,[1]-工件/基坐标系下偏移,[2]-工具坐标系下偏移；默认为0
            :param offset_pos:位姿偏移量,单位[mm][°]；默认为[0,0,0,0,0,0]
            :return:ret('0':成功 '1'：失败)
        '''

        # print('==============开始运动=============')
        # =======================如果没有设置joints,则用逆运动学算出来pose2joints=======================
        if joints == None:
            #print('==============开始逆运动学解算=============')
            HEAD = '/f/b'
            END = '/b/f'
            DELIMITOR = 'III'
            CNT = '52'
            CMD_ID = '377'
            DATA = f"GetInverseKin(" \
                      f"0," \
                      f"{','.join([str(p) for p in pose])}," \
                      f"-1)"
            LEN = str(len(DATA))
            cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
            # print(f'==============[发送逆运动学解算指令]：{cmd_str}==============')
            # 发送
            await self.send_command(cmd_str)
            # 接收
            receive_str = await self.receive_data()
            # print(f'==============[接收逆运动学解算指令]：{receive_str}==============')# /f/bIII52III377IIIyIIIxIII/b/f
            # 解码receive_str
            last_iii_pos = receive_str.rfind('III')
            second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
            joints = receive_str[second_last_iii_pos + 3:last_iii_pos]
            # print(f'==============[解码获得joints]：{joints}==============')# /f/bIII52III377IIIyIIIxIII/b/f

        # =======================如果没有pose,则用正运动学算出来joints2pose=======================
        if pose == None:
            #print('==============开始正运动学解算=============')
            HEAD = '/f/b'
            END = '/b/f'
            DELIMITOR = 'III'
            CNT = '52'
            CMD_ID = '377'
            DATA = f"GetForwardKin(" \
                   f"{','.join([str(j) for j in joints])})"
            LEN = str(len(DATA))
            cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
            #print(f'==============[发送正运动学解算指令]：{cmd_str}==============')
            # 发送
            await self.send_command(cmd_str)
            # 接收
            receive_str = await self.receive_data()
            #print(f'==============[接收正运动学解算指令]：{receive_str}==============')  # /f/bIII52III377IIIyIIIxIII/b/f
            # 解码receive_str
            last_iii_pos = receive_str.rfind('III')
            second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
            pose = receive_str[second_last_iii_pos + 3:last_iii_pos]
            #print(f'==============[解码获得pose]：{pose}==============')  # /f/bIII52III377IIIyIIIxIII/b/f

        # =======================控制运动MoveJ======================
        # cmd_str
        HEAD = '/f/b'
        END = '/b/f'
        DELIMITOR = 'III'
        CNT = '52'
        CMD_ID = '201'
        DATA = f"MoveJ(" \
               f"{','.join([str(p) for p in joints]) if type(joints)!=str else joints}," \
               f"{','.join([str(p) for p in pose]) if type(pose)!=str else pose}," \
               f"{str(tool)}," \
               f"{str(user)}," \
               f"{str(vel)}," \
               f"{str(acc)}," \
               f"{str(ovl)}," \
               f"{','.join([str(e) for e in exaxis_pos])}," \
               f"{str(blendT)},{str(offset_flag)}," \
               f"{','.join([str(o) for o in offset_pos])})"
        LEN = str(len(DATA))
        cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
        # print(f'==============[发送MoveJ(PTP)指令]：{cmd_str}==============')
        # 发送
        await self.send_command(cmd_str)
        # 接收
        receive_str = await self.receive_data()
        # 等待机械臂执行完指令
        # time.sleep(2)
        # print(f'==============[接收MoveJ(PTP)指令]：{receive_str}==============') # /f/bIII52III377IIIyIIIxIII/b/f
        # 解码receive_str
        last_iii_pos = receive_str.rfind('III')
        second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
        ret = receive_str[second_last_iii_pos + 3:last_iii_pos] #成功返回1,失败返回0
        # print(f'==============[解码获得运动是否成功ret]：{ret}==============')
        return ret # '0' or '1'

    async def get_pose(self):
        '''
        获取当前工具位姿,其中xyz是以mm为单位的,rxryrz是欧拉角,是以度为单位的
        :return: pose(x,y,z,rx,ry,rz)
        '''

        HEAD = '/f/b'
        END = '/b/f'
        DELIMITOR = 'III'
        CNT = '52'
        CMD_ID = '377'
        DATA = f"GetActualTCPPose()"
        LEN = str(len(DATA))
        cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
        # print(f'==============[发送获得当前末端位姿get_pose指令]：{cmd_str}==============')
        # 发送
        await self.send_command(cmd_str)
        # 接收
        receive_str = await self.receive_data()
        # print(f'==============[接收获得当前末端位姿get_pose指令]：{receive_str}==============') # /f/bIII52III377IIIyIIIxIII/b/f
        # 解码receive_str
        last_iii_pos = receive_str.rfind('III')
        second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
        pose = receive_str[second_last_iii_pos + 3:last_iii_pos]  # 返回位姿
        # 字符串转列表
        pose = [float(x) for x in pose.split(',')]
        # print(f'==============[解码获得末端位姿pose]：{pose}==============')
        return pose

    async def switch_electromagnet(self,bopen):
        '''
        开关电磁铁
        :param: bopen: 1(开) or 0(关)
        :return: ret
        '''

        HEAD = '/f/b'
        END = '/b/f'
        DELIMITOR = 'III'
        CNT = '52'
        CMD_ID = '210'
        DATA = f"SetToolDO(" \
               f"0," \
               f"{str(bopen)}," \
               f"0)"
        LEN = str(len(DATA))
        cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
        # print(f'==============[发送开关电磁铁指令]：{cmd_str}==============')
        # 发送
        await self.send_command(cmd_str)
        # 接收
        receive_str = await self.receive_data()# /f/bIII52III377IIIyIIIxIII/b/f
        # print(f'==============[接收开关电磁铁指令]：{receive_str}==============')
        # 解码receive_str
        last_iii_pos = receive_str.rfind('III')
        second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
        ret = receive_str[second_last_iii_pos + 3:last_iii_pos]  # ret
        # print(f'==============[解码获得是否开关成功]：{ret}==============')
        return ret

    async def plane_grasp(self,target2base_xyzrxryrz):
        '''
        平面抓取动作流程
        :param: target2base_xyzrxryrz:抓取目标的位姿
        :return: None
        '''

        # home状态
        await self.move(self.home_pose)
        time.sleep(3)
        # 达到上方0.5m处
        target2base_xyzrxryrz_above = target2base_xyzrxryrz
        target2base_xyzrxryrz_above[2] += 500
        await self.move(target2base_xyzrxryrz_above)
        time.sleep(3)
        # 开启夹抓
        await self.switch_electromagnet(0)
        # 从上到下直线运动到目标位置
        await self.move(target2base_xyzrxryrz)
        time.sleep(3)
        # 关闭夹抓
        await self.switch_electromagnet(1)
        # 拿起来到上方0.5m处
        await self.move(target2base_xyzrxryrz_above)
        time.sleep(3)


async def main():
    '''主函数'''
    # =====================设置parser=====================
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='Robot')
    parser.add_argument('--host', type=str, default="192.168.50.2", help='tcp host')  # 设置立方体的边长
    parser.add_argument('--port', type=int, default=8080,help='tcp port')  # 设置立方体的中心位置
    parser.add_argument('--joints', type=list, default=None, help='the target pose')  # 设置采样点的个数
    parser.add_argument('--pose', type=list, default=None,help='the target joints')  # 设置采样点的路径

    # =====================初始化参数=====================
    args = parser.parse_args()
    host = args.host
    port = args.port
    joints = args.joints
    pose = args.pose

    # ================创建机械臂控制对象并连接================
    robot = FR5ROBOT(host, port)
    await robot.connect()

    # ================获取位姿测试================
    pose = await robot.get_pose()
    print('pose: {}'.format(pose))

    # ================开关电磁铁测试================
    ret = await robot.switch_electromagnet(1) # 开
    time.sleep(1)
    ret = await robot.switch_electromagnet(0) # 关
    time.sleep(1)

    # ================逆运动学解算测试===============
    joints = await robot.GetInverseKin(pose=[341.672333, -139.154388, 289.773071, 179.591690, 12.606270, -3.254214])

    # ================正运动学解算测试===============
    pose = await robot.GetForwardKin(joints=[-1.521, -77.073, -126.277, -79.258, 89.968, -88.308])

    # ================PTP运动：笛卡尔坐标控制测试================
    home_pose = [-131.608,-268.481,211.671,-177.557,-0.462,-41.956]
    ret = await robot.move(pose=home_pose)
    print(f'ret:{ret}')
    time.sleep(3)
    
    # ================PTP运动：关节位置控制测试================
    joints = [-1.521, -77.073, -126.277, -79.258, 89.968, -88.308]
    ret = await robot.move(joints=joints)
    print(f'ret:{ret}')
    time.sleep(3)

    # ================断开连接================
    await robot.disconnect()

if  __name__ == '__main__':
    asyncio.run(main())
