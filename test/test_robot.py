'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.04.27(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description:测试FR5机械臂收发数据是否正确
@Usage: 参看Main()函数
'''
def test_move_pose_send_str():
    joints = [-168.847,-93.977,-93.118,-80.262,88.985,11.831]
    pose = [-558.082,27.343,208.135,-177.205,-0.450,89.288]
    tool = 1
    user = 0
    vel = 25
    acc = 100
    ovl = 100
    exaxis_pos = [0,0,0,0]
    blendT = -1
    offset_flag = 0
    offset_pos = [0.0] * 6

    HEAD = '/f/b'
    END = '/b/f'
    DELIMITOR = 'III'
    CNT = '52'
    CMD_ID = '201'
    #joints = '111.11,222.22,333.33,444,44,555.55,666.66'
    DATA = f"MoveJ(" \
           f"{','.join([str(j) for j in joints])}," \
           f"{','.join([str(p) for p in pose])}," \
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
    print(cmd_str)

def test_move_pose_receive_str():
    #receive_str = '/f/bIII52III201III1III1III/b/f'
    receive_str = '/f/bIII52III201III1III111.11,222.22,333.33,444,44,555.55,666.66III/b/f'
    last_iii_pos = receive_str.rfind('III')
    second_last_iii_pos = receive_str.rfind('III', 0, last_iii_pos - 1)
    ret = receive_str[second_last_iii_pos + 3:last_iii_pos]  # 成功返回1，失败返回0
    print(ret)

def test_GetInverseKin_send_str():
    pose = [-558.082, 27.343, 208.135, -177.205, -0.450, 89.288]
    HEAD = '/f/b'
    END = '/b/f'
    DELIMITOR = 'III'
    CNT = '52'
    CMD_ID = '377'
    DATA = f"GetInverseKin(" \
           f"1," \
           f"{','.join([str(p) for p in pose])}," \
           f"-1)"
    LEN = str(len(DATA))
    cmd_str = HEAD + DELIMITOR + CNT + DELIMITOR + CMD_ID + DELIMITOR + LEN + DELIMITOR + DATA + DELIMITOR + END
    print(cmd_str)

def main():
    test_move_pose_send_str()
    test_move_pose_receive_str()
    test_GetInverseKin_send_str()

if __name__ == "__main__":
    main()