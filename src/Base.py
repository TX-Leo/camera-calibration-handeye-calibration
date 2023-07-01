'''
@Author: 张恒硕
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Description: 串口方式控制机械臂基座沿着x轴电机的运动
@Usage: 参看Main()函数
'''

import struct
import time
import serial
import threading
import crcmod.predefined
from threading import Event

class FR5BASE(object):
    __uart_state = 0

    def __init__(self, com='/dev/tty.usbserial-1110', delay=.002,debug=False):
        # com = "COM30"
        # com="/dev/ttyTHS1"
        # com="/dev/ttyUSB0"
        # com="/dev/ttyAMA0"

        self.ser = serial.Serial(com, 115200)
        self.crc16 =crc16 = crcmod.predefined.Crc('modbus')

        self._delay_time = delay
        self._debug = debug
        self._time = 0
        self._error = 0
        self._x_tem = 0
        self._x_v = 0
        self._x_pulse = 0
        self._f1_tem = 0
        self._f2_tem = 0
        self._f3_tem = 0
        self._f4_tem = 0
        self._state = 0
        self.event = Event()


        if self._debug:
            print("cmd_delay=" + str(self._delay_time) + "s")

        if self.ser.isOpen():
            print("Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")

    def close_ser(self):
        self.ser.close()
        self.__uart_state = 0
        print("serial Close!")

    # 根据数据帧的类型来做出对应的解析
    # According to the type of data frame to make the corresponding parsing
    def __parse_data(self, ext_type, ext_data):
        # print("parse_data:", ext_data, ext_type)
        if ext_type == 0x10:
            # print(ext_data)
            #时间辍
            self._time = struct.unpack('<i', bytearray(ext_data[0:4]))[0]
            #电机错误代码，温度，电压，脉冲
            self._error = struct.unpack('<h', bytearray(ext_data[4:6]))[0]
            self._x_tem = struct.unpack('<b', bytearray(ext_data[6:7]))[0] 
            self._x_v = struct.unpack('<b', bytearray(ext_data[7:8]))[0] 
            self._x_pulse = struct.unpack('<h', bytearray(ext_data[8:10]))[0]
            #炸锅温度
            self._f1_tem = struct.unpack('<h', bytearray(ext_data[9:11]))[0]
            self._f2_tem = struct.unpack('<h', bytearray(ext_data[11:13]))[0]
            self._f3_tem = struct.unpack('<h', bytearray(ext_data[13:15]))[0]
            self._f4_tem = struct.unpack('<h', bytearray(ext_data[15:17]))[0]
            #电机和炸锅通信状态         
            self._state = struct.unpack('<b', bytearray(ext_data[15:16]))[0]
            #print(self._time,self._error,self._x_tem)
    #获取时间辍
    def _get_time(self):
        mcu_time = self._time
        return mcu_time
    #获取电机状态
    def _get_motor_state(self):
        error_code, motor_tem, motor_v, motor_pulse = self._error, self._x_tem, self._x_v, self._x_pulse
        return error_code, motor_tem, motor_v, motor_pulse
    #获取炸锅温度
    def _get_fryer_tem(self):
        f1, f2, f3, f4 = self._f1_tem, self._f2_tem, self._f3_tem, self._f4_tem
        return f1, f2, f3, f4 
    #获取通信状态
    def  _get_com_state(self):
        com_state = self._state
        return com_state     


    # 接收数据 receive data
    def __receive_data(self):
        while True:
            head1 = bytearray(self.ser.read())[0]
            if head1 == 0x55:
                head2 = bytearray(self.ser.read())[0]
                if head2 == 0xAA:
                    ext_type = bytearray(self.ser.read())[0]
                    if ext_type == 0x10:
                        ext_data = []
                        data_len = 21
                        while len(ext_data) < data_len:
                            value = bytearray(self.ser.read())[0]
                            ext_data.append(value)

                            #if len(ext_data) == data_len:
                            #    r_check_num = value
                            #else:
                            #    check_sum = check_sum + value 
                        check_sum = [head1,head2,ext_type]+ext_data[:-2]
                        crc16 = self.crc16.new(bytearray(check_sum)).digest()
                        little_endian_data = int.from_bytes(crc16, byteorder='big')
                        if little_endian_data == struct.unpack('<H', bytearray(ext_data[-2:]))[0]:
                            self.__parse_data(ext_type, ext_data)
                        else:
                            if self._debug:
                                print("check sum error:", len(check_sum), ext_type, ext_data)
            if self.event.is_set():
                self.close_ser()
                break



    # 开启接收和处理数据的线程
    # Start the thread that receives and processes data
    def create_receive_threading(self):
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                # task_receive.setDaemon(True)
                task_receive.daemon = True
                task_receive.start()
                print("----------------create receive threading--------------")
                self.__uart_state = 1
        except:
            print('---create_receive_threading error!---')
            pass
    
    #x轴电机移动
    def _ser_x_move(self,position,velocity=100):
        if position > 1250 or velocity >200:
            print('set x move error')
            return
        current_time = bytearray(struct.pack('<i',int(time.time())))
        move_position = bytearray(struct.pack('<h',position))
        vel = bytearray(struct.pack('<b',velocity))
        
        cmd =[0x55,0xAA,0x22,current_time[0],current_time[1],current_time[2],current_time[3],move_position[0],move_position[1],vel[0],0x00]
        crc16 = self.crc16.new(bytearray(cmd)).digest()
        cmd = cmd + [bytearray(crc16)[1],bytearray(crc16)[0]]
        self.ser.write(cmd)
        
        if self._debug:
            print("uartServo:", cmd)
        time.sleep(self._delay_time)
    
    #炸锅控制
    def _set_fryer(self,id,tem,switch):
        current_time = bytearray(struct.pack('<i',int(time.time())))
        id = bytearray(struct.pack('<b',id))
        tem = bytearray(struct.pack('<b',tem))
        switch =bytearray(struct.pack('<b',switch))
        cmd = [0x55,0xAA,0x23,current_time[0],current_time[1],current_time[2],current_time[3],id[0],tem[0],switch[0],0x00]
        crc16 = self.crc16.new(bytearray(cmd)).digest()
        cmd = cmd + [bytearray(crc16)[1],bytearray(crc16)[0]]
        self.ser.write(cmd)
    
    #下料机控制
    def _set_dispenser(self):
        cmd =[0x55,0xAA,0x24,0x01]
        crc16 = self.crc16.new(bytearray(cmd)).digest()
        cmd = cmd + [bytearray(crc16)[1],bytearray(crc16)[0]]
        self.ser.write(cmd)
    
    #Autobin
    def _set_Autobin(self):
        cmd =[0x55,0xAA,0x25,0x01]
        crc16 = self.crc16.new(bytearray(cmd)).digest()
        cmd = cmd + [bytearray(crc16)[1],bytearray(crc16)[0]]
        self.ser.write(cmd)



    # 重置小车flash保存的数据，恢复出厂默认值。
    # Reset the car flash saved data, restore the factory default value
    #def reset_flash_value(self):
    #    try:
    #        cmd = [0xff, 0xfe, 0x04, self.FUNC_RESET_FLASH, 0x5F]
    #        checksum = sum(cmd, 3) & 0xff
    #        cmd.append(checksum)
    #        self.ser.write(cmd)
    #        if self._debug:
    #            print("flash:", cmd)
    #        time.sleep(self._delay_time)
    #        time.sleep(.1)
    #    except:
    #        print('---reset_flash_value error!---')
    #        pass

    # 清除单片机自动发送过来的缓存数据
    # Clear the cache data automatically sent by the MCU
    def clear_auto_report_data(self):
        self._time = 0
        self._error, self._x_tem, self._x_v, self._x_pulse = 0, 0, 0, 0
        self._f1_tem, self._f2_tem, self._f3_tem, self._f4_tem = 0, 0, 0, 0
        self._state = 0

    # 获取底层单片机版本号，如1.1
    # Get the underlying microcontroller version number, such as 1.1
    #def get_version(self):
    #    if self.__version_H == 0:
    #        for i in range(0, 10):
    #            self.__request_data(self.FUNC_VERSION)
    #            if self.__version_H != 0:
    #                val = self.__version_H * 1.0
    #                self.__version = val + self.__version_L / 10.0
    #                if self._debug:
    #                    print("get_version:V{0}, i:{1}".format(self.__version, i))
    #                return self.__version
    #    else:
    #        return self.__version
    #    return -1


if __name__ == '__main__':
    # for test serial
    # s = serial.Serial('/dev/tty.usbserial-1230',115200)
    # data = s.read(30)
    # print(f'test receive data:{data}')

    # create a base
    base = FR5BASE()
    base.create_receive_threading()

    # # test print time
    # for i in range(10):
    #     print(f'time:{base._time}')

    # move to initial position
    base._ser_x_move(position=0, velocity=100)
    time.sleep(5)

    # move to position=200
    base._ser_x_move(position=1220, velocity=100)

    # print now_base_x
    for i in range(30):
        _, _, _, now_base_x = base._get_motor_state()
        print(f'now_base_x:{now_base_x}')
        time.sleep(0.5)