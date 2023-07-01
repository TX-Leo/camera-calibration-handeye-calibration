'''
@Author: TX-Leo(wangzhi)
@Date: 2023.04.06(created)-2023.06.29(last modified)
@Mail: tx.leo.wz@gmail.com | wangzhi@encosmart.com
@Description:测试realsense D435i相机的自动曝光功能
'''

import pyrealsense2 as rs

# 创建管道,使用config对象配置了深度流和彩色流的分辨率、帧率和数据格式等参数，最后启动相机。
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 获取了当前管道的活动配置(profile)，并使用配置获取了相机设备的深度传感器(depth_sensor)
profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()

# 获取相机设备的自动曝光参数(auto_exposure_param)，然后使用set_option()方法将自动曝光开关打开.
auto_exposure_param = depth_sensor.get_option(rs.option.enable_auto_exposure)
depth_sensor.set_option(rs.option.enable_auto_exposure, 1)

# 关闭并销毁管道
pipeline.stop()
