#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#

import sys
import os
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray

sys.path.append("..")
# from scservo_sdk import *                      # Uses FTServo SDK library

class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        # 创建订阅者，订阅名为 'angles' 的话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'angles',
            self.listener_callback,
            10)
        self.subscription  # 让订阅对象不被垃圾回收

        self.latest_angles = []
        self.last_angles = []
        self.change_angles = []

    def listener_callback(self, msg):
        self.last_angles = self.latest_angles
        self.latest_angles = msg.data  # 更新最近的数据
        #self.get_logger().info(f'Change angle: {self.change_angles}')
        
        # self.get_logger().info(f'Received angles: {self.latest_angles}')

# 拇指侧摆关节 ID 1 [128,2042]
# 拇指第二关节 ID 2 [2314,4073]
# 拇指质监关节 ID 3 [2468,4095]

InitPos = [0,1085,3193,3281,0,0,0,0,0,0]
ServoPos = [0,0,0,0,0,0,0,0,0]
ServoVel = [0,0,0,0,0,0,0,0,0]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
# portHandler = PortHandler('/dev/ttyUSB0')# ex) sWindows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PacketHandler instance
# Get methods and members of Protocol
# packetHandler = sms_sts(portHandler)
    
# Open port
# if portHandler.openPort():
#     print("Succeeded to open the port")
# else:
#     print("Failed to open the port")
#     quit()

# # Set port baudrate 1000000
# if portHandler.setBaudRate(1000000):
#     print("Succeeded to change the baudrate")
# else:
#     print("Failed to change the baudrate")
#     quit()

def ReadServoData():
    while True:
        for i in range(1,4):
            ServoPos[i] , ServoVel[i], scs_comm_result, scs_error = packetHandler.ReadPosSpeed(i)
            print('read')
            time.sleep(0.01)

def ServoControl():
    while True:
        for i in range(1,4):

            scs_comm_result, scs_error = packetHandler.WritePosEx(i, InitPos[i], 1000, 50)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
            time.sleep(0.001)

def main(args = None):
    rclpy.init(args=args)
    EncoderData = AngleSubscriber()
# 初始化舵机参数
    # for i in range(1,4):
    #     scs_comm_result, scs_error = packetHandler.ChangeLevelResponse(i, 0)

    #     if scs_comm_result != COMM_SUCCESS:
    #             print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    #     elif scs_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(scs_error))

    #     scs_comm_result, scs_error = packetHandler.ProtectCurrent(i, 1023)
        
    #     if scs_comm_result != COMM_SUCCESS:
    #             print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    #     elif scs_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(scs_error))

    # thread_ServoControl = threading.Thread(target=ServoControl)
    # thread_ReadServoData = threading.Thread(target=ReadServoData)
    #thread_ReadEncoder = threading.Thread(target=ReadEncoder)

    #thread_ServoControl.start()
    #thread_ReadServoData.start()
    #thread_ReadEncoder.start()

    try:
        rclpy.spin(EncoderData)
        print(EncoderData.latest_angles)
    except KeyboardInterrupt:
        pass
    finally:
        EncoderData.destroy_node()

    #thread_ServoControl.join()
    #thread_ReadServoData.join()
    #thread_ReadEncoder.join()

if __name__ =="__main__":
    main()


# Close port
# portHandler.closePort()
