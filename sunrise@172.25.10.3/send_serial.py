import serial
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

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

    def listener_callback(self, msg):
        self.latest_angles = msg.data  # 更新最近的数据
        self.get_logger().info(f'Received angles: {self.latest_angles}')


# 设置串口参数
SERIAL_PORT = '/dev/ttyS1'  # 串口设备路径
BAUD_RATE = 115200  # 波特率
TIMEOUT = 1  # 串口超时设置

# 打开串口
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

def send_angle(angle):
    # 保留两位小数
    angle_int = int(round(angle * 100))  # 乘以100并保留两位小数
    
    # 将整数转换为两个字节
    # 使用'!H'格式字符进行大端字节序编码（H代表无符号短整型）
    angle_bytes = struct.pack('!H', angle_int)
    
    # 通过串口发送数据
    ser.write(angle_bytes)

def main():
    rclpy.init()

    angle_subscriber = AngleSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(angle_subscriber)  # 获取并处理消息

            if angle_subscriber.latest_angles:  # 如果有收到角度数据
                angle0 = angle_subscriber.latest_angles[0]  # 获取第一个角度值
                print(f"Sending angle: {angle0}")
                send_angle(angle0)  # 发送角度值通过串口

    except KeyboardInterrupt:
        pass
    finally:
        angle_subscriber.destroy_node()
        rclpy.shutdown()
        ser.close()  # 关闭串口连接


if __name__ == "__main__":
    main()
