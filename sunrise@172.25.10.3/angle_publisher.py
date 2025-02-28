import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from MT6701_I2C import MT6701Direct, MT6701WithMux
import math

# source /opt/ros/humble/setup.bash

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'angles', 10)
        self.timer = self.create_timer(0.0001, self.timer_callback)


        # init MT6701
        self.mt6701_direct = MT6701Direct(bus_num=5)
        self.mt6701_with_mux = MT6701WithMux(bus_num=0)
        self.get_logger().info('AnglePublisher node initialized.')

    def timer_callback(self):
        # 初始化角度列表
        angles = [math.nan] * 8
        msg_angles = Float32MultiArray()

        # 创建Header并设置时间戳
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 设置时间戳

        # # direct connect
        # angle_direct = self.mt6701_direct.MT6701_I2C_read_angle()
        # if angle_direct is not None:
        #     angles[0] = angle_direct
        # else:
        #     angles[0] = math.nan

        # connect with mux
        for channel in range(8):
            angle_mux = self.mt6701_with_mux.MT6701_I2C_read_angle(channel)
            if angle_mux is not None:
                angles[channel] = angle_mux
            else:
                angles[channel] = math.nan

        # 将角度数据赋值给消息
        msg_angles.data = angles

        # 发布消息
        self.publisher_.publish(msg_angles)

        # 日志记录，显示时间戳和角度数据
        self.get_logger().info(f'timestamp: {header.stamp}, Publishing angles: {msg_angles.data}')

    def destroy_node(self):
        # 关闭 MT6701 的 I2C 连接
        self.mt6701_direct.close()
        self.mt6701_with_mux.close()
        super().destroy_node()

if __name__ == '__main__':
    rclpy.init()
    angle_publisher = AnglePublisher()
    try:
        rclpy.spin(angle_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        angle_publisher.destroy_node()
        rclpy.shutdown()
