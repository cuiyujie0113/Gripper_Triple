import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from MT6701_I2C import MT6701Direct, MT6701WithMux
import numpy as np
import threading
import time

class Exo_angles(Node):
    def __init__(self):
        super().__init__('exo_angles')
        self.mt6701_with_mux = MT6701WithMux(bus_num=0)
        self.angle_reorder_indices = [7, 6, 5, 2, 4, 3, 0, 1]
        self.reverse_angles = [-1,-1,-1,-1,-1,-1,1,1]

        self.pub_angles = np.zeros(8)
        self.raw_angles = np.zeros(8)
        self.init_angles = np.zeros(8)
        self.current_angles = np.zeros(8) 
        self.last_angles = np.zeros(8)
        self.angles_diff = np.zeros(8)
        self.fake_circles = np.zeros(8)

        self.pub = self.create_publisher(Float32MultiArray, '/Exo/Angles', 10)
        self.pub_msg = Float32MultiArray()

        self.INIT_SET = False

        # Start update_angles thread
        self.update_angles_thread = threading.Thread(target=self.update_angles)
        self.update_angles_thread.daemon = True
        self.update_angles_thread.start()

    def update_angles(self):
        while rclpy.ok():
            # Get 8 Encoder Angles
            angles_list = []
            for channel in range(8):
                angle_mux = self.mt6701_with_mux.MT6701_I2C_read_angle(channel)
                if angle_mux is not None:
                    angles_list.append(angle_mux)
                else:
                    angles_list.append(np.nan)
                    print(f"Error reading Encoder id={channel}")
            # Reorder the angles
            self.raw_angles = np.array(angles_list)[self.angle_reorder_indices]
            # print(self.raw_angles)
            # Preprocess the angles if already initialized
            if self.INIT_SET:
                valid_mask = ~np.isnan(self.raw_angles)
                self.current_angles[valid_mask] = self.raw_angles[valid_mask] - self.init_angles[valid_mask]
                self.current_angles[self.current_angles < 0] += 360
                self.angles_diff[valid_mask] = self.current_angles[valid_mask] - self.last_angles[valid_mask]
                self.fake_circles[valid_mask] -= self.angles_diff[valid_mask] > 180
                self.fake_circles[valid_mask] += self.angles_diff[valid_mask] < -180
                self.pub_angles = self.current_angles + self.fake_circles * 360
                self.pub_angles = self.pub_angles * self.reverse_angles
                self.last_angles = self.current_angles.copy()

            time.sleep(0.001) # 1K hz update

    def set_init_angles(self):
        # Wait for user's keyboard input to set initial angles
        input("Press ENTER to set initial angles...")

        self.init_angles = self.raw_angles.copy()
        self.INIT_SET = True

        print(f"Initial angles set: {self.init_angles}")
    
    def pub_timer_start(self):
        self.pub_timer = self.create_timer(0.01, self.pub_callback)  # 100Hz
        print("Start publishing angles...")

    def pub_callback(self):
        self.pub_msg.data = self.pub_angles.copy().tolist()
        self.pub.publish(self.pub_msg)
        self.get_logger().info(f"Publishing angles: {self.pub_msg.data}")

    def destroy_node(self):
        # 关闭 MT6701 的 I2C 连接
        self.update_angles_thread.join()
        self.mt6701_with_mux.close()
        super().destroy_node()


if __name__ == '__main__':
    rclpy.init()
    exo_angles = Exo_angles()

    try:
        exo_angles.set_init_angles()
        print("1s later to start publishing angles")
        time.sleep(1)
        exo_angles.pub_timer_start()
        rclpy.spin(exo_angles)

    except KeyboardInterrupt:
        pass

    finally:
        exo_angles.destroy_node()
        rclpy.shutdown()


    





