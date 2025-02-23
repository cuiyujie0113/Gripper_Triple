import mujoco
import mujoco.viewer
import numpy as np
import os
import rclpy
import time
from std_msgs.msg import Float32MultiArray
from angle_subscriber import AngleSubscriber
from position_control import PositionController

class JointStatePublisher:
    def __init__(self):
        self.node = rclpy.create_node("joint_state_publisher")
        self.publisher = self.node.create_publisher(Float32MultiArray, "joint_states", 10)
        self.rad_publisher = self.node.create_publisher(Float32MultiArray, "target_pos", 10)

    def publish(self, qpos, qpos_rad):
        msg = Float32MultiArray()
        msg.data = qpos.tolist()
        self.publisher.publish(msg)
        
        rad_msg = Float32MultiArray()
        rad_msg.data = qpos_rad.tolist()
        self.rad_publisher.publish(rad_msg)

# 加载 XML 文件
current_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(current_dir, "mjcf/Gripper3.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 初始化 ROS2 节点并订阅角度话题
rclpy.init()
angle_subscriber = AngleSubscriber()
joint_state_publisher = JointStatePublisher()

zero_angles = [320.07568359375, 46.60400390625, 42.78076171875, 72.39990234375, 155.3466796875, 128.759765625, 244.92919921875, 296.69677734375]

joint_names = [model.joint(i).name for i in range(model.njnt)]

circle = [0] * 8
previous_angle = [0] * 8
current_angle = [0] * 8
target_angle = [0] * 8

controllers = [PositionController(model, data, joint_name) for joint_name in joint_names]
angle_indices = [7, 6, 5, 2, 4, 3, 0, 1]

with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok() and viewer.is_running():
            rclpy.spin_once(angle_subscriber)
            angles = angle_subscriber.latest_angles

            for i, idx in enumerate(angle_indices):
                if not np.isnan(angles[idx]):
                    current_angle[idx] = angles[idx] - zero_angles[idx]
                    if current_angle[idx] < 0:
                        current_angle[idx] += 360

                    if current_angle[idx] - previous_angle[idx] > 180:
                        circle[idx] -= 1
                    elif current_angle[idx] - previous_angle[idx] < -180:
                        circle[idx] += 1

                    target_angle[idx] = current_angle[idx] + circle[idx] * 360
                    data.qpos[i] = np.deg2rad(target_angle[idx])
                    previous_angle[idx] = current_angle[idx]

                    # print(f"{joint_names[i]}: {target_angle[idx]} 度, {data.qpos[i]} 弧度")

            # 进行物理仿真一步
            mujoco.mj_step(model, data)
            
            # 发布关节状态
            joint_state_publisher.publish(data.qpos, np.array(target_angle))
            
            if viewer.is_running():
                viewer.sync()
    
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        angle_subscriber.destroy_node()
        joint_state_publisher.node.destroy_node()
        rclpy.shutdown()
