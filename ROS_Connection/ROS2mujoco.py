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

joint_names = [model.joint(i).name for i in range(model.njnt)]

controllers = [PositionController(model, data, joint_name) for joint_name in joint_names]

with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok() and viewer.is_running():
            rclpy.spin_once(angle_subscriber)
            angles = angle_subscriber.latest_angles
            for i in range(len(angles)):
                data.qpos[i] = np.deg2rad(angles[i])
                print(f"{joint_names[i]}: {angles[i]} 度, {data.qpos[i]} 弧度")

            # 进行物理仿真一步
            mujoco.mj_step(model, data)
            
            # 发布关节状态
            joint_state_publisher.publish(data.qpos, np.array(angles))
            
            if viewer.is_running():
                viewer.sync()
    
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        angle_subscriber.destroy_node()
        joint_state_publisher.node.destroy_node()
        rclpy.shutdown()
