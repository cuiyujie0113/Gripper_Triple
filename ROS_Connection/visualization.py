import mujoco
import mujoco.viewer
import numpy as np
import os
import rclpy
import time
from angle_subscriber import AngleSubscriber
from position_control import PositionController

# 加载 XML 文件
current_dir = os.path.dirname(os.path.abspath(__file__))
# MODEL_PATH = os.path.join(current_dir, "hinge.xml")
MODEL_PATH = os.path.join(current_dir, "fake_gripper.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 初始化 ROS2 节点并订阅角度话题
rclpy.init()
angle_subscriber = AngleSubscriber()

last_timestamp = None  # 记录上一次消息的时间


# 打开 MuJoCo Viewer 进行仿真
with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok() and viewer.is_running():
            # 读取 ROS2 中的角度值
            rclpy.spin_once(angle_subscriber)
            angles = angle_subscriber.latest_angles

            # 计算两次消息接收的时间间隔
            current_timestamp = time.time()
            if last_timestamp is not None:
                delta_time = current_timestamp - last_timestamp
                # print(f"Time interval between messages: {delta_time:.6f} seconds")
            last_timestamp = current_timestamp

            # 更新qpos
            for i in range(len(angles)):
                if not np.isnan(angles[i]):
                    data.qpos[i] = np.deg2rad(angles[i])
                    

            # 进行物理仿真一步
            mujoco.mj_step(model, data)

            # 确保更新 viewer，防止不同步问题
            if viewer.is_running():
                viewer.sync()  

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        angle_subscriber.destroy_node()
        rclpy.shutdown()
