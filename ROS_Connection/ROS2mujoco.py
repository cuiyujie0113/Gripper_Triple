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
MODEL_PATH = os.path.join(current_dir, "hinge.xml")
# MODEL_PATH = os.path.join(current_dir, "fake_gripper.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 初始化 ROS2 节点并订阅角度话题
rclpy.init()
angle_subscriber = AngleSubscriber()

last_timestamp = None  # 记录上一次消息的时间

zero_angles = [266.55029296875, 61.10595703125, 160.46630859375, 49.10888671875, 199.1162109375, 197.40234375, 267.42919921875, 12.63427734375]


joint_names = [model.joint(i).name for i in range(model.njnt)]

circle = [0, 0, 0, 0, 0, 0, 0, 0]
previous_angle = [0, 0, 0, 0, 0, 0, 0, 0]


controllers = [PositionController(model, data, joint_name) for joint_name in joint_names]

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

            # 更新控制器的目标位置
            for i,controller in enumerate(controllers):
                if not np.isnan(angles[i]):
                    current_angle = angles[i] - zero_angles[i]
                    if current_angle < 0:
                        current_angle += 360

                    if current_angle - previous_angle[i] > 180:
                        circle[i] -= 1
                    elif current_angle - previous_angle[i] < -180:
                        circle[i] += 1
                    
                    target_angle = current_angle + circle[i] * 360

                    controller.set_target_position(target_angle)
                    controller.update_control()
                    previous_angle[i] = current_angle

                    print(f"Joint {joint_names[i]}  target:{target_angle:.2f} degrees current:{current_angle:.2f} previous :{previous_angle[i]:.2f} degrees")

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
