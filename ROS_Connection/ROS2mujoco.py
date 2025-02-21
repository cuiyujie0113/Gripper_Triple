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
MODEL_PATH = os.path.join(current_dir, "mjcf/Gripper3.xml")
# MODEL_PATH = os.path.join(current_dir, "hinge.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 初始化 ROS2 节点并订阅角度话题
rclpy.init()
angle_subscriber = AngleSubscriber()

last_timestamp = None  # 记录上一次消息的时间

# 对应的零点角度，调整顺序以匹配角度索引
zero_angles = [283.7109375, 53.4375, 188.98681640625, 70.64208984375, 138.14208984375, 166.728515625, 126.01318359375, 138.6474609375]

# 关节名称
joint_names = [model.joint(i).name for i in range(model.njnt)]
# ['joint_thumb_1', 'joint_thumb_2', 'joint_thumb_3', 'joint_index_1', 'joint_index_2', 'joint_index_3', 'joint_mid_1', 'joint_mid_2']

# 角度值初始化
circle = [0] * 8
previous_angle = [0] * 8

# 创建控制器
controllers = [PositionController(model, data, joint_name) for joint_name in joint_names]

# 关节索引对应关系
angle_indices = [7, 6, 5, 2, 4, 3, 0, 1]  # 订阅数据在 angles 数组中的索引顺序

# 启动 MuJoCo Viewer 进行仿真
with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok() and viewer.is_running():
            # 读取 ROS2 中的角度值
            rclpy.spin_once(angle_subscriber)
            angles = angle_subscriber.latest_angles

            # 计算两次消息接收的时间间隔
            # current_timestamp = time.time()
            # if last_timestamp is not None:
            #     delta_time = current_timestamp - last_timestamp
            #     # print(f"Time interval between messages: {delta_time:.6f} seconds")
            # last_timestamp = current_timestamp

            # 更新控制器的目标位置
            for i, idx in enumerate(angle_indices):  # 按索引匹配数据
                if not np.isnan(angles[idx]):
                    current_angle = angles[idx] - zero_angles[idx]
                    if current_angle < 0:
                        current_angle += 360

                    if current_angle - previous_angle[idx] > 180:
                        circle[idx] -= 1
                    elif current_angle - previous_angle[idx] < -180:
                        circle[idx] += 1

                    target_angle = current_angle + circle[idx] * 360

                    # pos control
                    # controllers[idx].set_target_position(target_angle)
                    # controllers[idx].update_control()

                    # pos visual
                    data.qpos[i] = np.deg2rad(target_angle)

                    previous_angle[idx] = current_angle

                    print(f"Joint {joint_names[i]} target: {target_angle:.2f} degrees current: {current_angle:.2f} previous: {previous_angle[idx]:.2f} degrees circle: {circle[idx]}")

            # 进行物理仿真一步
            mujoco.mj_step(model, data)

            # 确保更新 viewer，防止不同步问题
            if viewer.is_running():
                viewer.sync()
                # sleep_time = 0.01

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        angle_subscriber.destroy_node()
        rclpy.shutdown()
