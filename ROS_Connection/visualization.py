import mujoco
import mujoco.viewer
import numpy as np
import os
import rclpy
import time

# 加载 XML 文件
current_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(current_dir, "mjcf/Gripper3.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 关节索引
joint_index_3 = 5

# 初始角度（单位：弧度）
initial_angle = np.deg2rad(0)  # 初始角度为 0 度
target_angle = np.deg2rad(180)  # 目标角度为 90 度
speed = np.deg2rad(1)  # 每次更新增加 1 度

# 赋值初始角度
data.qpos[joint_index_3] = initial_angle

# 初始化 ROS2
rclpy.init()

for i in range(model.njnt):
    data.qpos[i] = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok():  # 使用 rclpy.ok() 检查节点状态
            
            # 逐步增加角度
            if data.qpos[joint_index_3] < target_angle:
                data.qpos[joint_index_3] += speed

            print(f"{model.joint(joint_index_3).name}: {np.rad2deg(data.qpos[joint_index_3])} 度")

            # 进行仿真和渲染
            mujoco.mj_step(model, data)
            viewer.sync()  # 同步渲染

            time.sleep(0.05)  # 添加小延迟，控制速度
    except KeyboardInterrupt:
        print("退出程序...")
    finally:
        # 仅在 rclpy 仍然是活动状态时关闭
        if rclpy.ok():
            rclpy.shutdown()
