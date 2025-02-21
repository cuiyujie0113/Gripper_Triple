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

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 初始化 ROS2 节点并订阅角度话题
rclpy.init()
angle_subscriber = AngleSubscriber()

last_timestamp = None  # 记录上一次消息的时间

init_angles = [100, 100, 100, 100, 100, 100, 100, 100]

# 对应的零点角度，调整顺序以匹配角度索引
zero_angles = [0, 0, 0, 0, 0, 0, 0, 0]

# 关节名称
joint_names = [model.joint(i).name for i in range(model.njnt)]
print(joint_names)

# 角度值初始化
circle = [0] * 8
previous_angle = [0] * 8

# 创建控制器
controllers = [PositionController(model, data, joint_name) for joint_name in joint_names]

# 关节索引对应关系
angle_indices = [7, 6, 5, 2, 4, 3, 0, 1]  # 订阅数据在 angles 数组中的索引顺序

with mujoco.viewer.launch_passive(model, data) as viewer:
    try:
        while rclpy.ok():  # 使用 rclpy.ok() 来检查节点状态
            for i in range(model.njnt):
                data.qpos[i] = np.deg2rad(init_angles[i])
                print(f"{model.joint(i).name}: {data.qpos[i]}")
            mujoco.mj_step(model, data)
            viewer.sync()  # 同步渲染
            time.sleep(0.01)  # 添加一个小延迟以避免占用过多 CPU
    except KeyboardInterrupt:
        print("退出程序...")
    finally:
        # 确保 ROS 2 节点在程序退出时正确关闭
        rclpy.shutdown()