import mujoco
import mujoco.viewer
import glfw
import time

# 解析轨迹文件
trajectory = []
traj_path = '/media/cyj/DATA/大四下/UMI_Gripper3/ORB_SLAM3/CameraTrajectory.txt'
with open(traj_path, 'r') as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        
        # 提取位置和四元数（格式：x, y, z, w）
        pos = list(map(float, parts[1:4]))
        quat_xyzw = list(map(float, parts[4:8]))
        
        # 转换为MuJoCo需要的四元数格式（w, x, y, z）
        quat = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        trajectory.append((pos, quat))

# MuJoCo XML 场景（直接嵌入 Python 文件）
xml_string = """
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <light name="light" pos="0 0 4"/>
    <camera name="fixed" pos="0 -3 0" xyaxes="1 0 0 0 0 1"/>
    
    <body name="target" pos="0 0 0">
      <freejoint/>
      <!-- 用box表示目标物体 -->
      <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
    </body>

    
  </worldbody>
</mujoco>


"""

# 加载 MuJoCo 模型
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

# 创建查看器
viewer = mujoco.viewer.launch_passive(model, data)

# 设置播放速度控制
playback_speed = 0.01  # 调整此值改变播放速度

try:
    for pos, quat in trajectory:
        # 设置物体位姿
        data.qpos[0:3] = pos     # 前三个为位置
        data.qpos[3:7] = quat    # 后四个为四元数
        
        # 前向动力学计算
        mujoco.mj_forward(model, data)
        
        # 同步查看器
        viewer.sync()
        time.sleep(playback_speed)  # 控制播放速度

except KeyboardInterrupt:
    print("播放被用户中断")

finally:
    viewer.close()