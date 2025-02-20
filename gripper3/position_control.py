import mujoco
import numpy as np

class PositionController:
    def __init__(self, model, data, joint_name, kp=14, ki=0.0, kd=0.1):
        """s
        初始化单个关节的位控控制器。

        :param model: MuJoCo 模型对象
        :param data: MuJoCo 数据对象
        :param joint_name: 需要控制的关节名称
        :param kp: 比例增益
        :param ki: 积分增益
        :param kd: 微分增益
        :param dt: 控制循环的时间步长
        """
        self.model = model
        self.data = data
        self.joint_name = joint_name
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target_pos = 0.0  # 目标位置（弧度）
        self.integral_error = 0.0
        self.integral_limit = 50.0  # 限制积分项，防止积分饱和
        self.previous_pos = 0.0

        # 获取关节索引
        self.joint_id = model.joint(self.joint_name).id

    def set_target_position(self, target_deg):
        """设置目标关节位置（角度制）"""
        self.target_pos = np.deg2rad(target_deg) 
        

    def update_control(self):
        """执行一次位控计算，并应用控制力矩"""
        current_pos = self.data.qpos[self.joint_id]  # 获取当前关节位置

        # continue_pos = current_pos + self.circle * 2 * np.pi 

        # if self.joint_name == "hinge_7":
        #     print(f"Joint {self.joint_name} current position: {np.rad2deg(current_pos):.2f} degrees previous:{np.rad2deg(self.previous_pos):.2f} target position: {np.rad2deg(self.target_pos):.2f} degrees") 

            

        # 计算误差
        error = self.target_pos - current_pos

        # 积分误差计算（防止积分饱和）
        self.integral_error += error
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)

        # 计算微分项
        derivative = current_pos - self.previous_pos

        # 计算 PID 控制力矩
        torque = self.kp * error + self.ki * self.integral_error + self.kd * derivative

        # 施加力矩
        self.data.ctrl[self.joint_id] = torque

        # 更新前一次位置
        self.previous_pos = current_pos
