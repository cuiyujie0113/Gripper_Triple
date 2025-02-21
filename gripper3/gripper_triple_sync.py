import rclpy
import numpy as np

from gripper3.angle_subscriber import AngleSubscriber
from gripper3.position_control import PositionController
from gripper3.envs.gripper_triple_base import GripperTripleCfg, GripperTripleBase

class GripperTripleSync(GripperTripleBase):
    def __init__(self, config:GripperTripleCfg):
        super().__init__(config)
        self.angle_subscriber = AngleSubscriber()
        self.joint_names = [self.mj_model.joint(i).name for i in range(self.nj)]
        self.controllers = [PositionController(self.mj_model, self.mj_data, joint_name) for joint_name in self.joint_names]

        self.zero_angles = [266.55, 61.11, 160.47, 49.11, 199.12, 197.40, 267.43, 12.63]
        self.circle = [0] * self.nj
        self.previous_pos = [0] * self.nj
    
    def update_control_from_ros(self):
        rclpy.spin_once(self.angle_subscriber)
        angles = self.angle_subscriber.latest_angles

        for i, controller in enumerate(self.controllers):
            if not np.isnan(angles[i]):
                current_angle = angles[i] - self.zero_angles[i]
                if current_angle < 0:
                    current_angle += 360
                
                if current_angle - self.previous_pos[i] > 180:
                    self.circle[i] -= 1
                elif current_angle - self.previous_pos[i] < -180:
                    self.circle[i] += 1
                
                target_angle = current_angle + self.circle[i] * 360
                
                # pos control
                # controller.set_target_position(target_angle)
                # controller.update_control()

                # pos visual
                self.mj_data.qpos[i] = np.deg2rad(target_angle)

                self.previous_pos[i] = current_angle

                print(f"Joint {self.joint_names[i]} target: {target_angle:.2f} degrees, current: {current_angle:.2f}, previous: {self.previous_angle[i]:.2f}")

    def step(self):
        while self.running:
            self.update_control_from_ros()
            self.step(self.init_joint_pose[:self.nj])

if __name__ == "__main__":
    rclpy.init()
    cfg = GripperTripleCfg()
    exec_node = GripperTripleSync(cfg)
    exec_node.reset()
    try:
        exec_node.step()
    except KeyboardInterrupt: 
        print("Simulation interrupted by user.")
    finally:
        exec_node.angle_subscriber.destroy_node()
        rclpy.shutdown
