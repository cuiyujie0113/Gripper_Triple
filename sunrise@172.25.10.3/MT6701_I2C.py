import smbus2
import time
import threading

# MT6701 定义
MT6701_SLAVE_ADDR = 0x06  # MT6701 地址
MT6701_REG_ANGLE_14b = 0x03  # 14 位角度寄存器
MT6701_TIMEOUT = 0.05  # 超时时间（秒）

# TCA9548A 定义
TCA9548A_SLAVE_ADDR = 0x70  # TCA9548A 地址
TCA9548A_CHANNELS = [
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
]  # 通道选择


class MT6701Direct:
    """直接连接的 MT6701 操作类"""

    def __init__(self, bus_num):
        """初始化 MT6701"""
        self.bus = smbus2.SMBus(bus_num)

    def write_reg(self, reg, value):
        """向 MT6701 写入寄存器"""
        try:
            self.bus.write_byte_data(MT6701_SLAVE_ADDR, reg, value)
            return True
        except Exception as e:
            print(f"Error writing to MT6701 register {reg}: {e}")
            return False

    def read_reg(self, reg, length):
        """从 MT6701 读取寄存器"""
        try:
            data = self.bus.read_i2c_block_data(MT6701_SLAVE_ADDR, reg, length)
            return data
        except Exception as e:
            print(f"Error reading from MT6701 register {reg}: {e}")
            return None

    def get_angle(self):
        """获取 MT6701 的 14 位角度值"""
        temp = self.read_reg(MT6701_REG_ANGLE_14b, 2)
        if temp is None:
            return None, None

        # 组合 14 位角度值
        angle_int = (temp[0] << 6) | (temp[1] >> 2)
        angle = angle_int * 360 / 16384
        return angle_int, angle
    
    def MT6701_I2C_read_angle(self):
        try:
            angle_int, angle = self.get_angle()
            if angle is not None:
                return angle
            else:
                print("Failed to read angle.")
                return None
        except Exception as e:
            print(f"Error in MT6701_I2C_read_angle: {e}")
            return None

    def close(self):
        """关闭 I2C 连接"""
        self.bus.close()


class MT6701WithMux:
    """通过 TCA9548A MUX 连接的 MT6701 操作类"""

    def __init__(self, bus_num):
        """初始化 TCA9548A 和 MT6701"""
        self.bus = smbus2.SMBus(bus_num)
        self.i2c_lock = threading.Lock()

        
    def set_mux_channel(self, channel):
        """选择 TCA9548A 的通道"""
        with self.i2c_lock:
            if channel < 0 or channel > 7:
                raise ValueError("Invalid TCA9548A channel, must be 0-7.")
            try:
                self.bus.write_byte(TCA9548A_SLAVE_ADDR, TCA9548A_CHANNELS[channel])
                return True
            except Exception as e:
                print(f"Error selecting TCA9548A channel {channel}: {e}")
                return False

    def write_reg(self, reg, value, channel):
        """向通过 MUX 的 MT6701 写入寄存器"""
        with self.i2c_lock:
            if not self.set_mux_channel(channel):
                return False
            try:
                self.bus.write_byte_data(MT6701_SLAVE_ADDR, reg, value)
                return True
            except Exception as e:
                print(f"Error writing to MT6701 register {reg}: {e}")
                return False

    def read_reg(self, reg, length, channel):
        with self.i2c_lock:
            """从通过 MUX 的 MT6701 读取寄存器"""
            if not self.set_mux_channel(channel):
                return None
            try:
                data = self.bus.read_i2c_block_data(MT6701_SLAVE_ADDR, reg, length)
                return data
            except Exception as e:
                print(f"Error reading from MT6701 register {reg}: {e}")
                return None

    def get_angle(self, channel):
        with self.i2c_lock:
            """获取通过 MUX 的 MT6701 的 14 位角度值"""
            temp = self.read_reg(MT6701_REG_ANGLE_14b, 2, channel)
            if temp is None:
                return None, None

            # 组合 14 位角度值
            angle_int = (temp[0] << 6) | (temp[1] >> 2)
            angle = angle_int * 360 / 16384
            return angle_int, angle

    def MT6701_I2C_read_angle(self, channel):
        with self.i2c_lock:
            try:
                angle_int, angle = self.get_angle(channel)
                if angle is not None:
                    return angle
                else:
                    print("Failed to read angle.")
                    return None
            except Exception as e:
                print(f"Error in MT6701_I2C_read_angle: {e}")
                return None

    def close(self):
        with self.i2c_lock:
            """关闭 I2C 连接"""
            self.bus.close()