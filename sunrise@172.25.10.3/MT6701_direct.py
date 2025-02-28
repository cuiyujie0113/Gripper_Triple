import smbus2
import time

# MT6701 定义
MT6701_SLAVE_ADDR = 0x06  # I2C 地址
MT6701_REG_ANGLE_14b = 0x03  # 14 位角度寄存器
MT6701_TIMEOUT = 0.05  # 超时时间（秒）

class MT6701:
    def __init__(self, bus_num):
        self.bus = smbus2.SMBus(bus_num)

    def write_reg(self, reg, value):
        """写入单个寄存器"""
        try:
            self.bus.write_byte_data(MT6701_SLAVE_ADDR, reg, value)
            return True
        except Exception as e:
            print(f"Error writing to register {reg}: {e}")
            return False

    def read_reg(self, reg, length):
        """读取寄存器数据"""
        try:
            data = self.bus.read_i2c_block_data(MT6701_SLAVE_ADDR, reg, length)
            return data
        except Exception as e:
            print(f"Error reading from register {reg}: {e}")
            return None

    def get_angle(self):
        """获取 14 位角度值"""
        try:
            temp = self.read_reg(MT6701_REG_ANGLE_14b, 2)
            if temp is None:
                return None, None

            # 将高位和低位数据拼接成 14 位整数
            angle_int = (temp[0] << 6) | (temp[1] >> 2)
            angle = angle_int * 360 / 16384  # 转换为角度
            return angle_int, angle
        except Exception as e:
            print(f"Error getting angle: {e}")
            return None, None

    def close(self):
        """关闭 I2C 连接"""
        self.bus.close()
    
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


# 主程序示例
if __name__ == "__main__":
    # 初始化 I2C 5 总线
    mt6701 = MT6701(bus_num=5)

    try:
        print("Reading angle data from MT6701...")
        while True:
            angle_int, angle = mt6701.get_angle()
            if angle is not None:
                print(f"Raw Angle: {angle_int}, Angle (degrees): {angle:.2f}")
            else:
                print("Failed to read angle.")
            time.sleep(0.01)  # 每秒读取一次
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        mt6701.close()
