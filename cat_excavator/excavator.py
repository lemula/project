import socket
import json
import time
import re
import logging
from datetime import datetime
from smbus import SMBus

# PCA9685寄存器地址与通道定义（保持不变）
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09

CHANNEL_1 = 0  # 挖斗舵机  100
CHANNEL_2 = 1  # 大臂舵机  90
CHANNEL_3 = 2  # 小臂舵机  100-160
CHANNEL_4 = 3  # 回转电机
CHANNEL_5 = 4  # 液压泵电机
CHANNEL_6 = 5  # 灯组
CHANNEL_7 = 6  # 左直行电机
CHANNEL_8 = 7  # 右直行电机

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class PCA9685:
    """PCA9685 PWM控制器驱动（保持不变）"""

    def __init__(self, bus_num=7, address=0x40, frequency=50):
        self.bus = SMBus(bus_num)
        self.address = address
        self.frequency = frequency
        self.pwm_range = 4096  # 12位PWM分辨率
        self._reset()
        self._set_pwm_frequency(frequency)

    def _reset(self):
        self.bus.write_byte_data(self.address, PCA9685_MODE1, 0x00)
        self.bus.write_byte_data(self.address, PCA9685_MODE2, 0x04)
        time.sleep(0.005)

    def _set_pwm_frequency(self, frequency):
        prescale_value = 25000000.0 / self.pwm_range / float(frequency) - 1.0
        # prescale = int(round(prescale_value))
        prescale = 128
        old_mode = self.bus.read_byte_data(self.address, PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, PCA9685_MODE1, new_mode)
        self.bus.write_byte_data(self.address, PCA9685_PRESCALE, prescale)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, PCA9685_LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, PCA9685_LED0_ON_H + 4 * channel, on >> 8)
        self.bus.write_byte_data(self.address, PCA9685_LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, PCA9685_LED0_OFF_H + 4 * channel, off >> 8)


class ServoController:
    """舵机和电调控制器（保持不变）"""

    def __init__(self):
        self.pca = PCA9685(bus_num=7, address=0x40, frequency=50)
        self.servo_min = 100
        self.servo_max = 500
        self.esc_min = 100
        self.esc_mid = 300
        self.esc_max = 500
        self._initialize_escs()

    def _initialize_escs(self):
        logger.info("开始初始化电调...")

        # 舵机初始化到中位
        self.set_servo_angle(CHANNEL_1, 90)
        self.set_servo_angle(CHANNEL_2, 90)
        self.set_servo_angle(CHANNEL_3, 90)
        self.set_servo_angle(CHANNEL_5, 90)


        logger.info("电调和舵机初始化完成")

    def angle_to_pulse(self, angle):
        angle = max(0, min(180, angle))
        return int(self.servo_min + (self.servo_max - self.servo_min) * angle / 180)

    def speed_to_pulse(self, speed):
        speed = max(-100, min(100, speed))
        if speed >= 0:
            return int(self.esc_mid + (self.esc_max - self.esc_mid) * speed / 100)
        else:
            return int(self.esc_mid + (self.esc_mid - self.esc_min) * speed / 100)

    def set_servo_angle(self, channel, angle):
        pulse = self.angle_to_pulse(angle)
        self.pca.set_pwm(channel, 0, pulse)
        logger.debug(f"舵机通道 {channel} 角度设置为 {angle}° (脉冲: {pulse})")

    def set_motor_speed(self, channel, speed):
        pulse = self.speed_to_pulse(speed)
        self.pca.set_pwm(channel, 0, pulse)
        logger.debug(f"电机通道 {channel} 速度设置为 {speed}% (脉冲: {pulse})")


class JoystickReceiver:
    def __init__(self, listen_ip='192.168.2.62', listen_port=8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_address = (listen_ip, listen_port)
        self.latest_data = None  # 存储最新接收的完整数据
        self.last_receive_time = 0
        self.last_print_time = 0
        self.print_interval = 1.0
        self.connected = False
        self.controller = ServoController()

        # 编译正则表达式（匹配发送端的字符串格式）
        # 匹配 X1: 0.5 这种格式，支持X/Y/Z后跟数字
        self.axis_pattern = re.compile(r'([XYZW])(\d+):\s*([-\d.]+)')

    def start(self):
        try:
            self.sock.bind(self.listen_address)
            logger.info(f"开始在 {self.listen_address} 监听数据...")
            logger.info("按Ctrl+C停止程序")

            while True:
                data, addr = self.sock.recvfrom(1024)
                self.last_receive_time = time.time()

                try:
                    json_data = json.loads(data.decode('utf-8'))

                    # 处理握手
                    if json_data.get('type') == 'handshake':
                        self.connected = True
                        logger.info(f"收到来自 {addr} 的握手请求")
                        response = json.dumps({"status": "ready"})
                        self.sock.sendto(response.encode('utf-8'), addr)
                        logger.info(f"已与 {addr} 建立连接")
                    else:
                        self.latest_data = json_data
                        self.connected = True
                        self.process_data()

                        # 控制打印频率
                        current_time = time.time()
                        if current_time - self.last_print_time >= self.print_interval:
                            self.print_data()
                            self.last_print_time = current_time

                except json.JSONDecodeError:
                    logger.error(f"无效的JSON数据: {data.decode('utf-8')}")
                except Exception as e:
                    logger.error(f"处理数据出错: {e}")

        except KeyboardInterrupt:
            logger.info("\n用户中断,停止接收")
        finally:
            self.cleanup()

    def parse_joystick_string(self, data_str):
        """解析发送端的格式化字符串，提取控制参数"""
        params = {}
        if not data_str:
            return params

        # 查找所有轴数据（X1, Y1, Z1, X2等）
        matches = self.axis_pattern.findall(data_str)
        for axis, num, value in matches:
            # 组合成参数名，如 X1, Z2
            param_name = f"{axis}{num}"
            try:
                # 转换为浮点数并存储
                params[param_name] = float(value)
            except ValueError:
                logger.warning(f"无效的数值格式: {param_name}:{value}")

        return params

    def process_data(self):
        """处理数据并控制设备"""
        if not self.latest_data:
            return

        # 提取字符串格式数据
        data_str = self.latest_data.get("formatted_data", "")
        if not data_str:
            logger.warning("未找到有效数据字符串")
            return

        # 解析参数
        params = self.parse_joystick_string(data_str)
        if not params:
            logger.warning("未解析到任何控制参数")
            return

        # 执行控制逻辑（带异常处理）

        CHANNEL5_angle = -10;
        try:
            # 1. 回转控制 (X1)
            if 'X1' in params:
                if abs(params['X1']) > 0.15 or params['X1'] == 0:
                    self.controller.set_motor_speed(CHANNEL_4, params['X1'] * 20+5)
        except Exception as e:
            logger.error(f"转向控制失败: {e}")

        try:
            # 2. 小臂控制 (Y1)
            if 'Y1' in params:
                if abs(params['Y1']) > 0.15 or params['Y1'] == 0:
                    self.controller.set_servo_angle(CHANNEL_3, (params['Y1']) * 45 + 90)
                    y1_angle = -abs(params['Y1']) * 20
                    CHANNEL5_angle = min(CHANNEL5_angle, y1_angle)
        except Exception as e:
            logger.error(f"小臂控制失败: {e}")

        try:
            # 3. 挖斗控制 (X2)
            if 'X2' in params:
                if abs(params['X2']) > 0.15 or params['X2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_1, (params['X2']) * 45 + 90)
                    x2_angle = -abs(params['X2']) * 20
                    CHANNEL5_angle = min(CHANNEL5_angle, x2_angle)
        except Exception as e:
            logger.error(f"翻斗控制失败: {e}")

        try:
            # 4. 大臂控制 (Y2)
            if 'Y2' in params:
                if abs(params['Y2']) > 0.15 or params['Y2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_2, (params['Y2']) * 45 + 90)
                    y2_angle = -abs(params['Y2']) * 20
                    CHANNEL5_angle = min(CHANNEL5_angle, y2_angle)
        except Exception as e:
            logger.error(f"大臂控制失败: {e}")

        try:
            self.controller.set_motor_speed(CHANNEL_5, CHANNEL5_angle)
        except Exception as e:
            logger.error(f"CHANNEL_7 控制失败: {e}")

        try:
            # 5. 左直行电机控制 (Z1)
            if 'Z1' in params:
                if abs(params['Z1']) > 0.15 or params['Z1'] == 0:
                    self.controller.set_motor_speed(CHANNEL_7, params['Z1'] * 20+5 )

        except Exception as e:
            logger.error(f"左直行电机控制失败: {e}")

        try:
            # 6. 右直行电机控制 (Z2)
            if 'Z2' in params:
                if abs(params['Z2']) > 0.15 or params['Z2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_8, params['Z2'] * 20+5 )
        except Exception as e:
            logger.error(f"右直行电机控制失败: {e}")

    def print_data(self):
        """打印接收到的数据及解析结果"""
        if not self.latest_data:
            return

        try:
            # 提取时间戳
            timestamp = self.latest_data.get('timestamp')
            time_str = "无时间戳"
            if timestamp:
                time_str = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')

            # 打印原始字符串
            print(f"\n[{time_str}] 收到数据:")
            print("-" * 50)
            print(self.latest_data.get("formatted_data", "无数据"))
            print("-" * 50)

            '''
            # 打印解析后的参数
            params = self.parse_joystick_string(self.latest_data.get("formatted_data", ""))
            if params:
                print("解析的控制参数:")
                for key, value in params.items():
                    print(f"  {key}: {value}")
            print("=" * 50)
            '''
        except Exception as e:
            logger.error(f"打印数据出错: {e}")

    def cleanup(self):
        """清理资源并复位设备"""
        logger.info("清理资源并复位设备...")
        try:
            # 复位所有设备
            self.controller.set_servo_angle(CHANNEL_1, 90)
            self.controller.set_servo_angle(CHANNEL_2, 90)
            self.controller.set_servo_angle(CHANNEL_3, 90)
            
            self.controller.set_motor_speed(CHANNEL_5, 0)

            self.controller.set_motor_speed(CHANNEL_4, 5)
            self.controller.set_motor_speed(CHANNEL_7, 5)
            self.controller.set_motor_speed(CHANNEL_8, 5)
        except Exception as e:
            logger.error(f"设备复位失败: {e}")

        self.sock.close()
        logger.info("程序已退出")


if __name__ == "__main__":
    LISTEN_IP = '192.168.2.62'
    LISTEN_PORT = 8888
    try:
        receiver = JoystickReceiver(LISTEN_IP, LISTEN_PORT)
        receiver.start()
    except Exception as e:
        logger.critical(f"启动失败: {e}")