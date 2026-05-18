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

CHANNEL_1 = 0  # 直行电机  100
CHANNEL_2 = 1  # 转向电机  90
CHANNEL_3 = 2  # 回转电机  100-160
CHANNEL_4 = 3  # 小臂电机
CHANNEL_5 = 4  # 大臂电机
CHANNEL_6 = 5  # 翻斗电机
CHANNEL_7 = 6  # 液压泵电机
CHANNEL_8 = 7  # 差速锁舵机
CHANNEL_9 = 8  # qian支架舵机
CHANNEL_10 = 9  # hou支架舵机
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
        self.set_servo_angle(CHANNEL_2, 100)
        self.set_servo_angle(CHANNEL_4, 100)
        self.set_servo_angle(CHANNEL_5, 100)
        self.set_servo_angle(CHANNEL_6, 100)
        self.set_servo_angle(CHANNEL_7, 45)
        self.set_servo_angle(CHANNEL_8, 100)
        self.set_servo_angle(CHANNEL_9, 100)
        self.set_servo_angle(CHANNEL_10, 100)

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
