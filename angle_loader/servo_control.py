import logging
import time
from smbus import SMBus

# PCA9685 register address and channel definitions
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09

CHANNEL_1 = 0  # bucket servo
CHANNEL_2 = 1  # boom servo
CHANNEL_3 = 2  # throttle motor
CHANNEL_4 = 3  # steering servo
CHANNEL_5 = 4  # hydraulic pump
CHANNEL_6 = 5  # zdhz
CHANNEL_7 = 6  # gearbox
CHANNEL_8 = 7  # differential lock

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class PCA9685:
    """PCA9685 PWM controller driver."""

    def __init__(self, bus_num=7, address=0x40, frequency=50):
        self.bus = SMBus(bus_num)
        self.address = address
        self.frequency = frequency
        self.pwm_range = 4096
        self._reset()
        self._set_pwm_frequency(frequency)

    def _reset(self):
        self.bus.write_byte_data(self.address, PCA9685_MODE1, 0x00)
        self.bus.write_byte_data(self.address, PCA9685_MODE2, 0x04)
        time.sleep(0.005)

    def _set_pwm_frequency(self, frequency):
        prescale_value = 25000000.0 / self.pwm_range / float(frequency) - 1.0
        _ = prescale_value
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
    """Servo and ESC controller."""

    def __init__(self):
        self.pca = PCA9685(bus_num=7, address=0x40, frequency=50)
        self.servo_min = 100
        self.servo_max = 500
        self.esc_min = 100
        self.esc_mid = 300
        self.esc_max = 500
        self._initialize_escs()

    def _initialize_escs(self):
        logger.info("Starting ESC init...")
        self.set_servo_angle(CHANNEL_1, 90)
        self.set_servo_angle(CHANNEL_2, 90)
        self.set_servo_angle(CHANNEL_4, 90)
        self.set_servo_angle(CHANNEL_5, 90)
        self.set_servo_angle(CHANNEL_6, 180)
        self.set_servo_angle(CHANNEL_7, 90)
        self.set_servo_angle(CHANNEL_8, 90)
        logger.info("ESC and servo init done")

    def angle_to_pulse(self, angle):
        angle = max(0, min(180, angle))
        return int(self.servo_min + (self.servo_max - self.servo_min) * angle / 180)

    def speed_to_pulse(self, speed):
        speed = max(-100, min(100, speed))
        if speed >= 0:
            return int(self.esc_mid + (self.esc_max - self.esc_mid) * speed / 100)
        return int(self.esc_mid + (self.esc_mid - self.esc_min) * speed / 100)

    def set_servo_angle(self, channel, angle):
        pulse = self.angle_to_pulse(angle)
        self.pca.set_pwm(channel, 0, pulse)
        logger.debug(f"Servo channel {channel} angle {angle} (pulse {pulse})")

    def set_motor_speed(self, channel, speed):
        pulse = self.speed_to_pulse(speed)
        self.pca.set_pwm(channel, 0, pulse)
        logger.debug(f"Motor channel {channel} speed {speed} (pulse {pulse})")
