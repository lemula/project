import os
import threading

from angle_logger import AngleCsvLogger
from joystick_processor import JoystickProcessor
from dwj_read import ThreeAxisPotReader
from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_4,
    CHANNEL_5,
    ServoController,
    logger,
)
from udp_receiver import UdpJoystickReceiver


class TrackingController:
    def __init__(self, controller):
        self._controller = controller
        self._lock = threading.Lock()
        self._servo_angles = {}
        self._motor_speeds = {}

    def set_servo_angle(self, channel, angle):
        with self._lock:
            self._servo_angles[channel] = angle
        self._controller.set_servo_angle(channel, angle)

    def set_motor_speed(self, channel, speed):
        with self._lock:
            self._motor_speeds[channel] = speed
        self._controller.set_motor_speed(channel, speed)

    def get_servo_angle(self, channel, default=0.0):
        with self._lock:
            return self._servo_angles.get(channel, default)

    def get_motor_speed(self, channel, default=0.0):
        with self._lock:
            return self._motor_speeds.get(channel, default)


def main():
    listen_ip = '192.168.2.87'
    listen_port = 8888

    controller = ServoController()
    tracking = TrackingController(controller)
    processor = JoystickProcessor(tracking)

    try:
        pot_reader = ThreeAxisPotReader()
    except Exception as e:
        pot_reader = None
        logger.warning(f"电位计初始化失败: {e}")

    def on_data(data):
        processor.process_data(data)

    def on_print(data):
        processor.print_data(data)
        if not pot_reader:
            return
        try:
            pot_data = pot_reader.read()
            print(
                "pot: steer V={:.3f} angle={:.1f} | boom V={:.3f} angle={:.1f} | bucket V={:.3f} angle={:.1f}"
                .format(
                    pot_data["steer_v"],
                    pot_data["steer_angle"],

                    pot_data["boom_v"],
                    pot_data["boom_angle"],
                    pot_data["bucket_v"],
                    pot_data["bucket_angle"],
                )
            )
        except Exception as e:
            logger.error(f"电位计读取失败: {e}")

    receiver = UdpJoystickReceiver(
        listen_ip,
        listen_port,
        on_data=on_data,
        on_print=on_print,
        print_interval=1.0,
    )
    csv_logger = None
    if pot_reader:
        csv_logger = AngleCsvLogger(pot_reader, tracking, os.path.dirname(__file__), hz=10.0)

    try:
        if csv_logger:
            csv_logger.start()
        receiver.start()
    except Exception as e:
        logger.critical(f"启动失败: {e}")
    finally:
        if csv_logger:
            csv_logger.stop()
        processor.reset_devices()
        receiver.close()


if __name__ == "__main__":
    main()
