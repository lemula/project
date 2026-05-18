import re
from datetime import datetime

from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_7,
    CHANNEL_8,
    logger,
)


class JoystickProcessor:
    def __init__(self, controller):
        self.controller = controller
        self.axis_pattern = re.compile(r"([XYZW])(\d+):\s*([-\d.]+)")

    def parse_joystick_string(self, data_str):
        params = {}
        if not data_str:
            return params

        matches = self.axis_pattern.findall(data_str)
        for axis, num, value in matches:
            param_name = f"{axis}{num}"
            try:
                params[param_name] = float(value)
            except ValueError:
                logger.warning(f"invalid joystick value {param_name}:{value}")

        return params

    def process_data(self, latest_data):
        if not latest_data:
            return

        data_str = latest_data.get("formatted_data", "")
        if not data_str:
            logger.warning("no formatted joystick data found")
            return

        params = self.parse_joystick_string(data_str)
        if not params:
            logger.warning("no joystick control params parsed")
            return

        channel5_angle = -10

        try:
            if "X1" in params:
                if abs(params["X1"]) > 0.15 or params["X1"] == 0:
                    self.controller.set_motor_speed(CHANNEL_4, params["X1"] * 20 + 5)
        except Exception as e:
            logger.error(f"swing control failed: {e}")

        try:
            if "Y1" in params:
                if abs(params["Y1"]) > 0.15 or params["Y1"] == 0:
                    self.controller.set_servo_angle(CHANNEL_3, params["Y1"] * 45 + 90)
                    y1_angle = -abs(params["Y1"]) * 20
                    channel5_angle = min(channel5_angle, y1_angle)
        except Exception as e:
            logger.error(f"small arm control failed: {e}")

        try:
            if "X2" in params:
                if abs(params["X2"]) > 0.15 or params["X2"] == 0:
                    self.controller.set_servo_angle(CHANNEL_1, params["X2"] * 45 + 90)
                    x2_angle = -abs(params["X2"]) * 20
                    channel5_angle = min(channel5_angle, x2_angle)
        except Exception as e:
            logger.error(f"bucket control failed: {e}")

        try:
            if "Y2" in params:
                if abs(params["Y2"]) > 0.15 or params["Y2"] == 0:
                    self.controller.set_servo_angle(CHANNEL_2, params["Y2"] * 45 + 90)
                    y2_angle = -abs(params["Y2"]) * 20
                    channel5_angle = min(channel5_angle, y2_angle)
        except Exception as e:
            logger.error(f"big arm control failed: {e}")

        try:
            self.controller.set_motor_speed(CHANNEL_5, channel5_angle)
        except Exception as e:
            logger.error(f"hydraulic pump control failed: {e}")

        try:
            if "Z1" in params:
                if abs(params["Z1"]) > 0.15 or params["Z1"] == 0:
                    self.controller.set_motor_speed(CHANNEL_7, params["Z1"] * 20 + 5)
        except Exception as e:
            logger.error(f"left drive motor control failed: {e}")

        try:
            if "Z2" in params:
                if abs(params["Z2"]) > 0.15 or params["Z2"] == 0:
                    self.controller.set_servo_angle(CHANNEL_8, params["Z2"] * 20 + 5)
        except Exception as e:
            logger.error(f"right drive motor control failed: {e}")

    def print_data(self, latest_data):
        if not latest_data:
            return

        try:
            timestamp = latest_data.get("timestamp")
            time_str = "unknown time"
            if timestamp:
                time_str = datetime.fromtimestamp(timestamp).strftime("%H:%M:%S")

            print(f"\n[{time_str}] received data:")
            print("-" * 50)
            print(latest_data.get("formatted_data", "no data"))
            print("-" * 50)
        except Exception as e:
            logger.error(f"print data failed: {e}")

    def reset_devices(self):
        logger.info("resetting devices...")
        try:
            self.controller.set_servo_angle(CHANNEL_1, 90)
            self.controller.set_servo_angle(CHANNEL_2, 90)
            self.controller.set_servo_angle(CHANNEL_3, 90)

            self.controller.set_motor_speed(CHANNEL_5, 0)
            self.controller.set_motor_speed(CHANNEL_4, 5)
            self.controller.set_motor_speed(CHANNEL_7, 5)
            self.controller.set_motor_speed(CHANNEL_8, 5)
        except Exception as e:
            logger.error(f"device reset failed: {e}")

        logger.info("program exited")
