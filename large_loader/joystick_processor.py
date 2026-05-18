import re
from datetime import datetime

from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    logger,
)


class JoystickProcessor:
    def __init__(self, controller):
        self.controller = controller
        self.axis_pattern = re.compile(r'([XYZ])(\d+):\s*([-\d.]+)')

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
                logger.warning(f"参数值无效 {param_name}:{value}")

        return params

    def process_data(self, latest_data):
        if not latest_data:
            return

        data_str = latest_data.get("formatted_data", "")
        if not data_str:
            logger.warning("缺少格式化数据")
            return

        params = self.parse_joystick_string(data_str)
        if not params:
            logger.warning("未解析到控制参数")
            return

        channel5_angle = 90
        try:
            # 1. 转向 (X1)
            if 'X1' in params:
                if abs(params['X1']) > 0.15 or params['X1'] == 0:
                    self.controller.set_servo_angle(CHANNEL_4, params['X1'] * 45 + 90)
                    x1_angle = abs(params['X1']) * 30 + 90
                    channel5_angle = max(channel5_angle, x1_angle)
        except Exception as e:
            logger.error(f"转向控制失败: {e}")

        try:
            # 2. 大臂 (Y2)
            if 'Y2' in params:
                if abs(params['Y2']) > 0.15 or params['Y2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_2, params['Y2'] * 45 + 90)
                    y2_angle = abs(params['Y2']) * 30 + 90
                    channel5_angle = max(channel5_angle, y2_angle)
        except Exception as e:
            logger.error(f"大臂控制失败: {e}")

        try:
            # 3. 翻斗 (X2)
            if 'X2' in params:
                if abs(params['X2']) > 0.15 or params['X2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_1, params['X2'] * 45 + 90)
                    x2_angle = abs(params['X2']) * 30 + 90
                    channel5_angle = max(channel5_angle, x2_angle)
        except Exception as e:
            logger.error(f"翻斗控制失败: {e}")

        try:
            if channel5_angle != 90:
                self.controller.set_servo_angle(CHANNEL_5, 105)
        except Exception as e:
            logger.error(f"CHANNEL_5 控制失败: {e}")

        try:
            # 4. 直行 (Y1)
            if 'Y1' in params:
                if abs(params['Y1']) > 0.15 or params['Y1'] == 0:
                    self.controller.set_motor_speed(CHANNEL_3, -params['Y1'] * 50)
        except Exception as e:
            logger.error(f"直行电机控制失败: {e}")

    def print_data(self, latest_data):
        if not latest_data:
            return

        try:
            timestamp = latest_data.get('timestamp')
            time_str = "无时间戳"
            if timestamp:
                time_str = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')

            params = self.parse_joystick_string(latest_data.get("formatted_data", ""))
            if params:
                print(f"[{time_str}] 参数:")
                for key, value in params.items():
                    print(f"  {key}: {value}")
                print("=" * 50)
        except Exception as e:
            logger.error(f"打印数据失败: {e}")

    def reset_devices(self):
        logger.info("正在复位设备...")
        try:
            self.controller.set_servo_angle(CHANNEL_1, 90)
            self.controller.set_servo_angle(CHANNEL_2, 90)
            self.controller.set_motor_speed(CHANNEL_3, 0)
            self.controller.set_servo_angle(CHANNEL_4, 90)
            self.controller.set_servo_angle(CHANNEL_5, 90)
            self.controller.set_servo_angle(CHANNEL_6, 180)
            self.controller.set_servo_angle(CHANNEL_7, 90)
            self.controller.set_servo_angle(CHANNEL_8, 90)
        except Exception as e:
            logger.error(f"设备复位失败: {e}")
