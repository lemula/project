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
    CHANNEL_9,
    CHANNEL_10,
    logger,
)

class JoystickProcessor:
    def __init__(self, controller):
        self.controller = controller
        self.axis_pattern = re.compile(r'([XYZW])(\d+):\s*([-\d.]+)')

    def _gear_multiplier(self, value):
        if value is None:
            return 0.5
        try:
            value = float(value)
        except (TypeError, ValueError):
            return 0.5

        # Stepless: input range 0.5 -> -1 maps to multiplier 0.25 -> 1.0
        if value >= 0.5:
            return 0.5
        if value <= -1.0:
            return 1.5
        t = (0.5 - value) / 1.5
        return 0.5 + 1 * t

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
                logger.warning(f"参数值解析失败 {param_name}:{value}")

        return params

    def process_data(self, latest_data):

        if not latest_data:
            return

        """处理数据并控制设备"""
        data_str = latest_data.get("formatted_data", "")
        if not data_str:
            logger.warning("未找到有效数据字符串")
            return

        # 解析参数
        params = self.parse_joystick_string(data_str)
        if not params:
            logger.warning("未解析到任何控制参数")
            return

        # 执行控制逻辑（带异常处理）
        a3 = latest_data.get("A3")
        b3 = latest_data.get("B3")
        small_gain = self._gear_multiplier(a3)
        big_gain = self._gear_multiplier(b3)
        CHANNEL7_angle = 45
        try:
            # 1. 回转控制 (X1)
            if 'X1' in params:
                if abs(params['X1']) > 0.15 or params['X1'] == 0:
                    self.controller.set_motor_speed(CHANNEL_3, -params['X1'] * 50)
        except Exception as e:
            logger.error(f"回转控制异常: {e}")

        try:
             # 2. 小臂控制 (Y1)
            if 'Y1' in params:
                if abs(params['Y1']) > 0.15 or params['Y1'] == 0:
                    self.controller.set_servo_angle(CHANNEL_4, (-params['Y1']) * 45 * small_gain + 100)
                    y1_angle = abs(params['Y1']) * 45 * small_gain + 45
                    CHANNEL7_angle = max(CHANNEL7_angle, y1_angle)
        except Exception as e:
            logger.error(f"小臂控制异常: {e}")

        try:
            # 3. 翻斗控制 (X2)
            if 'X2' in params:
                if abs(params['X2']) > 0.15 or params['X2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_6, (params['X2']) * 45 + 100)
                    x2_angle = abs(params['X2']) * 30 + 45
                    CHANNEL7_angle = max(CHANNEL7_angle, x2_angle)
        except Exception as e:
            logger.error(f"翻斗控制异常: {e}")

        try:
            # 4. 大臂控制 (Y2)
            if 'Y2' in params:
                if abs(params['Y2']) > 0.15 or params['Y2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_5, (-params['Y2']) * 45 * big_gain + 100)
                    y2_angle = abs(params['Y2']) * 45 * big_gain + 45
                    CHANNEL7_angle = max(CHANNEL7_angle, y2_angle)
        except Exception as e:
            logger.error(f"大臂控制异常: {e}")

        try:
            # 5. 前支撑架 (W1)
            if 'W1' in params:
                if abs(params['W1']) > 0.15 or params['W1'] == 0:
                    self.controller.set_servo_angle(CHANNEL_9, (-params['W1']) * 45 + 100)
                    w1_angle = abs(params['W1']) * 45 + 45
                    CHANNEL7_angle = max(CHANNEL7_angle, w1_angle)
        except Exception as e:
            logger.error(f"前支撑控制异常: {e}")

        try:
            # 6. 后支撑架控制 (W2)
            if 'W2' in params:
                if abs(params['W2']) > 0.15 or params['W2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_10, (-params['W2']) * 45 + 100)
                    w2_angle = abs(params['W2']) * 45 + 45
                    CHANNEL7_angle = max(CHANNEL7_angle, w2_angle)
        except Exception as e:
            logger.error(f"后支撑控制异常: {e}")

        try:
            self.controller.set_servo_angle(CHANNEL_7, CHANNEL7_angle)
        except Exception as e:
            logger.error(f"CHANNEL_7 控制异常: {e}")

        try:
             # 5. 直行电机控制 (Z1)
            if 'Z1' in params:
                if abs(params['Z1']) > 0.15 or params['Z1'] == 0:
                    self.controller.set_motor_speed(CHANNEL_1, -params['Z1'] * 20 + 5)

        except Exception as e:
            logger.error(f"直行电机控制异常: {e}")

        try:
            # 6. 转向舵机控制 (Z2)
            if 'Z2' in params:
                if abs(params['Z2']) > 0.15 or params['Z2'] == 0:
                    self.controller.set_servo_angle(CHANNEL_2, params['Z2'] * 45 + 100)
        except Exception as e:
            logger.error(f"转向舵机控制异常: {e}")

    def print_data(self, latest_data):
        """打印接收到的数据及解析结果"""
        if not latest_data:
            return

        try:
            
            timestamp = latest_data.get('timestamp')
            time_str = "未知时间"
            if timestamp:
                time_str = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')

            # 打印原始字符串
            print(f"\n[{time_str}] 收到数据:")
            print("-" * 50)
            print(latest_data.get("formatted_data", "无数据"))
            print("-" * 50)

            '''
            # 打印解析后的参数
            params = self.parse_joystick_string(latest_data.get("formatted_data", ""))
            if params:
                print("解析参数:")
                for key, value in params.items():
                    print(f"  {key}: {value}")
            print("=" * 50)
            '''
        except Exception as e:
            logger.error(f"打印数据异常: {e}")

    def reset_devices(self):
        """清理资源并复位设备"""
        logger.info("清理资源并复位设备...")
        try:
            # 复位所有设备
            self.controller.set_servo_angle(CHANNEL_2, 100)
            self.controller.set_servo_angle(CHANNEL_4, 100)
            self.controller.set_servo_angle(CHANNEL_5, 100)
            self.controller.set_servo_angle(CHANNEL_6, 100)
            self.controller.set_servo_angle(CHANNEL_7, 45)
            self.controller.set_servo_angle(CHANNEL_8, 100)
            self.controller.set_servo_angle(CHANNEL_9, 100)
            self.controller.set_servo_angle(CHANNEL_10, 100)

            self.controller.set_motor_speed(CHANNEL_1, 0)
            self.controller.set_motor_speed(CHANNEL_3, 0)

        except Exception as e:
            logger.error(f"设备复位失败: {e}")

        logger.info("程序已退出")
