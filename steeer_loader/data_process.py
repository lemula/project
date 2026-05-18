import time

from dwj_read import ThreeAxisPotReader, STEER_MAX_ANGLE, STEER_MIN_ANGLE
from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    logger,
)


ANGLE_TOL = 4.0
SLOW_TOL = 15.0

PUMP_CHANNEL = CHANNEL_5
PUMP_NEUTRAL = 90.0
PUMP_MIN_ANGLE = 90.0
PUMP_MAX_ANGLE = 105.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def clamp_range(value, a_min, a_max):
    low = min(a_min, a_max)
    high = max(a_min, a_max)
    return clamp(value, low, high)


class PIDController:
    def __init__(self, kp, ki, kd, i_min, i_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_min = i_min
        self.i_max = i_max
        self._integral = 0.0
        self._prev_error = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def update(self, error, dt):
        if dt <= 0:
            return 0.0
        self._integral = clamp(self._integral + error * dt, self.i_min, self.i_max)
        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


AXES = {
    "steer": {
        "pot_key": "steer_angle",
        "a_min": float(STEER_MIN_ANGLE),
        "a_max": float(STEER_MAX_ANGLE),
        "servo_ch": CHANNEL_4,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
        "kp": 0.5,
        "ki": 0,
        "kd": 0.1,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    }
}


class DataProcessor:
    def __init__(self, controller):
        self.controller = controller
        self.pump_on = False
        self._pot_reader = ThreeAxisPotReader()
        steer_cfg = AXES["steer"]
        self._steer_pid = PIDController(
            steer_cfg["kp"],
            steer_cfg["ki"],
            steer_cfg["kd"],
            steer_cfg["i_min"],
            steer_cfg["i_max"],
        )
        self._steer_last_time = None

    @staticmethod
    def _clamp(value, v_min, v_max):
        return max(v_min, min(v_max, value))

    def _normalize_from_rest(self, value, v_rest, v_min):
        if value is None:
            return 0.0
        if v_rest == v_min:
            return 0.0
        norm = (v_rest - value) / (v_rest - v_min)
        return self._clamp(norm, 0.0, 1.0)

    def _steer_to_angle(self, steer):
        steer = 0.0 if steer is None else steer
        steer = self._clamp(steer, -1.0, 1.0)
        return steer * 40.0

    def _axis_to_angle(self, axis, gain=1.0):
        axis = 0.0 if axis is None else axis
        axis = self._clamp(axis, -1.0, 1.0)
        return 90.0 + axis * 45.0 * gain

    def angle_control(self, steer_input, boom_gain):
        steer_input = 0.0 if steer_input is None else steer_input
        steer_input = self._clamp(steer_input, -1.0, 1.0)
        boom_gain = 0.0 if boom_gain is None else boom_gain
        boom_gain = self._clamp(boom_gain, 0.0, 1.0)

        now = time.time()
        if self._steer_last_time is None:
            self._steer_last_time = now
            return
        dt = now - self._steer_last_time
        self._steer_last_time = now

        data = self._pot_reader.read()
        current_angle = data.get("steer_angle")

        t = (steer_input + 1.0) * 0.5
        target_angle = -(STEER_MIN_ANGLE + (STEER_MAX_ANGLE - STEER_MIN_ANGLE) * t)

        axis_cfg = AXES["steer"]
        if current_angle is None or target_angle is None:
            self.controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_neutral"])
            self.controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
            return

        target_angle = clamp_range(target_angle, axis_cfg["a_min"], axis_cfg["a_max"])
        error = target_angle - current_angle
        if axis_cfg.get("invert"):
            error = -error

        if abs(error) <= ANGLE_TOL:
            self.controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_neutral"])
            self.controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
            return

        output = self._steer_pid.update(error, dt)
        if output > 0:
            self.controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_open_pos"])
        else:
            self.controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_open_neg"])

        delta = clamp(abs(output) * boom_gain, 0.0, PUMP_MAX_ANGLE - PUMP_MIN_ANGLE)
        if abs(error) <= SLOW_TOL:
            delta = min(delta, (PUMP_MAX_ANGLE - PUMP_MIN_ANGLE) * 0.5)
        pump_angle = clamp(PUMP_NEUTRAL + delta, PUMP_MIN_ANGLE, PUMP_MAX_ANGLE)
        self.controller.set_servo_angle(PUMP_CHANNEL, pump_angle)

    def process(self, data):
        if not data:
            return

        wheel = data.get("wheel", {})
        tca = data.get("tca", {})
        stick = data.get("stick", {})

        steer = wheel.get("steer")
        forward = wheel.get("forward")
        reverse = wheel.get("reverse")

        stick_bucket = stick.get("axis0")
        stick_boom = stick.get("axis1")

        tca_axis0 = tca.get("axis0")
        tca_axis1 = tca.get("axis1")
        b07 = tca.get("b07")

        self.pump_on = bool(b07) if b07 is not None else False

        forward_norm = self._normalize_from_rest(forward, v_rest=1.0, v_min=-1.0)
        reverse_norm = self._normalize_from_rest(reverse, v_rest=1.0, v_min=-1.0)
        drive_speed = (forward_norm - reverse_norm) * 50.0

        pump_gain = self._normalize_from_rest(tca_axis0, v_rest=0.5, v_min=-1.0)*1.5
        boom_gain = self._normalize_from_rest(tca_axis1, v_rest=0.5, v_min=-1.0)*1.5

        bucket_angle = self._axis_to_angle(stick_bucket)
        boom_angle = self._axis_to_angle(stick_boom)

        if not self.pump_on:
            pump_angle = 90.0
        else:

            pump_angle = 90.0 + pump_gain * 15.0

        try:
            self.angle_control(steer, boom_gain)
        except Exception as exc:
            logger.error(f"Steer PID control failed: {exc}")

        try:
            self.controller.set_motor_speed(CHANNEL_3, drive_speed)
        except Exception as exc:
            logger.error(f"Drive control failed: {exc}")

        try:
            self.controller.set_servo_angle(CHANNEL_1, bucket_angle)
        except Exception as exc:
            logger.error(f"Bucket control failed: {exc}")

        try:
            self.controller.set_servo_angle(CHANNEL_2, boom_angle)
        except Exception as exc:
            logger.error(f"Boom control failed: {exc}")

        try:
            self.controller.set_servo_angle(CHANNEL_5, pump_angle)
        except Exception as exc:
            logger.error(f"Pump control failed: {exc}")

