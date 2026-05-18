import os
import sys
import threading
import time

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    ServoController,
)

# I2C config for ADS1115
ADS_ADDR = 0x48
ADS_GAIN = 1
ADS_RATE = 128

# Axis configuration:
# pot_ch: ADS1115 channel index (0-3)
# v_min/v_max: calibrated voltage range for the axis
# a_min/a_max: calibrated angle range for the axis
# servo_ch: PCA9685 channel for this axis
# s_min/s_max: servo command range (0-180)
AXES = {
    "boom": {
        "pot_ch": 0,
        "v_min": 0.26,
        "v_max": 2.53,
        "a_min": 0.0,
        "a_max": 120.0,
        "servo_ch": CHANNEL_2,
        "s_min": 0.0,
        "s_max": 180.0,
        "kp": 1.0,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
    },
    "steer": {
        "pot_ch": 2,
        "v_min": 0.40,
        "v_max": 2.60,
        "a_min": -90.0,
        "a_max": 90.0,
        "servo_ch": CHANNEL_4,
        "s_min": 0.0,
        "s_max": 180.0,
        "kp": 1.0,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
    },
    "bucket": {
        "pot_ch": 3,
        "v_min": 0.35,
        "v_max": 2.50,
        "a_min": 0.0,
        "a_max": 150.0,
        "servo_ch": CHANNEL_1,
        "s_min": 0.0,
        "s_max": 180.0,
        "kp": 1.0,
        "valve_neutral": 90.0,
        "valve_open_pos": 105.0,
        "valve_open_neg": 85.0,
    },
}

ANGLE_TOL = 2.0
SLOW_TOL = 10.0
LOOP_HZ = 10
PUMP_MAX_ANGLE = 135.0
PUMP_MIN_ANGLE = 45.0
PUMP_NEUTRAL = 90.0
PUMP_KP = 1.0

HYDRAULIC_AXES = {"boom", "bucket", "steer"}


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def voltage_to_angle(v, cfg):
    v = clamp(v, cfg["v_min"], cfg["v_max"])
    return (v - cfg["v_min"]) / (cfg["v_max"] - cfg["v_min"]) * (cfg["a_max"] - cfg["a_min"]) + cfg["a_min"]


def angle_to_servo_cmd(angle, cfg):
    angle = clamp(angle, cfg["a_min"], cfg["a_max"])
    return (angle - cfg["a_min"]) / (cfg["a_max"] - cfg["a_min"]) * (cfg["s_max"] - cfg["s_min"]) + cfg["s_min"]


def parse_targets(argv):
    targets = {"boom": 50.0, "steer": 0, "bucket": 120.0}
    for item in argv[1:]:
        if "=" not in item:
            continue
        key, val = item.split("=", 1)
        key = key.strip().lower()
        if key in targets:
            try:
                targets[key] = float(val)
            except ValueError:
                pass
    return targets


def parse_target_line(line, targets):
    parts = line.strip().split()
    for item in parts:
        if "=" not in item:
            continue
        key, val = item.split("=", 1)
        key = key.strip().lower()
        if key in targets:
            try:
                targets[key] = float(val)
            except ValueError:
                pass


def main():
    targets = parse_targets(sys.argv)
    controller = ServoController()

    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c, address=ADS_ADDR)
    ads.gain = ADS_GAIN
    ads.data_rate = ADS_RATE

    channels = {name: AnalogIn(ads, cfg["pot_ch"]) for name, cfg in AXES.items()}

    for name, cfg in AXES.items():
            current_angle = voltage_to_angle(channels[name].voltage, cfg)

    period = 1.0 / LOOP_HZ
    print("Targets:", targets)
    print("Input format: boom=50  steer=0 bucket=120 (type q to quit)")

    lock = threading.Lock()
    stop_event = threading.Event()

    def input_loop():
        while not stop_event.is_set():
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                stop_event.set()
                break
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                stop_event.set()
                break
            with lock:
                parse_target_line(line, targets)
                print("Targets updated:", targets)

    thread = threading.Thread(target=input_loop, daemon=True)
    thread.start()

    while not stop_event.is_set():
        current = {}
        for name, cfg in AXES.items():
            current[name] = voltage_to_angle(channels[name].voltage, cfg)

        with lock:
            current_targets = dict(targets)

        pump_required = False
        pump_angle = PUMP_NEUTRAL

        for name, cfg in AXES.items():
            error = current_targets[name] - current[name]

            if name in HYDRAULIC_AXES:
                if abs(error) <= ANGLE_TOL:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                else:
                    valve_angle = cfg["valve_open_pos"] if error > 0 else cfg["valve_open_neg"]
                    controller.set_servo_angle(cfg["servo_ch"], valve_angle)
                    pump_required = True
                    delta = clamp(PUMP_KP * abs(error), 0.0, PUMP_NEUTRAL - PUMP_MIN_ANGLE)
                    if abs(error) <= SLOW_TOL:
                        delta = min(delta, (PUMP_NEUTRAL - PUMP_MIN_ANGLE) * 0.5)
                    if error > 0:
                        pump_angle = max(pump_angle, PUMP_NEUTRAL + delta)
                    else:
                        pump_angle = min(pump_angle, PUMP_NEUTRAL - delta)

        if pump_required:
            controller.set_servo_angle(CHANNEL_5, pump_angle)
        else:
            controller.set_servo_angle(CHANNEL_5, PUMP_NEUTRAL)

        print(
            f"boom={current['boom']:.1f}  "
            f"steer={current['steer']:.1f} bucket={current['bucket']:.1f} "
            f"pump={pump_angle:.1f}"
        )
        time.sleep(period)


if __name__ == "__main__":
    main()
