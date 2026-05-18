import threading
import time

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from servo_control import CHANNEL_4, CHANNEL_5, ServoController

# I2C config for ADS1115
ADS_ADDR = 0x48
ADS_GAIN = 1
ADS_RATE = 128

# Steering axis calibration
STEER_CFG = {
    "pot_ch": 0,
    "v_min": 0.724,
    "v_max": 1.865,
    "a_min": 40,
    "a_max": -40,
    "valve_neutral": 90.0,
    "valve_open_pos": 120.0,
    "valve_open_neg": 60.0,
}

ANGLE_TOL = 2.0
SLOW_TOL = 10.0
LOOP_HZ = 10
PUMP_MAX_ANGLE = 135.0
PUMP_MIN_ANGLE = 45.0
PUMP_NEUTRAL = 90.0
PUMP_KP = 1.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def voltage_to_angle(v, cfg):
    v = clamp(v, cfg["v_min"], cfg["v_max"])
    return (v - cfg["v_min"]) / (cfg["v_max"] - cfg["v_min"]) * (cfg["a_max"] - cfg["a_min"]) + cfg["a_min"]


def main():
    target = {"steer": 0.0}
    controller = ServoController()

    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c, address=ADS_ADDR)
    ads.gain = ADS_GAIN
    ads.data_rate = ADS_RATE
    chan = AnalogIn(ads, STEER_CFG["pot_ch"])

    period = 1.0 / LOOP_HZ
    print("Input format: steer=0 or just 0 (type q to quit)")

    try:
        line = input("Initial target (deg): ").strip()
        if line:
            if line.lower() not in ("q", "quit", "exit"):
                value = float(line.split("=", 1)[-1])
                target["steer"] = value
    except (EOFError, KeyboardInterrupt, ValueError):
        pass

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
            try:
                if "=" in line:
                    key, val = line.split("=", 1)
                    if key.strip().lower() != "steer":
                        continue
                    value = float(val)
                else:
                    value = float(line)
            except ValueError:
                continue
            with lock:
                target["steer"] = value
                print("Target updated:", target)

    thread = threading.Thread(target=input_loop, daemon=True)
    thread.start()

    while not stop_event.is_set():
        current = voltage_to_angle(chan.voltage, STEER_CFG)
        with lock:
            target_angle = target["steer"]

        error = target_angle - current
        if abs(error) <= ANGLE_TOL:
            controller.set_servo_angle(CHANNEL_4, STEER_CFG["valve_neutral"])
            controller.set_servo_angle(CHANNEL_5, PUMP_NEUTRAL)
            pump_angle = PUMP_NEUTRAL
        else:
            valve_angle = STEER_CFG["valve_open_pos"] if error > 0 else STEER_CFG["valve_open_neg"]
            controller.set_servo_angle(CHANNEL_4, valve_angle)
            delta = clamp(PUMP_KP * abs(error), 0.0, PUMP_NEUTRAL - PUMP_MIN_ANGLE)
            if abs(error) <= SLOW_TOL:
                delta = min(delta, (PUMP_NEUTRAL - PUMP_MIN_ANGLE) * 0.5)
            if error > 0:
                pump_angle = clamp(PUMP_NEUTRAL + delta, PUMP_NEUTRAL, PUMP_MAX_ANGLE)
            else:
                pump_angle = clamp(PUMP_NEUTRAL + delta, PUMP_MIN_ANGLE, PUMP_NEUTRAL)
            controller.set_servo_angle(CHANNEL_5, pump_angle)

        print(f"steer={current:.1f} target={target_angle:.1f} pump={pump_angle:.1f}")
        time.sleep(period)


if __name__ == "__main__":
    main()
