import threading
import time
import tkinter as tk
import sys

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_4,
    CHANNEL_5,
    ServoController,
)

# I2C config for ADS1115
ADS_ADDR = 0x48
ADS_GAIN = 1
ADS_RATE = 128

AXES = {
    "steer": {
        "pot_ch": 0,
        "v_min": 1.833,
        "v_max": 0.753,
        "a_min": -40.0,
        "a_max": 40.0,
        "servo_ch": CHANNEL_4,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
    },
    "boom": {
        "pot_ch": 1,
        "v_min": 0.189,
        "v_max": 0.996,
        "a_min": 0.0,
        "a_max": 90.0,
        "servo_ch": CHANNEL_2,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
    },
    "bucket": {
        "pot_ch": 2,
        "v_min": 3.171,
        "v_max": 2.230,
        "a_min": -90.0,
        "a_max": 35.0,
        "servo_ch": CHANNEL_1,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
    },
}

ANGLE_TOL = 2.0
SLOW_TOL = 10.0
LOOP_HZ = 10
PUMP_MAX_ANGLE = 105
PUMP_MIN_ANGLE = 90
PUMP_NEUTRAL = 90.0
PUMP_KP = 1.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def voltage_to_angle(v, cfg):
    v_low = min(cfg["v_min"], cfg["v_max"])
    v_high = max(cfg["v_min"], cfg["v_max"])
    v = clamp(v, v_low, v_high)
    return (v - cfg["v_min"]) / (cfg["v_max"] - cfg["v_min"]) * (cfg["a_max"] - cfg["a_min"]) + cfg["a_min"]

def parse_targets(argv, defaults):
    targets = dict(defaults)
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
    controller = ServoController()

    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c, address=ADS_ADDR)
    ads.gain = ADS_GAIN
    ads.data_rate = ADS_RATE
    channels = {name: AnalogIn(ads, cfg["pot_ch"]) for name, cfg in AXES.items()}
    current = {name: voltage_to_angle(channels[name].voltage, cfg) for name, cfg in AXES.items()}
    targets = parse_targets(sys.argv, current)

    period = 1.0 / LOOP_HZ

    lock = threading.Lock()
    stop_event = threading.Event()

    def set_targets_from_ui():
        try:
            steer_val = float(steer_var.get())
            boom_val = float(boom_var.get())
            bucket_val = float(bucket_var.get())
        except ValueError:
            status_var.set("Invalid input")
            return
        with lock:
            targets["steer"] = steer_val
            targets["boom"] = boom_val
            targets["bucket"] = bucket_val
        status_var.set("Targets updated")

    def on_close():
        stop_event.set()
        root.destroy()

    root = tk.Tk()
    root.title("Angle Setpoint Control")
    root.protocol("WM_DELETE_WINDOW", on_close)

    steer_var = tk.StringVar(value=str(targets["steer"]))
    boom_var = tk.StringVar(value=str(targets["boom"]))
    bucket_var = tk.StringVar(value=str(targets["bucket"]))
    status_var = tk.StringVar(value="Ready")

    tk.Label(root, text="Steer angle").grid(row=0, column=0, padx=8, pady=6, sticky="e")
    tk.Entry(root, textvariable=steer_var, width=10).grid(row=0, column=1, padx=8, pady=6)

    tk.Label(root, text="Boom angle").grid(row=1, column=0, padx=8, pady=6, sticky="e")
    tk.Entry(root, textvariable=boom_var, width=10).grid(row=1, column=1, padx=8, pady=6)

    tk.Label(root, text="Bucket angle").grid(row=2, column=0, padx=8, pady=6, sticky="e")
    tk.Entry(root, textvariable=bucket_var, width=10).grid(row=2, column=1, padx=8, pady=6)

    tk.Button(root, text="Set targets", command=set_targets_from_ui).grid(
        row=3, column=0, columnspan=2, padx=8, pady=8
    )
    tk.Label(root, textvariable=status_var).grid(row=4, column=0, columnspan=2, padx=8, pady=6)

    def control_loop():
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
                if abs(error) <= ANGLE_TOL:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                else:
                    valve_angle = cfg["valve_open_pos"] if error > 0 else cfg["valve_open_neg"]
                    controller.set_servo_angle(cfg["servo_ch"], valve_angle)
                    pump_required = True
                    delta = clamp(PUMP_KP * abs(error), 0.0, PUMP_MAX_ANGLE - PUMP_MIN_ANGLE)
                    if abs(error) <= SLOW_TOL:
                        delta = min(delta, (PUMP_MAX_ANGLE - PUMP_MIN_ANGLE) * 0.5)
                    pump_angle = max(
                        pump_angle,
                        clamp(PUMP_NEUTRAL + delta, PUMP_MIN_ANGLE, PUMP_MAX_ANGLE),
                    )

        if pump_required:
            controller.set_servo_angle(CHANNEL_5, pump_angle)
        else:
            for cfg in AXES.values():
                controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
            controller.set_servo_angle(CHANNEL_5, PUMP_NEUTRAL)

            status_var.set(
                "steer={:.1f} boom={:.1f} bucket={:.1f} pump={:.1f}".format(
                    current["steer"], current["boom"], current["bucket"], pump_angle
                )
            )
            time.sleep(period)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    root.mainloop()


if __name__ == "__main__":
    main()
