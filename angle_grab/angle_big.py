import os
import sys
import threading
import time
import tkinter as tk

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LARGE_GARB_DIR = os.path.join(ROOT, "large_garb")
if LARGE_GARB_DIR not in sys.path:
    sys.path.insert(0, LARGE_GARB_DIR)

from servo_control import CHANNEL_5, CHANNEL_7, ServoController, logger
from dwj_read import PotentiometerReader, BIG_ARM_MIN_ANGLE, BIG_ARM_MAX_ANGLE

BIG_ARM_CFG = {
    "pot_key": "big_arm_angle",
    "a_min": float(BIG_ARM_MIN_ANGLE),
    "a_max": float(BIG_ARM_MAX_ANGLE),
    "servo_ch": CHANNEL_5,
    "valve_neutral": 100.0,
    "valve_open_pos": 145.0,
    "valve_open_neg": 55.0,
}

ANGLE_TOL = 2.0
SLOW_TOL = 10.0
LOOP_HZ = 10

PUMP_CHANNEL = CHANNEL_7
PUMP_NEUTRAL = 45.0
PUMP_MAX_ANGLE = 90
PUMP_KP = 2.0
PUMP_KI = 0.8
PUMP_KD = 0.2
PUMP_INT_LIMIT = 30.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def parse_targets(argv, default_value):
    target = default_value
    for item in argv[1:]:
        if "=" not in item:
            continue
        key, val = item.split("=", 1)
        key = key.strip().lower()
        if key not in ("big", "big_arm", "boom", "target"):
            continue
        try:
            target = float(val)
        except ValueError:
            pass
    return target


def main():
    controller = ServoController()
    reader = PotentiometerReader()
    data = reader.read()
    current = data.get(BIG_ARM_CFG["pot_key"])
    target = parse_targets(sys.argv, current)

    lock = threading.Lock()
    stop_event = threading.Event()
    period = 1.0 / LOOP_HZ
    prev_error = 0.0
    integral = 0.0

    def set_target_from_ui():
        try:
            val = float(target_var.get())
        except ValueError:
            status_var.set("Invalid input")
            return
        with lock:
            nonlocal_target[0] = val
        status_var.set("Target updated")

    def on_close():
        stop_event.set()
        try:
            controller.set_servo_angle(BIG_ARM_CFG["servo_ch"], BIG_ARM_CFG["valve_neutral"])
            controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
        except Exception as e:
            logger.error(f"Failed to reset valves: {e}")
        root.destroy()

    nonlocal_target = [target]
    root = tk.Tk()
    root.title("Big Arm Angle Control")
    root.protocol("WM_DELETE_WINDOW", on_close)

    target_var = tk.StringVar(value=str(target))
    status_var = tk.StringVar(value="Ready")

    tk.Label(root, text="Big Arm Target").grid(row=0, column=0, padx=8, pady=6, sticky="e")
    tk.Entry(root, textvariable=target_var, width=10).grid(row=0, column=1, padx=8, pady=6)

    tk.Button(root, text="Set Target", command=set_target_from_ui).grid(
        row=1, column=0, columnspan=2, padx=8, pady=8
    )
    tk.Label(root, textvariable=status_var).grid(row=2, column=0, columnspan=2, padx=8, pady=6)

    def control_loop():
        nonlocal prev_error, integral
        while not stop_event.is_set():
            data = reader.read()
            cur = data.get(BIG_ARM_CFG["pot_key"])

            with lock:
                tgt = nonlocal_target[0]

            pump_required = False
            pump_angle = PUMP_NEUTRAL

            if cur is not None:
                tgt = clamp(tgt, BIG_ARM_CFG["a_min"], BIG_ARM_CFG["a_max"])
                error = tgt - cur

                if abs(error) <= ANGLE_TOL:
                    controller.set_servo_angle(BIG_ARM_CFG["servo_ch"], BIG_ARM_CFG["valve_neutral"])
                else:
                    valve_angle = (
                        BIG_ARM_CFG["valve_open_pos"] if error > 0 else BIG_ARM_CFG["valve_open_neg"]
                    )
                    controller.set_servo_angle(BIG_ARM_CFG["servo_ch"], valve_angle)
                    pump_required = True
                    integral += error * period
                    integral = clamp(integral, -PUMP_INT_LIMIT, PUMP_INT_LIMIT)
                    derivative = (error - prev_error) / period
                    prev_error = error
                    pid = PUMP_KP * abs(error) + PUMP_KI * abs(integral) + PUMP_KD * abs(derivative)
                    delta = clamp(pid, 0.0, PUMP_MAX_ANGLE - PUMP_NEUTRAL)
                    if abs(error) <= SLOW_TOL:
                        delta = min(delta, (PUMP_MAX_ANGLE - PUMP_NEUTRAL) * 0.5)
                    pump_angle = max(pump_angle, PUMP_NEUTRAL + delta)

            if pump_required:
                controller.set_servo_angle(PUMP_CHANNEL, pump_angle)
            else:
                controller.set_servo_angle(BIG_ARM_CFG["servo_ch"], BIG_ARM_CFG["valve_neutral"])
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
                prev_error = 0.0
                integral = 0.0

            status_var.set("big={:.1f} target={:.1f} pump={:.1f}".format(cur, tgt, pump_angle))
            time.sleep(period)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    root.mainloop()


if __name__ == "__main__":
    main()

