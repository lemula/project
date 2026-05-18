import os
import sys
import threading
import time
import tkinter as tk

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from servo_control import (
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_4,
    CHANNEL_5,
    ServoController,
    logger,
)
from dwj_read import (
    ThreeAxisPotReader,
    STEER_MIN_ANGLE,
    STEER_MAX_ANGLE,
    BOOM_MIN_ANGLE,
    BOOM_MAX_ANGLE,
    BUCKET_MIN_ANGLE,
    BUCKET_MAX_ANGLE,
)

AXES = {
    "steer": {
        "pot_key": "steer_angle",
        "a_min": float(STEER_MIN_ANGLE),
        "a_max": float(STEER_MAX_ANGLE),
        "servo_ch": CHANNEL_4,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
        "kp": 2,
        "ki": 1,
        "kd": 0.1,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    },
    "boom": {
        "pot_key": "boom_angle",
        "a_min": float(BOOM_MIN_ANGLE),
        "a_max": float(BOOM_MAX_ANGLE),
        "servo_ch": CHANNEL_2,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
        "kp": 2,
        "ki": 1,
        "kd": 0.1,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    },
    "bucket": {
        "pot_key": "bucket_angle",
        "a_min": float(BUCKET_MIN_ANGLE),
        "a_max": float(BUCKET_MAX_ANGLE),
        "servo_ch": CHANNEL_1,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
        "kp": 2,
        "ki": 1,
        "kd": 0.1,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": True,
    },
}

ANGLE_TOL = 2.0
SLOW_TOL = 10.0
LOOP_HZ = 10

PUMP_CHANNEL = CHANNEL_5
PUMP_NEUTRAL = 90.0
PUMP_MIN_ANGLE = 90.0
PUMP_MAX_ANGLE = 105.0
PUMP_KP = 1.0


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


def main():
    controller = ServoController()
    reader = ThreeAxisPotReader()

    data = reader.read()
    current = {name: data.get(cfg["pot_key"]) for name, cfg in AXES.items()}
    targets = dict(current)

    lock = threading.Lock()
    stop_event = threading.Event()
    period = 1.0 / LOOP_HZ
    targets_active = False
    single_shot = False

    pid_by_axis = {
        name: PIDController(cfg["kp"], cfg["ki"], cfg["kd"], cfg["i_min"], cfg["i_max"])
        for name, cfg in AXES.items()
    }

    ui_alive = True

    def set_target_from_ui():
        nonlocal targets_active
        nonlocal single_shot
        try:
            tgt = float(target_var.get())
        except ValueError:
            status_var.set("Invalid input")
            return
        axis = axis_var.get()
        with lock:
            targets[axis] = tgt
            targets_active = True
            single_shot = True
        status_var.set("Target updated")

    def on_close():
        nonlocal ui_alive
        ui_alive = False
        stop_event.set()
        try:
            for cfg in AXES.values():
                controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
            controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
        except Exception as e:
            logger.error(f"Failed to reset valves: {e}")
        root.destroy()

    root = tk.Tk()
    root.title("Angleloader Single Axis Test")
    root.protocol("WM_DELETE_WINDOW", on_close)

    axis_var = tk.StringVar(value="steer")
    target_var = tk.StringVar(value=str(targets["steer"]))
    status_var = tk.StringVar(value="Ready")

    tk.Label(root, text="Axis").grid(row=0, column=0, padx=8, pady=6, sticky="e")
    tk.OptionMenu(root, axis_var, *AXES.keys()).grid(row=0, column=1, padx=8, pady=6, sticky="we")

    tk.Label(root, text="Target angle").grid(row=1, column=0, padx=8, pady=6, sticky="e")
    tk.Entry(root, textvariable=target_var, width=10).grid(row=1, column=1, padx=8, pady=6)

    tk.Button(root, text="Update target", command=set_target_from_ui).grid(
        row=2, column=0, columnspan=2, padx=8, pady=8
    )
    tk.Label(root, textvariable=status_var).grid(row=3, column=0, columnspan=2, padx=8, pady=6)

    last_time = time.time()

    def safe_set_status(text):
        if not ui_alive:
            return
        try:
            if not root.winfo_exists():
                return
            root.after(0, status_var.set, text)
        except RuntimeError:
            pass

    def on_axis_change(*_):
        nonlocal targets_active
        nonlocal single_shot
        axis = axis_var.get()
        current_val = current.get(axis)
        if current_val is not None:
            target_var.set(str(current_val))
        targets_active = False
        single_shot = False
        pid_by_axis[axis].reset()

    axis_var.trace_add("write", on_axis_change)

    def control_loop():
        nonlocal targets_active
        nonlocal single_shot
        nonlocal last_time
        while not stop_event.is_set():
            now = time.time()
            dt = now - last_time
            last_time = now

            data = reader.read()
            current_angles = {name: data.get(cfg["pot_key"]) for name, cfg in AXES.items()}
            with lock:
                current_targets = dict(targets)
                active = targets_active
                axis = axis_var.get()

            if not active:
                for cfg in AXES.values():
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
                safe_set_status("Waiting for target")
                for pid in pid_by_axis.values():
                    pid.reset()
                time.sleep(period)
                continue

            pump_required = False
            pump_angle = PUMP_NEUTRAL
            within_tol = False
            axis_cfg = AXES[axis]
            cur = current_angles.get(axis)
            tgt = current_targets.get(axis)

            for name, cfg in AXES.items():
                if name != axis:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])

            if cur is None or tgt is None:
                controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_neutral"])
            else:
                tgt = clamp_range(tgt, axis_cfg["a_min"], axis_cfg["a_max"])
                error = tgt - cur
                if axis_cfg.get("invert"):
                    error = -error
                if abs(error) <= ANGLE_TOL:
                    within_tol = True
                    controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_neutral"])
                else:
                    output = pid_by_axis[axis].update(error, dt)
                    if output > 0:
                        controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_open_pos"])
                    else:
                        controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_open_neg"])

                    pump_required = True
                    delta = clamp(PUMP_KP * abs(output), 0.0, PUMP_MAX_ANGLE - PUMP_MIN_ANGLE)
                    if abs(error) <= SLOW_TOL:
                        delta = min(delta, (PUMP_MAX_ANGLE - PUMP_MIN_ANGLE) * 0.5)
                    pump_angle = max(
                        pump_angle,
                        clamp(PUMP_NEUTRAL + delta, PUMP_MIN_ANGLE, PUMP_MAX_ANGLE),
                    )

            if pump_required:
                controller.set_servo_angle(PUMP_CHANNEL, pump_angle)
            else:
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)

            safe_set_status(
                "{} cur={:.1f} tgt={:.1f} pump={:.1f}".format(
                    axis,
                    cur if cur is not None else 0.0,
                    tgt if tgt is not None else 0.0,
                    pump_angle,
                )
            )

            if single_shot and within_tol:
                with lock:
                    targets_active = False
                    single_shot = False
            time.sleep(period)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    root.mainloop()


if __name__ == "__main__":
    main()
