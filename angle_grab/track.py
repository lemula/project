import os
import sys
import threading
import time
import tkinter as tk
from datetime import datetime

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from servo_control import (
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    ServoController,
    logger,
)
from dwj_read import (
    PotentiometerReader,
    BIG_ARM_MIN_ANGLE,
    BIG_ARM_MAX_ANGLE,
    SMALL_ARM_MIN_ANGLE,
    SMALL_ARM_MAX_ANGLE,
    BUCKET_MIN_ANGLE,
    BUCKET_MAX_ANGLE,
)
import curve

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

AXES = {
    "big_arm": {
        "pot_key": "big_arm_angle",
        "a_min": float(BIG_ARM_MIN_ANGLE),
        "a_max": float(BIG_ARM_MAX_ANGLE),
        "servo_ch": CHANNEL_5,
        "valve_neutral": 100.0,
        "valve_open_pos": 145.0,
        "valve_open_neg": 55.0,
        "kp": 2.0,
        "ki": 1.0,
        "kd": 0.2,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    },
    "small_arm": {
        "pot_key": "small_arm_angle",
        "a_min": float(SMALL_ARM_MIN_ANGLE),
        "a_max": float(SMALL_ARM_MAX_ANGLE),
        "servo_ch": CHANNEL_4,
        "valve_neutral": 100.0,
        "valve_open_pos": 145.0,
        "valve_open_neg": 55.0,
        "kp": 2.0,
        "ki": 1.0,
        "kd": 0.0,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    },
    "bucket": {
        "pot_key": "bucket_angle",
        "a_min": float(BUCKET_MIN_ANGLE),
        "a_max": float(BUCKET_MAX_ANGLE),
        "servo_ch": CHANNEL_6,
        "valve_neutral": 100.0,
        "valve_open_pos": 55.0,
        "valve_open_neg": 145.0,
        "kp": 2.0,
        "ki": 1.0,
        "kd": 0.0,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": True,
    },
}

ANGLE_TOL = 2.0
SLOW_TOL = 10.0
LOOP_HZ = 10
TARGET_LEAD_S = 0.5

PUMP_CHANNEL = CHANNEL_7
PUMP_NEUTRAL = 45.0
PUMP_MIN_ANGLE = 45.0
PUMP_MAX_ANGLE = 90.0
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


def write_plot(times, target_series, actual_series, out_path):
    if plt is None:
        return False
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for ax, name in zip(axes, ("big_arm", "small_arm", "bucket")):
        ax.plot(times, target_series[name], label=f"{name} target")
        ax.plot(times, actual_series[name], label=f"{name} actual")
        ax.set_ylabel("angle (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend()
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return True


def main():
    controller = ServoController()
    reader = PotentiometerReader()

    data = reader.read()
    current = {name: data.get(cfg["pot_key"]) for name, cfg in AXES.items()}
    lock = threading.Lock()
    stop_event = threading.Event()
    period = 1.0 / LOOP_HZ
    curve_active = False
    start_time = [None]
    plot_ready = False

    pid_by_axis = {
        name: PIDController(cfg["kp"], cfg["ki"], cfg["kd"], cfg["i_min"], cfg["i_max"])
        for name, cfg in AXES.items()
    }

    ui_alive = True

    def start_curve():
        nonlocal curve_active
        nonlocal plot_ready
        with lock:
            curve.set_start_values(current)
            curve_active = True
            start_time[0] = time.time()
            plot_ready = False
            plot_samples["t"].clear()
            for name in ("big_arm", "small_arm", "bucket"):
                plot_samples["target"][name].clear()
                plot_samples["actual"][name].clear()
        status_var.set("Curve started")

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
    root.title("Angle Garb Curve Track")
    root.protocol("WM_DELETE_WINDOW", on_close)
    status_var = tk.StringVar(value="Ready")

    tk.Label(
        root,
        text="Profile: multi-axis curve from current pose, following curve.py targets",
    ).grid(row=0, column=0, columnspan=2, padx=8, pady=6)
    tk.Button(root, text="Start curve", command=start_curve).grid(
        row=1, column=0, columnspan=2, padx=8, pady=8
    )
    tk.Label(root, textvariable=status_var).grid(row=2, column=0, columnspan=2, padx=8, pady=6)

    last_time = time.time()
    plot_samples = {
        "t": [],
        "target": {"big_arm": [], "small_arm": [], "bucket": []},
        "actual": {"big_arm": [], "small_arm": [], "bucket": []},
    }

    def safe_set_status(text):
        if not ui_alive:
            return
        try:
            if not root.winfo_exists():
                return
            root.after(0, status_var.set, text)
        except RuntimeError:
            pass

    def control_loop():
        nonlocal curve_active
        nonlocal last_time
        nonlocal plot_ready
        while not stop_event.is_set():
            now = time.time()
            dt = now - last_time
            last_time = now

            data = reader.read()
            current_angles = {name: data.get(cfg["pot_key"]) for name, cfg in AXES.items()}
            current.update(current_angles)
            with lock:
                active = curve_active
                t0 = start_time[0]

            if not active or t0 is None:
                for cfg in AXES.values():
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
                safe_set_status(
                    "Waiting for targets: big={:.1f} small={:.1f} bucket={:.1f}".format(
                        current_angles["big_arm"],
                        current_angles["small_arm"],
                        current_angles["bucket"],
                    )
                )
                for pid in pid_by_axis.values():
                    pid.reset()
                time.sleep(period)
                continue

            t = max(curve.T0, min(now - t0, curve.T3))
            t_lead = max(curve.T0, min(t + TARGET_LEAD_S, curve.T3))
            current_targets = curve.get_targets(t_lead)
            plot_targets = curve.get_targets(t)
            pump_required = False
            pump_angle = None
            all_within = True

            for name, cfg in AXES.items():
                cur = current_angles.get(name)
                tgt = current_targets.get(name)
                if not curve.MULTI_AXIS and name != curve.ACTIVE_AXIS:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                    continue

                if cur is None or tgt is None:
                    all_within = False
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                    continue

                tgt = clamp_range(tgt, cfg["a_min"], cfg["a_max"])
                error = tgt - cur
                if cfg.get("invert"):
                    error = -error
                if abs(error) <= ANGLE_TOL:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                else:
                    all_within = False
                    output = pid_by_axis[name].update(error, dt)
                    if output > 0:
                        controller.set_servo_angle(cfg["servo_ch"], cfg["valve_open_pos"])
                    else:
                        controller.set_servo_angle(cfg["servo_ch"], cfg["valve_open_neg"])

                    pump_required = True
                    delta = clamp(PUMP_KP * abs(output), 0.0, PUMP_MAX_ANGLE - PUMP_MIN_ANGLE)
                    if abs(error) <= SLOW_TOL:
                        delta = min(delta, (PUMP_MAX_ANGLE - PUMP_MIN_ANGLE) * 0.5)
                    candidate = clamp(PUMP_NEUTRAL + delta, PUMP_MIN_ANGLE, PUMP_MAX_ANGLE)
                    if pump_angle is None:
                        pump_angle = candidate
                    else:
                        pump_angle = max(pump_angle, candidate)

            plot_samples["t"].append(t)
            for name in ("big_arm", "small_arm", "bucket"):
                plot_val = plot_targets.get(name)
                if plot_val is None:
                    plot_val = current_angles.get(name, 0.0)
                plot_samples["target"][name].append(plot_val)
                plot_samples["actual"][name].append(current_angles.get(name, 0.0))

            if pump_required and pump_angle is not None:
                controller.set_servo_angle(PUMP_CHANNEL, pump_angle)
            else:
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)

            safe_set_status(
                "t={:.1f} big={:.1f} small={:.1f} bucket={:.1f} pump={:.1f}".format(
                    t,
                    current_angles.get("big_arm", 0.0),
                    current_angles.get("small_arm", 0.0),
                    current_angles.get("bucket", 0.0),
                    pump_angle if pump_angle is not None else PUMP_NEUTRAL,
                )
            )

            if t >= curve.T3 and all_within:
                curve_active = False
                if not plot_ready:
                    plot_ready = True
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    out_path = os.path.join(BASE_DIR, f"curve_plot_{timestamp}.png")
                    ok = write_plot(
                        list(plot_samples["t"]),
                        plot_samples["target"],
                        plot_samples["actual"],
                        out_path,
                    )
                    if ok:
                        safe_set_status(f"Curve done, plot saved: {os.path.basename(out_path)}")
                    else:
                        safe_set_status("Curve done, plot failed (matplotlib missing)")
            time.sleep(period)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    root.mainloop()


if __name__ == "__main__":
    main()
