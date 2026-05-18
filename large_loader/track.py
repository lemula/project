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
import curve

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

AXES = {
    "steer": {
        "pot_key": "steer_angle",
        "a_min": float(STEER_MIN_ANGLE),
        "a_max": float(STEER_MAX_ANGLE),
        "servo_ch": CHANNEL_4,
        "valve_neutral": 90.0,
        "valve_open_pos": 120.0,
        "valve_open_neg": 60.0,
        "kp": 1,
        "ki": 0.1,
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
        "valve_neutral": 97.0,
        "valve_open_pos": 127.0,
        "valve_open_neg": 67.0,
        "kp": 1.5,
        "ki": 0,
        "kd": 0,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    },
    "bucket": {
        "pot_key": "bucket_angle",
        "a_min": float(BUCKET_MIN_ANGLE),
        "a_max": float(BUCKET_MAX_ANGLE),
        "servo_ch": CHANNEL_1,
        "valve_neutral": 97.0,
        "valve_open_pos": 67.0,
        "valve_open_neg": 127.0,
        "kp": 0.8,
        "ki": 0,
        "kd": 0,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
    },
}

ANGLE_TOL = 2.0
SLOW_TOL = 15.0
LOOP_HZ = 10

PUMP_CHANNEL = CHANNEL_5
PUMP_NEUTRAL = 90.0
PUMP_MIN_ANGLE = 90.0
PUMP_MAX_ANGLE = 110.0
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
    labels = ("steer", "boom", "bucket")
    for idx, name in enumerate(labels):
        ax = axes[idx]
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
    reader = ThreeAxisPotReader()

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
            curve_active = True
            start_time[0] = time.time()
            plot_ready = False
            plot_samples["t"].clear()
            for name in plot_samples["target"]:
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
    root.title("Angleloader Curve Track")
    root.protocol("WM_DELETE_WINDOW", on_close)
    status_var = tk.StringVar(value="Ready")

    tk.Label(root, text="Profile: 0-3s -> 0,0,0 | 3-8s -> -30,40,10 | 8-10s -> 0,0,0").grid(
        row=0, column=0, columnspan=2, padx=8, pady=6
    )
    tk.Button(root, text="Start curve", command=start_curve).grid(
        row=1, column=0, columnspan=2, padx=8, pady=8
    )
    tk.Label(root, textvariable=status_var).grid(row=2, column=0, columnspan=2, padx=8, pady=6)

    last_time = time.time()
    plot_samples = {
        "t": [],
        "target": {"steer": [], "boom": [], "bucket": []},
        "actual": {"steer": [], "boom": [], "bucket": []},
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
            with lock:
                active = curve_active
                t0 = start_time[0]

            if not active or t0 is None:
                for cfg in AXES.values():
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
                safe_set_status(
                    "Waiting for targets: steer={:.1f} boom={:.1f} bucket={:.1f}".format(
                        current_angles["steer"],
                        current_angles["boom"],
                        current_angles["bucket"],
                    )
                )
                for pid in pid_by_axis.values():
                    pid.reset()
                time.sleep(period)
                continue

            t = max(curve.T0, min(now - t0, curve.T3))
            current_targets = curve.get_targets(t)
            pump_required = False
            pump_angle = PUMP_NEUTRAL
            all_within = True

            for name, cfg in AXES.items():
                cur = current_angles.get(name)
                tgt = current_targets.get(name)
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
                    pump_angle = max(
                        pump_angle,
                        clamp(PUMP_NEUTRAL + delta, PUMP_MIN_ANGLE, PUMP_MAX_ANGLE),
                    )

            plot_samples["t"].append(t)
            for name in ("steer", "boom", "bucket"):
                plot_samples["target"][name].append(current_targets.get(name, 0.0))
                plot_samples["actual"][name].append(current_angles.get(name, 0.0))

            if pump_required:
                controller.set_servo_angle(PUMP_CHANNEL, pump_angle)
            else:
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)

            safe_set_status(
                "t={:.1f} steer={:.1f} boom={:.1f} bucket={:.1f} pump={:.1f}".format(
                    t,
                    current_angles.get("steer", 0.0),
                    current_angles.get("boom", 0.0),
                    current_angles.get("bucket", 0.0),
                    pump_angle,
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