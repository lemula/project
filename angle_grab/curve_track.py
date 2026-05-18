import os
import sys
import threading
import time
import tkinter as tk

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LARGE_GARB_DIR = os.path.join(ROOT, "large_garb")
if LARGE_GARB_DIR not in sys.path:
    sys.path.insert(0, LARGE_GARB_DIR)

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
        "ki": 1,
        "kd": 0,
        "i_min": -30.0,
        "i_max": 30.0,
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
    },
    "bucket": {
        "pot_key": "bucket_angle",
        "a_min": float(BUCKET_MIN_ANGLE),
        "a_max": float(BUCKET_MAX_ANGLE),
        "servo_ch": CHANNEL_6,
        "valve_neutral": 100.0,
        "valve_open_pos": 145.0,
        "valve_open_neg": 55.0,
        "kp": 2.0,
        "ki": 1.0,
        "kd": 0.0,
        "i_min": -30.0,
        "i_max": 30.0,
    },
}

ANGLE_TOL = 1.0
SLOW_TOL = 10.0
LOOP_HZ = 10

PUMP_CHANNEL = CHANNEL_7
PUMP_NEUTRAL = 45.0
PUMP_MIN_ANGLE = 45.0
PUMP_MAX_ANGLE = 90.0
PUMP_KP = 1.0

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def clamp_range(value, a_min, a_max):
    low = min(a_min, a_max)
    high = max(a_min, a_max)
    return clamp(value, low, high)


def normalize_key(key):
    key = key.strip().lower()
    if key in ("big", "big_arm", "boom"):
        return "big_arm"
    if key in ("small", "small_arm", "stick"):
        return "small_arm"
    if key in ("bucket",):
        return "bucket"
    return None


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


def smoothstep(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def write_plot(times, target_vals, actual_vals):
    if plt is None:
        return False
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, target_vals, label="target")
    ax.plot(times, actual_vals, label="actual")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("angle (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    out_path = os.path.join(os.path.dirname(__file__), "curve_track_plot.png")
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
    active = False
    plot_ready = False

    pid_by_axis = {
        name: PIDController(cfg["kp"], cfg["ki"], cfg["kd"], cfg["i_min"], cfg["i_max"])
        for name, cfg in AXES.items()
    }

    start_time = [None]
    plot_samples = {"t": [], "target": [], "actual": []}

    def on_close():
        stop_event.set()
        try:
            for cfg in AXES.values():
                controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
            controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
        except Exception as e:
            logger.error(f"Failed to reset valves: {e}")
        root.destroy()

    def start_curve():
        nonlocal active
        nonlocal plot_ready
        axis_key = normalize_key(axis_var.get()) or "big_arm"
        with lock:
            params["axis"] = axis_key
            active = True
            start_time[0] = time.time()
            start_angle[0] = current.get(axis_key)
            plot_samples["t"].clear()
            plot_samples["target"].clear()
            plot_samples["actual"].clear()
            plot_ready = False
        status_var.set("Curve started")

    root = tk.Tk()
    root.title("Curve Track (Quadratic)")
    root.protocol("WM_DELETE_WINDOW", on_close)

    axis_var = tk.StringVar(value="big_arm")
    start_angle = [current.get("big_arm", 0.0)]
    status_var = tk.StringVar(value="Ready")

    tk.Label(root, text="Axis").grid(row=0, column=0, padx=8, pady=6, sticky="e")
    tk.OptionMenu(root, axis_var, *AXES.keys()).grid(row=0, column=1, padx=8, pady=6, sticky="we")

    tk.Label(root, text="Profile: 0-3s -> 90, 3-8s -> 50, 8-10s hold").grid(
        row=1, column=0, columnspan=2, padx=8, pady=6
    )

    tk.Button(root, text="Start", command=start_curve).grid(
        row=2, column=0, columnspan=2, padx=8, pady=8
    )
    tk.Label(root, textvariable=status_var).grid(row=3, column=0, columnspan=2, padx=8, pady=6)

    params = {"axis": "big_arm"}
    last_time = time.time()
    duration_val = 10.0

    def safe_set_status(text):
        root.after(0, status_var.set, text)

    def control_loop():
        nonlocal active
        nonlocal last_time
        nonlocal plot_ready
        while not stop_event.is_set():
            now = time.time()
            dt = now - last_time
            last_time = now

            data = reader.read()
            current_angles = {name: data.get(cfg["pot_key"]) for name, cfg in AXES.items()}

            with lock:
                is_active = active
                axis = params["axis"]
                t0 = start_time[0]

            if not is_active or t0 is None:
                for cfg in AXES.values():
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
                safe_set_status("Waiting to start")
                for pid in pid_by_axis.values():
                    pid.reset()
                time.sleep(period)
                continue

            t = max(0.0, min(now - t0, duration_val))
            s0 = start_angle[0] if start_angle[0] is not None else 0.0
            if t <= 3.0:
                u = smoothstep(t / 3.0)
                tgt = s0 + (90.0 - s0) * u
            elif t <= 8.0:
                u = smoothstep((t - 3.0) / 5.0)
                tgt = 90.0 + (50.0 - 90.0) * u
            else:
                tgt = 50.0
            axis_cfg = AXES[axis]
            cur = current_angles.get(axis)

            for name, cfg in AXES.items():
                if name != axis:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])

            pump_required = False
            pump_angle = PUMP_NEUTRAL

            if cur is None:
                controller.set_servo_angle(axis_cfg["servo_ch"], axis_cfg["valve_neutral"])
            else:
                tgt = clamp_range(tgt, axis_cfg["a_min"], axis_cfg["a_max"])
                error = tgt - cur
                if abs(error) <= ANGLE_TOL:
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

            plot_samples["t"].append(t)
            plot_samples["target"].append(tgt)
            plot_samples["actual"].append(cur if cur is not None else 0.0)

            if pump_required:
                controller.set_servo_angle(PUMP_CHANNEL, pump_angle)
            else:
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)

            status_var.set(
                "{} t={:.2f}/{:.2f} cur={:.1f} tgt={:.1f} pump={:.1f}".format(
                    axis,
                    t,
                    duration_val,
                    cur if cur is not None else 0.0,
                    tgt,
                    pump_angle,
                )
            )

            if t >= duration_val:
                active = False
                if not plot_ready:
                    plot_ready = True
                    ok = write_plot(
                        list(plot_samples["t"]),
                        list(plot_samples["target"]),
                        list(plot_samples["actual"]),
                    )
                    if ok:
                        safe_set_status("Curve done, plot saved: curve_track_plot.png")
                    else:
                        safe_set_status("Curve done, plot failed (matplotlib missing)")
            time.sleep(period)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    root.mainloop()


if __name__ == "__main__":
    main()
