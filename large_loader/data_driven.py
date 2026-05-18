import csv
import os
import sys
import time
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
        "kp": 0.5,
        "ki": 0,
        "kd": 0,
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
        "kp": 0.5,
        "ki": 0.3,
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
        "kp": 0.5,
        "ki": 0.3,
        "kd": 0,
        "i_min": -30.0,
        "i_max": 30.0,
        "invert": False,
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


def read_targets(path):
    targets = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row:
                continue
            ts = row.get("timestamp")
            try:
                ts = float(ts) if ts else None
            except ValueError:
                ts = None
            try:
                steer = float(row.get("steer_angle", 0.0))
                boom = float(row.get("boom_angle", 0.0))
                bucket = float(row.get("bucket_angle", 0.0))
            except ValueError:
                continue
            targets.append((ts, {"steer": steer, "boom": boom, "bucket": bucket}))
    return targets


def main():
    csv_path = os.path.join(BASE_DIR, "angle_log.csv")
    if not os.path.exists(csv_path):
        logger.error(f"CSV not found: {csv_path}")
        return

    target_rows = read_targets(csv_path)
    if not target_rows:
        logger.error("No target rows found in CSV")
        return

    controller = ServoController()
    reader = ThreeAxisPotReader()
    pid_by_axis = {
        name: PIDController(cfg["kp"], cfg["ki"], cfg["kd"], cfg["i_min"], cfg["i_max"])
        for name, cfg in AXES.items()
    }

    idx = 0
    period = 1.0 / LOOP_HZ
    last_time = time.time()
    start_time = time.time()
    base_ts = target_rows[0][0]
    next_switch_time = start_time
    samples = {
        "t": [],
        "target": {"steer": [], "boom": [], "bucket": []},
        "actual": {"steer": [], "boom": [], "bucket": []},
    }

    try:
        while idx < len(target_rows):
            now = time.time()
            dt = now - last_time
            last_time = now

            if now >= next_switch_time:
                ts, current_targets = target_rows[idx]
                idx += 1
                if idx < len(target_rows):
                    next_ts = target_rows[idx][0]
                    if ts is not None and next_ts is not None:
                        step = max(0.01, min(next_ts - ts, 1.0))
                    else:
                        step = 0.1
                    next_switch_time = now + step

            data = reader.read()
            current_angles = {name: data.get(cfg["pot_key"]) for name, cfg in AXES.items()}
            pump_required = False
            pump_angle = PUMP_NEUTRAL

            for name, cfg in AXES.items():
                cur = current_angles.get(name)
                tgt = current_targets.get(name)
                if cur is None or tgt is None:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                    continue

                tgt = clamp_range(tgt, cfg["a_min"], cfg["a_max"])
                error = tgt - cur
                if cfg.get("invert"):
                    error = -error
                if abs(error) <= ANGLE_TOL:
                    controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
                else:
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

            if pump_required:
                controller.set_servo_angle(PUMP_CHANNEL, pump_angle)
            else:
                controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)

            samples["t"].append(now - start_time)
            for name in ("steer", "boom", "bucket"):
                samples["target"][name].append(current_targets.get(name, 0.0))
                samples["actual"][name].append(current_angles.get(name, 0.0))

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for cfg in AXES.values():
            controller.set_servo_angle(cfg["servo_ch"], cfg["valve_neutral"])
        controller.set_servo_angle(PUMP_CHANNEL, PUMP_NEUTRAL)
        if samples["t"]:
            out_name = f"data_driven_plot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            out_path = os.path.join(BASE_DIR, out_name)
            ok = write_plot(samples["t"], samples["target"], samples["actual"], out_path)
            if ok:
                logger.info(f"Plot saved: {out_path}")
            else:
                logger.warning("Plot skipped (matplotlib missing)")


if __name__ == "__main__":
    main()
