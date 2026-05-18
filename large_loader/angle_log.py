import csv
import os
import threading
import time
from datetime import datetime

from servo_control import CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5


class AngleCsvLogger:
    def __init__(self, pot_reader, tracking, out_dir, hz=10.0):
        self._pot_reader = pot_reader
        self._tracking = tracking
        self._period = 1.0 / hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._out_path = os.path.join(out_dir, f"angle_log_{timestamp}.csv")

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self):
        with open(self._out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "timestamp",
                    "steer_angle",
                    "boom_angle",
                    "bucket_angle",
                    "steer_servo",
                    "boom_servo",
                    "bucket_servo",
                    "pump",
                    "motor_speed",
                ]
            )
            while not self._stop.is_set():
                try:
                    data = self._pot_reader.read()
                    writer.writerow(
                        [
                            time.time(),
                            data.get("steer_angle", 0.0),
                            data.get("boom_angle", 0.0),
                            data.get("bucket_angle", 0.0),
                            self._tracking.get_servo_angle(CHANNEL_4, 0.0),
                            self._tracking.get_servo_angle(CHANNEL_2, 0.0),
                            self._tracking.get_servo_angle(CHANNEL_1, 0.0),
                            self._tracking.get_servo_angle(CHANNEL_5, 0.0),
                            self._tracking.get_motor_speed(CHANNEL_3, 0.0),
                        ]
                    )
                    f.flush()
                except Exception:
                    pass
                time.sleep(self._period)