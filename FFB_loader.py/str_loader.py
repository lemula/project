import json
import socket
import time
from datetime import datetime

from data_process import DataProcessor
from dwj_read import ThreeAxisPotReader, STEER_MAX_ANGLE, STEER_MIN_ANGLE
from servo_control import ServoController


class Loader:
    def __init__(self, host="0.0.0.0", port=8888):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        self.controller = ServoController()
        self.processor = DataProcessor(self.controller)
        self.reader = ThreeAxisPotReader()
        self.last_print = 0.0
        self._steer_accum = 0.0
        self._last_target_steer = None
        self._steer_hold = 0.0

        print(f"Listening on {host}:{port}")

    def _compute_speed(self, data):
        wheel = data.get("wheel", {})
        forward = wheel.get("forward")
        reverse = wheel.get("reverse")
        forward_norm = self.processor._normalize_from_rest(forward, v_rest=1.0, v_min=-1.0)
        reverse_norm = self.processor._normalize_from_rest(reverse, v_rest=1.0, v_min=-1.0)
        return (forward_norm - reverse_norm) * 100.0

    def _compute_target_steer(self, data):
        wheel = data.get("wheel", {})
        steer = wheel.get("steer")
        steer = 0.0 if steer is None else steer
        steer = self.processor._clamp(steer, -1.0, 1.0)
        t = (steer + 1.0) * 0.5
        return STEER_MIN_ANGLE + (STEER_MAX_ANGLE - STEER_MIN_ANGLE) * t

    def run(self):
        try:
            while True:
                payload, src = self.sock.recvfrom(65535)
                try:
                    data = json.loads(payload.decode("utf-8"))
                except Exception:
                    continue

                if isinstance(data, dict) and data.get("type") == "handshake":
                    resp = {"status": "ready", "timestamp": datetime.now().timestamp()}
                    self.sock.sendto(json.dumps(resp).encode("utf-8"), src)
                    continue

                target_steer = self._compute_target_steer(data)
                if self._last_target_steer is None:
                    self._last_target_steer = target_steer
                else:
                    self._steer_accum += abs(target_steer - self._last_target_steer)
                    self._last_target_steer = target_steer

                if self._steer_accum >= 10.0:
                    self._steer_hold = data.get("wheel", {}).get("steer", 0.0)
                    self._steer_accum = 0.0

                data = dict(data)
                data.pop("tca", None)
                wheel = dict(data.get("wheel", {}))
                wheel["steer"] = self._steer_hold
                data["wheel"] = wheel

                self.processor.process(data)

                now = time.time()
                if now - self.last_print >= 1.0:
                    angles = self.reader.read()
                    speed = self._compute_speed(data)
                    target_steer = self._compute_target_steer(data)
                    print(
                        "steer={:.4f} target_steer={:.4f} boom={:.4f} "
                        "bucket={:.4f} speed={:.4f}".format(
                            angles.get("steer_angle", 0.0),
                            target_steer,
                            angles.get("boom_angle", 0.0),
                            angles.get("bucket_angle", 0.0),
                            speed,
                        )
                    )
                    self.last_print = now
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            self.sock.close()


if __name__ == "__main__":
    Loader(host="192.168.2.87", port=8888).run()
