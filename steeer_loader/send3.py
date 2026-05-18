import json
import socket
import time
from datetime import datetime

import pygame


def list_devices():
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    devices = []
    for i in range(count):
        js = pygame.joystick.Joystick(i)
        js.init()
        devices.append((i, js.get_name()))
    return devices


def pick_device(keywords):
    devices = list_devices()
    if not devices:
        return None

    for kw in keywords:
        for i, name in devices:
            if kw.lower() in (name or "").lower():
                js = pygame.joystick.Joystick(i)
                js.init()
                return js

    js = pygame.joystick.Joystick(devices[0][0])
    js.init()
    return js


def pick_device_excluding(exclude_names):
    devices = list_devices()
    if not devices:
        return None

    for i, name in devices:
        if name not in exclude_names:
            js = pygame.joystick.Joystick(i)
            js.init()
            return js

    return None


def detect_active_axes(js, count, duration=1.5):
    if js is None:
        return []

    n_axes = js.get_numaxes()
    if n_axes == 0:
        return []

    scores = [0.0] * n_axes
    last = [js.get_axis(i) for i in range(n_axes)]
    end_time = time.time() + duration

    while time.time() < end_time:
        pygame.event.pump()
        for i in range(n_axes):
            v = js.get_axis(i)
            scores[i] += abs(v - last[i])
            last[i] = v
        time.sleep(0.01)

    ranked = sorted(range(n_axes), key=lambda i: scores[i], reverse=True)
    return ranked[:count]


class ForkliftSender:
    def __init__(self, jetson_ip, jetson_port):
        pygame.init()
        pygame.joystick.init()

        self.wheel = pick_device(keywords=("Logitech", "G29", "G920", "Driving"))
        if self.wheel is None:
            raise RuntimeError("No controller devices detected.")

        self.tca = pick_device(keywords=("TCA", "Airbus", "Thrustmaster"))

        print(f"Wheel device: {self.wheel.get_name()}")
        if self.tca:
            print(f"TCA device: {self.tca.get_name()}")
        else:
            print("TCA device not found; only wheel axes will be sent.")

        exclude = {self.wheel.get_name()}
        if self.tca:
            exclude.add(self.tca.get_name())
        self.stick = pick_device_excluding(exclude_names=exclude)
        if self.stick:
            print(f"Stick device: {self.stick.get_name()}")
        else:
            print("Stick device not found; stick axes will be None.")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (jetson_ip, jetson_port)
        self.connected = False

        self.axis_steer = 0
        self.axis_forward = 2
        self.axis_reverse = 3
        self.tca_axis0 = 0
        self.tca_axis1 = 1
        self.stick_axis0 = 0
        self.stick_axis1 = 1

        print("Wheel axes fixed: steer=0 forward=2 reverse=3")

        if self.tca:
            tca_axes = detect_active_axes(self.tca, 2)
            if len(tca_axes) == 2:
                self.tca_axis0, self.tca_axis1 = tca_axes
                print(f"TCA axes auto-detected: {tca_axes}")
            else:
                print("TCA axes auto-detect failed; using defaults 0/1.")

        if self.stick:
            stick_axes = detect_active_axes(self.stick, 2)
            if len(stick_axes) == 2:
                self.stick_axis0, self.stick_axis1 = stick_axes
                print(f"Stick axes auto-detected: {stick_axes}")
            else:
                print("Stick axes auto-detect failed; using defaults 0/1.")

        self.deadband = 0.0
        self.precision = 4

        self.headless = True

    def _deadband(self, v):
        return 0.0 if abs(v) < self.deadband else v

    def read_axes(self):
        pygame.event.pump()
        steer = self._deadband(self.wheel.get_axis(self.axis_steer))
        forward = self._deadband(self.wheel.get_axis(self.axis_forward))
        reverse = self._deadband(self.wheel.get_axis(self.axis_reverse))

        tca0 = None
        tca1 = None
        tca_b07 = None
        if self.tca:
            tca0 = self._deadband(self.tca.get_axis(self.tca_axis0))
            tca1 = self._deadband(self.tca.get_axis(self.tca_axis1))
            if self.tca.get_numbuttons() > 7:
                tca_b07 = int(self.tca.get_button(7))

        stick0 = None
        stick1 = None
        if self.stick:
            stick0 = self._deadband(self.stick.get_axis(self.stick_axis0))
            stick1 = self._deadband(self.stick.get_axis(self.stick_axis1))

        return {
            "timestamp": datetime.now().timestamp(),
            "wheel": {
                "steer": steer,
                "forward": forward,
                "reverse": reverse,
            },
            "tca": {
                "axis0": tca0,
                "axis1": tca1,
                "b07": tca_b07,
            },
            "stick": {
                "axis0": stick0,
                "axis1": stick1,
            },
        }

    def verify_connection(self):
        try:
            handshake = json.dumps(
                {
                    "type": "handshake",
                    "timestamp": datetime.now().timestamp(),
                    "controllers": [
                        self.wheel.get_name(),
                        self.tca.get_name() if self.tca else None,
                        self.stick.get_name() if self.stick else None,
                    ],
                }
            )
            self.sock.sendto(handshake.encode("utf-8"), self.addr)
            self.sock.settimeout(3.0)
            response, addr = self.sock.recvfrom(1024)
            if addr == self.addr:
                resp_data = json.loads(response.decode("utf-8"))
                if resp_data.get("status") == "ready":
                    self.connected = True
                    return True
        except Exception:
            return False
        finally:
            self.sock.settimeout(None)
        return False

    def run(self, hz=20):
        interval = 1.0 / float(hz)
        last_print = 0.0
        try:
            if not self.verify_connection():
                print("Handshake failed; aborting.")
                return
            while True:
                t0 = time.time()

                data = self.read_axes()
                self.sock.sendto(json.dumps(data).encode("utf-8"), self.addr)

                if t0 - last_print >= 1.0:
                    ts = datetime.fromtimestamp(data["timestamp"]).strftime("%H:%M:%S")
                    def fmt(v):
                        return "None" if v is None else f"{v:.{self.precision}f}"
                    print(
                        f"[{ts}] steer={fmt(data['wheel']['steer'])} "
                        f"forward={fmt(data['wheel']['forward'])} "
                        f"reverse={fmt(data['wheel']['reverse'])} "
                        f"tca0={fmt(data['tca']['axis0'])} tca1={fmt(data['tca']['axis1'])} "
                        f"b07={data['tca']['b07']} "
                        f"stick0={fmt(data['stick']['axis0'])} stick1={fmt(data['stick']['axis1'])}"
                    )
                    last_print = t0

                elapsed = time.time() - t0
                time.sleep(max(0.0, interval - elapsed))
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.wheel:
            self.wheel.quit()
        if self.tca:
            self.tca.quit()
        if self.stick:
            self.stick.quit()
        pygame.quit()
        self.sock.close()


if __name__ == "__main__":
    JETSON_IP = "192.168.2.87"
    JETSON_PORT = 8888

    ForkliftSender(JETSON_IP, JETSON_PORT).run()