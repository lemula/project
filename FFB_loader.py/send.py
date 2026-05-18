import json
import os
import socket
import time
from datetime import datetime

import pygame
from evdev import InputDevice, ecodes, list_devices as evdev_list_devices

from FFB import Config, build_constant_effect, compute_align_torque, configure_device


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


def find_ffb_device(preferred_name, fallback="/dev/input/event6"):
    override = os.environ.get("FFB_DEVICE")
    if override:
        return override
    debug = os.environ.get("FFB_DEBUG") == "1"
    try:
        candidates = []
        for path in evdev_list_devices():
            try:
                dev = InputDevice(path)
                caps = dev.capabilities().get(ecodes.EV_FF, [])
                if ecodes.FF_CONSTANT not in caps:
                    continue
                candidates.append((path, dev.name or ""))
                if preferred_name and preferred_name.lower() in (dev.name or "").lower():
                    return path
            except Exception:
                continue
        if candidates:
            if debug:
                print("FFB candidates:", candidates)
            return candidates[0][0]
    except Exception:
        pass
    return fallback


class ForkliftSender:
    def __init__(self, jetson_ip, jetson_port):
        pygame.init()
        pygame.joystick.init()

        self.wheel = pick_device(keywords=("Logitech", "G29", "G920", "Driving"))
        if self.wheel is None:
            raise RuntimeError("No controller devices detected.")

        print(f"Wheel device: {self.wheel.get_name()}")

        exclude = {self.wheel.get_name()}
        self.stick = pick_device_excluding(exclude_names=exclude)
        if self.stick:
            print(f"Stick device: {self.stick.get_name()}")
        else:
            print("Stick device not found; stick axes will be None.")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (jetson_ip, jetson_port)
        self.connected = False

        # Force feedback (evdev) config
        self.ff_enabled = True
        self.ff_cfg = Config()
        self.ff_cfg.device = find_ffb_device(self.wheel.get_name(), fallback="/dev/input/event6")
        self.ff_cfg.hz = 100
        self.ff_cfg.print_hz = 0.0
        # Stronger defaults for noticeable feedback
        self.ff_cfg.deadband = 0.0
        self.ff_cfg.soft_zone = 0.0
        self.ff_cfg.min_torque = 0.15
        self.ff_cfg.max_torque = 1.0
        self.ff_cfg.linear_gain = 0.6
        self.ff_cfg.cubic_gain = 1.0
        self.ff_cfg.damping = 0.12
        self.ff_cfg.smooth = 0.2
        self.ff_cfg.gain = 1.0
        self.ff_cfg.hw_autocenter = 0.0

        self.ff_dev = None
        self.ff_effect_id = -1
        self.ff_last_pos = 0.0
        self.ff_last_time = time.time()
        self.ff_torque_f = None

        if self.ff_enabled:
            try:
                self.ff_dev = InputDevice(self.ff_cfg.device)
                try:
                    os.set_blocking(self.ff_dev.fd, False)
                except Exception:
                    pass
                caps = self.ff_dev.capabilities()
                ff_caps = caps.get(ecodes.EV_FF, [])
                if ecodes.FF_CONSTANT not in ff_caps:
                    print("FFB disabled: device does not support FF_CONSTANT.")
                    self.ff_dev.close()
                    self.ff_dev = None
                else:
                    configure_device(self.ff_dev, self.ff_cfg)
                    self._apply_ff_torque(0.0)
                    # Optional startup pulse to confirm FFB works
                    if os.environ.get("FFB_PULSE") == "1":
                        self._apply_ff_torque(0.3)
                        time.sleep(0.2)
                        self._apply_ff_torque(0.0)
                    print(f"FFB device: {self.ff_cfg.device} ({self.ff_dev.name})")
            except Exception as e:
                print(f"FFB init failed: {e}")
                self.ff_dev = None

        self.axis_steer = 0
        self.axis_forward = 2
        self.axis_reverse = 3
        self.stick_axis0 = 0
        self.stick_axis1 = 1

        print("Wheel axes fixed: steer=0 forward=2 reverse=3")

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
            "stick": {
                "axis0": stick0,
                "axis1": stick1,
            },
        }

    def update_force_feedback(self, steer_value):
        if self.ff_dev is None:
            return
        now = time.time()
        dt = max(1e-4, now - self.ff_last_time)
        vel = (steer_value - self.ff_last_pos) / dt
        self.ff_last_pos = steer_value
        self.ff_last_time = now

        torque_cmd = compute_align_torque(steer_value, vel, self.ff_cfg)
        if self.ff_torque_f is None:
            self.ff_torque_f = torque_cmd
        else:
            self.ff_torque_f = (1.0 - self.ff_cfg.smooth) * self.ff_torque_f + self.ff_cfg.smooth * torque_cmd
        torque_cmd = self.ff_torque_f

        self._apply_ff_torque(torque_cmd)

    def _apply_ff_torque(self, torque_cmd):
        if self.ff_dev is None:
            return
        eff = build_constant_effect(self.ff_effect_id, torque_cmd)
        self.ff_effect_id = self.ff_dev.upload_effect(eff)
        try:
            self.ff_dev.write(ecodes.EV_FF, self.ff_effect_id, 1)
        except Exception:
            pass

    def verify_connection(self):
        try:
            handshake = json.dumps(
                {
                    "type": "handshake",
                    "timestamp": datetime.now().timestamp(),
                    "controllers": [
                        self.wheel.get_name(),
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
                self.update_force_feedback(data["wheel"]["steer"])
                self.sock.sendto(json.dumps(data).encode("utf-8"), self.addr)

                if t0 - last_print >= 1.0:
                    ts = datetime.fromtimestamp(data["timestamp"]).strftime("%H:%M:%S")
                    def fmt(v):
                        return "None" if v is None else f"{v:.{self.precision}f}"
                    print(
                        f"[{ts}] steer={fmt(data['wheel']['steer'])} "
                        f"forward={fmt(data['wheel']['forward'])} "
                        f"reverse={fmt(data['wheel']['reverse'])} "
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
        if self.ff_dev:
            try:
                effz = build_constant_effect(self.ff_effect_id, 0.0)
                self.ff_dev.upload_effect(effz)
                self.ff_dev.write(ecodes.EV_FF, self.ff_effect_id, 1)
            except Exception:
                pass
            try:
                self.ff_dev.erase_effect(self.ff_effect_id)
            except Exception:
                pass
            try:
                self.ff_dev.close()
            except Exception:
                pass
        if self.wheel:
            self.wheel.quit()
        if self.stick:
            self.stick.quit()
        pygame.quit()
        self.sock.close()


if __name__ == "__main__":
    JETSON_IP = "192.168.2.87"
    JETSON_PORT = 8888

    ForkliftSender(JETSON_IP, JETSON_PORT).run()
