import json
import socket
import time
from datetime import datetime


class ForkliftReceiver:
    def __init__(self, host="0.0.0.0", port=8888):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        self.last_print = 0.0
        self.precision = 4
        print(f"Listening on {host}:{port}")

    def run(self):
        try:
            while True:
                data, src = self.sock.recvfrom(65535)
                try:
                    payload = json.loads(data.decode("utf-8"))
                except Exception as e:
                    print(f"Invalid packet from {src}: {e}")
                    continue

                if isinstance(payload, dict) and payload.get("type") == "handshake":
                    resp = {"status": "ready", "timestamp": datetime.now().timestamp()}
                    self.sock.sendto(json.dumps(resp).encode("utf-8"), src)
                    print(f"Handshake from {src}, replied ready.")
                    continue

                now = time.time()
                if now - self.last_print >= 1.0:
                    ts = payload.get("timestamp")
                    if ts is not None:
                        ts_str = datetime.fromtimestamp(ts).strftime("%H:%M:%S")
                    else:
                        ts_str = "N/A"

                    wheel = payload.get("wheel", {})
                    tca = payload.get("tca", {})
                    stick = payload.get("stick", {})

                    def fmt(v):
                        return "None" if v is None else f"{v:.{self.precision}f}"

                    print(
                        f"[{ts_str}] from {src} "
                        f"steer={fmt(wheel.get('steer'))} "
                        f"forward={fmt(wheel.get('forward'))} "
                        f"reverse={fmt(wheel.get('reverse'))} "
                        f"tca0={fmt(tca.get('axis0'))} tca1={fmt(tca.get('axis1'))} "
                        f"b07={tca.get('b07')} "
                        f"stick0={fmt(stick.get('axis0'))} stick1={fmt(stick.get('axis1'))}"
                    )
                    self.last_print = now
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            self.sock.close()


if __name__ == "__main__":
    ForkliftReceiver(host="0.0.0.0", port=8888).run()
