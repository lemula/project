import json
import socket
from datetime import datetime


class ForkliftReceiver:
    def __init__(self, host="0.0.0.0", port=8888):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        print(f"Listening on {host}:{port}")

    @staticmethod
    def _fmt(value):
        if value is None:
            return "None"
        if isinstance(value, float):
            return f"{value:.4f}"
        return str(value)

    def run(self):
        try:
            while True:
                packet, src = self.sock.recvfrom(65535)

                try:
                    payload = json.loads(packet.decode("utf-8"))
                except Exception as exc:
                    print(f"Invalid packet from {src}: {exc}")
                    continue

                ts = payload.get("timestamp")
                if ts is None:
                    ts_str = "N/A"
                else:
                    ts_str = datetime.fromtimestamp(ts).strftime("%H:%M:%S")

                wheel = payload.get("wheel", {})
                tca = payload.get("tca", {})
                stick = payload.get("stick", {})

                print(
                    f"[{ts_str}] from {src} "
                    f"steer={self._fmt(wheel.get('steer'))} "
                    f"forward={self._fmt(wheel.get('forward'))} "
                    f"reverse={self._fmt(wheel.get('reverse'))} "
                    f"tca0={self._fmt(tca.get('axis0'))} "
                    f"tca1={self._fmt(tca.get('axis1'))} "
                    f"b07={self._fmt(tca.get('b07'))} "
                    f"stick0={self._fmt(stick.get('axis0'))} "
                    f"stick1={self._fmt(stick.get('axis1'))}"
                )
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            self.sock.close()


if __name__ == "__main__":
    ForkliftReceiver(host="0.0.0.0", port=8888).run()
