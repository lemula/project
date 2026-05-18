import json
import socket
import time

from servo_control import logger


class UdpJoystickReceiver:
    def __init__(
        self,
        listen_ip="192.168.2.62",
        listen_port=8888,
        on_data=None,
        on_print=None,
        print_interval=1.0,
    ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_address = (listen_ip, listen_port)
        self.last_receive_time = 0
        self.last_print_time = 0
        self.print_interval = print_interval
        self.connected = False
        self.on_data = on_data
        self.on_print = on_print

    def start(self):
        self.sock.bind(self.listen_address)
        logger.info(f"listening on {self.listen_address}...")
        logger.info("press Ctrl+C to stop")

        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                self.last_receive_time = time.time()

                try:
                    json_data = json.loads(data.decode("utf-8"))

                    if json_data.get("type") == "handshake":
                        self.connected = True
                        logger.info(f"handshake received from {addr}")
                        response = json.dumps({"status": "ready"})
                        self.sock.sendto(response.encode("utf-8"), addr)
                        logger.info(f"ready response sent to {addr}")
                    else:
                        self.connected = True
                        if self.on_data:
                            self.on_data(json_data)

                        current_time = time.time()
                        if self.on_print and current_time - self.last_print_time >= self.print_interval:
                            self.on_print(json_data)
                            self.last_print_time = current_time

                except json.JSONDecodeError:
                    logger.error(f"invalid JSON data: {data.decode('utf-8')}")
                except Exception as e:
                    logger.error(f"failed to process joystick data: {e}")

        except KeyboardInterrupt:
            logger.info("stopped by user")

    def close(self):
        self.sock.close()
