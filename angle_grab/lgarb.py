import csv
import os
import threading
import time
from datetime import datetime

from joystick_processor import JoystickProcessor
from servo_control import ServoController, logger
from udp_receiver import UdpJoystickReceiver
from dwj_read import PotentiometerReader


class AngleCsvLogger:
    def __init__(self, pot_reader, out_path, hz=10.0):
        self._pot_reader = pot_reader
        self._out_path = out_path
        self._period = 1.0 / hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self):
        try:
            with open(self._out_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "timestamp",
                        "big_arm_angle",
                        "small_arm_angle",
                        "bucket_angle",
                        "big_arm_v",
                        "small_arm_v",
                        "bucket_v",
                    ]
                )
                while not self._stop.is_set():
                    try:
                        data = self._pot_reader.read()
                        writer.writerow(
                            [
                                time.time(),
                                data["big_arm_angle"],
                                data["small_arm_angle"],
                                data["bucket_angle"],
                                data["big_arm_v"],
                                data["small_arm_v"],
                                data["bucket_v"],
                            ]
                        )
                        f.flush()
                    except Exception as e:
                        logger.error(f"pot read failed: {e}")
                    time.sleep(self._period)
        except Exception as e:
            logger.error(f"csv logger failed: {e}")


if __name__ == "__main__":
    LISTEN_IP = '192.168.2.88'
    LISTEN_PORT = 8888
    controller = ServoController()
    processor = JoystickProcessor(controller)
    try:
        pot_reader = PotentiometerReader()
    except Exception as e:
        pot_reader = None
        logger.warning(f"电位计读取初始化失败: {e}")

    def print_with_pot(latest_data):
        processor.print_data(latest_data)
        if not pot_reader:
            return
        try:
            data = pot_reader.read()
            print(
                "pot: big_arm V={:.3f} angle={:.1f} | small_arm V={:.3f} angle={:.1f} | bucket V={:.3f} angle={:.1f}"
                .format(
                    data["big_arm_v"],
                    data["big_arm_angle"],
                    data["small_arm_v"],
                    data["small_arm_angle"],
                    data["bucket_v"],
                    data["bucket_angle"],
                )
            )
        except Exception as e:
            logger.error(f"电位计读取失败: {e}")
    receiver = UdpJoystickReceiver(
        LISTEN_IP,
        LISTEN_PORT,
        on_data=processor.process_data,
        on_print=print_with_pot,
        print_interval=1.0,
    )
    csv_logger = None
    if pot_reader:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(os.path.dirname(__file__), f"angle_log_{timestamp}.csv")
        csv_logger = AngleCsvLogger(pot_reader, out_path, hz=10.0)

    try:
        if csv_logger:
            csv_logger.start()
        receiver.start()
    except Exception as e:
        logger.critical(f"启动失败: {e}")
    finally:
        if csv_logger:
            csv_logger.stop()
        processor.reset_devices()
        receiver.close()
