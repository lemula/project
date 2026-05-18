from joystick_processor import JoystickProcessor
from servo_control import ServoController, logger
from udp_receiver import UdpJoystickReceiver


if __name__ == "__main__":
    LISTEN_IP = "192.168.2.62"
    LISTEN_PORT = 8888

    controller = ServoController()
    processor = JoystickProcessor(controller)
    receiver = UdpJoystickReceiver(
        LISTEN_IP,
        LISTEN_PORT,
        on_data=processor.process_data,
        on_print=processor.print_data,
        print_interval=1.0,
    )

    try:
        receiver.start()
    except Exception as e:
        logger.critical(f"startup failed: {e}")
    finally:
        processor.reset_devices()
        receiver.close()
