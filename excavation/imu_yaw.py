#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import struct
import serial


HEADER = bytes.fromhex("ab546500")
FRAME_LEN = 66


def open_ser(port, baud, tout=0.02):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=tout,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(0.05)
    ser.reset_input_buffer()
    return ser


def read_exact(ser, n):
    data = b""
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if chunk:
            data += chunk
    return data


def normalize_angle_deg(angle_deg):
    """把角度限制到 [-180, 180)"""
    while angle_deg >= 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def parse_frame(frame):
    """
    解析一帧数据
    返回:
        {
            'gx_deg_s', 'gy_deg_s', 'gz_deg_s',
            'ax_m_s2', 'ay_m_s2', 'az_m_s2',
            'timestamp_ms'
        }
    """
    gx, gy, gz = struct.unpack_from("<fff", frame, 23)   # deg/s
    ax, ay, az = struct.unpack_from("<fff", frame, 35)   # m/s^2
    ts_ms = struct.unpack_from("<Q", frame, 56)[0]       # ms

    return {
        "gx_deg_s": gx,
        "gy_deg_s": gy,
        "gz_deg_s": gz,
        "ax_m_s2": ax,
        "ay_m_s2": ay,
        "az_m_s2": az,
        "timestamp_ms": ts_ms
    }


def read_imu_frame(ser):
    """
    从串口字节流中读取一帧 IMU 数据
    成功返回 dict，失败返回 None
    """
    buf = b""

    while True:
        b1 = ser.read(1)
        if not b1:
            continue

        buf = (buf + b1)[-4:]
        if buf != HEADER:
            continue

        length = read_exact(ser, 2)
        rest = read_exact(ser, FRAME_LEN - 6)

        if len(length) + len(rest) != FRAME_LEN - 4:
            return None

        frame = HEADER + length + rest

        try:
            return parse_frame(frame)
        except Exception as e:
            print("解析失败:", e)
            return None


class IMUAxisTracker:
    """
    用于跟踪各轴角速度与积分角度
    可作为子函数/对象被调用，指定 axis='x'/'y'/'z'
    """
    def __init__(self):
        self.last_ts_ms = None
        self.angle_deg = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }
        self.rate_deg_s = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }

    def update(self, imu_data):
        """
        输入一帧 imu_data，更新角度和角速度
        """
        ts_ms = imu_data["timestamp_ms"]

        gx = imu_data["gx_deg_s"]
        gy = imu_data["gy_deg_s"]
        gz = imu_data["gz_deg_s"]

        self.rate_deg_s["x"] = gx
        self.rate_deg_s["y"] = gy
        self.rate_deg_s["z"] = gz

        if self.last_ts_ms is not None:
            dt = (ts_ms - self.last_ts_ms) * 1e-3
            if dt > 0:
                self.angle_deg["x"] += gx * dt
                self.angle_deg["y"] += gy * dt
                self.angle_deg["z"] += gz * dt

                self.angle_deg["x"] = normalize_angle_deg(self.angle_deg["x"])
                self.angle_deg["y"] = normalize_angle_deg(self.angle_deg["y"])
                self.angle_deg["z"] = normalize_angle_deg(self.angle_deg["z"])

        self.last_ts_ms = ts_ms

    def get_axis_data(self, axis="z", angle_unit="deg", rate_unit="deg/s"):
        """
        作为子函数调用时可指定轴:
            axis='x'/'y'/'z'

        返回:
            angle, rate
        """
        axis = axis.lower()
        if axis not in ("x", "y", "z"):
            raise ValueError("axis 必须是 'x'、'y' 或 'z'")

        angle = self.angle_deg[axis]
        rate = self.rate_deg_s[axis]

        if angle_unit == "rad":
            angle = math.radians(angle)
        elif angle_unit != "deg":
            raise ValueError("angle_unit 只能是 'deg' 或 'rad'")

        if rate_unit == "rad/s":
            rate = math.radians(rate)
        elif rate_unit != "deg/s":
            raise ValueError("rate_unit 只能是 'deg/s' 或 'rad/s'")

        return angle, rate


def read_axis_angle_and_rate(ser, tracker, axis="z", angle_unit="deg", rate_unit="deg/s"):
    """
    子函数接口：
    读取一帧 -> 更新状态 -> 返回指定轴的角度和角速度

    参数:
        ser: 串口对象
        tracker: IMUAxisTracker对象
        axis: 'x'/'y'/'z'
        angle_unit: 'deg' 或 'rad'
        rate_unit: 'deg/s' 或 'rad/s'

    返回:
        angle, rate
    """
    imu_data = read_imu_frame(ser)
    if imu_data is None:
        return None, None

    tracker.update(imu_data)
    return tracker.get_axis_data(axis=axis, angle_unit=angle_unit, rate_unit=rate_unit)


def main():
    port = "/dev/ttyTHS0"   # 根据你的设备修改
    baud = 460800

    ser = open_ser(port, baud)
    tracker = IMUAxisTracker()

    print(f"串口已打开: {port}, 波特率: {baud}")
    print("每100ms打印一次: yaw角(z轴积分角) 和 yaw_rate(z轴角速度)")

    last_print_time = time.time()

    try:
        while True:
            imu_data = read_imu_frame(ser)
            if imu_data is None:
                continue

            tracker.update(imu_data)

            now = time.time()
            if now - last_print_time >= 0.1:   # 100ms
                yaw_deg, yaw_rate_deg_s = tracker.get_axis_data(axis="z", angle_unit="deg", rate_unit="deg/s")

                print(
                    f"yaw = {yaw_deg:8.3f} deg, "
                    f"yaw_rate = {yaw_rate_deg_s:8.3f} deg/s"
                )

                last_print_time = now

    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()