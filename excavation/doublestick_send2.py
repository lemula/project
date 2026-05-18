# tca_rotate_0_1_2_sender.py
# 功能：
# 1) pygame 检测最多 3 个操作杆
# 2) 按需求重排索引：0->2, 1->0, 2->1 （new0=old1, new1=old2, new2=old0）
# 3) 重排后识别名字包含 "TCA" 的设备为 TCA
# 4) 读取 TCA 的 axis0/axis1 作为 A3/B3，UDP 发送并每秒打印一次

import pygame
import socket
import time
import json
from datetime import datetime


class DualJoystickSender:
    def __init__(self, jetson_ip, jetson_port):
        pygame.init()
        pygame.joystick.init()

        self.joysticks = []
        self.joystick_names = []
        self.joystick_guids = []

        self.tca_index = None

        # 网络配置
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.jetson_address = (jetson_ip, jetson_port)
        self.connected = False

        # 前两个普通操作杆轴映射（保持你原来的 4 轴：X/Y/Z/W）
        self.controller_mappings = [
            [0, 1, 4, 2],  # 普通杆1
            [0, 1, 4, 2],  # 普通杆2
        ]

        # 初始化最多三个操作杆
        joystick_count = pygame.joystick.get_count()
        print(f"检测到 {joystick_count} 个操作杆设备")

        use_count = min(3, joystick_count)
        if use_count == 0:
            raise SystemExit("未检测到任何操作杆，程序退出")

        for i in range(use_count):
            js = pygame.joystick.Joystick(i)
            js.init()
            name = js.get_name()
            guid = getattr(js, "get_guid", lambda: "N/A")()

            self.joysticks.append(js)
            self.joystick_names.append(name)
            self.joystick_guids.append(guid)

            print(f"已连接操作杆 {i}: {name} | guid={guid}")

        # ========= 关键：重排索引（0->2, 1->0, 2->1）=========
        # new0 = old1
        # new1 = old2
        # new2 = old0
        if len(self.joysticks) >= 3:
            print("\n执行：索引重排（0->2, 1->0, 2->1）...")

            old_js = self.joysticks[:]
            old_names = self.joystick_names[:]
            old_guids = self.joystick_guids[:]

            self.joysticks[0] = old_js[1]
            self.joysticks[1] = old_js[2]
            self.joysticks[2] = old_js[0]

            self.joystick_names[0] = old_names[1]
            self.joystick_names[1] = old_names[2]
            self.joystick_names[2] = old_names[0]

            self.joystick_guids[0] = old_guids[1]
            self.joystick_guids[1] = old_guids[2]
            self.joystick_guids[2] = old_guids[0]

            print("重排后设备顺序：")
            for i in range(3):
                print(f"  {i}: {self.joystick_names[i]} | guid={self.joystick_guids[i]}")
            print()
        else:
            print("\n提示：设备不足3个，无法按(0->2,1->0,2->1)重排。\n")

        # 重排后：识别 TCA（名字包含 'TCA'）
        self.tca_index = None
        for i, name in enumerate(self.joystick_names):
            if "tca" in (name or "").lower():
                self.tca_index = i
                break

        if self.tca_index is not None:
            print(f"已识别 TCA 设备索引: {self.tca_index} ({self.joystick_names[self.tca_index]})")
        else:
            print("提示：未识别到包含 'TCA' 的设备名，A3/B3 将不会发送。")

    def verify_connection(self):
        print(f"正在验证与 {self.jetson_address} 的连接...")
        try:
            handshake = json.dumps({
                "type": "handshake",
                "timestamp": datetime.now().timestamp(),
                "controllers": self.joystick_names
            })
            self.sock.sendto(handshake.encode("utf-8"), self.jetson_address)

            self.sock.settimeout(3.0)
            response, addr = self.sock.recvfrom(1024)

            if addr == self.jetson_address:
                resp_data = json.loads(response.decode("utf-8"))
                if resp_data.get("status") == "ready":
                    self.connected = True
                    print("连接成功，准备发送数据...")
                    return True

        except socket.timeout:
            print("连接超时，未收到响应")
        except json.JSONDecodeError:
            print("收到无效的响应格式")
        except Exception as e:
            print(f"连接验证失败: {e}")
        finally:
            self.sock.settimeout(None)

        return False

    def read_joystick_data(self, js_index):
        """读取普通操作杆数据并格式化（只对前两个普通杆做映射）"""
        if js_index >= len(self.joysticks):
            return None

        # TCA 不走普通映射
        if self.tca_index is not None and js_index == self.tca_index:
            return None

        if js_index >= len(self.controller_mappings):
            return None

        js = self.joysticks[js_index]
        mapping = self.controller_mappings[js_index]

        num_axes = js.get_numaxes()
        if max(mapping) >= num_axes:
            return (
                f"操作杆 {js_index + 1} ({self.joystick_names[js_index]}):\n"
                f"  轴数量不足：num_axes={num_axes}，mapping={mapping}\n"
            )

        X = js.get_axis(mapping[0])
        Y = js.get_axis(mapping[1])
        Z = js.get_axis(mapping[2])
        W = js.get_axis(mapping[3])

        return (
            f"操作杆 {js_index + 1} ({self.joystick_names[js_index]}):\n"
            f"  X{js_index + 1}: {round(X, 3)}\n"
            f"  Y{js_index + 1}: {round(Y, 3)}\n"
            f"  Z{js_index + 1}: {round(Z, 3)}\n"
            f"  W{js_index + 1}: {round(W, 3)}\n"
        )

    def read_tca_airbus_ab(self):
        """读取 TCA 的 axis0/axis1，并作为 A3/B3 返回"""
        if self.tca_index is None:
            return None, None, ""

        js = self.joysticks[self.tca_index]
        num_axes = js.get_numaxes()
        if num_axes < 2:
            msg = (
                f"TCA ({self.joystick_names[self.tca_index]}):\n"
                f"  轴数量不足：num_axes={num_axes}，无法读取 axis0/axis1\n"
            )
            return None, None, msg

        a3 = js.get_axis(0)
        b3 = js.get_axis(1)

        msg = (
            f"TCA ({self.joystick_names[self.tca_index]}):\n"
            f"  A3(axis0): {round(a3, 3)}\n"
            f"  B3(axis1): {round(b3, 3)}\n"
        )
        return a3, b3, msg

    def read_all_data(self):
        pygame.event.pump()

        data = {"timestamp": datetime.now().timestamp()}
        out = ""

        for i in range(len(self.joysticks)):
            s = self.read_joystick_data(i)
            if s:
                out += s + "\n"

        a3, b3, tca_s = self.read_tca_airbus_ab()
        if tca_s:
            out += tca_s + "\n"

        data["A3"] = a3
        data["B3"] = b3
        data["formatted_data"] = out.strip()
        return data

    def run(self):
        if not self.verify_connection():
            print("无法建立连接，程序退出")
            self.cleanup()
            return

        print("开始发送数据，按 Ctrl+C 停止...")
        last_print = time.time()

        try:
            while True:
                t0 = time.time()

                data = self.read_all_data()
                self.sock.sendto(json.dumps(data).encode("utf-8"), self.jetson_address)

                now = time.time()
                if now - last_print >= 1.0:
                    ts = datetime.fromtimestamp(data["timestamp"]).strftime("%H:%M:%S")
                    print(f"\n[{ts}]")
                    if data["formatted_data"]:
                        print(data["formatted_data"])
                    print(f"SEND -> A3={data['A3']}, B3={data['B3']}")
                    last_print = now

                # 10Hz
                elapsed = time.time() - t0
                time.sleep(max(0.1 - elapsed, 0))

        except KeyboardInterrupt:
            print("\n用户中断，停止发送")
        finally:
            self.cleanup()

    def cleanup(self):
        for js in self.joysticks:
            try:
                js.quit()
            except Exception:
                pass
        pygame.joystick.quit()
        pygame.quit()
        try:
            self.sock.close()
        except Exception:
            pass
        print("资源已释放，程序退出")


if __name__ == "__main__":
    JETSON_IP = "192.168.2.62"
    JETSON_PORT = 8888

    sender = DualJoystickSender(JETSON_IP, JETSON_PORT)
    sender.run()
