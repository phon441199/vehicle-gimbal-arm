#!/usr/bin/env python3
"""
gimbal_bridge.py  --  serial <-> ROS2 bridge for gimbal_level_hold_esp32.

Reads the firmware's CSV telemetry ("#T,ms,phi,tgt,err,v,d_hat,w_eso,dob")
and republishes each field as a std_msgs/Float32 topic so you can graph the
ESO on/off difference in PlotJuggler / rqt_plot. Also forwards commands:
publish a std_msgs/String to /gimbal/key and the bridge writes it to the
serial menu (e.g. "C" capture, "B" toggle ESO, "G 20", "A -0.8", "T" telem).

Firmware keeps its serial menu -- this bridge does NOT need micro-ROS on the
MCU, so the live tuning workflow is unchanged and the control loop is untouched.

Run (ROS2 sourced):
    python3 gimbal_bridge.py [/dev/ttyUSB0] [115200]

Needs: pyserial  (pip install pyserial  /  apt install python3-serial)
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial (or apt install python3-serial)")

FIELDS = ["phi", "target", "err", "v", "vpd", "d_hat", "w_eso", "dob"]  # after leading ms; comp = v-vpd


class GimbalBridge(Node):
    def __init__(self, port, baud):
        super().__init__("gimbal_bridge")
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.pubs = {f: self.create_publisher(Float32, f"/gimbal/{f}", 10) for f in FIELDS}
        self.create_subscription(String, "/gimbal/key", self.on_key, 10)
        self.get_logger().info(
            f"bridge up on {port}@{baud}. pub /gimbal/{{{','.join(FIELDS)}}}, sub /gimbal/key")
        self.alive = True
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.stdin_reader, daemon=True).start()  # type cmds here too

    def send(self, s):
        s = s.strip()
        if s:
            self.ser.write((s + "\n").encode())      # firmware parse_command reads until \n

    def on_key(self, msg):
        self.send(msg.data)                          # commands via ROS topic (scripting)

    def stdin_reader(self):
        # type menu commands (C/B/G 20/A -0.8/T ...) right in this terminal
        while self.alive:
            try:
                s = input()
            except (EOFError, KeyboardInterrupt):
                break
            self.send(s)

    def reader(self):
        while self.alive:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not line.startswith("#T,"):
                continue                              # ignore menu/human prints
            parts = line[3:].split(",")
            if len(parts) != len(FIELDS) + 1:         # ms + 7 fields
                continue
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                continue
            for name, val in zip(FIELDS, vals[1:]):
                self.pubs[name].publish(Float32(data=val))


def main():
    # plain positional args; rclpy.init() consumes any --ros-args
    port = "/dev/ttyUSB0"
    baud = 115200
    for a in sys.argv[1:]:
        if a.startswith("/dev/"):
            port = a
        elif a.isdigit():
            baud = int(a)
    rclpy.init()
    node = GimbalBridge(port, baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.alive = False
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
