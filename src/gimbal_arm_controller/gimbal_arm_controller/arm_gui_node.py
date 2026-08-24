#!/usr/bin/env python3
"""GimbalArm command console (tkinter).

A button/slider panel so you don't have to type `ros2 topic pub` for bring-up.
Publishes everything the arm_microros firmware listens to and shows the
/joint_states feedback (angles, motor torque, temperature, error/thermal flags).

Topics (domain 5):
  /arm_enable    Bool      AK40 shoulder+elbow active(true)/limp(false)
  /home          Empty     capture current pose as q=0 (pull arm vertical first!)
  /set_kp        Float32   MIT stiffness [Nm/rad]  (0..500)
  /joint_commands JointState  positions[1]=shoulder, [2]=elbow  [rad]
  /level_enable  Bool      wrist_pitch IMU cup-leveling on/off
  /set_wrist_kp  Float32   wrist leveling P gain [V/rad]  (0..40)
  /joint_states  JointState (subscribed)  feedback

Run via:  ./arm_gui.sh   (sources install/ + sets ROS_DOMAIN_ID=5)
"""
import os
os.environ.setdefault("ROS_DOMAIN_ID", "5")

import math
import socket
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32MultiArray
from collections import deque

# joint limits [deg] (mirror joint_config.py / firmware JLIM)
YAW_MIN, YAW_MAX = -90.0, 90.0     # base_yaw DS3240 servo [deg]
SH_MIN, SH_MAX = -90.0, 90.0
EL_MIN, EL_MAX = -120.0, 120.0
WKP_MIN, WKP_MAX = 0.0, 40.0       # pitch leveling P [V/rad]
WKP2_MAX = 60.0                    # roll leveling P (heavier axis, 40->60)
WKD_MIN, WKD_MAX = 0.0, 2.0        # wrist leveling D
DSC_MIN, DSC_MAX = -2.0, 2.0       # DOB authority (sign matters)
DBW_MIN, DBW_MAX = 1.0, 9.0        # DOB observer bandwidth [Hz] (< ~9Hz resonance)
JN_MIN,  JN_MAX  = 0.3, 200.0      # ESO inertia J_nom in 1e-4 kg·m^2 (bare GM4108=0.68; +cup pitch ~60)
MKP_MIN, MKP_MAX = 0.0, 300.0      # AK40 arm MIT stiffness [Nm/rad] (HW range 0..500)
MKD_MIN, MKD_MAX = 0.0, 3.0        # AK40 arm MIT damping [Nm/(rad/s)] (HW range 0..5)
GRAV_MIN, GRAV_MAX = -2.0, 2.0     # gravity-comp tff scale (0=off, ~1 nominal, sign empirical)
SGAIN_MIN, SGAIN_MAX = -2.0, 2.0   # active-suspension authority (sign matters: + rejects, flip if it amplifies)
SHAKER_OFFSET_PER_HZ = 200.0       # excitation table: drive offset per Hz (400->~2Hz; approx, open-loop)
SHAKER_OFFSET_MAX = 987            # servo drive offset cap (firmware constrains 0..987)
JOINT_NAMES = ["base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]


class Shaker:
    """Link to the standalone excitation-table ESP32 (servo_exciter firmware).

    NOT on the ROS graph.  Two transports, same single-char protocol:
      - serial : USB @115200
      - tcp    : WiFi, port 3333 (ESP32 powered from 5V, no USB)
    Commands: c=run fwd  a=run rev  s=stop  y=sync  d=debug  f=flip  1/2=Kp-/+  3/4=Ki-/+
              k=state  m=I2C scan  i=IMU diag  v<int>=drive offset (excitation freq ~off/200 Hz)
    A reader thread parses incoming lines into self.rx; the tk loop drains it (thread-safe).
    """
    def __init__(self):
        self.mode = None                  # 'serial' | 'tcp' | None
        self.ser = None
        self.sock = None
        self.rx = deque(maxlen=400)
        self._buf = ""
        self._stop = False

    def connect_serial(self, port, baud=115200):
        import serial                       # lazy: GUI still runs if pyserial absent
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.mode = "serial"
        self._start_reader()

    def connect_tcp(self, host, port=3333):
        self.disconnect()
        s = socket.create_connection((host, port), timeout=3.0)
        s.settimeout(0.1)
        self.sock = s
        self.mode = "tcp"
        self._start_reader()

    def _start_reader(self):
        self._buf = ""
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def disconnect(self):
        self._stop = True
        for h in (self.ser, self.sock):
            try:
                if h is not None:
                    h.close()
            except Exception:
                pass
        self.ser = self.sock = None
        self.mode = None

    def is_open(self):
        if self.mode == "serial":
            return self.ser is not None and getattr(self.ser, "is_open", False)
        if self.mode == "tcp":
            return self.sock is not None
        return False

    def send(self, s):
        if not self.is_open():
            return
        data = s.encode()
        try:
            if self.mode == "serial":
                self.ser.write(data)
            else:
                self.sock.sendall(data)
        except Exception as e:
            self.rx.append(f"[tx error] {e}")

    def _reader(self):
        while not self._stop:
            try:
                if self.mode == "serial" and self.ser is not None:
                    data = self.ser.read(64)
                elif self.mode == "tcp" and self.sock is not None:
                    try:
                        data = self.sock.recv(256)
                        if data == b"":          # peer closed
                            break
                    except socket.timeout:
                        data = b""
                else:
                    break
            except Exception:
                break
            if data:
                self._buf += data.decode(errors="replace")
                while "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    self.rx.append(line.rstrip("\r"))


class ArmConsole(Node):
    def __init__(self):
        super().__init__("arm_gui")
        self.pub_enable = self.create_publisher(Bool, "/arm_enable", 10)
        self.pub_home = self.create_publisher(Empty, "/home", 10)
        self.pub_cmd = self.create_publisher(JointState, "/joint_commands", 10)
        self.pub_level = self.create_publisher(Bool, "/level_enable", 10)
        self.pub_cfg = self.create_publisher(Float32MultiArray, "/wrist_cfg", 10)
        self.last_state = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        self.last_state = msg

    # --- publish helpers ---
    def send_enable(self, on):
        self.pub_enable.publish(Bool(data=bool(on)))

    def send_home(self):
        self.pub_home.publish(Empty())

    def send_level(self, on):
        self.pub_level.publish(Bool(data=bool(on)))

    def send_cfg(self, kp, kd, dob_scale, dob_bw, dob_on, jnom, kp2, kd2, dob_scale2,
                 mit_kp, mit_kd, grav, susp_gain=0.0, susp_dgain=0.0, susp_en=False, set_offset=0.0):
        # /wrist_cfg = [..8, 9=set_offset trig, 10=MIT Kp, 11=MIT Kd, 12=grav_scale,
        #               13=susp_gain, 14=susp_dgain, 15=susp_enable]  (firmware capacity 16)
        m = Float32MultiArray()
        m.data = [float(kp), float(kd), float(dob_scale), float(dob_bw),
                  1.0 if dob_on else 0.0, float(jnom), float(kp2), float(kd2),
                  float(dob_scale2), float(set_offset), float(mit_kp), float(mit_kd), float(grav),
                  float(susp_gain), float(susp_dgain), 1.0 if susp_en else 0.0]
        self.pub_cfg.publish(m)

    def send_cmd(self, yaw_rad, sh_rad, el_rad):
        m = JointState()
        m.name = JOINT_NAMES
        m.position = [float(yaw_rad), float(sh_rad), float(el_rad), 0.0, 0.0]
        self.pub_cmd.publish(m)


class Gui:
    def __init__(self, node):
        self.node = node
        self.arm_on = False
        self.level_on = False
        self.susp_on = False                      # active-suspension master enable
        self.shaker = Shaker()                    # excitation-table serial link (raw, not ROS)

        self.root = tk.Tk()                       # must exist before any tk.*Var
        self.live = tk.BooleanVar(value=True)
        self._dragging = False                    # user holding a cmd slider -> pause feedback-sync
        self._syncing  = False                    # programmatic slider set -> suppress publish
        self.disp_hist  = deque(maxlen=200)       # tip displacement d_u history [mm] (active-susp strip chart)
        self.disp_scale = 0.1                      # mm, peak-hold auto-scale (snap up, decay slow -> shrink stays visible)
        self.root.title("GimbalArm Console (domain 5)")
        self.root.configure(padx=10, pady=10)
        self._build()
        self.root.after(100, self._refresh)

    def _build(self):
        r = self.root

        # ---- AK40 arm ----
        af = ttk.LabelFrame(r, text="arm (base_yaw servo / shoulder / elbow)")
        af.grid(row=0, column=0, sticky="ew", pady=4)

        self.b_enable = tk.Button(af, text="ARM: LIMP", width=14, bg="#ccc",
                                  command=self._toggle_arm)
        self.b_enable.grid(row=0, column=0, padx=4, pady=4)
        tk.Button(af, text="HOME (vertical!)", command=self.node.send_home,
                  bg="#ffe0a0").grid(row=0, column=1, padx=4)
        ttk.Label(af, text="base_yaw [deg]").grid(row=1, column=0, sticky="w", padx=4)
        self.yaw = tk.DoubleVar(value=0.0)
        self.yaw_scale = ttk.Scale(af, from_=YAW_MIN, to=YAW_MAX, variable=self.yaw, length=220,
                  command=lambda e: self._on_slider())
        self.yaw_scale.grid(row=1, column=1, columnspan=2, sticky="ew")
        self.yaw_lbl = ttk.Label(af, text="0")
        self.yaw_lbl.grid(row=1, column=3, padx=4)

        ttk.Label(af, text="shoulder [deg]").grid(row=2, column=0, sticky="w", padx=4)
        self.sh = tk.DoubleVar(value=0.0)
        self.sh_scale = ttk.Scale(af, from_=SH_MIN, to=SH_MAX, variable=self.sh, length=220,
                  command=lambda e: self._on_slider())
        self.sh_scale.grid(row=2, column=1, columnspan=2, sticky="ew")
        self.sh_lbl = ttk.Label(af, text="0")
        self.sh_lbl.grid(row=2, column=3, padx=4)

        ttk.Label(af, text="elbow [deg]").grid(row=3, column=0, sticky="w", padx=4)
        self.el = tk.DoubleVar(value=0.0)
        self.el_scale = ttk.Scale(af, from_=EL_MIN, to=EL_MAX, variable=self.el, length=220,
                  command=lambda e: self._on_slider())
        self.el_scale.grid(row=3, column=1, columnspan=2, sticky="ew")
        for s in (self.yaw_scale, self.sh_scale, self.el_scale):   # pause feedback-sync while user drags
            s.bind("<ButtonPress-1>",   lambda e: setattr(self, "_dragging", True))
            s.bind("<ButtonRelease-1>", lambda e: setattr(self, "_dragging", False))
        self.el_lbl = ttk.Label(af, text="0")
        self.el_lbl.grid(row=3, column=3, padx=4)

        ttk.Checkbutton(af, text="live send", variable=self.live).grid(row=4, column=0, padx=4)
        tk.Button(af, text="SEND cmd", command=self._send_cmd).grid(row=4, column=1, padx=4, pady=4)
        tk.Button(af, text="zero sliders", command=self._zero).grid(row=4, column=2, padx=4)
        tk.Button(af, text="IMU zero @ VERTICAL (calib)", command=self._set_imu_offset).grid(
            row=5, column=0, columnspan=3, padx=4, pady=2, sticky="ew")
        tk.Button(af, text="wrist abs-zero @ here (save)", command=self._wrist_zero).grid(
            row=6, column=0, columnspan=3, padx=4, pady=2, sticky="ew")

        # ---- wrist leveling + ESO/DOB ----
        wf = ttk.LabelFrame(r, text="wrist_pitch IMU leveling + ESO/DOB")
        wf.grid(row=1, column=0, sticky="ew", pady=4)
        self.b_level = tk.Button(wf, text="LEVEL: OFF", width=14, bg="#ccc",
                                 command=self._toggle_level)
        self.b_level.grid(row=0, column=0, padx=4, pady=4)
        self.dob = tk.BooleanVar(value=True)
        ttk.Checkbutton(wf, text="DOB comp", variable=self.dob,
                        command=self._send_cfg).grid(row=0, column=1, columnspan=2, padx=4)

        # cfg sliders -> /wrist_cfg [kp, kd, dob_scale, dob_bw, dob_on, J(e-4)]
        self.wkp  = tk.DoubleVar(value=10.0)   # HW-tuned 2026-06-24 (stable w/ DOB)
        self.wkd  = tk.DoubleVar(value=0.25)
        self.wkp2 = tk.DoubleVar(value=40.0)   # roll (heavier axis) -- higher than pitch
        self.wkd2 = tk.DoubleVar(value=0.49)
        self.dsc  = tk.DoubleVar(value=-0.43)  # pitch DOB auth. NEG = disturbance rejection (textbook)
        self.dsc2 = tk.DoubleVar(value=0.0)    # roll DOB auth (separate; 0=off, roll diverged at -0.43)
        self.dbw  = tk.DoubleVar(value=6.78)
        self.jnom = tk.DoubleVar(value=0.68)   # J in 1e-4 kg·m^2 (bare GM4108); raise for cup
        self.mkp  = tk.DoubleVar(value=120.0)  # AK40 arm stiffness [Nm/rad] (impedance)
        self.mkd  = tk.DoubleVar(value=0.6)    # AK40 arm damping [Nm/(rad/s)]
        self.grav = tk.DoubleVar(value=0.0)    # gravity-comp tff scale (0=off; tune ~1, sign empirical)
        self.sgain  = tk.DoubleVar(value=0.0)  # active-susp displacement-cancel authority (0=off)
        self.sdgain = tk.DoubleVar(value=0.0)  # active-susp sky-hook velocity-damping authority
        rows = [("MIT Kp arm",     self.mkp,  MKP_MIN, MKP_MAX),
                ("MIT Kd arm",     self.mkd,  MKD_MIN, MKD_MAX),
                ("grav comp",      self.grav, GRAV_MIN, GRAV_MAX),
                ("Kp pitch",       self.wkp,  WKP_MIN, WKP_MAX),
                ("Kd pitch",       self.wkd,  WKD_MIN, WKD_MAX),
                ("Kp roll",        self.wkp2, WKP_MIN, WKP2_MAX),
                ("Kd roll",        self.wkd2, WKD_MIN, WKD_MAX),
                ("DOB auth pitch", self.dsc,  DSC_MIN, DSC_MAX),
                ("DOB auth roll",  self.dsc2, DSC_MIN, DSC_MAX),
                ("DOB bw[Hz]",     self.dbw,  DBW_MIN, DBW_MAX),
                ("J [e-4]",        self.jnom, JN_MIN,  JN_MAX)]
        self.cfg_lbls = {}
        for i, (name, var, lo, hi) in enumerate(rows, start=1):
            ttk.Label(wf, text=name).grid(row=i, column=0, sticky="w", padx=4)
            ttk.Scale(wf, from_=lo, to=hi, variable=var, length=180,
                      command=lambda e: self._send_cfg()).grid(row=i, column=1,
                                                               columnspan=2, sticky="ew")
            lbl = ttk.Label(wf, text=f"{var.get():.2f}")
            lbl.grid(row=i, column=3, padx=4)
            self.cfg_lbls[name] = (lbl, var)

        # ---- active suspension (harmonic excitation -> tip-displacement rejection) ----
        # moves shoulder+elbow to hold the tip still in world frame (band-pass cup accel ->
        # damped 2R Jacobian).  OFF by default; ramp gain on HW, flip sign if it amplifies.
        xf = ttk.LabelFrame(r, text="active suspension (tip-displacement rejection : shoulder+elbow)")
        xf.grid(row=3, column=0, columnspan=2, sticky="ew", pady=4)
        self.b_susp = tk.Button(xf, text="SUSP: OFF", width=14, bg="#ccc",
                                command=self._toggle_susp)
        self.b_susp.grid(row=0, column=0, rowspan=2, padx=4, pady=4)
        ttk.Label(xf, text="susp gain (disp)").grid(row=0, column=1, sticky="w", padx=4)
        ttk.Scale(xf, from_=SGAIN_MIN, to=SGAIN_MAX, variable=self.sgain, length=220,
                  command=lambda e: self._send_cfg()).grid(row=0, column=2, sticky="ew")
        self.sgain_lbl = ttk.Label(xf, text="0.00")
        self.sgain_lbl.grid(row=0, column=3, padx=4)
        ttk.Label(xf, text="susp dgain (sky-hook)").grid(row=1, column=1, sticky="w", padx=4)
        ttk.Scale(xf, from_=SGAIN_MIN, to=SGAIN_MAX, variable=self.sdgain, length=220,
                  command=lambda e: self._send_cfg()).grid(row=1, column=2, sticky="ew")
        self.sdgain_lbl = ttk.Label(xf, text="0.00")
        self.sdgain_lbl.grid(row=1, column=3, padx=4)
        self.susp_lbl = ttk.Label(xf, text="susp off | tip d_u --  v_u -- | off sh -- el --",
                                  font=("monospace", 9))
        self.susp_lbl.grid(row=2, column=0, columnspan=4, sticky="w", padx=6, pady=2)

        # ---- E-STOP ----
        tk.Button(r, text="STOP ALL (limp + level off)", bg="#e04030", fg="white",
                  font=("TkDefaultFont", 11, "bold"), command=self._estop).grid(
            row=2, column=0, sticky="ew", pady=6)

        # ---- feedback ----
        ff = ttk.LabelFrame(r, text="/joint_states feedback")
        ff.grid(row=2, column=1, sticky="new", padx=(12, 0), pady=4)
        self.fb = tk.Text(ff, width=52, height=9, font=("monospace", 9))
        self.fb.grid(row=0, column=0, padx=4, pady=4)
        self.fb.insert("1.0", "waiting for /joint_states ...")
        self.fb.config(state="disabled")

        # ---- tip displacement d_u (active-susp output) strip chart [mm] ----
        # replaces transmissibility: ratio is unreliable here (accel/all-axis, and the shoulder
        # excitation IMU gets contaminated once suspension moves the joint).  d_u (effort[16]) =
        # band-passed in-plane tip displacement = exactly what the controller drives toward 0.
        tf = ttk.LabelFrame(r, text="tip displacement d_u (active-susp output) [mm] -- shrinks as gain rises")
        tf.grid(row=1, column=1, sticky="new", padx=(12, 0), pady=4)
        self.tr_canvas = tk.Canvas(tf, width=360, height=130, bg="white",
                                   highlightthickness=1, highlightbackground="#bbb")
        self.tr_canvas.grid(row=0, column=0, padx=4, pady=4)
        self.tr_ratio = ttk.Label(tf, text="d_u --", font=("TkDefaultFont", 11, "bold"))
        self.tr_ratio.grid(row=1, column=0, sticky="w", padx=6, pady=2)

        # ---- excitation table (standalone servo_exciter ESP32, raw serial -- NOT ROS) ----
        # drives the base plate at a fixed ~2Hz; read excit/response/ratio in the panel above.
        sf = ttk.LabelFrame(r, text="excitation table (servo_exciter ESP32 -- USB/WiFi, ~off/200 Hz)")
        sf.grid(row=0, column=1, sticky="new", padx=(12, 0), pady=4)
        cf = ttk.Frame(sf); cf.grid(row=0, column=0, columnspan=3, sticky="w", padx=4, pady=2)
        ttk.Label(cf, text="USB").pack(side="left")
        self.shaker_port = tk.StringVar(value="/dev/ttyUSB0")   # ESP32 = ttyUSB* (arm Teensy = ttyACM0)
        ttk.Entry(cf, textvariable=self.shaker_port, width=12).pack(side="left", padx=(2, 2))
        tk.Button(cf, text="connect", command=self._shaker_connect_serial).pack(side="left")
        ttk.Label(cf, text="   WiFi").pack(side="left")
        self.shaker_ip = tk.StringVar(value="192.168.0.50")     # = ESP32 IP printed on its serial boot (STA)
        ttk.Entry(cf, textvariable=self.shaker_ip, width=14).pack(side="left", padx=(2, 2))
        tk.Button(cf, text="connect", command=self._shaker_connect_wifi).pack(side="left")
        tk.Button(cf, text="disconnect", command=self._shaker_disconnect).pack(side="left", padx=(8, 0))
        self.shaker_status = ttk.Label(cf, text="off", foreground="#888")
        self.shaker_status.pack(side="left", padx=(8, 0))
        # run controls (momentary -- firmware latches the mode)
        tk.Button(sf, text="START (fwd)", bg="#a0d8a0", width=11,
                  command=lambda: self._shaker_cmd("c")).grid(row=1, column=0, padx=4, pady=2)
        tk.Button(sf, text="STOP", bg="#e0a0a0", width=11,
                  command=lambda: self._shaker_cmd("s")).grid(row=1, column=1, padx=4, pady=2)
        tk.Button(sf, text="reverse (a)", width=11,
                  command=lambda: self._shaker_cmd("a")).grid(row=1, column=2, padx=4, pady=2)
        tk.Button(sf, text="sync (y)", width=9,
                  command=lambda: self._shaker_cmd("y")).grid(row=2, column=0, padx=4)
        tk.Button(sf, text="state (k)", width=9,
                  command=lambda: self._shaker_cmd("k")).grid(row=2, column=1, padx=4)
        tk.Button(sf, text="flip (f)", width=9,
                  command=lambda: self._shaker_cmd("f")).grid(row=2, column=2, padx=4)
        kf = ttk.Frame(sf); kf.grid(row=3, column=0, columnspan=3, sticky="w", padx=4, pady=2)
        ttk.Label(kf, text="sync Kp").pack(side="left")
        tk.Button(kf, text="-", width=2, command=lambda: self._shaker_cmd("1")).pack(side="left")
        tk.Button(kf, text="+", width=2, command=lambda: self._shaker_cmd("2")).pack(side="left", padx=(0, 8))
        ttk.Label(kf, text="Ki").pack(side="left")
        tk.Button(kf, text="-", width=2, command=lambda: self._shaker_cmd("3")).pack(side="left")
        tk.Button(kf, text="+", width=2, command=lambda: self._shaker_cmd("4")).pack(side="left")
        ttk.Label(sf, text="freq [Hz]").grid(row=4, column=0, sticky="w", padx=4)
        self.shaker_hz = tk.DoubleVar(value=2.0)
        ttk.Entry(sf, textvariable=self.shaker_hz, width=8).grid(row=4, column=1, sticky="w")
        tk.Button(sf, text="set freq", command=self._shaker_setfreq).grid(row=4, column=2, padx=4, pady=2)
        self.shaker_log = tk.Text(sf, width=52, height=5, font=("monospace", 8))
        self.shaker_log.grid(row=5, column=0, columnspan=3, padx=4, pady=4)
        self.shaker_log.insert("1.0", "(not connected -- set port, click connect)\n")
        self.shaker_log.config(state="disabled")

    # ---- actions ----
    def _toggle_arm(self):
        self.arm_on = not self.arm_on
        self.node.send_enable(self.arm_on)
        self.b_enable.config(text="ARM: ACTIVE" if self.arm_on else "ARM: LIMP",
                             bg="#80d080" if self.arm_on else "#ccc")

    def _toggle_level(self):
        self.level_on = not self.level_on
        self.node.send_level(self.level_on)
        self.b_level.config(text="LEVEL: ON" if self.level_on else "LEVEL: OFF",
                            bg="#80d080" if self.level_on else "#ccc")

    def _toggle_susp(self):
        self.susp_on = not self.susp_on
        self.b_susp.config(text="SUSP: ON" if self.susp_on else "SUSP: OFF",
                           bg="#80d080" if self.susp_on else "#ccc")
        self._send_cfg()                          # push susp_enable + current gains (ON = firmware reseeds)

    def _estop(self):
        self.arm_on = False
        self.level_on = False
        self.susp_on = False
        self.node.send_enable(False)
        self.node.send_level(False)
        self._send_cfg()                          # push susp_enable=0 (active suspension off)
        self.b_enable.config(text="ARM: LIMP", bg="#ccc")
        self.b_level.config(text="LEVEL: OFF", bg="#ccc")
        self.b_susp.config(text="SUSP: OFF", bg="#ccc")

    def _send_cfg(self):
        for lbl, var in self.cfg_lbls.values():
            lbl.config(text=f"{var.get():.2f}")
        self.sgain_lbl.config(text=f"{self.sgain.get():.2f}")
        self.sdgain_lbl.config(text=f"{self.sdgain.get():.2f}")
        self.node.send_cfg(self.wkp.get(), self.wkd.get(), self.dsc.get(),
                           self.dbw.get(), self.dob.get(), self.jnom.get() * 1e-4,
                           self.wkp2.get(), self.wkd2.get(), self.dsc2.get(),
                           self.mkp.get(), self.mkd.get(), self.grav.get(),
                           self.sgain.get(), self.sdgain.get(), self.susp_on)

    def _on_slider(self):
        self.yaw_lbl.config(text=f"{self.yaw.get():.0f}")
        self.sh_lbl.config(text=f"{self.sh.get():.0f}")
        self.el_lbl.config(text=f"{self.el.get():.0f}")
        if self.live.get() and not self._syncing:   # _syncing = programmatic set from feedback, don't publish
            self._send_cmd()

    def _send_cmd(self):
        self.node.send_cmd(math.radians(self.yaw.get()), math.radians(self.sh.get()), math.radians(self.el.get()))

    def _zero(self):
        self.yaw.set(0.0)
        self.sh.set(0.0)
        self.el.set(0.0)
        self._on_slider()

    def _set_imu_offset(self):   # one-shot: AK40 joint zero from IMU gravity-angle (/wrist_cfg[9]=1)
        self.node.send_cfg(self.wkp.get(), self.wkd.get(), self.dsc.get(),
                           self.dbw.get(), self.dob.get(), self.jnom.get() * 1e-4,
                           self.wkp2.get(), self.wkd2.get(), self.dsc2.get(),
                           self.mkp.get(), self.mkd.get(), self.grav.get(),
                           self.sgain.get(), self.sdgain.get(), self.susp_on, set_offset=1.0)

    def _wrist_zero(self):       # capture wrist encoders as joint q=0 + persist EEPROM (/wrist_cfg[9]=2)
        self.node.send_cfg(self.wkp.get(), self.wkd.get(), self.dsc.get(),
                           self.dbw.get(), self.dob.get(), self.jnom.get() * 1e-4,
                           self.wkp2.get(), self.wkd2.get(), self.dsc2.get(),
                           self.mkp.get(), self.mkd.get(), self.grav.get(),
                           self.sgain.get(), self.sdgain.get(), self.susp_on, set_offset=2.0)

    # ---- excitation table (serial / WiFi) ----
    def _shaker_connect_serial(self):
        port = self.shaker_port.get().strip()
        try:
            self.shaker.connect_serial(port)
            self._shaker_set_status(f"USB {port}")
        except Exception as e:
            self._shaker_log(f"[USB connect FAIL] {e}")

    def _shaker_connect_wifi(self):
        ip = self.shaker_ip.get().strip()
        try:
            self.shaker.connect_tcp(ip, 3333)
            self._shaker_set_status(f"WiFi {ip}:3333")
        except Exception as e:
            self._shaker_log(f"[WiFi connect FAIL] {e}")

    def _shaker_disconnect(self):
        try:
            if self.shaker.is_open():
                self.shaker.send("s")             # stop before dropping the link
        except Exception:
            pass
        self.shaker.disconnect()
        self._shaker_set_status("off")

    def _shaker_set_status(self, s):
        self.shaker_status.config(text=s, foreground=("#080" if s != "off" else "#888"))
        self._shaker_log(f"[{s}]")

    def _shaker_cmd(self, ch):
        if not self.shaker.is_open():
            self._shaker_log("[not connected]")
            return
        self.shaker.send(ch)

    def _shaker_log(self, s):
        self.shaker_log.config(state="normal")
        self.shaker_log.insert("end", s + "\n")
        if int(self.shaker_log.index("end-1c").split(".")[0]) > 200:   # keep bounded
            self.shaker_log.delete("1.0", "100.0")
        self.shaker_log.see("end")
        self.shaker_log.config(state="disabled")

    def _shaker_setfreq(self):
        if not self.shaker.is_open():
            self._shaker_log("[not connected]")
            return
        try:
            hz = float(self.shaker_hz.get())
        except Exception:
            self._shaker_log("[bad freq]")
            return
        off = max(0, min(SHAKER_OFFSET_MAX, int(round(hz * SHAKER_OFFSET_PER_HZ))))
        self.shaker.send(f"v{off}\n")        # firmware 'v<int>' sets drive offset (speed)
        self._shaker_log(f"[set ~{hz:.2f}Hz -> offset {off}] verify actual via transmissibility")

    def _draw_tr(self):                            # tip-displacement strip chart (signed d_u [mm], zero midline)
        c = self.tr_canvas; c.delete("all")
        w = int(c["width"]); h = int(c["height"])
        mid = h / 2.0
        c.create_line(3, mid, w - 3, mid, fill="#ddd")          # zero line
        d = list(self.disp_hist)
        if len(d) < 2:
            return
        pk = max(abs(x) for x in d)
        # peak-hold auto-scale: snap up, decay slow (floor 0.05mm) so a SHRINKING envelope stays
        # visible a few seconds instead of the chart instantly re-zooming to fill.
        self.disp_scale = max(pk, self.disp_scale * 0.98, 0.05)
        n = len(d)
        flat = []
        for i in range(n):
            flat += [i * (w - 6) / (n - 1) + 3, mid - (d[i] / self.disp_scale) * (mid - 6)]
        c.create_line(*flat, fill="#d62728", width=2)
        c.create_text(5, 8, anchor="w", text=f"scale +/-{self.disp_scale:.2f} mm", fill="#999",
                      font=("TkDefaultFont", 7))

    def _refresh(self):
        st = self.node.last_state
        if st is not None:
            def g(arr, i, d=0.0):
                return arr[i] if len(arr) > i else d
            yaw = math.degrees(g(st.position, 0))
            sh = math.degrees(g(st.position, 1))
            el = math.degrees(g(st.position, 2))
            wp = math.degrees(g(st.position, 3))
            # keep cmd sliders synced to actual joint angle (unless user is dragging) so a
            # nudge moves RELATIVE to where the arm is -- no jump after HOME / e-stop / desync.
            if not self._dragging:
                self._syncing = True
                self.yaw.set(yaw); self.sh.set(sh); self.el.set(el)
                self.yaw_lbl.config(text=f"{yaw:.0f}")
                self.sh_lbl.config(text=f"{sh:.0f}"); self.el_lbl.config(text=f"{el:.0f}")
                self._syncing = False
            tau_sh = g(st.effort, 1)
            tau_el = g(st.effort, 2)
            # velocity field packs telemetry: [flags, temp_sh, temp_el, err_sh, err_el]
            flags = int(g(st.velocity, 0))
            t_sh, t_el = g(st.velocity, 1), g(st.velocity, 2)
            e_sh, e_el = int(g(st.velocity, 3)), int(g(st.velocity, 4))
            warn = []
            if flags & 1: warn.append("SH THERMAL-LIMP")
            if flags & 2: warn.append("EL THERMAL-LIMP")
            if flags & 4: warn.append("sh warn")
            if flags & 8: warn.append("el warn")
            if flags & 16: warn.append("SH POS-ESTOP")
            if flags & 32: warn.append("EL POS-ESTOP")
            # wrist: effort[0]=status bits, [3]=est_phi, [4]=vpd
            wst = int(g(st.effort, 0))
            wphi = math.degrees(g(st.effort, 3))
            wvpd = g(st.effort, 4)
            wflag = (f"FOC:{'ok' if wst & 1 else 'X'} | IMU:{'ok' if wst & 2 else 'X'} | "
                     f"level:{'ON' if wst & 4 else 'off'} | {'DOB' if wst & 8 else 'PD'}")
            q1i = math.degrees(g(st.effort, 14)); q2i = math.degrees(g(st.effort, 15))
            mpu = f"MPU sh:{'ok' if wst & 32 else 'X'} el:{'ok' if wst & 64 else 'X'}"
            txt = (f"base_yaw : {yaw:7.1f} deg   (DS3240 servo, open-loop)\n"
                   f"shoulder : {sh:7.1f} deg   tau {tau_sh:+6.2f} Nm  "
                   f"temp {t_sh:3.0f}C err {e_sh}\n"
                   f"elbow    : {el:7.1f} deg   tau {tau_el:+6.2f} Nm  "
                   f"temp {t_el:3.0f}C err {e_el}\n"
                   f"wrist_p  : {wp:7.1f} deg   [{wflag}]\n"
                   f"  tilt phi {wphi:+6.1f} deg   vpd {wvpd:+6.2f} V\n"
                   f"IMU q  : sh {q1i:+6.1f}  el {q2i:+6.1f} deg  [{mpu}]\n"
                   f"\nflags: {', '.join(warn) if warn else 'ok'}")
            self.fb.config(state="normal")
            self.fb.delete("1.0", "end")
            self.fb.insert("1.0", txt)
            self.fb.config(state="disabled")
            # active-suspension telemetry: effort[16]=tip disp_u[m] [17]sh off [18]el off [19]vel_u, bit128=on
            du = g(st.effort, 16); vu = g(st.effort, 19)
            soff_sh = math.degrees(g(st.effort, 17)); soff_el = math.degrees(g(st.effort, 18))
            self.susp_lbl.config(
                text=f"susp {'ON' if wst & 128 else 'off'} | tip d_u {du * 1000:+6.1f} mm  "
                     f"v_u {vu * 1000:+6.0f} mm/s | off sh {soff_sh:+.2f} el {soff_el:+.2f} deg")
            # tip-displacement strip chart [mm] -- active-susp output; watch the envelope shrink as gain rises
            self.disp_hist.append(du * 1000.0)
            self._draw_tr()
            excit = g(st.effort, 11)                                # excitation accel RMS (confirm input present)
            pk = max((abs(x) for x in self.disp_hist), default=0.0)
            self.tr_ratio.config(text=f"d_u peak {pk:5.2f} mm   |   excit {excit:.3f} m/s2")
        # drain excitation-table serial RX (reader thread -> tk main thread) into its log
        while self.shaker.rx:
            self._shaker_log(self.shaker.rx.popleft())
        self.root.after(100, self._refresh)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = ArmConsole()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    gui = Gui(node)
    try:
        gui.run()
    finally:
        # safety: limp arm + leveling off on window close
        node.send_enable(False)
        node.send_level(False)
        try:                                       # stop + release the excitation table
            if gui.shaker.is_open():
                gui.shaker.send("s")
                gui.shaker.disconnect()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
