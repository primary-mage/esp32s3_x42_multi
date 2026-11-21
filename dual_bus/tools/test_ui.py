#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESP32-S3 双总线电机 · 测试上位机

用途：调试单台电机的运动模式（位置/速度）、参数取值范围，以及两路 IO 状态。
不涉及任何机构逻辑——不自动回零、不做软限位、不做插补。

全局 ID：A 路（UART1, GPIO4/5）→ 1..255；B 路（UART2, GPIO8/9）→ 1001..1255

运行：
    python3 tools/test_ui.py
    python3 tools/test_ui.py --port /dev/ttyACM0
"""
from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先运行：python3 -m pip install -r requirements.txt")

from port_utils import default_port, list_serial_ports

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
ID_IN_TEXT = re.compile(r"ID\s*(\d+)", re.I)

BAUDRATE = 115200
POLL_MS = 500          # 状态轮询周期
IO_POLL_MS = 200       # IO 轮询周期


def id_to_name(mid: int) -> str:
    """全局 ID -> 友好名：1..255 → A1..，1001..1255 → B1.."""
    return f"B{mid - 1000}" if mid >= 1000 else f"A{mid}"


class LinkError(Exception):
    pass


# ───────────────────────── 串口客户端 ─────────────────────────
class MotorClient:
    def __init__(self, log=None):
        self.ser: serial.Serial | None = None
        self.lock = threading.Lock()
        self.timeout = 3.0
        self.log = log or (lambda _s: None)

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str) -> None:
        self.close()
        self.ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.05,
                                 write_timeout=1.0)
        time.sleep(0.35)              # 等 USB 枚举稳定
        self.ser.reset_input_buffer()

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def transact(self, cmd: str, timeout: float | None = None) -> list[str]:
        """发一行命令，返回 CMD> 之后、DONE 之前的响应行。"""
        if not self.is_open:
            raise ConnectionError("串口未连接")
        timeout = timeout or self.timeout
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode("ascii"))
            self.ser.flush()
            buf = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                chunk = self.ser.read(512)
                if chunk:
                    buf += chunk
                    if b"CMD> DONE" in buf:
                        break
        text = ANSI_RE.sub("", buf.decode("utf-8", "replace"))
        out = []
        for line in text.replace("\r", "").split("\n"):
            line = line.strip()
            if line.startswith("CMD>"):
                body = line[4:].strip()
                if body and body not in ("DONE", "READY"):
                    out.append(body)
        return out

    # ── 具体命令 ──
    def scan(self, lo: int = 1, hi: int = 16, timeout: float = 8.0) -> list[dict]:
        motors = []
        for body in self.transact(f"SCAN {lo} {hi}", timeout=timeout):
            p = body.split()
            if len(p) == 6 and p[0] == "FOUND":
                motors.append(dict(id=int(p[1]), bus=p[2], addr=int(p[3]),
                                   fw=int(p[4]), hw=int(p[5])))
        return motors

    def motors(self) -> list[dict]:
        motors = []
        for body in self.transact("MOTORS", timeout=2.0):
            p = body.split()
            if len(p) == 6 and p[0] == "FOUND":
                motors.append(dict(id=int(p[1]), bus=p[2], addr=int(p[3]),
                                   fw=int(p[4]), hw=int(p[5])))
        return motors

    def status(self, ident: int) -> dict | None:
        for body in self.transact(f"STAT {ident}", timeout=1.5):
            p = body.split()
            if len(p) == 8 and p[0] == "STAT":
                return dict(id=int(p[1]), en=int(p[2]), inpos=int(p[3]),
                            stall=int(p[4]), clog=int(p[5]),
                            pos=int(p[6]), speed=int(p[7]))
        return None

    def io(self) -> dict | None:
        for body in self.transact("IO?", timeout=1.5):
            p = body.split()
            # IO IN 15 <lv> OUT 16 <lv>
            if len(p) == 7 and p[0] == "IO" and p[1] == "IN":
                return dict(in_pin=int(p[2]), in_level=int(p[3]),
                            out_pin=int(p[5]), out_level=int(p[6]))
        return None

    def io_out(self, level: int) -> int:
        for body in self.transact(f"IO OUT {level}", timeout=1.5):
            if body.startswith("OK IO OUT"):
                return int(body.split()[-1])
            if body.startswith("ERR"):
                raise LinkError(body)
        raise LinkError("设置 MOS 无应答")

    def _check(self, cmd: str, ok_prefix: str = "", timeout: float = 3.0, **kw) -> None:
        for body in self.transact(cmd, timeout=timeout, **kw):
            if body.startswith("ERR"):
                raise LinkError(body)

    def pos(self, ident, cw, pulses, rpm, accel, absolute, sync=False):
        d = "CW" if cw else "CCW"
        m = "ABS" if absolute else "REL"
        cmd = f"POS {ident} {d} {int(pulses)} {int(rpm)} {int(accel)} {m}"
        if sync:
            cmd += " SYNC"
        self._check(cmd, "OK POS")

    def go(self):
        self._check("GO", "OK GO")

    def spd(self, ident, cw, rpm, accel):
        self._check(f"SPD {ident} {'CW' if cw else 'CCW'} {int(rpm)} {int(accel)}")

    def stop(self, ident=0):
        self._check(f"STP {ident}")

    def enable(self, ident, on):
        self._check(f"ENA {ident} {1 if on else 0}")

    def zero(self, ident):
        self._check(f"ZERO {ident}")

    def release(self, ident):
        self._check(f"REL {ident}")


# ───────────────────────── 界面 ─────────────────────────
class App(tk.Tk):
    def __init__(self, port: str | None = None):
        super().__init__()
        self.title("双总线电机测试台")
        self.geometry("640x860")
        self.minsize(600, 780)

        self.client = MotorClient(log=self.log)
        self.q: queue.Queue = queue.Queue()
        self.motors: list[dict] = []
        self.current_id: int | None = None   # 主线程维护，轮询线程只读它
        self.connected = False
        self._poll_stop = threading.Event()
        self._poll_thread: threading.Thread | None = None

        self._build()
        self.refresh_ports()
        self.after(60, self._drain)
        if port:
            self.port_var.set(port)

    # ---------- 布局 ----------
    def _build(self):
        pad = dict(padx=10, pady=5)
        big = ("Sans", 11)

        # 连接
        conn = ttk.LabelFrame(self, text=" 连接 ")
        conn.pack(fill="x", **pad)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, state="readonly")
        self.port_box.grid(row=0, column=0, padx=(10, 4), pady=(10, 4), sticky="we")
        ttk.Button(conn, text="刷新", width=6, command=self.refresh_ports).grid(
            row=0, column=1, padx=2, pady=(10, 4))
        self.btn_conn = ttk.Button(conn, text="连接", width=8, command=self.on_connect)
        self.btn_conn.grid(row=0, column=2, padx=(2, 10), pady=(10, 4))
        self.lbl_online = ttk.Label(conn, text="未连接", foreground="#666")
        self.lbl_online.grid(row=1, column=0, columnspan=3, padx=10, pady=(0, 10),
                             sticky="w")
        conn.columnconfigure(0, weight=1)

        # 目标电机
        mf = ttk.LabelFrame(self, text=" 目标电机 ")
        mf.pack(fill="x", **pad)
        ttk.Label(mf, text="电机").grid(row=0, column=0, padx=(10, 4), pady=8)
        self.motor_var = tk.StringVar()
        self.motor_box = ttk.Combobox(mf, textvariable=self.motor_var, width=18)
        self.motor_box.grid(row=0, column=1, padx=4, pady=8, sticky="w")
        self.motor_box.bind("<<ComboboxSelected>>", lambda _e: self.on_motor_changed())
        ttk.Button(mf, text="重新扫描", command=self.on_scan).grid(
            row=0, column=2, padx=6, pady=8)
        self.lbl_where = ttk.Label(mf, text="—", foreground="#666")
        self.lbl_where.grid(row=0, column=3, padx=(4, 10), pady=8, sticky="w")
        mf.columnconfigure(3, weight=1)

        # 状态行
        st = ttk.Frame(mf)
        st.grid(row=1, column=0, columnspan=4, padx=10, pady=(0, 6), sticky="we")
        self.lbl_status = tk.Label(st, text="位置 --   转速 --   到位 --   堵转 --",
                                   anchor="w", font=big,
                                   bg="#2b2b2b", fg="#f0f0f0", padx=8, pady=6)
        self.lbl_status.pack(fill="x")

        btns = ttk.Frame(mf)
        btns.grid(row=2, column=0, columnspan=4, padx=10, pady=(0, 10), sticky="we")
        for i, (txt, fn) in enumerate((
                ("使能", lambda: self.on_enable(True)),
                ("失能", lambda: self.on_enable(False)),
                ("位置清零", self.on_zero),
                ("解除堵转", self.on_release))):
            b = ttk.Button(btns, text=txt, command=fn)
            b.pack(side="left", expand=True, fill="x", padx=2)
        self.motor_btns = btns.winfo_children()

        # 位置控制
        pf = ttk.LabelFrame(self, text=" 位置控制 ")
        pf.pack(fill="x", **pad)
        ttk.Label(pf, text="方向").grid(row=0, column=0, padx=(10, 2), pady=6, sticky="w")
        self.pos_dir = tk.StringVar(value="CW")
        ttk.Radiobutton(pf, text="CW", value="CW", variable=self.pos_dir).grid(
            row=0, column=1, sticky="w")
        ttk.Radiobutton(pf, text="CCW", value="CCW", variable=self.pos_dir).grid(
            row=0, column=2, sticky="w")

        ttk.Label(pf, text="脉冲").grid(row=0, column=3, padx=(10, 2), sticky="e")
        self.pos_pulses = tk.StringVar(value="3200")
        ttk.Entry(pf, textvariable=self.pos_pulses, width=9).grid(row=0, column=4, sticky="w")

        ttk.Label(pf, text="转速 rpm").grid(row=1, column=0, padx=(10, 2), pady=6, sticky="w")
        self.pos_rpm = tk.StringVar(value="30")
        ttk.Entry(pf, textvariable=self.pos_rpm, width=9).grid(row=1, column=1, sticky="w")

        ttk.Label(pf, text="加速档").grid(row=1, column=3, padx=(10, 2), sticky="e")
        self.pos_accel = tk.StringVar(value="10")
        ttk.Entry(pf, textvariable=self.pos_accel, width=9).grid(row=1, column=4, sticky="w")

        ttk.Label(pf, text="模式").grid(row=2, column=0, padx=(10, 2), pady=6, sticky="w")
        self.pos_mode = tk.StringVar(value="ABS")
        ttk.Radiobutton(pf, text="ABS 绝对", value="ABS", variable=self.pos_mode).grid(
            row=2, column=1, columnspan=2, sticky="w")
        ttk.Radiobutton(pf, text="REL 相对", value="REL", variable=self.pos_mode).grid(
            row=2, column=3, sticky="w")
        self.btn_pos = tk.Button(pf, text="执行 POS", command=self.on_pos,
                                 height=2, bg="#1976d2", fg="white",
                                 activebackground="#1565c0", activeforeground="white")
        self.btn_pos.grid(row=0, column=5, rowspan=3, padx=10, pady=6, sticky="ns")

        # 速度控制
        sf = ttk.LabelFrame(self, text=" 速度控制 ")
        sf.pack(fill="x", **pad)
        ttk.Label(sf, text="方向").grid(row=0, column=0, padx=(10, 2), pady=8, sticky="w")
        self.spd_dir = tk.StringVar(value="CW")
        ttk.Radiobutton(sf, text="CW", value="CW", variable=self.spd_dir).grid(
            row=0, column=1, sticky="w")
        ttk.Radiobutton(sf, text="CCW", value="CCW", variable=self.spd_dir).grid(
            row=0, column=2, sticky="w")
        ttk.Label(sf, text="转速 rpm").grid(row=0, column=3, padx=(10, 2), sticky="e")
        self.spd_rpm = tk.StringVar(value="100")
        ttk.Entry(sf, textvariable=self.spd_rpm, width=9).grid(row=0, column=4, sticky="w")
        ttk.Label(sf, text="加速档").grid(row=1, column=0, padx=(10, 2), pady=8, sticky="w")
        self.spd_accel = tk.StringVar(value="10")
        ttk.Entry(sf, textvariable=self.spd_accel, width=9).grid(row=1, column=1, sticky="w")

        self.btn_spd = tk.Button(sf, text="持续运转\nSPD", command=self.on_spd,
                                 height=2, bg="#6a1b9a", fg="white",
                                 activebackground="#4a148c", activeforeground="white")
        self.btn_spd.grid(row=0, column=5, rowspan=2, padx=(10, 4), pady=6, sticky="ns")
        self.btn_stop = tk.Button(sf, text="停止\nSTP", command=self.on_stop_this,
                                  height=2, bg="#ef6c00", fg="white",
                                  activebackground="#e65100", activeforeground="white")
        self.btn_stop.grid(row=0, column=6, rowspan=2, padx=(0, 10), pady=6, sticky="ns")

        # IO
        iof = ttk.LabelFrame(self, text=" IO ")
        iof.pack(fill="x", **pad)
        self.lbl_in = tk.Label(iof, text="限位开关 GPIO15：—", anchor="w",
                               font=big, padx=8, pady=6, bg="#e8e8e8")
        self.lbl_in.grid(row=0, column=0, columnspan=3, padx=10, pady=(8, 4), sticky="we")

        ttk.Label(iof, text="MOS GPIO16").grid(row=1, column=0, padx=(10, 6), pady=(0, 10),
                                               sticky="w")
        self.btn_mos_lo = tk.Button(iof, text="低电平 0", command=lambda: self.on_mos(0))
        self.btn_mos_lo.grid(row=1, column=1, padx=2, pady=(0, 10))
        self.btn_mos_hi = tk.Button(iof, text="高电平 1", command=lambda: self.on_mos(1))
        self.btn_mos_hi.grid(row=1, column=2, padx=2, pady=(0, 10))
        self.lbl_out = ttk.Label(iof, text="当前 0")
        self.lbl_out.grid(row=1, column=3, padx=10, pady=(0, 10), sticky="w")
        iof.columnconfigure(3, weight=1)

        # 急停
        ef = ttk.Frame(self)
        ef.pack(fill="x", **pad)
        self.btn_estop = tk.Button(ef, text="全 部 急 停   (STP 0)", command=self.on_estop,
                                   height=2, font=("Sans", 13, "bold"),
                                   bg="#c62828", fg="white",
                                   activebackground="#b71c1c", activeforeground="white")
        self.btn_estop.pack(fill="x")

        # 日志
        lf = ttk.LabelFrame(self, text=" 日志 ")
        lf.pack(fill="both", expand=True, **pad)
        self.txt = tk.Text(lf, height=8, font=("Monospace", 9), state="disabled")
        self.txt.pack(fill="both", expand=True, padx=8, pady=6)

        self._set_enabled(False)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- 状态 ----------
    def log(self, msg: str) -> None:
        self.q.put(("log", f"[{time.strftime('%H:%M:%S')}] {msg}"))

    def _drain(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "log":
                    self.txt.config(state="normal")
                    self.txt.insert("end", payload + "\n")
                    self.txt.see("end")
                    self.txt.config(state="disabled")
                elif kind == "status":
                    self._apply_status(payload)
                elif kind == "io":
                    self._apply_io(payload)
                elif kind == "motors":
                    self._apply_motors(payload)
                elif kind == "online":
                    self.lbl_online.config(text=payload)
        except queue.Empty:
            pass
        self.after(60, self._drain)

    def _apply_status(self, st: dict | None):
        if not st:
            self.lbl_status.config(text="位置 --   转速 --   （无应答）")
            return
        inpos = "✓" if st["inpos"] else "—"
        stall = "✓ 堵转!" if st["stall"] else "—"
        clog = "  ⚠保护" if st["clog"] else ""
        self.lbl_status.config(
            text=f"位置 {st['pos']}   转速 {st['speed']}   到位 {inpos}   堵转 {stall}{clog}")

    def _apply_io(self, io: dict | None):
        if not io:
            return
        lv = io["in_level"]
        if lv:
            self.lbl_in.config(text=f"限位开关 GPIO{io['in_pin']}：● 已触发 (3.3V)",
                               bg="#fff3cd")
        else:
            self.lbl_in.config(text=f"限位开关 GPIO{io['in_pin']}：○ 未触发 (0V)",
                               bg="#e8e8e8")
        self.lbl_out.config(text=f"当前 {io['out_level']}")

    def _apply_motors(self, motors: list[dict]):
        self.motors = motors
        values = [f"{id_to_name(m['id'])}  (ID {m['id']})" for m in motors]
        self.motor_box["values"] = values
        if values:
            if not self.motor_var.get() or self.motor_var.get() not in values:
                self.motor_var.set(values[0])
            self.lbl_online.config(
                text="在线 " + "  ".join(id_to_name(m["id"]) for m in motors),
                foreground="#2e7d32")
            self._set_motor_controls(True)
        else:
            self.motor_var.set("")
            self.lbl_online.config(text="未发现电机", foreground="#c62828")
            self._set_motor_controls(False)
        self.on_motor_changed()

    def _set_enabled(self, on: bool):
        state = "normal" if on else "disabled"
        self.btn_conn.config(text="断开" if on else "连接")
        self._set_motor_controls(on)

    def _set_motor_controls(self, on: bool):
        state = "normal" if on else "disabled"
        for w in (self.btn_pos, self.btn_spd, self.btn_stop, self.btn_estop):
            w.config(state=state)
        for w in self.motor_btns:
            w.config(state=state)
        for w in (self.btn_mos_lo, self.btn_mos_hi):
            w.config(state=state)

    def selected_id(self) -> int | None:
        text = self.motor_var.get().strip()
        m = ID_IN_TEXT.search(text)
        if m:
            return int(m.group(1))
        try:
            return int(text.split()[0])
        except (ValueError, IndexError):
            return None

    def on_motor_changed(self):
        mid = self.selected_id()
        self.current_id = mid
        info = next((m for m in self.motors if m["id"] == mid), None)
        if info:
            bus = "A(UART1 GPIO4/5)" if info["bus"] == "A" else "B(UART2 GPIO8/9)"
            self.lbl_where.config(
                text=f"{bus} 地址 {info['addr']} · fw {info['fw']} / hw {info['hw']}")
        else:
            self.lbl_where.config(text="—")

    # ---------- 串口 ----------
    def refresh_ports(self):
        ports = list_serial_ports()
        values = [f"{d} — {s}" if s else d for d, s in ports]
        self.port_box["values"] = values
        if values and self.port_var.get() not in values:
            guess = default_port()
            self.port_var.set(next((v for v in values if v.startswith(guess or "\0")),
                                   values[0]))

    def _port(self) -> str:
        return self.port_var.get().split(" — ")[0].strip()

    def on_connect(self):
        if self.connected:
            self._disconnect()
            return

        port = self._port()
        if not port:
            messagebox.showwarning("没有串口", "未检测到串口，请插好板子后点「刷新」。")
            return
        if not __import__("os").path.exists(port):
            self.refresh_ports()
            messagebox.showerror("串口不存在", f"{port} 不存在了，板子可能重新枚举过。\n"
                                 "请重新选择端口。")
            return
        try:
            self.client.connect(port)
            bodies = self.client.transact("PING", timeout=1.5)
            if "PONG" not in bodies:
                raise LinkError("板子无应答（PING 失败），确认烧的是本工程固件")
        except Exception as exc:
            self.client.close()
            self.log(f"连接失败：{exc}")
            messagebox.showerror("连接失败", f"{port}\n\n{exc}")
            return

        self.connected = True
        self._set_enabled(True)
        self.log(f"已连接 {port}")
        self.on_scan()

    def _disconnect(self):
        self._stop_polling()
        self.client.close()
        self.connected = False
        self._set_enabled(False)
        self.lbl_online.config(text="未连接", foreground="#666")
        self.log("已断开")

    def on_scan(self):
        if not self.connected:
            return
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def _scan_worker(self):
        self._stop_polling()      # 扫描期间先别轮询，避免命令排队拖慢扫描
        try:
            motors = self.client.scan(1, 16)
            self.q.put(("motors", motors))
            self.log(f"扫描完成，发现 {len(motors)} 台：" +
                     (", ".join(f"{id_to_name(m['id'])}" for m in motors) or "无"))
            self._start_polling()
        except Exception as exc:
            self.log(f"扫描失败：{exc}")

    # ---------- 轮询 ----------
    def _start_polling(self):
        self._stop_polling()
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _stop_polling(self):
        if self._poll_thread:
            self._poll_stop.set()
            self._poll_thread = None

    def _poll_loop(self):
        last_status = 0.0
        while not self._poll_stop.is_set():
            try:
                now = time.time()
                if now - last_status >= POLL_MS / 1000.0:
                    mid = self.current_id          # 只读主线程写好的值
                    if mid is not None:
                        self.q.put(("status", self.client.status(mid)))
                    last_status = now
                if self._poll_stop.is_set():
                    break
                self.q.put(("io", self.client.io()))
            except Exception as exc:
                self.log(f"轮询中断：{exc}")
                break
            time.sleep(IO_POLL_MS / 1000.0)

    # ---------- 动作 ----------
    def _refresh_now(self):
        """动作执行后立刻拉一次状态，避免画面要等下一个轮询周期才更新。"""
        def work():
            try:
                mid = self.current_id
                if mid is not None:
                    self.q.put(("status", self.client.status(mid)))
                self.q.put(("io", self.client.io()))
            except Exception:
                pass
        threading.Thread(target=work, daemon=True).start()

    def _act(self, fn, what: str):
        if not self.connected:
            return
        try:
            fn()
            self.log(what)
            self._refresh_now()
        except Exception as exc:
            self.log(f"{what} 失败：{exc}")
            if isinstance(exc, (LinkError, ConnectionError, serial.SerialException)):
                messagebox.showerror("操作失败", str(exc))

    def _int(self, var: tk.StringVar, name: str, lo: int, hi: int) -> int | None:
        try:
            v = int(var.get().strip())
        except ValueError:
            messagebox.showwarning("参数错误", f"{name} 必须是整数")
            return None
        if not (lo <= v <= hi):
            messagebox.showwarning("参数错误", f"{name} 应在 {lo}~{hi} 之间")
            return None
        return v

    def on_pos(self):
        mid = self.selected_id()
        if mid is None:
            return
        pulses = self._int(self.pos_pulses, "脉冲数", 0, 0x7FFFFFFF)
        rpm = self._int(self.pos_rpm, "转速", 1, 1000)
        accel = self._int(self.pos_accel, "加速档", 0, 255)
        if None in (pulses, rpm, accel):
            return
        cw = self.pos_dir.get() == "CW"
        absolute = self.pos_mode.get() == "ABS"
        self._act(lambda: self.client.pos(mid, cw, pulses, rpm, accel, absolute),
                  f"POS {id_to_name(mid)} {'CW' if cw else 'CCW'} {pulses}脉冲 "
                  f"{rpm}rpm acc{accel} {'ABS' if absolute else 'REL'}")

    def on_spd(self):
        mid = self.selected_id()
        if mid is None:
            return
        rpm = self._int(self.spd_rpm, "转速", 1, 1000)
        accel = self._int(self.spd_accel, "加速档", 0, 255)
        if None in (rpm, accel):
            return
        cw = self.spd_dir.get() == "CW"
        self._act(lambda: self.client.spd(mid, cw, rpm, accel),
                  f"SPD {id_to_name(mid)} {'CW' if cw else 'CCW'} {rpm}rpm acc{accel}")

    def on_stop_this(self):
        mid = self.selected_id()
        if mid is None:
            return
        self._act(lambda: self.client.stop(mid), f"STP {id_to_name(mid)}")

    def on_estop(self):
        self._act(lambda: self.client.stop(0), "STP 0  全部急停")

    def on_enable(self, on: bool):
        mid = self.selected_id()
        if mid is None:
            return
        self._act(lambda: self.client.enable(mid, on),
                  f"ENA {id_to_name(mid)} {1 if on else 0}")

    def on_zero(self):
        mid = self.selected_id()
        if mid is None:
            return
        self._act(lambda: self.client.zero(mid), f"ZERO {id_to_name(mid)} 位置清零")

    def on_release(self):
        mid = self.selected_id()
        if mid is None:
            return
        self._act(lambda: self.client.release(mid), f"REL {id_to_name(mid)} 解除堵转")

    def on_mos(self, level: int):
        if not self.connected:
            return
        self._act(lambda: self.client.io_out(level),
                  f"MOS GPIO16 → {'高 1' if level else '低 0'}")

    def on_close(self):
        self._stop_polling()
        self.client.close()
        self.destroy()


def main():
    ap = argparse.ArgumentParser(description="双总线电机测试台")
    ap.add_argument("--port", help="串口，例如 /dev/ttyACM0")
    args = ap.parse_args()
    App(port=args.port).mainloop()


if __name__ == "__main__":
    main()
