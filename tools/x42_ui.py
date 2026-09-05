#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
张大头 X42S (Emm42_V5.0) 三电机上位机

链路：PC --USB(115200)--> ESP32-S3 --UART(GPIO4/5)--> 电机 x3
协议：ASCII 行命令，ESP 回复以 "CMD> " 开头，每条命令以 "CMD> DONE" 结束。

运行：python3 x42_ui.py
依赖：pip install pyserial
"""
from __future__ import annotations

import argparse
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import serial

from port_utils import default_controller_port


# ================= 串口链路 =================

class Link:
    """PC <-> ESP32 串口通信，线程安全。"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.5):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.timeout = timeout
        self.lock = threading.Lock()

    def transact(self, cmdline: str) -> list[str]:
        """发送一行命令，收集所有 CMD> 行直到 DONE 或超时。"""
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmdline + "\n").encode())
            lines: list[str] = []
            deadline = time.time() + self.timeout
            while time.time() < deadline:
                data = self.ser.readline()
                if not data:
                    continue
                text = data.decode(errors="replace").strip()
                if text.startswith("CMD>"):
                    body = text[5:].strip()
                    lines.append(body)
                    if body == "DONE":
                        break
            return lines

    def close(self) -> None:
        with self.lock:
            try:
                self.ser.close()
            except Exception:
                pass


# ================= 上位机窗口 =================

class MotorPanel(ttk.LabelFrame):
    """单个电机的控制面板。"""

    def __init__(self, parent: tk.Widget, motor_id: int, app: "X42App"):
        super().__init__(parent, text=f"电机 {motor_id}")
        self.id = motor_id
        self.app = app

        self.enable_var = tk.BooleanVar(value=True)
        self.pos_var = tk.StringVar(value="0.0")
        self.speed_var = tk.StringVar(value="100")
        self.accel_var = tk.StringVar(value="10")
        self.dir_var = tk.StringVar(value="CW")
        self.mode_var = tk.StringVar(value="REL")
        self.status_var = tk.StringVar(value="未连接")

        grid = dict(padx=4, pady=2)
        r = 0
        ttk.Checkbutton(self, text="使能", variable=self.enable_var,
                        command=self.on_enable).grid(row=r, column=0, sticky="w", **grid)
        ttk.Label(self, textvariable=self.status_var).grid(row=r, column=1,
                                                           columnspan=3, sticky="w", **grid)
        r += 1
        ttk.Label(self, text="目标位置(mm)").grid(row=r, column=0, sticky="e", **grid)
        ttk.Entry(self, textvariable=self.pos_var, width=10).grid(row=r, column=1, **grid)
        ttk.Label(self, text="速度(RPM)").grid(row=r, column=2, sticky="e", **grid)
        ttk.Entry(self, textvariable=self.speed_var, width=8).grid(row=r, column=3, **grid)
        r += 1
        ttk.Label(self, text="加速度档位").grid(row=r, column=0, sticky="e", **grid)
        ttk.Entry(self, textvariable=self.accel_var, width=8).grid(row=r, column=1, **grid)
        ttk.Combobox(self, textvariable=self.dir_var, values=("CW", "CCW"),
                     width=5, state="readonly").grid(row=r, column=2, **grid)
        ttk.Combobox(self, textvariable=self.mode_var, values=("REL", "ABS"),
                     width=5, state="readonly").grid(row=r, column=3, **grid)
        r += 1
        ttk.Button(self, text="执行定位", command=self.on_go).grid(row=r, column=0, **grid)
        ttk.Button(self, text="停止", command=self.on_stop).grid(row=r, column=1, **grid)
        ttk.Button(self, text="设为零点", command=self.on_zero).grid(row=r, column=2, **grid)
        ttk.Button(self, text="存零点(掉电保存)",
                   command=self.on_ozero).grid(row=r, column=3, **grid)
        r += 1
        ttk.Button(self, text="回零", command=self.on_home).grid(row=r, column=0, **grid)
        ttk.Button(self, text="解除堵转", command=self.on_release).grid(row=r, column=1, **grid)

    # ---- 动作 ----
    def _cmd(self, line: str) -> list[str]:
        if not self.app.link:
            messagebox.showwarning("未连接", "请先连接串口")
            return []
        try:
            return self.app.link.transact(line)
        except serial.SerialException as e:
            self.app.log(f"[电机{self.id}] 串口异常: {e}")
            return []

    def on_enable(self) -> None:
        self._cmd(f"ENA {self.id} {1 if self.enable_var.get() else 0}")

    def build_pos_cmd(self) -> str | None:
        """根据面板当前设置生成 POS 命令行，参数非法返回 None"""
        try:
            mm = float(self.pos_var.get())
            speed = int(self.speed_var.get())
            accel = int(self.accel_var.get())
        except ValueError:
            messagebox.showerror("参数错误", f"电机{self.id}: 位置/速度/加速度必须是数字")
            return None
        pulses = int(round(mm / self.app.lead_mm.get() * self.app.pulses_rev.get()))
        return f"POS {self.id} {self.dir_var.get()} {pulses} {speed} {accel} {self.mode_var.get()}"

    def on_go(self) -> None:
        cmd = self.build_pos_cmd()
        if cmd is None:
            return
        self.app.log(f"[电机{self.id}] {cmd}")
        for line in self._cmd(cmd):
            self.app.log(f"  {line}")

    def on_stop(self) -> None:
        self._cmd(f"STP {self.id}")

    def on_zero(self) -> None:
        """当前位置清零（不保存，掉电恢复）"""
        for line in self._cmd(f"ZERO {self.id}"):
            self.app.log(f"[电机{self.id}] {line}")

    def on_ozero(self) -> None:
        """存单圈零点（掉电保存）"""
        for line in self._cmd(f"OZERO {self.id}"):
            self.app.log(f"[电机{self.id}] {line}")

    def on_home(self) -> None:
        """电机1 用固件 X 轴状态机（碰撞->退圈->设零点）；2/3 用原生回零"""
        if self.id == 1:
            self.app.log("[电机1] XHOME（碰撞找端 -> 退一圈 -> 设零点）")
            for line in self._cmd("XHOME"):
                self.app.log(f"  {line}")
            threading.Thread(target=self._poll_xstate, daemon=True).start()
        else:
            mode = self.app.home_mode.get().split()[0]
            self.app.log(f"[电机{self.id}] 触发回零(模式{mode})...")
            for line in self._cmd(f"HOME {self.id} {mode}"):
                self.app.log(f"  {line}")

    def _poll_xstate(self) -> None:
        states = {0: "空闲", 1: "回零中", 2: "反向退圈中", 3: "设零点中",
                  4: "完成", 5: "失败"}
        last = None
        for _ in range(120):
            if not self.app.link:
                return
            try:
                lines = self.app.link.transact("XSTATE")
            except Exception:
                break
            for line in lines:
                parts = line.split()
                if len(parts) == 3 and parts[0] == "XSTATE":
                    state = int(parts[1])
                    if state != last:
                        last = state
                        self.app.after(0, self.app.log,
                                       f"[电机1回零] {states.get(state, state)}")
                    if state in (4, 5):
                        return
            time.sleep(0.8)

    def on_release(self) -> None:
        """解除堵转保护"""
        for line in self._cmd(f"REL {self.id}"):
            self.app.log(f"[电机{self.id}] {line}")


class SyncFrame(ttk.LabelFrame):
    """电机2+3 同步控制框（Y轴平台）：两机收到的命令完全一致，同时起步。"""

    def __init__(self, parent: tk.Widget, app: "X42App"):
        super().__init__(parent, text="电机2+3 同步控制（Y轴，两机命令完全一致）")
        self.app = app

        self.pos_var = tk.StringVar(value="0.0")
        self.speed_var = tk.StringVar(value="100")
        self.accel_var = tk.StringVar(value="10")
        self.dir_var = tk.StringVar(value="CW")
        self.mode_var = tk.StringVar(value="REL")

        g = dict(padx=4, pady=2)
        ttk.Label(self, text="目标位置(mm)").grid(row=0, column=0, sticky="e", **g)
        ttk.Entry(self, textvariable=self.pos_var, width=10).grid(row=0, column=1, **g)
        ttk.Label(self, text="速度(RPM)").grid(row=0, column=2, sticky="e", **g)
        ttk.Entry(self, textvariable=self.speed_var, width=8).grid(row=0, column=3, **g)
        ttk.Label(self, text="加速度档位").grid(row=0, column=4, sticky="e", **g)
        ttk.Entry(self, textvariable=self.accel_var, width=6).grid(row=0, column=5, **g)
        ttk.Combobox(self, textvariable=self.dir_var, values=("CW", "CCW"),
                     width=5, state="readonly").grid(row=0, column=6, **g)
        ttk.Combobox(self, textvariable=self.mode_var, values=("REL", "ABS"),
                     width=5, state="readonly").grid(row=0, column=7, **g)
        ttk.Button(self, text="同步执行", command=self.on_go).grid(row=0, column=8, **g)
        ttk.Button(self, text="同步停止", command=self.on_stop).grid(row=0, column=9, **g)
        ttk.Button(self, text="同步回零", command=self.on_home).grid(row=0, column=10, **g)

    def _tx(self, cmd: str) -> None:
        if not self.app.link:
            messagebox.showwarning("未连接", "请先连接串口")
            return None
        return self.app.link.transact(cmd)

    def _log(self, cmd: str) -> None:
        self.app.log(f"[同步] {cmd}")

    def on_go(self) -> None:
        try:
            mm = float(self.pos_var.get())
            speed = int(self.speed_var.get())
            accel = int(self.accel_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "位置/速度/加速度必须是数字")
            return
        pulses = int(round(mm / self.app.lead_mm.get() * self.app.pulses_rev.get()))
        args = f"{self.dir_var.get()} {pulses} {speed} {accel} {self.mode_var.get()}"
        # 两机收到一模一样的押住命令，广播后同时起步
        for mid in (2, 3):
            cmd = f"POS {mid} {args} SYNC"
            self._log(cmd)
            lines = self._tx(cmd) or []
            for line in lines:
                self.app.log(f"  {line}")
        self._log("GO（两机同时起步）")
        for line in self._tx("GO") or []:
            self.app.log(f"  {line}")

    def on_stop(self) -> None:
        for mid in (2, 3):
            self._log(f"STP {mid}")
            for line in self._tx(f"STP {mid}") or []:
                self.app.log(f"  {line}")

    def on_home(self) -> None:
        """电机2/3 双机回零状态机：同时碰撞找端 -> 两机都到位 -> 退圈 -> 设零点"""
        self._log("PHOME（双机回零状态机启动）")
        lines = self._tx("PHOME") or []
        for line in lines:
            self.app.log(f"  {line}")
        threading.Thread(target=self._poll_pstate, daemon=True).start()

    def _poll_pstate(self) -> None:
        states = {0: "空闲", 1: "回零中(先到的等后到的)", 2: "反向退圈中",
                  3: "设零点中", 4: "完成", 5: "失败"}
        last = None
        for _ in range(240):   # 最多约 3 分钟
            if not self.app.link:
                return
            try:
                lines = self.app.link.transact("PSTATE")
            except Exception:
                break
            for line in lines:
                parts = line.split()
                if len(parts) == 4 and parts[0] == "PSTATE":
                    state = int(parts[1])
                    if state != last:
                        last = state
                        self.app.after(0, self.app.log,
                                       f"[同步回零] {states.get(state, state)}")
                    if state in (4, 5):
                        return
            time.sleep(0.8)


class X42App(tk.Tk):
    def __init__(self, port: str | None = None):
        super().__init__()
        self.title("张大头 X42S 三电机上位机")
        self.link: Link | None = None
        self._poll_alive = threading.Event()

        self.port_var = tk.StringVar(value=port or default_controller_port())
        self.lead_mm = tk.DoubleVar(value=4.0)       # 丝杆导程 mm/圈
        self.pulses_rev = tk.IntVar(value=3200)      # 每圈脉冲（16细分）
        self.conn_var = tk.StringVar(value="未连接")

        self._build_top()
        self._build_home()
        SyncFrame(self, self).pack(fill="x", padx=6, pady=3)
        self.panels = [MotorPanel(self, i, self) for i in range(1, 4)]
        for p in self.panels:
            p.pack(fill="x", padx=6, pady=3)
        self._build_bottom()

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_top(self) -> None:
        top = ttk.Frame(self)
        top.pack(fill="x", padx=6, pady=4)
        ttk.Label(top, text="串口").pack(side="left")
        ttk.Entry(top, textvariable=self.port_var, width=14).pack(side="left", padx=4)
        ttk.Button(top, text="刷新", command=self.on_refresh_port).pack(side="left", padx=2)
        ttk.Label(top, text="导程(mm/圈)").pack(side="left")
        ttk.Entry(top, textvariable=self.lead_mm, width=6).pack(side="left", padx=4)
        ttk.Label(top, text="每圈脉冲").pack(side="left")
        ttk.Entry(top, textvariable=self.pulses_rev, width=6).pack(side="left", padx=4)
        self.conn_btn = ttk.Button(top, text="连接", command=self.on_connect)
        self.conn_btn.pack(side="left", padx=4)
        ttk.Button(top, text="全部停止", command=self.on_stop_all).pack(side="left", padx=4)
        ttk.Label(top, textvariable=self.conn_var).pack(side="left", padx=8)

    def _build_bottom(self) -> None:
        self.logbox = scrolledtext.ScrolledText(self, height=10, state="disabled")
        self.logbox.pack(fill="both", expand=True, padx=6, pady=4)

    def on_refresh_port(self) -> None:
        port = default_controller_port()
        if port:
            self.port_var.set(port)

    def _build_home(self) -> None:
        """回零参数全局配置（写入所有电机）"""
        frm = ttk.LabelFrame(self, text="回零参数（多圈碰撞回零=模式2，写入全部电机）")
        frm.pack(fill="x", padx=6, pady=3)
        self.home_mode = tk.StringVar(value="2 多圈碰撞")
        self.home_dir = tk.StringVar(value="CW")
        self.home_speed = tk.StringVar(value="80")
        self.home_timeout = tk.StringVar(value="30000")
        self.home_clog_rpm = tk.StringVar(value="50")
        self.home_clog_ma = tk.StringVar(value="900")
        self.home_clog_ms = tk.StringVar(value="80")
        self.home_auto = tk.BooleanVar(value=False)

        g = dict(padx=4, pady=2)
        ttk.Label(frm, text="模式").grid(row=0, column=0, sticky="e", **g)
        ttk.Combobox(frm, textvariable=self.home_mode, width=12, state="readonly",
                     values=("0 单圈就近", "1 单圈方向", "2 多圈碰撞", "3 多圈限位")
                     ).grid(row=0, column=1, **g)
        ttk.Label(frm, text="方向").grid(row=0, column=2, sticky="e", **g)
        ttk.Combobox(frm, textvariable=self.home_dir, values=("CW", "CCW"),
                     width=5, state="readonly").grid(row=0, column=3, **g)
        ttk.Label(frm, text="回零速度RPM").grid(row=0, column=4, sticky="e", **g)
        ttk.Entry(frm, textvariable=self.home_speed, width=6).grid(row=0, column=5, **g)
        ttk.Label(frm, text="超时ms").grid(row=0, column=6, sticky="e", **g)
        ttk.Entry(frm, textvariable=self.home_timeout, width=7).grid(row=0, column=7, **g)
        ttk.Label(frm, text="碰撞转速RPM").grid(row=1, column=0, sticky="e", **g)
        ttk.Entry(frm, textvariable=self.home_clog_rpm, width=6).grid(row=1, column=1, **g)
        ttk.Label(frm, text="碰撞电流mA").grid(row=1, column=2, sticky="e", **g)
        ttk.Entry(frm, textvariable=self.home_clog_ma, width=6).grid(row=1, column=3, **g)
        ttk.Label(frm, text="碰撞时间ms").grid(row=1, column=4, sticky="e", **g)
        ttk.Entry(frm, textvariable=self.home_clog_ms, width=6).grid(row=1, column=5, **g)
        ttk.Checkbutton(frm, text="上电自动回零",
                        variable=self.home_auto).grid(row=1, column=6, columnspan=2, sticky="w", **g)
        ttk.Button(frm, text="写入参数(存芯片)", command=self.on_write_home).grid(row=2, column=0, columnspan=3, **g)
        ttk.Button(frm, text="读取参数", command=self.on_read_home).grid(row=2, column=3, columnspan=2, **g)
        ttk.Button(frm, text="查回零状态", command=self.on_hstat).grid(row=2, column=5, columnspan=3, **g)

    def on_write_home(self) -> None:
        if not self.link:
            messagebox.showwarning("未连接", "请先连接串口")
            return
        mode = self.home_mode.get().split()[0]
        cmd = (f"HPARAM {{id}} {mode} {self.home_dir.get()} {self.home_speed.get()} "
               f"{self.home_timeout.get()} {self.home_clog_rpm.get()} "
               f"{self.home_clog_ma.get()} {self.home_clog_ms.get()} "
               f"{1 if self.home_auto.get() else 0}")
        for mid in (1, 2, 3):
            try:
                lines = self.link.transact(cmd.format(id=mid))
            except Exception as e:
                self.log(f"[电机{mid}] {e}")
                continue
            for line in lines:
                self.log(f"[电机{mid}] {line}")

    def on_read_home(self) -> None:
        if not self.link:
            return
        for line in self.link.transact("HPARAMR 1"):
            self.log(f"[回零参数] {line}")

    def on_hstat(self) -> None:
        if not self.link:
            return
        for line in self.link.transact("HSTAT 1"):
            self.log(f"[回零状态] {line}")

    def log(self, msg: str) -> None:
        self.logbox.configure(state="normal")
        self.logbox.insert("end", f"{time.strftime('%H:%M:%S')} {msg}\n")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    # ---- 连接 / 轮询 ----
    def on_connect(self) -> None:
        if self.link:
            self.on_close()
            return
        try:
            self.link = Link(self.port_var.get())
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            return
        self.conn_var.set("已连接")
        self.conn_btn.configure(text="断开")
        self.log(f"已连接 {self.port_var.get()}")
        self._poll_alive.set()
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def _poll_loop(self) -> None:
        while self._poll_alive.is_set():
            try:
                lines = self.link.transact("STAT 0")
            except Exception as e:
                self.after(0, self.log, f"轮询异常: {e}")
                time.sleep(1.0)
                continue
            updates = []
            for line in lines:
                parts = line.split()
                if len(parts) == 8 and parts[0] == "STAT":
                    _, mid, en, inpos, stall, clog, pos, spd = parts
                    mm = int(pos) * self.lead_mm.get() / 65536.0
                    text = f"en={en} 到位={inpos} 堵转={stall} {mm:+.2f}mm {spd}RPM"
                    updates.append((int(mid), text))
                elif parts and parts[0] == "ERR":
                    updates.append((None, " ".join(parts[1:])))
            self.after(0, self._apply_updates, updates)
            time.sleep(0.5)

    def _apply_updates(self, updates) -> None:
        for mid, text in updates:
            if mid is None:
                continue
            for p in self.panels:
                if p.id == mid:
                    p.status_var.set(text)

    def on_stop_all(self) -> None:
        if self.link:
            self.link.transact("STP 0")
            self.log("已发送全部停止")

    def on_close(self) -> None:
        self._poll_alive.clear()
        if self.link:
            self.link.close()
            self.link = None
        self.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="X42 debug interface")
    parser.add_argument("--port", help="controller serial port, for example COM4 or /dev/ttyACM0")
    X42App(parser.parse_args().port).mainloop()
