#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
X42 三轴机器底层封装（用户 UI 专用，隐藏协议细节）

- X42Link：PC <-> ESP32 串口链路（协议：CMD> 前缀，CMD> DONE 结尾）
- Machine：机器模型：mm/脉冲换算、软限位、状态读取、XY 直线插补运动、回零

机器：电机1 = X 轴丝杆（导程 4mm，16细分 3200脉冲/圈）
      电机2/3 = Y 轴刚性龙门（命令一致，同时起步）
"""
from __future__ import annotations

import math
import threading
import time

import serial

# ================= 默认机器参数（高级设置可改） =================
DEFAULT_LEAD_MM = 4.0        # 丝杆导程
DEFAULT_PULSES_REV = 3200    # 每圈脉冲（16细分）
DEFAULT_TRAVEL_X = (0.0, 180.0)   # X 行程 mm
DEFAULT_TRAVEL_Y = (0.0, 180.0)   # Y 行程 mm（回零点在顶部，UI 显示顶部=180mm）
DEFAULT_ACCEL = 10           # 加速度档位 0~255
DEFAULT_SPEED_MM_S = 20.0    # 默认线速度


class LinkError(Exception):
    """通信/运动错误，消息面向用户。"""


# ================= 串口链路 =================

class X42Link:
    """PC <-> ESP32 串口通信，线程安全。"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 3.0):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.timeout = timeout
        self.lock = threading.Lock()

    def transact(self, cmdline: str) -> list[str]:
        """发送一行命令，收集所有 CMD> 行直到 DONE 或超时；空响应自动重试一次。"""
        with self.lock:
            for _attempt in range(2):
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
                if lines:
                    return lines
                time.sleep(0.1)   # 偶发丢帧，重试一次
            return []

    def close(self) -> None:
        with self.lock:
            try:
                self.ser.close()
            except Exception:
                pass


# ================= 机器模型 =================

class Machine:
    def __init__(self, link: X42Link):
        self.link = link
        self.lead_mm = DEFAULT_LEAD_MM
        self.pulses_per_rev = DEFAULT_PULSES_REV
        self.travel_x = DEFAULT_TRAVEL_X
        self.travel_y = DEFAULT_TRAVEL_Y
        self.accel = DEFAULT_ACCEL
        # 回零后工作区位于"碰撞端部的反方向"（退圈方向），默认 X+/Y+ = CCW
        self.x_invert = True
        self.y_invert = True
        self.y3_invert = False   # 电机3 镜像安装时勾选
        self.homed = False       # 回零后置 True（绝对坐标基准）
        # 用户工作流：禁用固件堵转自动恢复（否则守护会擅自运动并清零位置），
        # 堵转情况由上位机检测并报错。
        try:
            link.transact("GUARD 0")
        except Exception:
            pass

    # ---------- 换算 ----------
    def _per_mm(self) -> float:
        """脉冲数/mm（命令侧）"""
        return self.pulses_per_rev / self.lead_mm

    def mm_to_pulses(self, mm: float) -> int:
        """行程 mm -> 电机脉冲数（位置命令用）"""
        return int(round(abs(mm) * self._per_mm()))

    def raw_to_mm(self, raw: int) -> float:
        """电机 raw 位置（65536/圈，编码器单位）-> mm"""
        return raw * self.lead_mm / 65536.0

    def rpm_for(self, mm_s: float) -> int:
        """线速度 mm/s -> 电机转速 RPM（整数，最低 1）。"""
        return max(1, int(round(mm_s * 60.0 / self.lead_mm)))

    def check_bounds(self, x: float, y: float) -> None:
        if not (self.travel_x[0] <= x <= self.travel_x[1] + 0.5):
            raise LinkError(f"X 越界: {x:.1f}mm（行程 {self.travel_x[0]}-{self.travel_x[1]}mm）")
        if not (self.travel_y[0] <= y <= self.travel_y[1] + 0.5):
            raise LinkError(f"Y 越界: {y:.1f}mm（行程 {self.travel_y[0]}-{self.travel_y[1]}mm）")

    # ---------- 状态 ----------
    def status(self) -> dict[int, dict]:
        """返回 {电机id: {en, inpos, stall, clog, pos_raw, speed}}。"""
        out: dict[int, dict] = {}
        for line in self.link.transact("STAT 0"):
            p = line.split()
            if len(p) == 8 and p[0] == "STAT":
                out[int(p[1])] = dict(
                    en=int(p[2]), inpos=int(p[3]), stall=int(p[4]),
                    clog=int(p[5]), pos_raw=int(p[6]), speed=int(p[7]))
        return out

    def position(self) -> tuple[float, float]:
        """当前位置 (x_mm, y_mm)。"""
        st = self.status()
        if 1 not in st or 2 not in st:
            raise LinkError("电机状态读取失败（检查连接/供电）")
        x = self.raw_to_mm(st[1]["pos_raw"])
        y = self.raw_to_mm(st[2]["pos_raw"])
        if self.x_invert:
            x = -x
        if self.y_invert:
            y = -y
        return x, y

    # ---------- 运动 ----------
    def move_axis(self, axis: str, pos: float, speed_mm_s: float = DEFAULT_SPEED_MM_S,
                  progress=None, stop_check=None) -> None:
        """单轴绝对定位。axis: 'X'（电机1）或 'Y'（电机2/3 同步）。"""
        axis = axis.upper()
        if axis == "X":
            lo, hi = self.travel_x
            if not (lo <= pos <= hi + 0.5):
                raise LinkError(f"X 越界: {pos:.1f}mm（行程 {lo}-{hi}mm）")
            if not self.homed:
                raise LinkError("请先回零再运动")
            cx, _ = self.position()
            dist = abs(pos - cx)
            if dist < 0.02:
                return
            px = int(round(-pos * self._per_mm())) if self.x_invert \
                else int(round(pos * self._per_mm()))
            d = "CW" if px >= 0 else "CCW"
            cmds = [f"POS 1 {d} {abs(px)} {self.rpm_for(speed_mm_s)} {self.accel} ABS SYNC"]
        elif axis == "Y":
            lo, hi = self.travel_y
            if not (lo <= pos <= hi + 0.5):
                raise LinkError(f"Y 越界: {pos:.1f}mm（行程 {lo}-{hi}mm）")
            if not self.homed:
                raise LinkError("请先回零再运动")
            _, cy = self.position()
            dist = abs(pos - cy)
            if dist < 0.02:
                return
            py = int(round(-pos * self._per_mm())) if self.y_invert \
                else int(round(pos * self._per_mm()))
            py3 = -py if self.y3_invert else py
            d = "CW" if py >= 0 else "CCW"
            d3 = d if not self.y3_invert else ("CCW" if d == "CW" else "CW")
            rpm = self.rpm_for(speed_mm_s)
            cmds = [f"POS 2 {d} {abs(py)} {rpm} {self.accel} ABS SYNC",
                    f"POS 3 {d3} {abs(py3)} {rpm} {self.accel} ABS SYNC"]
        else:
            raise LinkError(f"未知轴: {axis}")

        st0 = self.status()
        start_pos = {i: st0[i]["pos_raw"] for i in (1, 2, 3) if i in st0}
        for c in cmds:
            for line in self.link.transact(c):
                if line.startswith("ERR"):
                    raise LinkError(f"运动命令被拒: {line}")
        self.link.transact("GO")
        time.sleep(0.1)
        timeout = dist / max(speed_mm_s, 1.0) * 2 + 20.0
        self.wait_move_done(start_pos, timeout, progress, stop_check)

    def move_to(self, x: float, y: float, speed_mm_s: float = DEFAULT_SPEED_MM_S,
                progress=None, stop_check=None) -> None:
        """
        直线插补运动到 (x, y)：按 ΔX:ΔY 比例分配两轴速度，
        押住三机命令后广播同时起步，两轴同时到达。
        progress(): 每次轮询回调（更新 UI）；stop_check(): 返回 True 则中止并抛 LinkError。
        """
        self.check_bounds(x, y)
        if not self.homed:
            raise LinkError("请先回零再运动")
        cx, cy = self.position()
        dx = x - cx
        dy = y - cy
        dist = math.hypot(dx, dy)
        if dist < 0.02:
            return

        # 记录出发位置（用于确认运动真的开始，避免旧到位标志误判）
        st0 = self.status()
        start_pos = {i: st0[i]["pos_raw"] for i in (1, 2, 3) if i in st0}

        ax = dx / dist
        ay = dy / dist

        # 绝对位置模式：脉冲数 = 相对零点的绝对目标坐标（方向字节 = 符号）
        per = self._per_mm()
        px = int(round(-x * per)) if self.x_invert else int(round(x * per))
        py = int(round(-y * per)) if self.y_invert else int(round(y * per))
        py3 = -py if self.y3_invert else py
        cmds: list[str] = []
        if abs(dx) > 0.02:
            d = "CW" if px >= 0 else "CCW"
            cmds.append(f"POS 1 {d} {abs(px)} "
                        f"{self.rpm_for(abs(ax) * speed_mm_s)} {self.accel} ABS SYNC")
        if abs(dy) > 0.02:
            d = "CW" if py >= 0 else "CCW"
            d3 = d if not self.y3_invert else ("CCW" if d == "CW" else "CW")
            rpm = self.rpm_for(abs(ay) * speed_mm_s)
            cmds.append(f"POS 2 {d} {abs(py)} {rpm} {self.accel} ABS SYNC")
            cmds.append(f"POS 3 {d3} {abs(py3)} {rpm} {self.accel} ABS SYNC")

        for c in cmds:
            for line in self.link.transact(c):
                if line.startswith("ERR"):
                    raise LinkError(f"运动命令被拒: {line}")
        self.link.transact("GO")
        time.sleep(0.1)
        timeout = dist / max(speed_mm_s, 1.0) * 2 + 20.0
        self.wait_move_done(start_pos, timeout, progress, stop_check)

    def wait_move_done(self, start_pos, timeout: float, progress=None, stop_check=None) -> None:
        """等待运动真正开始并完成：先看到位置变化，再等到位（防旧到位标志误判）。
        到位判定需连续两次确认（容忍偶发丢帧）。"""
        t0 = time.time()
        started = False
        inpos_streak = 0
        while time.time() - t0 < timeout:
            if stop_check and stop_check():
                self.stop()
                raise LinkError("用户中止运动")
            st = self.status()
            if not st:
                continue   # 丢帧，跳过这一轮
            if any(st.get(i, {}).get("clog") for i in (1, 2, 3)):
                raise LinkError("堵转保护触发，请检查机械")
            cur = {i: st[i]["pos_raw"] for i in (1, 2, 3) if i in st}
            if not started:
                moved = any(abs(cur.get(i, 0) - start_pos.get(i, 0)) > 200
                            for i in (1, 2, 3))
                if moved:
                    started = True
                # 注意：不设"瞬间完成"捷径——电机收到命令到真正起步
                # 有几十毫秒延迟，必须确认位置真的变了才算开始
            else:
                if all(st.get(i, {}).get("inpos") for i in (1, 2, 3)):
                    inpos_streak += 1
                    if inpos_streak >= 2:
                        return
                else:
                    inpos_streak = 0
            if progress:
                progress()
            time.sleep(0.08)
        raise LinkError("运动超时未到位")

    def stop(self) -> None:
        self.link.transact("STP 0")

    # ---------- 回零 ----------
    def home(self, progress=None) -> None:
        """X（电机1）与 Y（2/3）并行回零，等待两者都完成。"""
        for cmd in ("XHOME", "PHOME"):
            for line in self.link.transact(cmd):
                if line.startswith("ERR") and "BUSY" not in line:
                    raise LinkError(f"回零启动失败: {line}")

        t0 = time.time()
        while time.time() - t0 < 120:
            x_done = any(l.startswith("XSTATE 4") or l.startswith("XSTATE 5")
                         for l in self.link.transact("XSTATE"))
            y_done = any(l.startswith("PSTATE 4") or l.startswith("PSTATE 5")
                         for l in self.link.transact("PSTATE"))
            if x_done and y_done:
                if any(l.startswith("XSTATE 5") for l in self.link.transact("XSTATE")) or \
                   any(l.startswith("PSTATE 5") for l in self.link.transact("PSTATE")):
                    raise LinkError("回零失败（超时或碰撞异常）")
                # 等各电机收尾稳定（到位标志全部置位）
                t1 = time.time()
                while time.time() - t1 < 3.0:
                    st = self.status()
                    if st and all(st.get(i, {}).get("inpos") for i in (1, 2, 3)):
                        self.homed = True
                        return
                    time.sleep(0.1)
                self.homed = True
                return
            if progress:
                progress()
            time.sleep(0.3)
        raise LinkError("回零超时")
