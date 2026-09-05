#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
X42 三轴平台 · 用户界面（化学实验演示仪器）

操作员流程（打开软件即引导）：
  ① 首页：确认设备连接 → 提示执行复位（回零）→ 复位完成显示"已就绪"
  ② 选择轨迹 → 进入操作页：执行/暂停/停止（曲线预览 + 当前段高亮）
  ③ 轨迹制作在「轨迹管理」页（二级页面，正常操作不进入）

设计说明：
  - 轨迹 = 时间轴曲线：横轴时间(s)、纵轴位置(mm)，X/Y 两轨，关键帧间直线插值
  - 段速度 = Δ距离/Δ时间，超过速度上限自动限速（编辑器中标红提示）
  - Y 轴坐标约定（仅 UI 层）：顶部 = 行程上限，底部 = 0，向上为正；
    机器/固件坐标不变，App 内统一镜像换算（Y_UI = 行程上限 - Y_机器）
  - 位置单位 mm，导程/脉冲/限位等细节隐藏在轨迹管理页的"高级设置"

运行：python3 user_ui.py
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from x42link import (DEFAULT_ACCEL, DEFAULT_LEAD_MM, DEFAULT_PULSES_REV,
                     DEFAULT_SPEED_MM_S, DEFAULT_TRAVEL_X, DEFAULT_TRAVEL_Y,
                     LinkError, Machine, X42Link)
from curve_editor import CurveEditor, build_segments, interp_track
from port_utils import default_controller_port

TRAJ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectories")


# ================= 轨迹数据：时间轴曲线 =================

class Trajectory:
    """tracks = {"X": [[t_s, pos_mm], ...], "Y": [[t_s, pos_mm], ...]}
    相邻关键帧直线插值；段速度 = Δ位置/Δ时间，超限标红。
    stirs = [{"t0", "t1", "freq_hz", "amp_mm"}, ...]：Y 轴搅拌段（支线任务），
    窗口内 Y 曲线必须平坦（Y 被振动占用），X 照常运动。"""

    def __init__(self, name="未命名", speed_limit_mm_s=40.0, duration_s=30.0,
                 tracks=None, stirs=None):
        self.name = name
        self.speed_limit_mm_s = speed_limit_mm_s
        self.duration_s = duration_s
        self.tracks = tracks or {"X": [[0.0, 0.0], [duration_s, 0.0]],
                                 "Y": [[0.0, 0.0], [duration_s, 0.0]]}
        self.stirs = stirs or []

    def to_dict(self):
        return {"name": self.name, "speed_limit_mm_s": self.speed_limit_mm_s,
                "duration_s": self.duration_s,
                "tracks": {"X": [list(p) for p in self.tracks["X"]],
                           "Y": [list(p) for p in self.tracks["Y"]]},
                "stirs": [dict(s) for s in self.stirs]}

    @classmethod
    def from_dict(cls, d):
        stirs = [{"t0": float(s["t0"]), "t1": float(s["t1"]),
                  "freq_hz": float(s["freq_hz"]), "amp_mm": float(s["amp_mm"])}
                 for s in d.get("stirs", [])]
        if "tracks" in d:
            tracks = {ax: [[float(t), float(p)] for t, p in d["tracks"].get(ax, [])]
                      for ax in ("X", "Y")}
            return cls(d.get("name", "未命名"),
                       float(d.get("speed_limit_mm_s", 40.0)),
                       float(d.get("duration_s", 30.0)), tracks, stirs)
        # 兼容旧版步骤表格式：每步一个关键帧，5 秒/步，直线过渡
        steps = [(str(s["axis"]).upper(), float(s["pos"])) for s in d.get("steps", [])]
        dur = max(5.0 * len(steps), 10.0)
        tracks = {"X": [[0.0, 0.0], [dur, 0.0]], "Y": [[0.0, 0.0], [dur, 0.0]]}
        for i, (ax, p) in enumerate(steps, start=1):
            tracks[ax].append([i * 5.0, p])
        return cls(d.get("name", "未命名"), 40.0, dur, tracks, stirs)


def validate_stirs(traj: Trajectory, y_top: float, extra: dict | None = None) -> list:
    """严格校验搅拌段，返回错误列表（空 = 合法）：
    - 时间范围 0≤t0<t1≤总时长；频率 0.1~10Hz；振幅>0
    - 振动期间 Y 曲线必须平坦（Y 被振动占用）
    - 中心±振幅始终在行程内（0~y_top）
    - 搅拌段互不重叠"""
    errs: list = []
    stirs = [dict(s) for s in traj.stirs]
    if extra:
        stirs.append(dict(extra))
    dur = traj.duration_s
    ytrack = traj.tracks.get("Y", [])
    for i, s in enumerate(stirs):
        t0, t1, f, a = s["t0"], s["t1"], s["freq_hz"], s["amp_mm"]
        if not (0.0 <= t0 < t1 <= dur + 1e-6):
            errs.append(f"搅拌段{i + 1}: 时间 {t0:g}~{t1:g}s 非法（需 0≤开始<结束≤总时长 {dur:g}s）")
            continue
        if not (0.1 <= f <= 10.0):
            errs.append(f"搅拌段{i + 1}: 频率 {f:g}Hz 超出 0.1~10Hz")
        if a <= 0:
            errs.append(f"搅拌段{i + 1}: 振幅必须 > 0")
            continue
        # 振动期间 Y 曲线必须平坦
        yc = interp_track(ytrack, t0)
        if abs(interp_track(ytrack, t1) - yc) > 0.05:
            errs.append(f"搅拌段{i + 1}: Y 曲线在 {t0:g}~{t1:g}s 内不平坦（振动期间 Y 不能位移）")
            continue
        for t, _ in ytrack:
            if t0 - 1e-6 < t < t1 + 1e-6 and abs(interp_track(ytrack, t) - yc) > 0.05:
                errs.append(f"搅拌段{i + 1}: Y 曲线在 {t0:g}~{t1:g}s 内不平坦（振动期间 Y 不能位移）")
                break
        # 限位：中心±振幅始终在行程内
        if not (a - 1e-6 <= yc <= y_top - a + 1e-6):
            errs.append(f"搅拌段{i + 1}: Y={yc:.1f}mm 振动 ±{a:g}mm 越界（行程 0~{y_top:g}mm）")
    srt = sorted(stirs, key=lambda s: s["t0"])
    for i in range(len(srt) - 1):
        if srt[i]["t1"] > srt[i + 1]["t0"] + 1e-6:
            errs.append(f"搅拌段重叠: {srt[i]['t0']:g}~{srt[i]['t1']:g} 与 "
                        f"{srt[i + 1]['t0']:g}~{srt[i + 1]['t1']:g}")
            break
    return errs


def list_trajectories() -> list[str]:
    if not os.path.isdir(TRAJ_DIR):
        return []
    return sorted(f[:-5] for f in os.listdir(TRAJ_DIR) if f.endswith(".json"))


# ================= 主窗口 =================

class App(tk.Tk):
    def __init__(self, port: str | None = None):
        super().__init__()
        self.title("实验演示平台控制台")
        self.geometry("860x640")

        self.link: X42Link | None = None
        self.machine: Machine | None = None
        self.ops_lock = threading.Lock()
        self.connected = False
        self.homing = False
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.exec_thread: threading.Thread | None = None
        self._stir_center_y: float | None = None

        self.port_var = tk.StringVar(value=port or default_controller_port())
        self.conn_var = tk.StringVar(value="● 未连接")
        self.pos_var = tk.StringVar(value="X=---  Y=---")
        self.home_state_var = tk.StringVar(value="等待连接")
        self.busy_var = tk.StringVar(value="空闲")
        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED_MM_S)
        self.selected_traj_name = tk.StringVar(value="")
        self.traj = Trajectory()
        self.traj.tracks = self._empty_tracks(self.traj.duration_s)

        self._build_notebook()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # 线程安全：后台线程只投递消息，主线程统一消费（tkinter 非线程安全）
        self.uiq: queue.Queue = queue.Queue()
        self.after(80, self._drain_uiq)

    # ---------- 主线程 UI 消息消费 ----------
    def _ui(self, kind: str, *args) -> None:
        self.uiq.put((kind, args))

    def _drain_uiq(self):
        try:
            while True:
                kind, args = self.uiq.get_nowait()
                self._handle_ui(kind, args)
        except queue.Empty:
            pass
        self.after(80, self._drain_uiq)

    def _handle_ui(self, kind, args):
        if kind == "pos":
            self.pos_var.set(f"X={args[0]:.1f}mm  Y={args[1]:.1f}mm")
        elif kind == "log":
            self.log(args[0])
        elif kind == "busy":
            self.busy_var.set(args[0])
        elif kind == "home_state":
            self.home_state_var.set(args[0])
            self.home_btn.configure(state="normal" if not self.homing else "disabled")
        elif kind == "highlight":
            self.preview.set_highlight(args[0], args[1])
        elif kind == "clear_progress":
            self.preview.clear_progress()
        elif kind == "run_btns":
            self.run_btn.configure(state=args[0])
            self.pause_btn.configure(state=args[1])
        elif kind == "connected":
            self.connected = True
            self.conn_var.set("● 已连接")
            self.conn_btn.configure(text="断开")
            self.home_state_var.set("⚠ 设备未复位，请执行复位")
            self.home_btn.configure(state="normal")
            self._refresh_traj_list()
        elif kind == "conn_failed":
            self.conn_btn.configure(state="normal")
            messagebox.showerror("连接失败", args[0])
        elif kind == "update_sel":
            axis, pos = args
            self.editor.set_selected_pos(pos)
            self._on_curve_changed()
            self.log(f"关键帧已更新为当前 {axis} 位置 {pos:.1f}mm")
        elif kind == "error":
            messagebox.showerror(args[0], args[1])

    # ================= 布局 =================

    def _build_notebook(self):
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)
        self.tab_home = ttk.Frame(self.nb)
        self.tab_run = ttk.Frame(self.nb)
        self.tab_manage = ttk.Frame(self.nb)
        self.nb.add(self.tab_home, text="① 首页")
        self.nb.add(self.tab_run, text="② 操作")
        self.nb.add(self.tab_manage, text="③ 轨迹管理")
        self._build_home()
        self._build_run()
        self._build_manage()
        self._build_log()

    # ---------- ① 首页：回零引导 ----------
    def _build_home(self):
        frm = ttk.Frame(self.tab_home, padding=20)
        frm.pack(fill="both", expand=True)

        ttk.Label(frm, text="设备连接", font=("", 12, "bold")).pack(anchor="w", pady=(0, 6))
        row = ttk.Frame(frm)
        row.pack(fill="x")
        ttk.Label(row, textvariable=self.conn_var, font=("", 11)).pack(side="left")
        ttk.Entry(row, textvariable=self.port_var, width=13).pack(side="right", padx=2)
        ttk.Button(row, text="刷新", command=self._refresh_port).pack(side="right", padx=2)
        self.conn_btn = ttk.Button(row, text="连接", command=self.on_connect)
        self.conn_btn.pack(side="right", padx=4)

        ttk.Separator(frm).pack(fill="x", pady=14)

        ttk.Label(frm, text="第一步 · 复位（回零）", font=("", 12, "bold")).pack(anchor="w", pady=(0, 6))
        self.home_state_lbl = ttk.Label(frm, textvariable=self.home_state_var,
                                        font=("", 16, "bold"))
        self.home_state_lbl.pack(anchor="w", pady=6)
        ttk.Label(frm, text="复位时设备会自动运动到两端限位后回到零点，请确保平台周围无遮挡。",
                  foreground="#666").pack(anchor="w")
        self.home_btn = ttk.Button(frm, text="开始复位", command=self.on_home, width=20)
        self.home_btn.pack(anchor="w", pady=8)

        ttk.Separator(frm).pack(fill="x", pady=14)

        ttk.Label(frm, text="第二步 · 选择实验轨迹", font=("", 12, "bold")).pack(anchor="w", pady=(0, 6))
        row2 = ttk.Frame(frm)
        row2.pack(fill="x")
        self.traj_combo = ttk.Combobox(row2, textvariable=self.selected_traj_name,
                                       state="readonly", width=24)
        self.traj_combo.pack(side="left", padx=(0, 6))
        self.traj_combo.bind("<<ComboboxSelected>>", lambda e: self._on_traj_selected())
        ttk.Button(row2, text="刷新列表", command=self._refresh_traj_list).pack(side="left")
        ttk.Button(row2, text="进入操作 ▶", command=self._enter_run, width=16).pack(side="right")

        ttk.Separator(frm).pack(fill="x", pady=14)
        ttk.Button(frm, text="轨迹管理（制作/修改轨迹，仅调试人员使用）",
                   command=lambda: self.nb.select(self.tab_manage)).pack(anchor="sw")

    def _refresh_traj_list(self):
        names = list_trajectories()
        self.traj_combo["values"] = names
        if names and not self.selected_traj_name.get():
            self.selected_traj_name.set(names[0])
            self._on_traj_selected()

    def _refresh_port(self):
        port = default_controller_port()
        if port:
            self.port_var.set(port)

    def _on_traj_selected(self):
        name = self.selected_traj_name.get()
        path = os.path.join(TRAJ_DIR, f"{name}.json")
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                self.traj = Trajectory.from_dict(json.load(f))
            self.log(f"已选择轨迹: {name}")
            self.dur_var.set(self.traj.duration_s)
            self.limit_var.set(self.traj.speed_limit_mm_s)
            self.editor.set_tracks(self.traj.tracks, self.traj.duration_s)
            self.editor.limit = self.traj.speed_limit_mm_s
            self.editor.redraw()
            self._refresh_stirs()

    def _enter_run(self):
        if not self.selected_traj_name.get():
            messagebox.showwarning("未选择轨迹", "请先在首页选择一条轨迹")
            return
        if not (self.machine and self.machine.homed):
            messagebox.showwarning("未复位", "请先执行复位（回零）")
            return
        self.nb.select(self.tab_run)

    # ---------- ② 操作页 ----------
    def _build_run(self):
        frm = ttk.Frame(self.tab_run, padding=16)
        frm.pack(fill="both", expand=True)

        top = ttk.Frame(frm)
        top.pack(fill="x", pady=(0, 8))
        ttk.Label(top, textvariable=self.selected_traj_name, font=("", 13, "bold")).pack(side="left")
        ttk.Label(top, textvariable=self.pos_var, font=("", 11)).pack(side="left", padx=20)
        ttk.Button(top, text="← 返回首页", command=lambda: self.nb.select(self.tab_home)).pack(side="right")

        # 只读曲线预览（执行时高亮当前段）
        self.preview = CurveEditor(frm, self.traj.tracks, self.traj.duration_s,
                                   self._travel(), self.traj.speed_limit_mm_s,
                                   readonly=True, height=360)
        self.preview.pack(fill="both", expand=True, pady=4)

        ctrl = ttk.Frame(frm)
        ctrl.pack(fill="x", pady=8)
        ttk.Label(ctrl, textvariable=self.busy_var).pack(side="left")
        self.run_btn = ttk.Button(ctrl, text="▶ 执行", command=self.on_run, width=12)
        self.run_btn.pack(side="right", padx=4)
        self.pause_btn = ttk.Button(ctrl, text="⏸ 暂停", command=self.on_pause,
                                    state="disabled", width=10)
        self.pause_btn.pack(side="right", padx=4)
        ttk.Button(ctrl, text="⏹ 停止", command=self.on_stop, width=10).pack(side="right", padx=4)

    def _travel(self):
        m = self.machine
        return {"X": (m.travel_x[1] - m.travel_x[0]) if m else DEFAULT_TRAVEL_X[1],
                "Y": (m.travel_y[1] - m.travel_y[0]) if m else DEFAULT_TRAVEL_Y[1]}

    def _refresh_run_preview(self):
        self.preview.set_tracks(self.traj.tracks, self.traj.duration_s)
        self.preview.set_stirs(self.traj.stirs)
        self.preview.limit = self.traj.speed_limit_mm_s
        self.preview.redraw()

    # ---------- ③ 轨迹管理页 ----------
    def _build_manage(self):
        frm = ttk.Frame(self.tab_manage, padding=16)
        frm.pack(fill="both", expand=True)

        top = ttk.Frame(frm)
        top.pack(fill="x", pady=(0, 6))
        ttk.Label(top, text="总时长(s)").pack(side="left")
        self.dur_var = tk.DoubleVar(value=self.traj.duration_s)
        ttk.Entry(top, textvariable=self.dur_var, width=6).pack(side="left", padx=2)
        ttk.Label(top, text="速度上限(mm/s)").pack(side="left", padx=(8, 2))
        self.limit_var = tk.DoubleVar(value=self.traj.speed_limit_mm_s)
        ttk.Entry(top, textvariable=self.limit_var, width=6).pack(side="left", padx=2)
        ttk.Button(top, text="应用参数", command=self.on_apply_curve_params).pack(side="left", padx=6)
        ttk.Label(top, text="拖动改位置/时间 · 双击输入 · 点击空白添加 · Delete 删除",
                  foreground="#888").pack(side="right")

        self.editor = CurveEditor(frm, self.traj.tracks, self.traj.duration_s,
                                  self._travel(), self.traj.speed_limit_mm_s,
                                  on_change=self._on_curve_changed)
        self.editor.pack(fill="both", expand=True, pady=4)

        jog = ttk.LabelFrame(frm, text="示教 / 点动")
        jog.pack(fill="x", pady=(6, 2))
        self.jog_axis_var = tk.StringVar(value="X")
        self.jog_pos_var = tk.StringVar(value="0.0")
        self.jog_step_var = tk.StringVar(value="5")
        ttk.Combobox(jog, textvariable=self.jog_axis_var, values=("X", "Y"),
                     width=5, state="readonly").pack(side="left", padx=4, pady=6)
        ttk.Label(jog, text="目标(mm)").pack(side="left")
        ttk.Entry(jog, textvariable=self.jog_pos_var, width=7).pack(side="left", padx=2)
        ttk.Button(jog, text="移动到位置", command=self.on_jog_to).pack(side="left", padx=6)
        ttk.Label(jog, text="步长(mm)").pack(side="left", padx=(8, 2))
        ttk.Entry(jog, textvariable=self.jog_step_var, width=5).pack(side="left")
        ttk.Button(jog, text="X-", width=4, command=lambda: self.on_jog_rel("X", -1)).pack(side="left", padx=2)
        ttk.Button(jog, text="X+", width=4, command=lambda: self.on_jog_rel("X", +1)).pack(side="left", padx=2)
        ttk.Button(jog, text="Y-", width=4, command=lambda: self.on_jog_rel("Y", -1)).pack(side="left", padx=2)
        ttk.Button(jog, text="Y+", width=4, command=lambda: self.on_jog_rel("Y", +1)).pack(side="left", padx=2)
        ttk.Button(jog, text="用当前实际位置更新选中关键帧",
                   command=self.on_update_selected).pack(side="right", padx=6)

        vib = ttk.LabelFrame(frm, text="搅拌段（Y 轴支线任务：该时间段内 Y 轴振动，X 照常运动）")
        vib.pack(fill="x", pady=(6, 2))
        self.stir_t0_var = tk.StringVar(value="5")
        self.stir_t1_var = tk.StringVar(value="15")
        self.stir_f_var = tk.StringVar(value="2")
        self.stir_a_var = tk.StringVar(value="5")
        ttk.Label(vib, text="开始s").pack(side="left", padx=(6, 2))
        ttk.Entry(vib, textvariable=self.stir_t0_var, width=5).pack(side="left")
        ttk.Label(vib, text="结束s").pack(side="left", padx=(6, 2))
        ttk.Entry(vib, textvariable=self.stir_t1_var, width=5).pack(side="left")
        ttk.Label(vib, text="频率Hz").pack(side="left", padx=(6, 2))
        ttk.Entry(vib, textvariable=self.stir_f_var, width=5).pack(side="left")
        ttk.Label(vib, text="振幅±mm").pack(side="left", padx=(6, 2))
        ttk.Entry(vib, textvariable=self.stir_a_var, width=5).pack(side="left")
        ttk.Button(vib, text="添加搅拌段", command=self.on_stir_add).pack(side="left", padx=8)
        ttk.Button(vib, text="删除选中", command=self.on_stir_del).pack(side="left", padx=4)
        self.stir_list = tk.Listbox(vib, height=3, width=36)
        self.stir_list.pack(side="left", padx=8, pady=4)

        save = ttk.Frame(frm)
        save.pack(fill="x", pady=4)
        ttk.Label(save, text="轨迹名").pack(side="left")
        ttk.Entry(save, textvariable=self.selected_traj_name, width=14).pack(side="left", padx=2)
        ttk.Button(save, text="保存轨迹", command=self.on_save).pack(side="left", padx=4)
        ttk.Button(save, text="从文件加载", command=self.on_load).pack(side="left", padx=4)
        ttk.Button(save, text="新建轨迹", command=self._new_trajectory).pack(side="left", padx=4)
        ttk.Button(save, text="清空", command=self.on_clear_tracks).pack(side="left", padx=4)
        ttk.Button(save, text="回到首页", command=lambda: self.nb.select(self.tab_home)).pack(side="right")

        self._build_advanced(frm)

    def _on_curve_changed(self):
        self.traj.tracks = self.editor.tracks
        self._refresh_run_preview()

    def _y_top(self) -> float:
        """UI 坐标下 Y 的顶部位置（= 行程上限，回零位置）"""
        return self.machine.travel_y[1] if self.machine else DEFAULT_TRAVEL_Y[1]

    def _empty_tracks(self, dur: float) -> dict:
        """空轨迹：X 在 0，Y 落在最上方（回零后的停靠位置）"""
        ytop = self._y_top()
        return {"X": [[0.0, 0.0], [dur, 0.0]],
                "Y": [[0.0, ytop], [dur, ytop]]}

    def _new_trajectory(self):
        """新建空轨迹（默认端点 + 默认参数）"""
        self.traj = Trajectory()
        self.traj.tracks = self._empty_tracks(self.traj.duration_s)
        self.selected_traj_name.set("未命名")
        self.dur_var.set(self.traj.duration_s)
        self.limit_var.set(self.traj.speed_limit_mm_s)
        self.editor.set_tracks(self.traj.tracks, self.traj.duration_s)
        self.editor.limit = self.traj.speed_limit_mm_s
        self.editor.redraw()
        self._refresh_stirs()
        self._refresh_run_preview()
        self.log("已新建空轨迹")

    def on_clear_tracks(self):
        """清空当前轨迹内容（保留时长/限速/名称），Y 线落到最上方"""
        dur = self.traj.duration_s
        self.traj.tracks = self._empty_tracks(dur)
        self.traj.stirs = []
        self.editor.set_tracks(self.traj.tracks, dur)
        self.editor.redraw()
        self._refresh_stirs()
        self._refresh_run_preview()
        self.log("轨迹已清空")

    def on_apply_curve_params(self):
        try:
            new_dur = float(self.dur_var.get())
            new_limit = float(self.limit_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "时长/限速必须是数字")
            return
        old_dur = self.traj.duration_s
        self.traj.duration_s = new_dur
        self.traj.speed_limit_mm_s = new_limit
        for ax in ("X", "Y"):
            tr = self.traj.tracks[ax]
            if tr and abs(tr[-1][0] - old_dur) < 0.05:
                tr[-1][0] = new_dur
        self.editor.set_tracks(self.traj.tracks, new_dur)
        self.editor.limit = new_limit
        self.editor.redraw()
        self._refresh_stirs()
        self._refresh_run_preview()

    def on_update_selected(self):
        if not self._need_machine():
            return
        sel = self.editor.get_selected()
        if not sel:
            messagebox.showwarning("未选择", "请先在曲线上选中一个关键帧")
            return
        threading.Thread(target=self._update_sel_worker, args=(sel[0],), daemon=True).start()

    def _update_sel_worker(self, axis):
        try:
            with self.ops_lock:
                x, y = self.machine.position()
        except LinkError as e:
            self._ui("error", "读取位置失败", str(e))
            return
        self._ui("update_sel", axis,
                 x if axis == "X" else self._y_disp(y))

    # ================= 搅拌段（Y 轴支线任务） =================

    def _refresh_stirs(self):
        """搅拌段列表 + 编辑器/预览搅拌带同步"""
        self.stir_list.delete(0, "end")
        for s in sorted(self.traj.stirs, key=lambda s: s["t0"]):
            self.stir_list.insert("end",
                                  f"{s['t0']:.1f}~{s['t1']:.1f}s  "
                                  f"{s['freq_hz']:g}Hz ±{s['amp_mm']:g}mm")
        self.editor.set_stirs(self.traj.stirs)
        self._refresh_run_preview()

    def on_stir_add(self):
        try:
            t0 = float(self.stir_t0_var.get())
            t1 = float(self.stir_t1_var.get())
            f = float(self.stir_f_var.get())
            a = float(self.stir_a_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "时间/频率/振幅必须是数字")
            return
        errs = validate_stirs(self.traj, self._y_top(),
                              extra={"t0": t0, "t1": t1, "freq_hz": f, "amp_mm": a})
        if errs:
            messagebox.showerror("搅拌段无效", "\n".join(errs))
            return
        self.traj.stirs.append({"t0": t0, "t1": t1, "freq_hz": f, "amp_mm": a})
        self._refresh_stirs()
        self.log(f"已添加搅拌段: {t0:g}~{t1:g}s {f:g}Hz ±{a:g}mm")

    def on_stir_del(self):
        sel = self.stir_list.curselection()
        if not sel:
            messagebox.showwarning("未选择", "请先在列表中选中搅拌段")
            return
        srt = sorted(self.traj.stirs, key=lambda s: s["t0"])
        del srt[sel[0]]
        self.traj.stirs = srt
        self._refresh_stirs()
        self.log("已删除搅拌段")

    def _build_advanced(self, parent):
        self.adv_visible = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent, text="高级设置（正常使用无需修改）", variable=self.adv_visible,
                        command=self._toggle_adv).pack(anchor="w", pady=(6, 0))
        self.adv = ttk.LabelFrame(parent, text="高级设置")
        self.lead_var = tk.DoubleVar(value=DEFAULT_LEAD_MM)
        self.ppr_var = tk.IntVar(value=DEFAULT_PULSES_REV)
        self.accel_var = tk.IntVar(value=DEFAULT_ACCEL)
        self.tx0_var = tk.DoubleVar(value=DEFAULT_TRAVEL_X[0])
        self.tx1_var = tk.DoubleVar(value=DEFAULT_TRAVEL_X[1])
        self.ty0_var = tk.DoubleVar(value=DEFAULT_TRAVEL_Y[0])
        self.ty1_var = tk.DoubleVar(value=DEFAULT_TRAVEL_Y[1])
        self.xinv_var = tk.BooleanVar(value=True)
        self.yinv_var = tk.BooleanVar(value=True)
        self.y3inv_var = tk.BooleanVar(value=False)
        g = dict(padx=3, pady=2)
        r = 0
        ttk.Label(self.adv, text="导程mm").grid(row=r, column=0, sticky="e", **g)
        ttk.Entry(self.adv, textvariable=self.lead_var, width=6).grid(row=r, column=1, **g)
        ttk.Label(self.adv, text="每圈脉冲").grid(row=r, column=2, sticky="e", **g)
        ttk.Entry(self.adv, textvariable=self.ppr_var, width=6).grid(row=r, column=3, **g)
        ttk.Label(self.adv, text="加速度档位").grid(row=r, column=4, sticky="e", **g)
        ttk.Entry(self.adv, textvariable=self.accel_var, width=5).grid(row=r, column=5, **g)
        r += 1
        ttk.Label(self.adv, text="X行程min/max").grid(row=r, column=0, sticky="e", **g)
        ttk.Entry(self.adv, textvariable=self.tx0_var, width=6).grid(row=r, column=1, **g)
        ttk.Entry(self.adv, textvariable=self.tx1_var, width=6).grid(row=r, column=2, **g)
        ttk.Label(self.adv, text="Y行程min/max").grid(row=r, column=3, sticky="e", **g)
        ttk.Entry(self.adv, textvariable=self.ty0_var, width=6).grid(row=r, column=4, **g)
        ttk.Entry(self.adv, textvariable=self.ty1_var, width=6).grid(row=r, column=5, **g)
        r += 1
        ttk.Checkbutton(self.adv, text="X反向", variable=self.xinv_var).grid(row=r, column=0, **g)
        ttk.Checkbutton(self.adv, text="Y反向", variable=self.yinv_var).grid(row=r, column=1, **g)
        ttk.Checkbutton(self.adv, text="电机3镜像", variable=self.y3inv_var).grid(row=r, column=2, columnspan=2, **g)
        ttk.Button(self.adv, text="应用", command=self.on_apply_adv).grid(row=r, column=4, **g)

    def _build_log(self):
        self.log_visible = tk.BooleanVar(value=False)
        ttk.Checkbutton(self, text="运行日志", variable=self.log_visible,
                        command=self._toggle_log).pack(anchor="w", padx=8)
        self.logbox = scrolledtext.ScrolledText(self, height=5, state="disabled")

    def _toggle_adv(self):
        if self.adv_visible.get():
            self.adv.pack(fill="x", padx=4, pady=4)
        else:
            self.adv.pack_forget()

    def _toggle_log(self):
        if self.log_visible.get():
            self.logbox.pack(fill="x", padx=6, pady=2)
        else:
            self.logbox.pack_forget()

    # ================= 通用 =================

    def log(self, msg: str):
        self.logbox.configure(state="normal")
        self.logbox.insert("end", f"{time.strftime('%H:%M:%S')} {msg}\n")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    def _need_machine(self) -> bool:
        if not self.connected or not self.machine:
            messagebox.showwarning("未连接", "请先连接设备")
            return False
        return True

    def _set_busy(self, text: str):
        self.busy_var.set(text)

    # ---------- Y 轴 UI 镜像（顶部=上限、底部=0、向上为正） ----------
    def _y_disp(self, y_machine: float) -> float:
        """机器坐标 -> UI 显示坐标"""
        return self.machine.travel_y[1] - y_machine

    def _y_machine(self, y_ui: float) -> float:
        """UI 坐标 -> 机器坐标"""
        return self.machine.travel_y[1] - y_ui

    # ================= 连接 =================

    def on_connect(self):
        if self.connected:
            self.on_close()
            return
        self.conn_btn.configure(state="disabled")
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        try:
            link = X42Link(self.port_var.get())
            machine = Machine(link)   # 内部会发 GUARD 0
        except Exception as e:
            self._ui("conn_failed", str(e))
            return
        self.link = link
        self.machine = machine
        self._ui("connected")
        self._ui("log", "已连接 " + self.port_var.get())
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def _poll_loop(self):
        while self.connected and self.machine:
            try:
                with self.ops_lock:
                    x, y = self.machine.position()
                self._ui("pos", x, self._y_disp(y))
            except Exception:
                pass
            time.sleep(0.3)

    # ================= 回零 =================

    def on_home(self):
        if not self._need_machine() or self.homing:
            return
        self.homing = True
        self.home_btn.configure(state="disabled")
        self.home_state_var.set("⏳ 复位中，请勿操作...")
        threading.Thread(target=self._home_worker, daemon=True).start()

    def _home_worker(self):
        with self.ops_lock:
            try:
                self.machine.home()
                self._ui("home_state", "✅ 设备已就绪")
                self._ui("log", "复位（回零）完成")
            except LinkError as e:
                self._ui("home_state", "❌ 复位失败，请重试")
                self._ui("log", f"复位失败: {e}")
                self._ui("error", "复位", str(e))
            finally:
                self.homing = False

    # ================= 示教/点动 =================

    def on_jog_to(self):
        if not self._need_machine():
            return
        try:
            pos = float(self.jog_pos_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "位置必须是数字")
            return
        axis = self.jog_axis_var.get()
        threading.Thread(target=self._jog_worker, args=(axis, pos), daemon=True).start()

    def on_jog_rel(self, axis, sign):
        if not self._need_machine():
            return
        try:
            step = float(self.jog_step_var.get()) * sign
        except ValueError:
            messagebox.showerror("参数错误", "步长必须是数字")
            return
        threading.Thread(target=self._jog_rel_worker, args=(axis, step), daemon=True).start()

    def _jog_rel_worker(self, axis, delta):
        with self.ops_lock:
            self._ui("busy", "点动中...")
            try:
                x, y = self.machine.position()
                if axis == "X":
                    disp = x + delta
                    target = disp
                else:
                    disp = self._y_disp(y) + delta   # UI 坐标：Y+ 向上
                    target = self._y_machine(disp)
                self.machine.move_axis(axis, target, self.speed_var.get())
                self._ui("log", f"点动完成 {axis} -> {disp:.1f}mm")
            except LinkError as e:
                self._ui("log", f"点动失败: {e}")
            finally:
                self._ui("busy", "空闲")

    def _jog_worker(self, axis, pos):
        with self.ops_lock:
            self._ui("busy", "运动中...")
            try:
                target = pos if axis == "X" else self._y_machine(pos)  # 输入为 UI 坐标
                self.machine.move_axis(axis, target, self.speed_var.get())
                self._ui("log", f"已到达 {axis} = {pos:.1f}mm")
            except LinkError as e:
                self._ui("log", f"运动失败: {e}")
            finally:
                self._ui("busy", "空闲")

    # ================= 执行 =================

    def on_run(self):
        if not self._need_machine():
            return
        if not self.machine.homed:
            messagebox.showwarning("未复位", "请先在首页执行复位")
            return
        if any(len(self.traj.tracks[ax]) < 2 for ax in ("X", "Y")):
            messagebox.showwarning("空轨迹", "轨迹没有内容")
            return
        errs = validate_stirs(self.traj, self._y_top())
        if errs:
            messagebox.showerror("搅拌段校验失败", "\n".join(errs))
            return
        if self.exec_thread and self.exec_thread.is_alive():
            return
        self.stop_event.clear()
        self.pause_event.clear()
        self._ui("run_btns", "disabled", "normal")
        self.exec_thread = threading.Thread(target=self._run_worker, daemon=True)
        self.exec_thread.start()

    def _run_worker(self):
        with self.ops_lock:
            self._ui("busy", "执行轨迹")
            stirs = sorted(self.traj.stirs, key=lambda s: s["t0"])
            stir_times = [t for s in stirs for t in (s["t0"], s["t1"])]
            segs = build_segments(self.traj.tracks, self.traj.duration_s,
                                  extra_times=tuple(stir_times))
            error_message = None
            try:
                for i, (t0, t1, x0, y0, x1, y1) in enumerate(segs):
                    if self.stop_event.is_set():
                        self._ui("log", "轨迹已停止")
                        return
                    while self.pause_event.is_set() and not self.stop_event.is_set():
                        time.sleep(0.2)
                    if self.stop_event.is_set():
                        self._ui("log", "轨迹已停止")
                        return
                    # 搅拌窗口内：Y 被振动占用（曲线已校验平坦），X 照常运动
                    stir = next((s for s in stirs
                                 if s["t0"] - 1e-6 <= t0 < s["t1"] - 1e-6), None)
                    if stir:
                        self._run_stir_slice(stir, t0, t1, x0, x1)
                        continue
                    dx, dy = x1 - x0, y1 - y0
                    # 兜底：钳位到行程内（防止手工改 JSON / 旧数据越界）
                    x1c = min(max(x1, 0.0), self.machine.travel_x[1])
                    y1c = min(max(y1, 0.0), self.machine.travel_y[1])
                    if x1c != x1 or y1c != y1:
                        self._ui("log", f"  段 {i + 1} 目标越界已钳位: ({x1:.1f},{y1:.1f}) -> ({x1c:.1f},{y1c:.1f})")
                        x1, y1 = x1c, y1c
                        dx, dy = x1 - x0, y1 - y0
                    if abs(dx) < 0.02 and abs(dy) < 0.02:
                        continue   # 静止段跳过
                    self._ui("highlight", t0, t1)
                    self._ui("log", f"→ 段 {i + 1}: {t0:.1f}s~{t1:.1f}s -> ({x1:.1f}, {y1:.1f})")
                    dist = (dx * dx + dy * dy) ** 0.5
                    speed = min(dist / max(t1 - t0, 0.05), self.traj.speed_limit_mm_s)
                    while True:
                        if self.stop_event.is_set():
                            raise LinkError("已停止")
                        try:
                            # 曲线为 UI 坐标，Y 换算成机器坐标下发
                            self.machine.move_to(x1, self._y_machine(y1), speed,
                                                 stop_check=self.stop_event.is_set)
                            break
                        except LinkError as e:
                            if self.pause_event.is_set():
                                while self.pause_event.is_set() and not self.stop_event.is_set():
                                    time.sleep(0.2)
                                continue
                            raise
                    self._ui("log", f"  段 {i + 1} 完成")
                self._ui("log", "轨迹执行完成")
            except LinkError as e:
                if self.stop_event.is_set():
                    self._ui("log", "轨迹已停止")
                else:
                    self._ui("log", f"执行中断: {e}")
                    error_message = str(e)
            finally:
                self._ui("busy", "空闲")
                self._ui("run_btns", "normal", "disabled")
                self._ui("clear_progress")
                # Queue cleanup before the modal dialog so the UI is usable
                # immediately after the operator acknowledges the error.
                if error_message:
                    self._ui("error", "执行", error_message)

    def _run_stir_slice(self, stir, t0, t1, x0, x1):
        """搅拌窗口内的一个时间片：起点启动振动；X 照常运动；终点等振动结束"""
        start = abs(t0 - stir["t0"]) < 1e-6
        end = abs(t1 - stir["t1"]) < 1e-6
        if start:
            # Keep an independent physical reference. A lost FD acknowledgement
            # is tolerated only when the mechanism demonstrably returns here.
            _, self._stir_center_y = self.machine.position()
            dur_s = max(1, int(round(stir["t1"] - stir["t0"])))
            self._ui("highlight", stir["t0"], stir["t1"])
            self._ui("log", f"→ 搅拌 {stir['t0']:.1f}~{stir['t1']:.1f}s: "
                            f"{stir['freq_hz']:g}Hz ±{stir['amp_mm']:g}mm")
            self.machine.vib_start("Y", stir["freq_hz"], stir["amp_mm"], dur_s,
                                   mirror=self.y3inv_var.get())
        # 窗口内 X 照常运动（Y 曲线平坦，无 Y 位移）
        if abs(x1 - x0) >= 0.02:
            spd = min(abs(x1 - x0) / max(t1 - t0, 0.05),
                      self.traj.speed_limit_mm_s)
            self.machine.move_axis("X", x1, spd,
                                   stop_check=self.stop_event.is_set)
        if end:
            # 等固件振动自然结束（定时驱动，偶数半周期后回中心）
            st = 1
            deadline = time.monotonic() + max(15.0, (t1 - t0) + 10.0)
            query_error = None
            while not self.stop_event.is_set():
                if time.monotonic() >= deadline:
                    detail = f"（最后错误：{query_error}）" if query_error else ""
                    self.machine.vib_stop()
                    self._ui("log", "搅拌状态查询超时，已请求结束振动，继续执行轨迹" + detail)
                    self._stir_center_y = None
                    return
                try:
                    st, cyc, ms, phase, code = self.machine.vib_state()
                    query_error = None
                except LinkError as e:
                    # The vibration task keeps running independently on the ESP32.
                    # Do not interrupt a normal physical motion for one lost read.
                    query_error = str(e)
                    time.sleep(0.3)
                    continue
                if st in (2, 3, 4):
                    break
                time.sleep(0.3)
            if self.stop_event.is_set():
                raise LinkError("已停止")
            af = (cyc / 2.0) / (ms / 1000.0) if ms > 0 else 0.0
            if st == 3:
                if phase.startswith("CLOG_"):
                    raise LinkError(f"搅拌失败：{phase} 堵转保护触发（驱动器错误码 {code}）")
                tolerated, detail = self._tolerate_vib_comm_fault(phase, code, cyc)
                if not tolerated:
                    raise LinkError(detail)
                self._ui("log", "搅拌通信告警已恢复：" + detail)
            self._ui("log", f"  搅拌段完成（实际 {af:.1f}Hz）")
            self._stir_center_y = None

    def _tolerate_vib_comm_fault(self, phase, code, cycles):
        """Treat post-motion VIB transport faults as non-blocking warnings."""
        if phase.startswith("CLOG_"):
            return False, f"搅拌失败：{phase} 堵转保护触发（驱动器错误码 {code}）"
        if cycles > 0:
            return True, f"阶段 {phase} 应答异常（错误码 {code}），已完成 {cycles} 个半周期"
        return False, f"搅拌未确认启动：阶段 {phase}，错误码 {code}"

    def on_pause(self):
        self.pause_event.set()
        if self.machine:
            self.machine.stop()
        self.log("暂停请求")

    def on_stop(self):
        self.stop_event.set()
        if self.machine:
            self.machine.stop()
        self.log("停止请求")

    # ================= 保存/加载 =================

    def on_save(self):
        name = self.selected_traj_name.get() or "未命名"
        self.traj.name = name
        os.makedirs(TRAJ_DIR, exist_ok=True)
        path = os.path.join(TRAJ_DIR, f"{name}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.traj.to_dict(), f, ensure_ascii=False, indent=2)
        self.log(f"轨迹已保存: {path}")
        self._refresh_traj_list()
        self.selected_traj_name.set(name)

    def on_load(self):
        name = self.selected_traj_name.get()
        path = os.path.join(TRAJ_DIR, f"{name}.json")
        if not os.path.exists(path):
            messagebox.showerror("文件不存在", path)
            return
        with open(path, encoding="utf-8") as f:
            self.traj = Trajectory.from_dict(json.load(f))
        self.dur_var.set(self.traj.duration_s)
        self.limit_var.set(self.traj.speed_limit_mm_s)
        self.editor.set_tracks(self.traj.tracks, self.traj.duration_s)
        self.editor.limit = self.traj.speed_limit_mm_s
        self.editor.redraw()
        self._refresh_stirs()
        self.log(f"轨迹已加载: {path}")

    # ================= 高级设置 =================

    def on_apply_adv(self):
        if not self.machine:
            return
        m = self.machine
        try:
            m.lead_mm = self.lead_var.get()
            m.pulses_per_rev = self.ppr_var.get()
            m.accel = self.accel_var.get()
            m.travel_x = (self.tx0_var.get(), self.tx1_var.get())
            m.travel_y = (self.ty0_var.get(), self.ty1_var.get())
            m.x_invert = self.xinv_var.get()
            m.y_invert = self.yinv_var.get()
            m.y3_invert = self.y3inv_var.get()
        except Exception as e:
            messagebox.showerror("参数错误", str(e))
            return
        self.log("高级参数已应用")

    # ================= 关闭 =================

    def on_close(self):
        self.stop_event.set()
        self.connected = False
        if self.link:
            self.link.close()
            self.link = None
        self.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="X42 user interface")
    parser.add_argument("--port", help="controller serial port, for example COM4 or /dev/ttyACM0")
    App(parser.parse_args().port).mainloop()
