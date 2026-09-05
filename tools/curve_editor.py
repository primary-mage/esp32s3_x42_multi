#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
时间轴曲线编辑器（剪辑软件风格）

- 两条轨道：X 轴（上半）、Y 轴（下半），横轴=时间(s)，纵轴=距离(mm)
- 关键帧：点击曲线空白处添加（位置取插值，不改变曲线形状）
          拖动改时间/位置（端点锁定在 0s 与总时长）
          双击输入精确值
          选中后 Delete 删除
- 相邻关键帧直线插值；段速度 = Δ位置/Δ时间，超过限速的段标红
- readonly 模式用于操作页预览；set_highlight/set_playhead 显示执行进度
"""
from __future__ import annotations

import tkinter as tk
from tkinter import simpledialog


def interp_track(track: list, t: float) -> float:
    """曲线插值：track = 按时间排序的 [[t, pos], ...]"""
    if not track:
        return 0.0
    if t <= track[0][0]:
        return track[0][1]
    if t >= track[-1][0]:
        return track[-1][1]
    for i in range(len(track) - 1):
        t0, p0 = track[i]
        t1, p1 = track[i + 1]
        if t0 <= t <= t1:
            if t1 == t0:
                return p1
            return p0 + (p1 - p0) * (t - t0) / (t1 - t0)
    return track[-1][1]


def normalize_tracks(tracks: dict) -> dict:
    """按时间排序，同一时间只保留最后一个点（去重复）"""
    out = {}
    for ax in ("X", "Y"):
        pts = sorted(tracks.get(ax, []), key=lambda p: p[0])
        dedup = []
        for t, p in pts:
            if dedup and abs(dedup[-1][0] - t) < 1e-6:
                dedup[-1][1] = p
            else:
                dedup.append([float(t), float(p)])
        out[ax] = dedup
    return out


def build_segments(tracks: dict, duration_s: float) -> list:
    """合并两轴关键帧时间点 -> 段列表 [(t0, t1, x0, y0, x1, y1), ...]"""
    tracks = normalize_tracks(tracks)
    times = {0.0, duration_s}
    for ax in ("X", "Y"):
        for t, _ in tracks.get(ax, []):
            times.add(t)
    times = sorted(times)
    segs = []
    for i in range(len(times) - 1):
        t0, t1 = times[i], times[i + 1]
        x0 = interp_track(tracks.get("X", []), t0)
        x1 = interp_track(tracks.get("X", []), t1)
        y0 = interp_track(tracks.get("Y", []), t0)
        y1 = interp_track(tracks.get("Y", []), t1)
        segs.append((t0, t1, x0, y0, x1, y1))
    return segs


class CurveEditor(tk.Canvas):
    AXES = ("X", "Y")
    COLORS = {"X": "#2b7de9", "Y": "#2ca02c"}

    def __init__(self, parent, tracks: dict, duration_s: float,
                 travel: dict, speed_limit_mm_s: float = 40.0,
                 readonly: bool = False, on_change=None,
                 width: int = 780, height: int = 460):
        super().__init__(parent, width=width, height=height, bg="#1e1e1e",
                         highlightthickness=0)
        self.tracks = normalize_tracks(tracks)
        self.duration_s = duration_s
        self.travel = travel            # {"X": 180.0, "Y": 180.0}
        self.limit = speed_limit_mm_s
        self.readonly = readonly
        self.on_change = on_change
        self.width = width
        self.height = height
        self.sel = None                 # (axis, index)
        self.drag_offset = None         # (dt, dpos)
        self.highlight = None           # (t0, t1)
        self.playhead = None            # t
        if not readonly:
            self.bind("<Button-1>", self._on_press)
            self.bind("<B1-Motion>", self._on_drag)
            self.bind("<ButtonRelease-1>", self._on_release)
            self.bind("<Double-Button-1>", self._on_double)
            self.bind("<Delete>", lambda e: self.delete_selected())
            self.bind("<BackSpace>", lambda e: self.delete_selected())
        self.redraw()

    # ---------- 坐标映射 ----------
    def _lane_rect(self, axis):
        """轨道矩形 (x0, y0, x1, y1)：左边留刻度，内边距"""
        ml, mt, mr, mb = 44, 14, 16, 8
        half = self.height / 2
        y0 = mt if axis == "X" else half + mt
        y1 = half - mb if axis == "X" else self.height - mb
        return ml, y0, self.width - mr, y1

    def _t2x(self, t):
        x0, _, x1, _ = self._lane_rect("X")
        return x0 + (t / self.duration_s) * (x1 - x0)

    def _x2t(self, x):
        x0, _, x1, _ = self._lane_rect("X")
        t = (x - x0) / (x1 - x0) * self.duration_s
        return min(max(t, 0.0), self.duration_s)

    def _pos2y(self, axis, pos):
        _, y0, _, y1 = self._lane_rect(axis)
        tr = self.travel.get(axis, 200.0)
        return y1 - (pos / tr) * (y1 - y0)

    def _y2pos(self, axis, y):
        _, y0, _, y1 = self._lane_rect(axis)
        tr = self.travel.get(axis, 200.0)
        return min(max((y1 - y) / (y1 - y0) * tr, 0.0), tr)

    def _hit(self, x, y):
        best, bd = None, 9
        for ax in self.AXES:
            for i, (t, p) in enumerate(self.tracks[ax]):
                d = ((self._t2x(t) - x) ** 2 + (self._pos2y(ax, p) - y) ** 2) ** 0.5
                if d < bd:
                    best, bd = (ax, i), d
        return best

    # ---------- 绘制 ----------
    def redraw(self):
        self.delete("all")
        for ax in self.AXES:
            self._draw_lane(ax)
        if self.highlight:
            self._draw_highlight(*self.highlight)
        if self.playhead is not None:
            x = self._t2x(self.playhead)
            self.create_line(x, 8, x, self.height - 4, fill="#ff5555", width=2)
        for ax in self.AXES:
            self._draw_curve(ax)
            self._draw_points(ax)

    def _draw_lane(self, axis):
        x0, y0, x1, y1 = self._lane_rect(axis)
        tr = self.travel.get(axis, 200.0)
        self.create_text(12, (y0 + y1) / 2, text=axis, fill="#fff", font=("", 12, "bold"))
        # 网格：1s 竖线、10mm 横线
        for t in range(0, int(self.duration_s) + 1, 1):
            x = self._t2x(t)
            self.create_line(x, y0, x, y1, fill="#333")
            if t % 5 == 0:
                self.create_text(x, y1 + 8, text=str(t), fill="#999", font=("", 8))
        for mm in range(0, int(tr) + 1, 10):
            y = self._pos2y(axis, mm)
            self.create_line(x0, y, x1, y, fill="#333")
            self.create_text(x0 - 4, y, text=str(mm), fill="#999", font=("", 8), anchor="e")
        # 边框
        self.create_rectangle(x0, y0, x1, y1, outline="#555")

    def _draw_curve(self, axis):
        tr = self.tracks[axis]
        if len(tr) < 2:
            return
        pts = [self._t2x(t) for t, _ in tr] + []
        for i in range(len(tr) - 1):
            t0, p0 = tr[i]
            t1, p1 = tr[i + 1]
            speed = abs(p1 - p0) / max(t1 - t0, 1e-6)
            color = "#ff6666" if speed > self.limit else self.COLORS[axis]
            self.create_line(self._t2x(t0), self._pos2y(axis, p0),
                             self._t2x(t1), self._pos2y(axis, p1),
                             fill=color, width=2)

    def _draw_points(self, axis):
        for i, (t, p) in enumerate(self.tracks[axis]):
            x, y = self._t2x(t), self._pos2y(axis, p)
            fill = "#fff" if self.sel == (axis, i) else self.COLORS[axis]
            r = 7
            self.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline="#000")
            if self.sel == (axis, i):
                self.create_text(x, y - 14, text=f"{p:.1f}mm",
                                 fill="#fff", font=("", 9))

    def _draw_highlight(self, t0, t1):
        x0, _, x1, _ = self._lane_rect("X")
        a = self._t2x(t0)
        b = self._t2x(t1)
        self.create_rectangle(a, 6, b, self.height - 6, fill="#ffffff",
                              stipple="gray25", outline="")

    # ---------- 交互 ----------
    def _on_press(self, ev):
        hit = self._hit(ev.x, ev.y)
        if hit:
            self.sel = hit
            ax, i = hit
            t, p = self.tracks[ax][i]
            self.drag_offset = (t - self._x2t(ev.x), p - self._y2pos(ax, ev.y))
        else:
            ax = "X" if ev.y < self.height / 2 else "Y"
            t = round(self._x2t(ev.x), 1)
            p = interp_track(self.tracks[ax], t)
            p = min(max(p, 0.0), self.travel.get(ax, 200.0))
            if t == 0.0 or t == round(self.duration_s, 1):
                self.redraw()
                return   # 端点已存在
            self.tracks[ax].append([t, p])
            self.tracks[ax].sort(key=lambda q: q[0])
            self.sel = None
            self._notify()
        self.redraw()

    def _on_drag(self, ev):
        if not self.sel or not self.drag_offset:
            return
        ax, i = self.sel
        t0 = self.tracks[ax][i][0]
        if t0 <= 0.01 or t0 >= self.duration_s - 0.01:
            t = t0   # 端点时间锁定
        else:
            t = round(self._x2t(ev.x) + self.drag_offset[0], 1)
            t = min(max(t, 0.01), self.duration_s - 0.01)
        p = round(self._y2pos(ax, ev.y) + self.drag_offset[1], 1)
        p = min(max(p, 0.0), self.travel.get(ax, 200.0))
        self.tracks[ax][i] = [t, p]
        self.tracks[ax].sort(key=lambda q: q[0])
        if self.tracks[ax][i] != [t, p]:
            self.sel = None
        self.redraw()

    def _on_release(self, ev):
        self.drag_offset = None
        self._notify()

    def _on_double(self, ev):
        hit = self._hit(ev.x, ev.y)
        if not hit:
            return
        ax, i = hit
        t, p = self.tracks[ax][i]
        s = simpledialog.askstring("关键帧", f"时间(s),位置(mm)\n例如 {t},{p}",
                                   initialvalue=f"{t},{p}", parent=self)
        if not s:
            return
        try:
            parts = [v.strip() for v in s.split(",")]
            nt = float(parts[0])
            np_ = float(parts[1])
        except Exception:
            return
        nt = min(max(nt, 0.0), self.duration_s)
        np_ = min(max(np_, 0.0), self.travel.get(ax, 200.0))
        self.tracks[ax][i] = [round(nt, 1), round(np_, 1)]
        self.tracks[ax].sort(key=lambda q: q[0])
        self._notify()
        self.redraw()

    def delete_selected(self):
        if not self.sel:
            return
        ax, i = self.sel
        t = self.tracks[ax][i][0]
        if t <= 0.01 or t >= self.duration_s - 0.01:
            return   # 端点不可删
        del self.tracks[ax][i]
        self.sel = None
        self._notify()
        self.redraw()

    def _notify(self):
        if self.on_change:
            self.on_change()

    # ---------- 外部接口 ----------
    def get_selected(self):
        return self.sel

    def set_selected_pos(self, pos: float):
        """把选中关键帧的位置设为指定值（示教用）"""
        if not self.sel:
            return
        ax, i = self.sel
        tr = self.travel.get(ax, 200.0)
        self.tracks[ax][i][1] = min(max(pos, 0.0), tr)
        self._notify()
        self.redraw()

    def set_tracks(self, tracks, duration_s):
        self.tracks = normalize_tracks(tracks)
        self.duration_s = duration_s
        self.sel = None
        self.redraw()

    def set_highlight(self, t0, t1):
        self.highlight = (t0, t1)
        self.redraw()

    def set_playhead(self, t):
        self.playhead = t
        self.redraw()

    def clear_progress(self):
        self.highlight = None
        self.playhead = None
        self.redraw()
