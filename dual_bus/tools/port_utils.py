"""串口自动识别：优先找 ESP32-S3 的原生 USB 串口。

ESP32-S3 通过 USB Serial/JTAG 接入电脑时，厂商 ID 固定是 0x303A
（Espressif），设备描述里通常含 "Espressif" 或 "JTAG"。
板子上如果还接了 CH340 之类的 USB-TTL，会出现第二个串口，需要区分开。
"""

from __future__ import annotations

import os
import re

from serial.tools import list_ports

ESPRESSIF_VID = 0x303A

# udev 规则里给 ESP32-S3 建的固定别名（见 README）。
# 有它的话，板子重新枚举成 ttyACM1/ACM2 也不会影响使用。
STABLE_LINK = "/dev/esp32s3"

# 主板自带的 legacy 串口（/dev/ttyS0..ttyS31），对我们的用途没有意义，
# 留着只会把下拉框撑满，直接过滤掉。
_LEGACY_TTY = re.compile(r"/dev/ttyS\d+$")


def stable_port() -> str:
    """若存在固定别名则返回它，否则返回空串。"""
    return STABLE_LINK if os.path.exists(STABLE_LINK) else ""


def list_serial_ports() -> list[tuple[str, str]]:
    """返回 [(设备名, 描述), ...]，按设备名排序；固定别名排在最前。"""
    out = []
    link = stable_port()
    if link:
        out.append((link, "ESP32-S3 固定别名（推荐）"))

    for p in list_ports.comports():
        if _LEGACY_TTY.match(p.device):
            continue
        if link and os.path.realpath(p.device) == os.path.realpath(link):
            continue        # 已经用别名了，不用再重复列一遍
        desc = p.description or ""
        if p.vid is not None:
            desc = f"{desc} [{p.vid:04X}:{p.pid:04X}]" if p.pid is not None else desc
        out.append((p.device, desc))
    return out


def _is_espressif(port) -> bool:
    text = " ".join(
        filter(None, (port.description, port.manufacturer, port.product))
    ).lower()
    return port.vid == ESPRESSIF_VID or "espressif" in text or "jtag" in text


def default_port() -> str:
    """挑一个最可能是 ESP32-S3 的串口；识别不出就返回空串。"""
    link = stable_port()
    if link:
        return link
    ports = list(list_ports.comports())
    for port in ports:
        if _is_espressif(port):
            return port.device
    # 只有一个串口时，基本可以确定就是它
    if len(ports) == 1:
        return ports[0].device
    return ""
