#!/usr/bin/env python3
"""ESP32 固件模拟器：用 PTY 虚拟串口扮演双总线固件，用于无硬件验证 UI。"""
import os, pty, select, threading, time, tty


class FakeFirmware:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        tty.setraw(self.slave)          # 关掉回显，否则自己的输出会被当命令读回
        self.path = os.ttyname(self.slave)
        self.motors = {1: dict(bus="A", addr=1, fw=207, hw=8980),
                       1001: dict(bus="B", addr=1, fw=207, hw=8980)}
        self.state = {i: dict(en=1, inpos=1, stall=0, clog=0, pos=0, speed=0)
                      for i in self.motors}
        self.io_in, self.io_out = 0, 0
        self.cmds = []
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()
        self._send("CMD> READY\n")
        self._send("CMD> FOUND 1 A 1 207 8980\n")
        self._send("CMD> FOUND 1001 B 1 207 8980\n")
        self._send("CMD> SCAN 2\n")
        self._send("CMD> DONE\n")

    def _send(self, text):
        try:
            os.write(self.master, text.encode())
        except BlockingIOError:
            pass

    def _loop(self):
        buf = b""
        while not self._stop.is_set():
            r, _, _ = select.select([self.master], [], [], 0.1)
            if not r:
                continue
            try:
                buf += os.read(self.master, 4096)
            except (BlockingIOError, OSError):
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle(line.decode("utf-8", "replace").strip())

    def _handle(self, line):
        if not line:
            self._send("CMD> DONE\n")
            return
        if line.startswith("CMD>"):    # 回显行，忽略
            return
        self.cmds.append(line)
        p = line.split()
        op = p[0]
        if op == "PING":
            self._send("CMD> PONG\n")
        elif op in ("SCAN", "MOTORS"):
            for i, m in self.motors.items():
                self._send(f"CMD> FOUND {i} {m['bus']} {m['addr']} {m['fw']} {m['hw']}\n")
            self._send(f"CMD> {op} {len(self.motors)}\n")
        elif op == "STAT" and len(p) >= 2:
            ident = int(p[1])
            for i in ([ident] if ident else self.motors):
                s = self.state[i]
                self._send(f"CMD> STAT {i} {s['en']} {s['inpos']} {s['stall']} "
                           f"{s['clog']} {s['pos']} {s['speed']}\n")
        elif op == "POS" and len(p) >= 7:
            ident = int(p[1])
            pulses = int(p[3])
            s = self.state[ident]
            s["pos"] = (s["pos"] - pulses) if p[2] == "CW" else (s["pos"] + pulses)
            self._send(f"CMD> OK POS {ident}\n")
        elif op == "SPD" and len(p) >= 5:
            ident = int(p[1])
            self.state[ident]["speed"] = -int(p[3]) if p[2] == "CW" else int(p[3])
            self._send(f"CMD> OK SPD {ident}\n")
        elif op == "STP":
            if int(p[1]) == 0:
                for s in self.state.values():
                    s["speed"] = 0
            else:
                self.state[int(p[1])]["speed"] = 0
            self._send(f"CMD> OK STP {p[1]}\n")
        elif op == "ENA" and len(p) >= 3:
            self.state[int(p[1])]["en"] = int(p[2])
            self._send(f"CMD> OK ENA {p[1]} {p[2]}\n")
        elif op == "ZERO" and len(p) >= 2:
            self.state[int(p[1])]["pos"] = 0
            self._send(f"CMD> OK ZERO {p[1]}\n")
        elif op == "REL" and len(p) >= 2:
            self._send(f"CMD> OK REL {p[1]}\n")
        elif op == "IO?":
            self._send(f"CMD> IO IN 15 {self.io_in} OUT 16 {self.io_out}\n")
        elif op == "IO" and len(p) >= 3 and p[1] == "OUT":
            self.io_out = int(p[2])
            self._send(f"CMD> OK IO OUT 16 {self.io_out}\n")
        elif op == "GO":
            self._send("CMD> OK GO\n")
        else:
            self._send(f"CMD> ERR CMD {op}\n")
        self._send("CMD> DONE\n")

    def close(self):
        self._stop.set()
        try:
            os.close(self.master)
            os.close(self.slave)
        except OSError:
            pass


if __name__ == "__main__":
    import sys
    fw = FakeFirmware()
    print(f"模拟固件已就绪，虚拟串口：{fw.path}")
    print("另开一个终端：python3 tools/test_ui.py --port " + fw.path)
    print("（Ctrl+C 退出）")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        fw.close()
