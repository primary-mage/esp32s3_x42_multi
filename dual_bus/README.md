# 双总线电机测试固件 + 测试上位机

ESP32-S3 用**两路独立 UART** 同时驱张大头 X42 闭环步进，自带一个纯调试用的 Python 上位机。
不含任何机构逻辑（不自动回零、不做软限位、不做插补），专门用来**试参数、试运动模式**。

---

## 一、这个分支和 `main` 是什么关系

本仓库现在有**两个版本的固件**，分别放在不同位置，互不影响：

| | `main` 分支（根目录） | `dual-bus` 分支 · `dual_bus/` 目录（本页） |
|---|---|---|
| 用途 | 特定机构的三轴平台（实验演示仪） | 通用电机测试台 |
| 总线 | 单条 UART1（GPIO4/5） | **双条 UART1(4/5) + UART2(8/9)** |
| 电机 | 固定 3 台，地址 1/2/3 | **自动扫描，地址任意**，两路地址空间独立 |
| 上层功能 | 龙门回零、时间轴振动、轨迹播放 | 只有位置/速度/IO，纯测试 |
| 额外 IO | 无 | 限位开关输入 + MOS 输出 |
| 模块 | main + pair_home + vib + stall_guard | 去掉了这三个机构相关模块 |

两个版本**共用同一份电机驱动** `zdt_x42.c/h`，逐字节一致——驱动本来就按「电机自带总线号」
设计，双总线不需要改它。

```
仓库根目录（main 分支的固件，未改动）
└── dual_bus/            ← 本分支新增，独立的 ESP-IDF 工程
    ├── main/
    │   ├── zdt_x42.c/h     ← 与根目录完全相同的驱动，零改动
    │   ├── motor_bus.c/h   ← 新写：双总线初始化、自动扫描、ID 编码、两路 IO
    │   ├── console_cmd.c/h ← 新写：精简测试命令集
    │   └── main.c
    └── tools/
        ├── test_ui.py      ← 测试上位机
        ├── fake_firmware.py← 固件模拟器（无硬件调 UI）
        └── port_utils.py   ← 串口自动识别
```

> 这是独立工程，编译时要先进 `dual_bus/` 目录，不要和根目录的旧固件搞混。

---

## 二、硬件接线

```
ESP32-S3                       电机 / 外部
UART1  GPIO4 (TX) ──── R/A/H ──── 总线 A（地址 1~255）
       GPIO5 (RX) ──── T/B/L
UART2  GPIO8 (TX) ──── R/A/H ──── 总线 B（地址 1~255，与 A 独立的地址空间）
       GPIO9 (RX) ──── T/B/L
GPIO15 ──── 微动开关 ──── GND / 3V3     输入，内部下拉
GPIO16 ──── 外部 MOS 模块信号           输出，上电默认低
GND    ──── 共地
```

- 两路总线**地址空间完全独立**：A 路 1 号和 B 路 1 号是两台不同的电机，互不干扰。
- 微动开关未压到 = 0V（内部下拉），压到 = 3.3V。
- GPIO15/16 避开了 UART(4/5、8/9)、USB(19/20) 和启动绑定脚(0/3/45/46)。
- 引脚都定义在 `main/motor_bus.h` 顶部，改硬件只需改那里。

---

## 三、全局 ID 编码

上层只用一个 ID 就能指到任意总线上的任意电机：

```
全局 ID = 总线号 × 1000 + 电机地址
  总线 A（UART1, GPIO4/5） →    1 .. 255     显示为 A1, A2 ...
  总线 B（UART2, GPIO8/9） → 1001 .. 1255    显示为 B1, B2 ...
```

例：`ID 3` = A 路地址 3；`ID 1001` = B 路地址 1。

用编码而不是查表，好处是**自动识别不影响稳定性**——多插一台电机不会让其他 ID 整体挪位。

---

## 四、编译烧录

需要 ESP-IDF v5.2.x（在 v5.2.6 上验证）。

```bash
source ~/projects/activate-esp-idf.sh
cd dual_bus                  # 注意：是新固件目录，不是仓库根目录
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

固件约 219KB，分区剩余 79%。

---

## 五、测试上位机

```bash
cd dual_bus
python3 -m pip install -r requirements.txt
python3 tools/test_ui.py
python3 tools/test_ui.py --port /dev/ttyACM0
```

界面功能：

- **连接**：自动选中 ESP32 那个串口（按 VID 0x303A），连上后自动扫一遍电机
- **目标电机**：下拉选 `A1` / `B1`…，旁边显示它挂哪条总线、地址多少、固件硬件版本
- **状态条**：位置 / 转速 / 到位 / 堵转，500ms 刷新（操作后立即刷新一次）
- **位置控制**：方向、脉冲数、转速 rpm、加速档、ABS/REL → 执行 POS
- **速度控制**：方向、转速、加速档 → 持续运转 SPD / 停止 STP
- **IO**：限位开关 GPIO15 实时电平（200ms 刷新，触发时变黄底）、MOS GPIO16 手动高低
- **全部急停**：`STP 0` 停掉所有已发现电机

> `位置` 显示的是电机位置计数器的**原始值**，一圈 65536 计数，不是脉冲数。
> 丝杆导程 4mm 时 1mm ≈ 16384 计数。本 UI 不做换算（纯测试用途）。

### 没有硬件时怎么调 UI

工程自带固件模拟器，用 Linux PTY 造一个虚拟串口，行为与真实固件一致：

```bash
# 终端 1：起模拟器，它会打印虚拟串口路径
python3 tools/fake_firmware.py
#   模拟固件已就绪，虚拟串口：/dev/pts/3

# 终端 2：UI 连到虚拟串口
python3 tools/test_ui.py --port /dev/pts/3
```

模拟器里固定有两台电机（A1、B1），支持全部命令。真机到手后换成真实串口即可，UI 不用改。

---

## 六、串口协议

一行一条命令，空格分词，回车结束，每条命令以 `CMD> DONE` 收尾。

| 命令 | 作用 | 典型响应 |
|---|---|---|
| `PING` | 心跳 | `CMD> PONG` |
| `SCAN [from] [to]` | 扫描两条总线（默认 1~16） | `CMD> FOUND 1 A 1 207 8980` ×N → `CMD> SCAN N` |
| `MOTORS` | 列出已登记电机（不重扫） | 同上 + `CMD> MOTORS N` |
| `ENA <id> <0\|1>` | 使能 / 去使能 | `CMD> OK ENA 1 1` |
| `POS <id> <CW\|CCW> <pulses> <rpm> <accel> <ABS\|REL> [SYNC]` | 位置模式 | `CMD> OK POS 1` |
| `GO` | 所有在用总线广播同步起步 | `CMD> OK GO` |
| `SPD <id> <CW\|CCW> <rpm> <accel>` | 速度模式（持续转） | `CMD> OK SPD 1` |
| `STP <id\|0>` | 停止；0 = 全部 | `CMD> OK STP 0` |
| `ZERO <id>` | 位置清零 | `CMD> OK ZERO 1` |
| `REL <id>` | 解除堵转保护 | `CMD> OK REL 1` |
| `STAT <id\|0>` | 状态 | `CMD> STAT <id> <en> <inpos> <stall> <clog> <pos_raw> <speed>` |
| `IO?` | 读两路 IO | `CMD> IO IN 15 0 OUT 16 0` |
| `IO OUT <0\|1>` | 设 MOS 电平 | `CMD> OK IO OUT 16 1` |

上电先打 `CMD> READY`，然后自动扫一遍并输出 `FOUND` 行。

### 手动发命令

```bash
python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
time.sleep(0.3); s.reset_input_buffer()
for c in ('PING', 'SCAN 1 8', 'STAT 1', 'IO?'):
    s.write((c+'\n').encode()); time.sleep(0.5)
    print(f'>>> {c}'); print(s.read(8192).decode('utf-8','replace'))
EOF
```

---

## 七、实现说明

### 多总线为什么不改驱动

`zdt_x42.c` 本来就是按「电机自带总线号」设计的：

```c
static SemaphoreHandle_t s_bus_mutex[UART_NUM_MAX];   // 每条总线一把锁
static bool              s_bus_installed[UART_NUM_MAX];
```

`zdt_x42_transact()` 里取锁、`uart_flush_input()`、`uart_write_bytes()`、读应答
全部用 `m->uart_num`，所以初始化两条总线、把不同电机分到不同总线上即可并发工作，
**`zdt_x42.c/h` 一行没改**。

唯一特殊的是 `GO` 广播：`zdt_x42_sync_start(m)` 只会在 `m->uart_num` 那条总线上广播，
所以 `motor_sync_start_all()` 会遍历已登记电机，每条在用总线各广播一次。

### 自动识别（扫描）的代价

电机地址 1~255，空地址只能靠超时判定。默认事务超时 200ms，扫 255 个地址要一分多钟。

所以扫描用**独立的短超时**：`probe.timeout_ms = 25`。
有电机的地址通常 1~3ms 就应答，25ms 足够；扫 16 个地址 × 2 条总线约 **0.8 秒**。
扫描结束后，正式登记的句柄切回正常 200ms 超时。

⚠️ **地址 0 是广播地址**，扫它会触发总线上所有电机同时应答、帧互相撞车，代码里强制从 1 开始。

### 为什么去掉旧工程的那三个模块

`pair_home.c`（龙门回零）、`vib.c`（振动搅拌）、`stall_guard.c`（堵转自动恢复）
都是针对旧机构写的。尤其 `stall_guard` 会在堵转时**自动反向退一圈并清零位置**，
调试单机电机会严重干扰判断，所以本分支没有带过来，堵转状态由上位机看 `STAT` 自行处理。

---

## 八、常见问题

**扫描不到电机**
- 确认电机供电（24V）已上、屏幕亮、`P_Serial=UART_FUN`
- TX/RX 是否接对：ESP32 的 TX(GPIO4) 接电机 **R/A/H**，RX(GPIO5) 接 **T/B/L**
- 地址是否超出扫描范围：用 `SCAN 1 255` 扩大范围（会慢一些）
- 校验方式是否为出厂默认 0x6B

**Linux 串口没权限**
```bash
sudo usermod -aG dialout $USER    # 重新登录生效
```

**位置数值看不懂**
`pos_raw` 是电机位置计数器原始值，**一圈 = 65536 计数**，不是脉冲数。
看相对变化即可，本 UI 不做 mm 换算。

**点了按钮没反应**
看界面底部日志，每次操作都会记录发出的命令和结果；失败会显示 `ERR` 原因。
