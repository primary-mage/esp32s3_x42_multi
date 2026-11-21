#pragma once
/**
 * @file console_cmd.h
 * @brief 上位机 ASCII 命令行（USB Serial/JTAG 控制台）。
 *
 * 协议：一行一条命令，空格分词，回车结束，大小写敏感。
 * ESP 每条命令处理完回复 "CMD> DONE"；上电就绪先打印 "CMD> READY"。
 *
 *   PING                                    心跳，回 CMD> PONG
 *   SCAN [from] [to]                        扫描两条总线（默认 1~16）
 *                                           CMD> FOUND <id> <bus> <addr> <fw> <hw>
 *                                           CMD> SCAN <count>
 *   MOTORS                                  列出已登记的电机（不重新扫描）
 *   ENA <id> <0|1>                          使能 / 去使能
 *   POS <id> <CW|CCW> <pulses> <rpm> <accel> <ABS|REL> [SYNC]
 *                                           位置模式；SYNC = 押住等 GO 同步起步
 *   GO                                      所有在用总线广播同步起步
 *   SPD <id> <CW|CCW> <rpm> <accel>         速度模式（持续转，直到 STP）
 *   STP <id|0>                              停止；0 = 全部
 *   ZERO <id>                               位置清零（掉电不保存）
 *   REL <id>                                解除堵转保护
 *   STAT <id|0>                             CMD> STAT <id> <en> <inpos> <stall> <clog> <pos_raw> <speed>
 *   IO?                                     CMD> IO IN 15 <0|1> OUT 16 <0|1>
 *   IO OUT <0|1>                            CMD> OK IO OUT 16 <0|1>
 *   HELP                                    命令列表
 *
 * 全局 ID：总线 A(UART1,GPIO4/5) → 1..255；总线 B(UART2,GPIO8/9) → 1001..1255
 */
#include "esp_err.h"

/** 创建控制台任务（内部会先做一次默认范围扫描） */
esp_err_t console_cmd_start(void);
