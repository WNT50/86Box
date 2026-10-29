# HP Vectra 45945A ROM 分析笔记

**镜像**：`vectra45925_64k_combined.bin`（65,536 B，由两片 32 KB EPROM 奇偶交错拼成）
**来源**：VCFed thread 78146「HP Vectra 286 45945A」（作者 SpaceDog707，2020-09-19 上传的 EPROM 备份）
**BIOS 版本**：`HP45925 09/01/85 (C)Copyright 1985 Hewlett-Packard Corporation`

## 1. 片序与完整性

| 文件 | 原始名 | 角色 | SHA256 |
|---|---|---|---|
| `vectra45945a_even_1818-3784.bin` | `HP_Vectra_45945A_XU183_1818-3784.bin` | **even（低字节）** | `6f3670e3e0b04dbb35be19be803ce7ddf59a368703abbbb9d4c8329c7839bbe0` |
| `vectra45945a_odd_1818-3785.bin` | `HP_Vectra_45945A_XU191_1818-3785.bin` | **odd（高字节）** | `1506b1d4e70fa8d0440eff0903afc9ebfd54afd65eb2792baebf3fc69d9530f8` |
| `vectra45925_64k_combined.bin` | — | 拼装结果 | `613502c514d29d7a6dda33d818c3b891230554bdc192bbc48ca0e2acc956995f` |

判定依据：只有 `XU183=even / XU191=odd` 这一种拼法能得到连续版权串与正确的复位向量；
反向拼接会把字符串打散成 `4P95520 /9108/5C(C)porygith1 89 5eHlwte-taPkcra`。

| 校验 | 值 |
|---|---|
| 8 位和 | `0x00` |
| 复位向量 @ `0xFFFF0` | `EA 5B E0 00 F0` → `jmp F000:E05B` |

## 2. I/O 端口引用统计（`ndisasm -b 16 -o 0xF0000`，36,846 行）

| 端口 | 引用数 | 角色 |
|---|---|---|
| `0x60` | 8 | 8041 数据（键盘/命令返回） |
| `0x61` | 13 | PPI Port B（NMI/扬声器/A20 等） |
| `0x64` | 20 | 8041 状态/命令 |
| `0x68` | 3 | HP 8041 控制缓冲 |
| `0x69` | 4 | HP 8041 SVC 状态 |
| `0x6A` | 12 | 清 HPINT |
| `0x6C` | 20 | **HIL 数据** |
| `0x6D` | 21 | **HIL 地址/索引** |
| `0x6E` | 3 | **HIL 错误/状态** |
| `0x6F` | 8 | **HIL 就绪位** |

## 3. HIL 主机控制器（U777）实测语义

与 MAME `hp_hil.cpp` 的 MLC 模型**不同**——Vectra 是"索引 + 数据 + 状态"的 8 位寄存器接口：

| 端口 | 语义 | ROM 证据 |
|---|---|---|
| `0x6D` | 地址/索引；bit3 = 1 表示命令模式 | F3832 `or al,0x8` / F3834 `out 0x6d,al`；F3891 同 |
| `0x6C` | 数据 | F3838 `mov al,0x10` / F383A `out 0x6c,al`；F3897 `mov al,0x7` / F3899 `out 0x6c,al` |
| `0x6E` | 错误/状态 | F3846 `in al,0x6e` / F3848 `jnz`；F38A3 `in al,0x6e` / F38A5 `cmp al,0` |
| `0x6F` | bit0 = 完成位 | 握手 F38B7：`in al,0x6f` / F38C7 `test al,0x1` / `jnz ok`，超时 0x3E8 × 0x180 次延时 |

命令字节：**0x07 = 执行（写）**、**0x10 = 读请求**。

- 写：`out 0x6D, addr|0x08` → `out 0x6C, data` → 轮询 `0x6F` bit0 → 读 `0x6E` 查错
- 读：`out 0x6D, addr` → `out 0x6C, 0x10` → 等 bit0 → `in 0x6C` 取数

关键例程：F38B7（等完成）、F3830（读寄存器）、F386A（写寄存器）、F371B（设备轮询/POL 循环）。

## 4. 8041 键盘控制器实测

- 自检：`in al,0x60` 比对 **0x55**（BAT OK），不符报错（F1DC2–F1DC8）
- 命令 **0x45** 写 `0x60` → 读 8041 的 Port 1（HP 路径数据）（F1DEC–F1DF1）
- `out 0x6A,al` 后轮询 `in al,0x69` 直到 0（清 SVC 完成）（F1DCC–F1DD8、F1DFA–F1E0C）

## 5. 相关中断向量（Vol2 Table 2.1）

| INT | 地址 | 用途 |
|---|---|---|
| 33 | 0CC | HP Mouse（EX-BIOS 驱动） |
| 68 | 1A0 | 8041 服务请求 ISR |
| 69 | 1A4 | 键盘 OBF ISR |
| 6C | 1B0 | HP-HIL 控制器 ISR |
| 6F | 1BC | EX-BIOS 入口 |

## 6. 复现方法

```bash
python3 tools/verify_rom.py --even roms/machines/hpvectra/vectra45945a_even_1818-3784.bin \
                            --odd  roms/machines/hpvectra/vectra45945a_odd_1818-3785.bin \
                            --out  vectra45925_64k.bin --disasm
```

脚本会校验 8 位和、复位向量、版本串，并统计 HIL/8041 端口引用。
