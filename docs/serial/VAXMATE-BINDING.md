# VAXmate 串口接线（新 backend API）

更新日期：2026-11-02
基线：上游 master `1441bd9ec`（含串口三层重构）+ `vaxmate-wd2010`

## 1. 三层结构

| 层 | 文件 | 职责 |
|---|---|---|
| 端口表 | `src/include/86box/serial.h` | `com_ports[]` 只放策略：`enabled` / `device` / `chip` / `ops` / `char_port` |
| 芯片 | `src/device/serial_scn26x1.c` | 寄存器、MR/CR 时序、收发、中断 |
| 后端 | `serial_backend_t` | 字节来源与去向：chardev 或板上的模拟设备 |

芯片不再自己占地址，也不再有 `serial_t`；板卡注册端口，芯片把 `A1:A0` 直接当寄存器号用。

## 2. VAXmate 的接法

VAXmate 的 SCN2661 在 `0x0C40–0x0C43`，只接 DEC 三键鼠标，**不是 COM 口**，
所以不用 `com_ports` 槽位：

```c
/* m_vaxmate.c */
device_add(&vaxmate_mouse_device);
vaxmate_uart = device_add_params(&scn2661c_device, (void *) 0xff); /* C part, no COM port */
io_sethandler(0x0c40, 0x0004,
              scn26x1_read, NULL, NULL,
              scn26x1_write, NULL, NULL,
              vaxmate_uart);
scn26x1_set_irq(vaxmate_uart, 0x1000);                    /* IRQ12 */
scn26x1_set_backend(vaxmate_uart, vaxmate_mouse_backend());
```

芯片侧为此只新增一处：

- `scn26x1_set_backend(void *priv, const serial_backend_t *be)`
  直连后端，绕过 `com_ports`；设置后立刻 `scn26x1_rx_try_read()` 取第一个字节。

「不占 COM 口」不需要另立 device_t：`device_add_common()` 对 `params` 做的是
`init_dev->local |= (uintptr_t) params`（基值 `3 << 8`，或上 `0xff` 后低字节即端口号 ff），传 `(void *) 0xff` 就把 `local` 置成 `0xff`，
`scn26x1_init()` 里的 `port = info->local & 0xff` 因此落在 `SERIAL_MAX` 之外，端口表注册被跳过。
设备身份仍按 `local` 记账，与真正的 `scn2661_device`（`local = 0`）互不冲突。

## 2.1 用的是 C 变体（SCN2661C，5.0688 MHz）

芯片族按 BRCLK 分成两个部件，`device_t.local` 的高字节存变体号（低字节留端口号）：

| 器件 | BRCLK | 速率表 | `local` |
|---|---|---|---:|
| `scn2661b_device` | 4.9152 MHz | 45.5 … 38400 | `2 << 8` |
| `scn2661c_device` | 5.0688 MHz | 50 … 19200 | `3 << 8` |

VAXmate 的鼠标 USART 走 **SCN2661C**，判据链：

1. 数据手册（`ocrpage/scn2661_epci/ch6_1_ELECTRICAL_CHARACTERISTICS.md`）：
   `fBRG Baud rate generator (2661 C) 1.0 / 5.0688 / 5.0738` MHz，C 件的 BRCLK 就是 5.0688。
2. ROM 初始化 `MR2 = 0x7C`：高半字节 7 = 16× 时钟，低半字节 C → 分频 66。
3. 66 × 16 × 4800 = **5,068,800 Hz**，与 C 件的 BRCLK 精确吻合；4.9152 MHz 下同一档只有 4654 baud。
4. 5.0688 MHz 的 4800 档在 C 表（`ch1_1_…` 第 269 行，`1100 → 4800 / 66`），B 表同档的分频值不同。

## 3. 鼠标如何实现后端

`mouse_vaxmate.c` 保留 DEC 协议（`D`/`P`/`R`/`T`/`Z`、三字节位置报告、四字节自检），
只把投递方式换掉：

| 旧接口 | 新接口 |
|---|---|
| `scn26x1_set_rx_pull_handler()` + 手动投字节 | `backend.read_byte()` 每次给一个字节 |
| `scn26x1_set_tx_handler()` | `backend.write_byte()` |
| `scn26x1_init(port, irq, priv)` | 板卡 `io_sethandler` + `scn26x1_set_irq()` |

`read_byte()` 每次只出一个字节是刻意的：2661 只有一个 RDR，芯片在 CPU 读走之后
立刻再拉一次，四字节自检因此在一个轮询周期内排空——这正是 POST `F22A2` 那个
微秒级短循环需要的。

## 4. 踩到的坑：DCD 未建模导致接收器永久关闭

`scn26x1_receiver_on()` 原文：

```c
if (!(dev->cr & SCN_CR_RXEN)) return 0;
return !dev->dcd;          /* 复位值 1 = 未断言 → 接收器不开 */
```

鼠标板不建模 modem 线，`dcd` 永远停在 1，于是命令进得去（`Tx 50`）、字节出不来，
状态寄存器恒读 `05h`（TXRDY|TXEMT，RXRDY 不置位）。

`scn26x1_cts_on()` 本来就写明「没有东西驱动 CTS 就当它已断言」，DCD 漏了同一条规则。
补上后恢复：

```c
if (!dev->modem_set && ((dev->be == NULL) || (dev->be->status == NULL)))
    return 1;
```

## 5. 验证

| 项 | 证据 |
|---|---|
| 构建 | `[810/810] Linking CXX executable src/86Box`，零警告 |
| `mtest` 引导块（`run-vaxmate/mtest.asm`） | 屏显 `BDA 330:53 FF 00 F0` / `MOUSE RX: 90 00 00 90 0A 00 …`（帧格式修正后；早期版本是 `C4 …` 一族，两者含义不同，见 `NOTES-VAXMATE-MOUSE.md`） |
| `irqtest` 引导块（`run-vaxmate/irqtest.asm`） | 每份三字节报告触发 **3 次 IRQ12**（`INT 0x74` 计数），四步合计 12 |
| 芯片日志 | `[0C40] write 00 <- 50` → `Tx 50` → `Rx 90` / `Rx 00` / `Rx 00` |
| 整机 | VAXmate Information System 主菜单（MS-Windows / VT220 / VT240 / VAXmate Network 齐全）|
| Windows 1.0 | 指针四方向 1:1 跟随、左键可弹菜单（`NOTES-VAXMATE-MOUSE.md` §4） |

## 6. 迁移备忘

- 旧 `src/include/86box/serial_scn26x1.h` 已删除（其中 `scn26x1_rx_byte` /
  `rx_free` / `set_tx_handler` / `set_rx_pull_handler` 在新 API 中都没有对应物）。
- `serial_backend_from_device()` 刻意不提供 `read_byte`：那条路是「设备直接推 FIFO」。
  需要「设备有队列、芯片按需来取」时，实现完整 `serial_backend_t` 并直连（如本机鼠标）。
- 芯片的 `be_store` 只用于适配器；直连后端是静态生命周期，不需要 store。
- 想改 `local` 又不想另建 device_t 时用 `device_add_params()`：它只做 `|=`，
  清不掉已有位；本例基值 0，或 `0xff` 正好得到 0xFF。
