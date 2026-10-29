# HP Vectra (45945A) → 86Box 移植方案

> 依据：HP 官方手册 45961-90001（Vol1 Hardware, 1985-08）、45961-90002（Vol2 System BIOS, 1985）、
> 45945-90012（BIOS TRM, 1988-02）。原文分章在 `~/documents/ocrpage/HP_Vectra_*/ch*.md`。
> 目标机：初代 HP Vectra 45945A，1985-10，HP 第一台 IBM AT 兼容机。

---

## 1. 硬件基线（手册实测规格）

| 项 | 规格 | 出处 |
|---|---|---|
| CPU | Intel 80286 @ **8 MHz**（16 MHz XTAL ÷2） | Vol1 ch2 |
| 系统固件 | **64 KB**（F0000–FFFFF），可加 192 KB 模块 → 最多 256 KB | Vol1 ch1 |
| 板载 RAM | 256 KB / 640 KB，板载可扩至 640 KB；加专有卡至 **3.64 MB** | Vol1 ch1 |
| DMA | 8237 ×**2**（0x000–0x01F 主、0x0C0–0x0DF 从）+ 页寄存器 74LS612（0x080–0x09F） | Vol1 Table 2.6 |
| 中断控制器 | 8259 ×**3**（主 0x20–0x3F、从2 0x0A0–0x0BF、从3 0x07C–0x07D） | Vol1 Table 2.6 |
| 定时器 | **8254**（0x040–0x05F，三通道 + 状态读回） | Vol1 Table 2.6 |
| RTC | MC146818（0x070/0x071） | Vol1 Table 2.6 |
| 键盘控制器 | **HP 8041**（0x060/0x064 应用路径；0x068/0x069/0x06A HP 路径） | Vol1 ch4 |
| HIL 主机控制器 | **U777**（HP 自研 IC，0x06C–0x06F；AP 60 Hz，CERBINT 中断） | Vol1 ch4 |
| 视频 | **HP45981A 多模式卡**：6845 @ 3D0–3D7，3D8 模式/3D9 颜色/3DA 状态/3DD 扩展；3B0–3BF 另区 | Vol1 ch7 |
| 软驱 | NEC765 兼容（0x3F0–0x3F7 主 / 0x370–0x377 副） | Vol1 Table 2.6 |
| 硬盘 | **HP45816A 20 MB / HP45817A 40 MB**，ST506/ST412（0x1F0–0x1F8） | Vol1 ch9 |
| 扩展槽 | 7 个 I/O 通道槽（I/O 仅 0x100–0x3FF） | Vol1 Table 2.6 |

### 视频卡端口明细（Vol1 ch7, Table「I/O Registers」）

| I/O | 读 | 写 |
|---|---|---|
| 3D0 | CRTC 未定义 | CRTC 寄存器索引 |
| 3D1 | CRTC 数据 | CRTC 数据 |
| 3D2/3D4/3D6 | — | 索引副本 |
| 3D3/3D5/3D7 | CRTC 数据 | CRTC 数据副本 |
| 3D8 | — | 模式控制 |
| 3D9 | — | 颜色选择 |
| 3DA | 状态寄存器 | 禁止 |
| 3DB/3DC | 清/置光笔标志 | 同 |
| 3DD | — | 扩展控制 |

特性：8 页字符 RAM（32 KB，全访问交织，不需等回扫）、8×16 点阵字符格、下划线/单色灰度、
HP 或工业标准字符集可切、15 kHz（200 线）/ 25 kHz（400 线）双扫描率（不同时）、光笔接口。

### 中断向量（Vol2 Table 2.1）

| INT | 地址 | 用途 |
|---|---|---|
| 33 | 0CC | HP Mouse（EX-BIOS DRVR） |
| 40 | 100 | Alternate Diskette（STD） |
| 41 / 46 | 104 / 118 | 硬盘参数表 0 / 1（STD，HP 类型表） |
| **68** | 1A0 | **8041 服务请求 ISR** |
| **69** | 1A4 | **键盘 OBF ISR** |
| 6A/6B | 1A8 | 保留 |
| **6C** | 1B0 | **HP-HIL 控制器 ISR** |
| 6D/6E | 1B4 | 保留（第三片 8259 用） |
| **6F** | 1BC | **EX-BIOS 入口** |
| 70–77 | 1C0– | RTC + IRQ 8–15 |

---

## 2. 86Box 现状盘点

| 组件 | 86Box 现状 | 位置 | 差距 |
|---|---|---|---|
| 80286 | ✅ `CPU_PKG_286`，`MACHINE_TYPE_286` | `machine_table.c:55`、286 条目自 3713 起 | 只需 8 MHz 主频 |
| 8237 ×2 | ✅ `dma_init()` + `dma16_init()` | `machine.c:204`、`m_at_common.c:71` | 无 |
| 8259 ×2 | ✅ `pic_init()` / `pic2_init()` | `src/pic.c:989` / `:1113` | **缺第三片** |
| 8254 | ✅ `PIT_8254`（`IS_AT` 时自动） | `machine.c:206` | 无 |
| MC146818 + NVR | ✅ `nvr_at_device` | `machine_table.c` 各 AT 条目 | CMOS 默认值需按 HP |
| NEC765 软控 | ✅ `fdc_at_device`（0x3F0–0x3F7） | `m_at_286.c` 各 init | 副地址 0x370 未接 |
| ST506 硬盘 | ✅ `st506_at_wd1003_device`（0x1F0 x1、0x1F1 x7、0x3F6） | `src/disk/hdc_st506_at.c:780-784` | 端口差 0x1F8；HP 类型表在 BIOS 内，需能禁自带 ROM |
| 键盘控制器 | ❌ 只有 8042 固件（`keyboard_at_device`、`kbc_at_device`） | `src/device/keyboard_at.c`、`keyboard.c` | **8041 语义不同**，且无 0x68–0x6A HP 路径 |
| HP-HIL 主机控制器 | ❌ 全无 | — | 需新写（U777，0x6C–0x6F） |
| 45981A 视频卡 | ❌ 只有 MDA/CGA/EGA/HGC 等标准卡 | `src/video/vid_mda.c`、`vid_cga.c`（3D0–3D7 在 :98/:166） | 需新卡：6845 部分可照 CGA，扩 8 页/8×16/25 kHz/3DD |
| 通用 6845 设备 | ❌ 没有独立 CRTC 设备，各卡自带实现 | — | 从 `vid_cga.c` 抄 |
| ISA 槽 | ✅ `MACHINE_AT`（ISA + ISA16） | `machine.h:53` | 无 |
| 0x60–0x6F 冲突 | ✅ 安全：`port_6x` 只接 0x61/0x62/0x63 | `src/port_6x.c:187-202` | 0x68–0x6A、0x6C–0x6F 空闲可用 |

---

## 3. 缺失件与实现思路

### 3.1 【P0】BIOS ROM：✅ 已取得（2026-10-29）

- 需要 64 KB 系统固件（典型 2×32 KB 或 4×16 KB，`bios_load_interleaved()` 方式同 IBM AT：`m_at_286.c:169`）。
- 公开渠道**未见 dump**：MAME 无 Vectra 驱动、`hash/*.xml` 无记录；bitsavers 只有 `pdf/hp/vectra/` 手册
  （`bits/HP/` 目录无 vectra）。86Box-roms 只有后期机型 `vectra500mt`/`vectra52`/`vectra54`，不能混用。
- 获取路径（按可行性）：
  1. 真机 dump（27C256/27C512 ×2，需 45945A 主板或有人代 dump）；
  2. VCF / HP 博物馆 / hpmuseum.net 的固件镜像；
  3. 德/意二手市场整机买回自行 dump。
- **验收判据**：`0xFFFF0` 处应有远跳（`EA xx xx 00 F0`），且 64 KB 校验和自洽；
  若为 256 KB 扩展配置，需确认 ROM 模块的映射窗口（E0000 起）。

#### 3.1.1 联网搜索结论（2026-10-29）

| 渠道 | 结果 |
|---|---|
| MAME | 无 Vectra 驱动；`src/mame/mame.lst`、`hash/*.xml` 均无 vectra 条目 |
| 86Box / 86Box-roms | 只有后期机型：PR #415（430FX Vectra）、#591/#594（486N）、#598（386/16N、20N）、issue #8254；`roms/machines/` 只有 `vectra500mt`/`vectra52`/`vectra54` |
| bitsavers `bits/HP/` | 有 `HP_150/`、`HP85/`、`IPC/`、`1000_firmware/` 等固件目录，**无 Vectra** |
| bitsavers `pdf/hp/vectra/` | 仅 4 份手册（已在手） |
| hpmuseum.net | `exhibit.php?catdoc=42` 只有 Vectra 文档 PDF；`display_item.php?hw=125` 是 Vectra 条目页，无 ROM |
| theretroweb.com | 无 45945A（主板库主要是 486 以后） |
| GitHub | `search/repositories?q=vectra+bios` 无相关仓库；code search 需认证 |
| archive.org / web.archive.org | 本机网络不可达（超时，非 404） |
| **VCFed thread 78146**「HP Vectra 286 45945A」(SpaceDog707) | **有 EPROM 备份附件**，原帖写 "Here are backups of the EPROM BIOS files."；本机直连 403、真浏览器 403、`allorigins`/`codetabs`/`corsproxy`/`thingproxy`/`r.jina.ai`/`translate.goog` 代理全部失败（CloudFront 按出口 IP 封数据中心段） |
| vogons.org | 403（同为 Cloudflare 前置） |

**已取得结果（用户从 VCFed 帖 78146 下载的附件，2026-10-29）**：

| 项 | 值 |
|---|---|
| 源文件 | `HP_Vectra_45945A_XU183_1818-3784.bin`、`HP_Vectra_45945A_XU191_1818-3785.bin`（各 32,768 B，zip 39.4 KB） |
| 交错方向 | **XU183 = even（低字节）、XU191 = odd（高字节）**；反向拼接得到乱码串（"4P95520 /9108/5C(C)porygith"），据此判定无误 |
| 拼装镜像 | `vectra45925_64k_combined.bin`，65,536 B，**8 位校验和 = 0x00** |
| 复位向量 | `0xFFFF0: EA 5B E0 00 F0` → `jmp F000:E05B` |
| 版本串 | `HP45925 09/01/85 (C)Copyright 1985 Hewlett-Packard Corporation` |
| 落盘 | `~/86Box-roms/machines/hpvectra/`（even/odd/combined 三份） |
| SHA256 | combined `613502c5…56995f`；even `6f3670e3…839bbe0`；odd `1506b1d4…9530f8` |

接入方式：`bios_load_interleaved(fn[0]=even, fn[1]=odd, 0x000F0000, 65536, 0)`（与 `m_at_286.c:169` 一致）。
ROM 内可见串已证实 HIL 路径：`Internal HPHIL Interface`、`Boot Device`、`No Device`、`QWERTY Pad`。

### 3.2 【P0】`kbc_hp_8041`：8041 键盘控制器 + HP 路径

新文件 `src/device/kbc_hp_8041.c` + `src/include/86box/kbc_hp.h`。

| 端口 | 读 | 写 |
|---|---|---|
| 0x60 | 键盘数据缓冲（应用路径） | 命令缓冲 |
| 0x64 | 8041 状态 | 命令 |
| 0x68 | — | HP 8041 控制缓冲（Port 1 的缓冲版） |
| 0x69 | HP 8041 SVC 状态 | — |
| 0x6A | — | 清 HPINT（Clear Processing Done） |

**ROM 实测**（`vectra45925_64k_combined.bin`）：

- 上电自检读 `in al,0x60` 比对 **0x55**（BAT OK），不符即报错（F1DC2–F1DC8）；
- 命令 **0x45** 写 `0x60` / 状态读 `0x64` → 读 8041 的 Port 1（HP 路径数据）（F1DEC–F1DF1）；
- `out 0x6A,al` 后轮询 `in al,0x69` 直到为 0（清 SVC 完成）（F1DCC–F1DD8、F1DFA–F1E0C）；
- 端口引用计数：0x60 = 8 处、0x64 = 20 处、0x68 = 3 处、0x69 = 4 处、0x6A = 12 处。

三条输出线的语义必须建模（这三条是 Vectra 与 IBM AT 的分水岭）：
- **AP**：60 Hz 方波，送 HIL 控制器做轮询 → 用 `timer` 产生，上升沿触发 HIL POL；
- **RC**：拉低即复位 80286（Ctrl-Alt-Del、从保护模式回实模式）→ 调 `machine_reset()`/`cpu_leave_protected()`；
- **A20_GATE**：直接决定 A20 掩码（86Box 有 `mem_a20_alt`/`mem_a20_recalc` 机制）。

扫描码：8041 把 HIL 键盘报文转成 PC set-1 扫描码送 0x60（Vol2 ch5 有转换表）。
先照 `keyboard_at.c` 的扫描码表 + `kbc_at_device` 的 0x60/0x64 语义实现子集，再补 8041 私有命令。

### 3.3 【P0】`hp_hil`：U777 主机控制器

新文件 `src/device/hp_hil.c`。

**⚠️ ROM 实测：Vectra 的 U777 与 MAME 的 MLC 模型不同**。反汇编 `vectra45925_64k_combined.bin`：

| 端口 | 角色（ROM 实测） | 证据 |
|---|---|---|
| `0x6D` | 地址/索引；**bit3 = 1 表示命令模式** | `or al,0x8` + `out 0x6d,al`（F3832/F3891） |
| `0x6C` | 数据 | `mov al,0x10` + `out 0x6c,al`（F3838）；`out 0x6c,0x07`（F3897） |
| `0x6E` | 错误/状态读 | `in al,0x6e` + `cmp al,0x0` + `jnz fail`（F3846/F38A3） |
| `0x6F` | 就绪位：**bit0 = 1 完成** | 握手例程 F38B7：`in al,0x6f` / `test al,0x1` / `jnz ok` |

命令字节：**0x07 = 执行（写）**、**0x10 = 读请求**。
写时序：`out 0x6D, addr|0x08` → `out 0x6C, data` → 轮询 `0x6F` bit0 → 查 `0x6E` 错误。
读时序：`out 0x6D, addr` → `out 0x6C, 0x10` → 等 bit0 → `in 0x6C` 取数。

**结论**：U777 是"索引 + 数据 + 状态"的 8 位寄存器接口，**没有** MAME 的 16 位帧 FIFO / R0–R3 语义。
MAME 只能借用两层：设备链与地址分配（`hil_write(uint16_t*)` 串联、ACF 递减）、设备侧报文格式
（bit11 命令位 / bit10-8 地址 / bit7-0 数据 / POL 广播）——这些是 HIL 总线协议层，与主机寄存器接口无关。

参考（MAME 侧，仅供协议层）：
- 4 个 HIL 端口，链式设备模型：`hil_write(uint16_t *frame)` 返回是否消费、`next` 串联；
- 报文格式（`hlebase.cpp:69-71`）：bit11 = 1 命令 / 0 数据、bit10-8 = 设备地址（0 = 广播）、bit7-0 = 数据；
- 16 深 FIFO，满则置 FOF 并拉 **CERBINT**（→ INT 6C）；
- AP 60 Hz 上升沿发 POL，设备轮询返回数据；
- 设备侧最小集：HIL 键盘（103 键，扫描码 `((((row+1)^8)<<4)+(col<<1))`，MAME `hlekbd.cpp:485`）+
  HIL 鼠标（IDD 序列 `0x68`/`0x12`/`0xc2`/`0x1e`，`hlemouse.cpp:83-88`）。
- 不建模的部分（MAME 同样未做）：位级时序、传输错误恢复、冲突仲裁。

### 3.4 【P0】`vid_vectra_45981a`：多模式视频卡

新文件 `src/video/vid_vectra.c`，从 `vid_cga.c` 派生（6845 访问在 `:98`/`:166`，正好是 3D0–3D7）：

| 步骤 | 内容 |
|---|---|
| 1 | 复制 6845 寄存器读写（19 个，索引/数据 + 3D2–3D7 副本） |
| 2 | 3D8 模式控制、3D9 颜色选择（CGA 同名同址，语义按手册） |
| 3 | 3DA 状态寄存器（回扫/显示使能位按 Vol1 ch7 表） |
| 4 | 3DD 扩展控制（Vectra 私有） |
| 5 | 字符内存 32 KB、8 页（80×25 时每页 2000 字 = 4000 B；40×25 时 16 页） |
| 6 | 8×16 字符格 + 两套字符集 ROM（HP / 工业标准） |
| 7 | 双扫描率 15 kHz / 25 kHz（`vid_cga` 只有 15 kHz，需扩出 400 线模式） |
| 8 | 图形模式（Vol1 ch7 「Graphics Mode」节，640×…位图） |

注意：86Box 显示器模型按行频/分辨率换算，25 kHz 400 线要确认不超过 `vid_*` 允许的 `monitor` 表。

### 3.5 【P1】第三片 8259（`pic3`）

`src/pic.c` 现有 `pic_init()`(989) / `pic2_init()`(1113)，都是全局单例 + 固定端口。第三片在
**0x07C/0x07D**，与既有两片不共享端口，建议：
- 在 `src/pic.c` 增 `pic3_init()`，复用 8259 状态机但用独立寄存器组；
- 级联到主片某条 IRQ（等 ROM 反汇编确认，或按 86Box 现有做法把第三片输出 OR 进主片未用线）；
- 若第三片实际只服务 HIL/EX-BIOS 少数源，也可先做成"哑片"：可读 ISR/IMR、写 EOI 即清，
  直到 ROM 证明必须真实中断。

### 3.6 【P1】硬盘端口补齐

`hdc_st506_at.c:780-784` 接 0x1F0(1) + 0x1F1(7) + 0x3F6(1)。Vectra 用 0x1F0–0x1F8：
- 补 0x1F8（HP 卡上的额外状态/诊断口，语义待 ROM 确认）；
- 若 HP BIOS 自己带类型表（INT 41/46 由 BIOS 填），用 `st506_at_wd1003` 时需确认它不覆盖 BIOS 的表；
- 20 MB 几何（615/4/17 之类）与 40 MB 几何按手册 ch9 的表配。

### 3.7 【P2】其余

- 专有内存扩展卡（3.64 MB）：86Box 的 `machine_t.ram` 直接给 max = 3648 KB 即可，不必模拟卡本身；
- 串/并口卡：86Box 有 `lpt`/`serial` 设备，按 0x378/0x2F8 挂上即可；
- 副软控地址 0x370–0x377：`fdc_at_device` 只有 0x3F0 一组，需要时补。

---

## 4. 分阶段实施

### Phase 0：ROM（阻塞项，1–2 天，取决于能否 dump）
1. 拿到 64 KB BIOS，确认片数/位宽（`bios_load_interleaved` vs `bios_load`）。
2. `ndisasm -b 16` 反汇编 F000:0000 起，标出 POST 对 0x68/0x69/0x6A/0x6C–0x6F 的访问点。
3. 结论决定 Phase 1 的桩要做多真。

### Phase 1：骨架能 POST（3–5 天，约 300 行）
1. `machine_table.c` 增条目（`internal_name = "hpvectra"`）：

```c
{
    .name = "[ISA] HP Vectra", .internal_name = "hpvectra",
    .type = MACHINE_TYPE_286, .chipset = MACHINE_CHIPSET_DISCRETE,
    .init = machine_at_vectra_init,
    .cpu = { .package = CPU_PKG_286, .min_bus = 8000000, .max_bus = 8000000, ... },
    .bus_flags = MACHINE_AT,
    .ram = { .min = 256, .max = 3648, .step = 256 },
    .kbc_device = &kbc_hp_8041_device, .kbc_p1 = <AP|RC|A20 掩码>,
    .nvr_device = &nvr_at_device, .nvr_params = NVR_AT,
    .device = &vectra_45945a_device,           /* BIOS 配置 */
    .vid_device = &vid_vectra_45981a_device,
}
```

2. `machine_at_vectra_init()`：`bios_load_interleaved(..., 0x000f0000, 65536, 0)` →
   `machine_at_common_init(model)` → `pic3_init()` → `fdc_at_device`（若 `FDC_INTERNAL`）。
3. 先用 `kbc_at_device` + `vid_cga_device` 顶上，目标：**POST 跑到提示符/软驱引导**。
4. 记录 POST 卡点：若卡在 0x68/0x6C 探测，立刻上 Phase 2a 的桩。

### Phase 2：三件专有件（1–2 周）
- 2a `kbc_hp_8041`（400–600 行）：0x60/0x64 子集 + 0x68/0x69/0x6A + AP/RC/A20；
- 2b `hp_hil`（600–900 行）：4 端口 + FIFO + POL/ACF + 键盘/鼠标设备；
- 2c `vid_vectra_45981a`（900–1500 行）：6845 + 3D8/3D9/3DA/3DD + 8 页 + 8×16 + 双行频；
- 2d `pic3`（150–250 行）。

### Phase 3：磁盘与 CMOS（3–5 天）
- 0x1F8 补齐、20/40 MB 几何、CMOS 默认值（HP 的 0x70/0x71 位定义）；
- HP 硬盘类型表验证：`INT 41/46` 指向 BIOS 内的表，确认 BIOS 能正确读出几何。

### Phase 4：细节（按需）
内存扩展、串并口、25 kHz 显示、光笔、EX-BIOS 的 RAM 索取接口（适配卡从 BIOS 拿 RAM）。

---

## 5. 代码落点清单

| 文件 | 动作 |
|---|---|
| `src/machine/m_at_286.c` | 新增 `machine_at_vectra_init()` |
| `src/machine/machine_table.c` | 新增 286 条目 |
| `src/include/86box/machine.h` | 声明 init |
| `src/device/kbc_hp_8041.c` | 新文件（8041 + HP 路径） |
| `src/include/86box/kbc_hp.h` | 新文件 |
| `src/device/hp_hil.c` | 新文件（U777 MLC + 设备链） |
| `src/include/86box/hp_hil.h` | 新文件 |
| `src/video/vid_vectra.c` | 新文件（45981A） |
| `src/include/86box/vid_vectra.h` | 新文件 |
| `src/pic.c` / `src/include/86box/pic.h` | 增 `pic3_init()` |
| `src/disk/hdc_st506_at.c` | 补 0x1F8 |
| `src/CMakeLists.txt`（或 `src/device/CMakeLists.txt`） | 加新源文件 |
| `roms/machines/hpvectra/*.bin` | BIOS ROM |

---

## 6. 工作量估算

| 件 | 行数 | 难度 | 说明 |
|---|---|---|---|
| machine init + 表项 | 60–100 | 低 | 照 `machine_at_ibmat_init` |
| `kbc_hp_8041` | 400–600 | 中 | 8041 命令集需反汇编确认 |
| `hp_hil` | 600–900 | 中高 | MAME 有 1569 行可裁 |
| `vid_vectra` | 900–1500 | 高 | 6845 可抄，8 页/双行频/字符集要新写 |
| `pic3` | 150–250 | 低中 | 端口独立，状态机可复用 |
| 硬盘/CMOS 细节 | 100–200 | 低 | |
| **合计** | **≈2300–3500** | | 不含 ROM 获取与调试 |

对比：Z-100（MAME 1032 行骨架、定制软控、双 CPU 切换、S-100 全无）与 Nimbus（三片 ULA + 8031 +
bit-bang SCSI）都比 Vectra 重；Vectra 的 AT 外设几乎全是标准件，真缺口就三件 + 一片 PIC。

---

## 7. 风险与验证

| 风险 | 影响 | 应对 |
|---|---|---|
| **无 BIOS dump** | 项目无法验证，只能写不能跑 | Phase 0 优先；无 ROM 时先写骨架 + 桩，等 ROM |
| 8041 命令集与 8042 不同 | POST 卡在键盘自检 | 反汇编 POST 的 0x64 写序列；按需实现 |
| HIL 无设备时 POST 超时 | POST 挂起 | 先给"空 HIL"（POL 立刻返回无设备），再挂键盘 |
| 视频卡 25 kHz 400 线 | 86Box 显示模型超范围/黑屏 | 先只做 15 kHz，25 kHz 作为可选模式 |
| 硬盘类型表 | 引导扇区读不出 | 用 HP BIOS 的表，不启用 86Box 自带 ROM |
| 第三片 8259 用途不明 | 多做或漏做 | 先哑片，等 ROM 证据 |

验证顺序（每步都要有可观测结果）：
1. ROM 校验和 + 复位向量；
2. POST 第一条端口访问（对比反汇编预期）；
3. 视频出字（哪怕是乱码，证明 6845 时序对）；
4. 软驱引导 360 KB DOS；
5. HIL 键盘按键 → INT 9/INT 16 路径；
6. 硬盘分区表可读 → 引导。

---

## 8. 附：与既有工作的关系

- HP 150（8088 + 触摸屏 + HIL）：MAME/86Box 皆无，Vectra 的 HIL 实现可直接复用给 HP 150。
- Siemens PC-D/PC-X：卡在 OMTI 5100/5300 与定制视频，Vectra 不比它难。
- 86Box 已有的 80186 片内外设（`src/chipset/intel_8018x.c`，WNT50）与 3477-S 线无关，本方案不涉及。
