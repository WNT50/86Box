# 3477-S 逆向：跑批与观测工具（80186 分支）

这些脚本是本机（慢机）调试时用的；在**快机**上跑同一构建即可。全部探针默认关，
必须用环境变量显式打开。

## 1. 构建

```bash
mkdir -p build/regular && cd build/regular
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ../..
ninja            # 产物: build/regular/src/86Box
```

## 2. 跑一轮（脚本自己生成 86box.cfg）

```bash
# 用法: run3477.sh [秒数] [标签] [testmode] [testsw] [软盘镜像]
TM=0 bash run3477.sh 1500 long3477b 0      # 1500 秒；TM=0 = 系统 ROS（真实路径）
# 日志: run3477/86box-<标签>.log    配置: run3477/86box.cfg
```

要点（都是踩过的坑）：

- **时长是第 1 个位置参数**，只给 `timeout` 不管用（脚本内部默认 150 s）。
- `run3477.sh` 每次用 here-doc **重生成 cfg**，手工改 cfg 会被覆盖；要改就改脚本。
- **软盘 turbo 在 cfg 里没有对应的键**，只能由机器代码 `fdd_set_turbo(&drives[0], 1)`
  设置（本分支已在 3477 的 init 路径里打开；`EPOCH3477_NOTURBO=1` 可关）。
- 无声卡的环境要加 `ALSOFT_DRIVERS=null SDL_AUDIODRIVER=dummy`，否则
  OpenAL/ALSA 初始化失败后会触发一次 abort 把跑批打断。

## 3. 探针门控（默认全关）

| 环境变量 | 观察对象 |
|---|---|
| `EPOCH3477_TESTMODE=0` | 切到 System ROS（lower 32K），真实路径 |
| `EPOCH3477_FCRD=1` | 客人对 `F000` 字卡窗口的**读**（取字 vs 机型自检） |
| `EPOCH3477_FCWR=1` | 字卡窗口的**写**（页锁存协议） |
| `EPOCH3477_GLYPH=1` | 渲染器的文本取字（码 → 字模槽 → 点阵） |
| `EPOCH3477_GLYPH24=1` | Font 24 取字：`base` + 拼好的 32 位行 |
| `EPOCH3477_INJECT=1` | 打开机侧字形注入（对照实验用；默认关） |
| `EPOCH3477_PLANE=1` | 显示平面窗写普查（次数 / 地址范围 / 值直方图） |
| `EPOCH3477_HIRAM=1` | `E800-EC00` 写入（投递落点） |
| `EPOCH3477_MODE=1` | 每次 `crtmode` 写（04/10/94 vs 09/0A） |
| `EPOCH3477_LCDATA=1` | 经 `0x3D0/0x3D1` 写入的 (索引,值) 对 |
| `EPOCH3477_VDUMP=1` / `EPOCH3477_VD2=1` | 客人数据在 `epoch->vram` 的落点与块结构 |
| `EPOCH3477_NOFDCDROP=1` | 关掉 FDC 结果相写丢弃（A/B 用） |
| `EPOCH3477_EBANK` / `EBTRC` | `E800` 体窗口 / `0x400` 写跟踪 |

一次观测常用的组合：

```bash
ALSOFT_DRIVERS=null SDL_AUDIODRIVER=dummy \
EPOCH3477_TESTMODE=0 EPOCH3477_FCRD=1 EPOCH3477_GLYPH24=1 EPOCH3477_PLANE=1 \
EPOCH3477_HIRAM=1 EPOCH3477_MODE=1 EPOCH3477_LCDATA=1 \
TM=0 bash 3477-tools/run3477.sh 1500 fast 0
```

## 4. 看结果

```bash
python3 3477-tools/g24view.py run3477/86box-fast.log 4   # 取字 → ASCII 点阵
grep -a PLANEW  run3477/86box-fast.log | tail -5          # 平面写采样
grep -a LCDATA  run3477/86box-fast.log | tail -20         # 0x3D0/0x3D1 写入
grep -a 'HB PC' run3477/86box-fast.log | sort -u          # 走到过哪些 PC
```

## 5. 已知事实（便于对照你那边跑出来的东西）

- 客人：字符码 → CRAM（23,615 个码 + 16,385 属性）；点阵 → 显示平面窗
  （≥196,608 次写，值 `00/07/FF/3F`，地址限在 `+0x0000..+0x1B7F`）。
- 渲染器（font24）：字形 = `epoch->vram[base + 0..3]`，**每字形 128 字节**、每行 4 字节。
- 客人数据落点 `vram[0x0000..0x1B7F]`（≈55 个 128 B 槽）—— 与渲染器取字槽**地址重叠**。
- 客人**从未**写 `E800-EC00`（`HIRAMW=0`）；对字卡窗口只有 2 次读（机型自检 `F5F8D/F5F94`）。
