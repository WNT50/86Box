#!/bin/bash
# 3477-S 逆向跑批（memory-check 第 ⑥ 步 / cron 用）：固定配置跑 K 轮，
# 按"判据"决定要不要出声——只有出现**实质新情况**才打印，其余一律静默。
# 判据（任一成立才报）：
#   ① 出现新模式值（已知集合 {04,10,94} 之外，尤其 09/0A = APA 图形）
#   ② 平面写 > 256 次，或 distinct > 4（⇒ 真位图数据，不是 00/07 填充）
#   ③ hiram(E800-EC00) 写 > 0（⇒ 客人开始投递字模）
#   ④ GLYPH 取字出现非全 0（⇒ 发生器里有客人送的点阵）
#   ⑤ 停机点 PC 与上一次记录不同（⇒ 走到新阶段）
# 不报的：日志行数、D0/90 次数、平面写 0~8 这类抖动。
# 软盘 turbo 默认开（run3477.sh 生成的 cfg 带 fdd_01_turbo=1，TURBO=0 可关）。
set -u
S=/home/ubuntu/documents/86box
OUT=$S/battery
KNOWN_MODES="04 10 94"
mkdir -p "$OUT"
exec 9>"$OUT/.lock" || exit 0
flock -n 9 || exit 0           # 已有跑批在跑就安静退出

# 内存太紧跑不动（本机 3.7 GiB，引擎+网关已占 3.2 GiB）
AV=$(free -m | awk '/Mem:/{print $7}')
[ "${AV:-0}" -lt 200 ] && exit 0

TS=$(date +%Y%m%d-%H%M)
K=${K:-2}

export EPOCH3477_TESTMODE=0 EPOCH3477_PLANE=1 EPOCH3477_MODE=1 EPOCH3477_HIRAM=1 EPOCH3477_GLYPH=1
for i in $(seq 1 "$K"); do
    bash "$S/tools/killvm.sh" >/dev/null 2>&1
    TM=0 timeout 160 bash "$S/run3477.sh" >"$OUT/run-$TS-$i.log" 2>&1
    cp "$S/run3477/86box-base.log" "$OUT/emu-$TS-$i.log" 2>/dev/null
done

python3 - "$OUT" "$TS" "$KNOWN_MODES" <<'PY'
import sys, glob, re, os
out, ts, known = sys.argv[1], sys.argv[2], set(sys.argv[3].split())
state = os.path.join(out, ".criteria")
prev_stops = set(open(state).read().split()) if os.path.exists(state) else set()

runs, hits = [], []
for f in sorted(glob.glob(os.path.join(out, f"emu-{ts}-*.log"))):
    t = open(f, errors="ignore").read()
    stop  = (re.findall(r"3477: HB PC ([0-9A-F:]+)", t) or ["-"])[-1]
    modes = sorted(set(re.findall(r"CRTMODE <- ([0-9A-F]{2})", t)))
    pl    = re.findall(r"PLANEW n=(\d+)[^\n]*distinct=(\d+)", t)
    pw, pd = (int(pl[-1][0]), int(pl[-1][1])) if pl else (0, 0)
    hiram = len(re.findall(r"HIRAMW", t))
    gz    = bool(re.search(r"GLYPH [^\n]*b=(?!00 00 00 00 00 00 00 00)[0-9A-F]{2}", t))
    r = dict(f=os.path.basename(f), lines=len(t.splitlines()), stop=stop, modes=modes,
             pw=pw, pd=pd, hiram=hiram, gz=gz)
    runs.append(r)
    if set(modes) - known:     hits.append((r, "新模式 " + ",".join(sorted(set(modes) - known))))
    if pw > 256 or pd > 4:     hits.append((r, f"平面写上量 n={pw} distinct={pd}"))
    if hiram > 0:              hits.append((r, f"hiram 写 {hiram} 次（投递迹象）"))
    if gz:                     hits.append((r, "取字出现非全 0（发生器有客人点阵）"))
    if prev_stops and stop != "-" and stop not in prev_stops:
        hits.append((r, f"停机点进入新位置 {stop}"))

with open(state, "a") as fh:
    for r in runs:
        if r["stop"] != "-":
            fh.write(r["stop"] + "\n")

if not hits:
    sys.exit(0)                                  # 静默：判据一条都没成立

print("🔬 3477-S 跑批（判据命中）▫️ " + ts)
for r, why in hits:
    print(f"   ⚡ {why}")
    print(f"      ▶ {r['f']}: 行 {r['lines']} ▫️ 停 {r['stop']} ▫️ 模式 {','.join(r['modes']) or '-'} ▫️ "
          f"平面写 {r['pw']} (distinct {r['pd']}) ▫️ hiram {r['hiram']} ▫️ 非零点阵 {r['gz']}")
print(f"   📌 停机点已记录，日志在 {os.path.basename(out)}/")
PY
exit 0
