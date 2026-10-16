#!/bin/bash
# 3477-S 复现跑批（我的独立 worktree；不碰用户分支）
# 用法: run3477.sh [秒数] [标签] [testmode] [testsw] [软盘镜像]
export DISPLAY=:99
BIN=/home/ubuntu/documents/86box/src-3477/build/gdbstub/src/86Box
ROMS1=/home/ubuntu/86Box-roms
ROMS2=/home/ubuntu/documents/86box/roms-3477
d=/home/ubuntu/documents/86box/run3477
SECS=${1:-150}
TAG=${2:-base}
TM=${3:-1}
SW=${4:-0}
IMG=${5:-}
# turbo 默认 OFF；可用 TURBO=1 或 EPOCH3477_TURBO=1 显式打开
TURBO=${TURBO:-${EPOCH3477_TURBO:-0}}
mkdir -p "$d"

pgrep -f "Xvfb :99" >/dev/null || { Xvfb :99 -screen 0 1280x800x24 >/dev/null 2>&1 & sleep 2; }

# 跑前清残留：本构建目录的旧实例（曾有 1 天 10 小时的僵尸实例吃满 CPU 拖垮跑批）
LEFT=$(pgrep -f "$BIN" 2>/dev/null)
if [ -n "$LEFT" ]; then
    echo "⚠ 清理残留 86Box 实例: $LEFT"
    kill -9 $LEFT 2>/dev/null
    sleep 1
fi

cat > "$d/86box.cfg" <<EOF
[General]
vid_renderer = qt_software
sound = 0

[Machine]
machine = ibm3477
cpu_family = 80186
cpu_speed = 12000000
cpu_use_dynarec = 0
testmode = $TM
testsw = $SW
mem_size = 640

[Video]
gfxcard = internal


[Input devices]
keyboard_type = keyboard_ps55
mouse_type = none
tablet_type = none

[Floppy and CD-ROM drives]
fdd_01_type = 35_2hd
fdd_01_turbo = ${TURBO:-0}
fdd_02_type = none
EOF
KBD=${6:-keyboard_ps55}   # 3477-S 用 PS/55 键盘（有 FLAG_PS2_KBD）；此前默认 keyboard_at 导致 0xF0 被拒
sed -i "s/^keyboard_type = .*/keyboard_type = $KBD/" "$d/86box.cfg"

setsid bash -c "cd '$d' && exec '$BIN' -C '$d/86box.cfg' -R '$ROMS1' -R '$ROMS2' -L '$d/86box-$TAG.log' ${IMG:+-I A:$IMG} >'$d/stdout-$TAG.log' 2>&1" </dev/null &
pid=$!
echo "跑 $SECS 秒（tag=$TAG, pid=$pid）…"
sleep "$SECS"
# 整组回收：只杀 pid 会漏掉子进程，残留实例会一直吃 CPU
kill -9 -"$pid" 2>/dev/null
kill -9 "$pid" 2>/dev/null
sleep 1
# 兜底：再按构建目录清一次
pgrep -f "$BIN" 2>/dev/null | xargs -r kill -9 2>/dev/null
sleep 1
echo "日志: $d/86box-$TAG.log  $(wc -l < "$d/86box-$TAG.log") 行"
echo "--- 尾部 ---"; tail -5 "$d/86box-$TAG.log"
