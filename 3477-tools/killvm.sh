#!/bin/bash
# 停掉所有 86Box 实例（含 retail 与 gdbstub 两棵树）。
# 单独放成脚本，避免调用方命令行里出现匹配串而被 pkill 误杀。
for pat in 'src-3477/build/regular/src/86Box' 'src-3477/build/gdbstub/src/86Box'; do
    pids=$(pgrep -f "$pat" 2>/dev/null)
    [ -n "$pids" ] && { echo "停 $pat: $pids"; kill -9 $pids 2>/dev/null; }
done
sleep 1
pgrep -f 'src-3477/build' >/dev/null && echo "⚠ 仍有残留" || echo "已清空"
