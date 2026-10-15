#!/usr/bin/env python3
"""把 3477-S 的 font24 取字探针日志（EPOCH3477_GLYPH24=1）还原成可读点阵。

用法: python3 g24view.py [日志路径] [最多显示几个字形]
每行 32 位，'#' = 亮点、'.' = 暗点；同一个 base 的连续行组成一个字形。
"""
import re, sys, itertools, collections

path = sys.argv[1] if len(sys.argv) > 1 else "/home/ubuntu/86box-work/run3477/86box-g24.log"
limit = int(sys.argv[2]) if len(sys.argv) > 2 else 6

pat = re.compile(r"G24 n=(\d+) base=([0-9A-F]+) font=([0-9A-F]{8})")
rows = [(m.group(2), int(m.group(3), 16)) for m in pat.finditer(open(path, errors="ignore").read())]
print(f"  取字行数: {len(rows)}")

groups = []
for base, grp in itertools.groupby(rows, key=lambda r: r[0]):
    g = [w for _, w in grp]
    groups.append((base, g))

print(f"  字形数: {len(groups)}  base 范围: {groups[0][0]}..{groups[-1][0]}" if groups else "  无数据")

for base, g in groups[:limit]:
    print(f"  --- base={base} 行数={len(g)} 非零字={sum(1 for w in g if w)} ---")
    for w in g[:16]:
        print("     " + "".join('#' if (w >> (31 - i)) & 1 else '.' for i in range(32)))

# 统计：哪些 base 出现过，非零行数分布
nz = collections.Counter(sum(1 for w in g if w) for _, g in groups)
print("  非零行数分布:", dict(sorted(nz.items())))
