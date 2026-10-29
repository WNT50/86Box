#!/usr/bin/env python3
"""拼装并校验 HP Vectra 45945A 的两片 EPROM 备份。

用法:
    python3 verify_rom.py --even <even.bin> --odd <odd.bin> [--out combined.bin] [--disasm]

校验项: 片大小 / 交错方向 / 8 位校验和 / 复位向量 / 版本串；
--disasm 额外调用 ndisasm 统计 8041 与 HIL 端口引用。
"""
import argparse
import pathlib
import re
import subprocess
import sys

VERSION_RE = re.compile(rb"HP\d{5}\s+\d\d/\d\d/\d\d")
PORTS = [0x60, 0x61, 0x64, 0x68, 0x69, 0x6A, 0x6C, 0x6D, 0x6E, 0x6F]


def interleave(even: bytes, odd: bytes) -> bytes:
    img = bytearray(len(even) + len(odd))
    img[0::2] = even
    img[1::2] = odd
    return bytes(img)


def describe(img: bytes) -> dict:
    return {
        "len": len(img),
        "sum8": sum(img) & 0xFF,
        "reset": img[0xFFF0:0xFFF5].hex(" "),
        "jmp": f"F000:{int.from_bytes(img[0xFFF1:0xFFF3], 'little'):04X}",
        "version": (VERSION_RE.search(img) or [b"(not found)"])[0].decode("ascii", "ignore")
        if VERSION_RE.search(img)
        else "(not found)",
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--even", required=True)
    ap.add_argument("--odd", required=True)
    ap.add_argument("--out", default="vectra_combined.bin")
    ap.add_argument("--disasm", action="store_true")
    args = ap.parse_args()

    even = pathlib.Path(args.even).read_bytes()
    odd = pathlib.Path(args.odd).read_bytes()
    if len(even) != len(odd):
        print(f"片大小不一致: even={len(even)} odd={len(odd)}", file=sys.stderr)
        return 1
    if len(even) != 32768:
        print(f"警告: 片大小 {len(even)}，期望 32768", file=sys.stderr)

    # 两种拼法都算，取复位向量为 EA 且校验和自洽的那个
    cands = []
    for name, img in (("even=--even / odd=--odd", interleave(even, odd)),
                      ("even=--odd / odd=--even", interleave(odd, even))):
        d = describe(img)
        ok_reset = img[0xFFF0] == 0xEA
        ok_sum = d["sum8"] == 0x00
        cands.append((ok_reset + ok_sum, name, img, d))

    cands.sort(key=lambda x: -x[0])
    score, name, img, d = cands[0]
    print(f"采用拼法: {name}")
    for label, img2, d2 in ((c[1], c[2], c[3]) for c in cands):
        flag = "✔" if d2["reset"].startswith("ea") else "✘"
        print(f"  {flag} {label}: sum8={d2['sum8']:02X} reset={d2['reset']} ver={d2['version']}")

    if d["sum8"] != 0x00:
        print("警告: 8 位校验和不为 0 —— 片序可能反了或有坏字节", file=sys.stderr)

    pathlib.Path(args.out).write_bytes(img)
    print(f"已写出 {args.out} ({len(img)} B)")

    if args.disasm:
        dis = subprocess.run(["ndisasm", "-b", "16", "-o", "0xF0000", args.out],
                             capture_output=True, text=True)
        text = dis.stdout
        print(f"\n反汇编 {len(text.splitlines())} 行，端口引用:")
        for p in PORTS:
            n = len(re.findall(rf"\b(?:out|in)\s+(?:al|ax),\s*0x{p:x}\b", text, re.I))
            n += len(re.findall(rf"\bout\s+0x{p:x},\s*(?:al|ax)\b", text, re.I))
            if n:
                print(f"  0x{p:02x}: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
