# HP Vectra 45945A: interrupt controller structure

Source: HP Vectra hardware reference, Volume 1, **Figure 9 "Interrupt
Controller Structure"** (page supplied 2026-10-31). This is the board's own
wiring, so it overrides anything we inferred from POST behaviour.

## Master: INT 1 (IRQ 0-7)

| IRQ | Vector | Vector address | Source |
|---:|---|---|---|
| 0 | 08h | 000020h | Timer 0 output |
| 1 | 09h | 000024h | **Cascade from controller #3** |
| 2 | 0Ah | 000028h | **Cascade from controller #2** |
| 3 | 0Bh | 00002Ch | Serial port 2 |
| 4 | 0Ch | 000030h | Serial port 1 |
| 5 | 0Dh | 000034h | Parallel port 1 |
| 6 | 0Eh | 000038h | Flexible disc controller |
| 7 | 0Fh | 00003Ch | Parallel port 2 |

The master's vector base is **XT style (08h)**, not the AT 20h. Anything in
`m_at_286.c` that assumes the AT remap does not apply to this machine.

## Slave #2: INT 2 (IRQ 8-15)

| IRQ | Vector | Vector address | Source |
|---:|---|---|---|
| 8 | 70h | 0001C0h | Real time clock |
| 9 | 71h | 0001C4h | IRQ9 - I/O channel |
| 10 | 72h | 0001C8h | Reserved |
| 11 | 73h | 0001CCh | Reserved |
| 12 | 74h | 0001D0h | Reserved |
| 13 | 75h | 0001D4h | Coprocessor (80287) |
| 14 | 76h | 0001D8h | Hard disc controller |
| 15 | 77h | 0001DCh | Reserved |

## Slave #3: INT 3 (IRQ 16-23), ports 7Ch-7Dh

| IRQ | Vector | Vector address | Source |
|---:|---|---|---|
| 16 | 68h | 0001A0h | **SVC request - 8041** |
| 17 | 69h | 0001A4h | **OBF - 8041** |
| 18 | 6Ah | 0001A8h | Reserved |
| 19 | 6Bh | 0001ACh | Reserved |
| 20 | 6Ch | 0001B0h | **HPHIL controller** |
| 21 | 6Dh | 0001B4h | Reserved |
| 22 | 6Eh | 0001B8h | Reserved |
| 23 | 6Fh | 0001BCh | Reserved |

IRQ 16/17 are the third controller's lines 0 and 1; IRQ 20 is its line 4.
`INT 3`'s output is wired to master IRQ 1, `INT 2`'s to master IRQ 2.

## What this settles

1. `pic3_init()` is right in direction: third 8259 on 7Ch-7Dh, cascaded into
   master IR1. Since upstream #8561 the cascade has to go through
   `pic_attach_slave(&pic3, 1)`, otherwise `master_irq` stays 0 and the PIC is
   fed into IRQ 0.
2. `picint_lines()` covers only 0-15 (`pic` and `pic2`). There is **no way to
   raise a third-PIC line yet**; a small `pic3_int(line, set)` is needed that
   sets `pic3.irr`, recomputes `int_pending`, and lets the existing update path
   propagate it into the master through `master_irq`.
3. `src/device/hp_hil.c` never raises an interrupt at all. POST waits at
   `F000:92AB` on the HP-HIL interrupt, which per this figure is **third PIC
   line 4 -> IRQ 20 -> vector 6Ch** (its ISR sits at `F000:552D`).
4. `kbc_hp_8041.c` only does the port-level loopback; its SVC and OBF lines are
   third PIC lines 0 and 1 (IRQ 16/17, vectors 68h/69h) and are still unused.
