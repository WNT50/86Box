/* 86Box: MCS-48 interpreter for the Mindset system controller. */
/* See src/include/86box/mcs48.h for the reference note. */
/* Minimal MCS-48 (8048/8049/8042) interpreter, written for the Mindset's
   system controller.  Instruction encodings follow Intel's MCS-48 Family
   User's Manual; the port wiring is the caller's business.
   Standalone test build: gcc -O2 -o t mcs48_core.c mcs48_test.c */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <86box/mcs48.h>


struct mcs48 {
    uint8_t  rom[4096];
    uint32_t rom_mask;
    uint8_t  ram[128];
    uint8_t  ext[256];   /* MOVX space: the board's register file */

    uint8_t  a, psw;
    uint8_t  p1, p2;
    int      t0, t1;
    uint8_t  t;                 /* timer/counter */
    int      tf;                /* timer flag */
    int      t_running, cnt_running;
    int      t_prescale;      /* machine cycles toward the next timer tick */
    int      irq_en, tcnti_en;
    int      f0, f1;
    int      mb;                /* MB0/MB1: program memory bank */
    int      int_pending;
    uint16_t pc;                /* 12-bit */
    uint16_t dbuf;              /* 12-bit, next PC while executing */
    int      cycles;
    uint8_t  bus_out;           /* last OUTL BUS value */
    int      halted;

    mcs48_hooks h;
};

mcs48 *
mcs48_create(void)
{
    return (mcs48 *) calloc(1, sizeof(mcs48));
}

uint8_t *
mcs48_rom(mcs48 *c)
{
    return c->rom;
}

void
mcs48_set_rom_mask(mcs48 *c, uint32_t mask)
{
    c->rom_mask = mask;
}

uint8_t *
mcs48_ext(mcs48 *c)
{
    return c->ext;
}

mcs48_hooks *
mcs48_hooks_of(mcs48 *c)
{
    return &c->h;
}

void
mcs48_destroy(mcs48 *c)
{
    free(c);
}

void mcs48_reset(mcs48 *c)
{
    c->a = 0; c->psw = 0x08; c->p1 = 0xff; c->p2 = 0xff;
    c->t = 0; c->tf = 0; c->t_running = 0; c->cnt_running = 0;
    c->irq_en = 0; c->tcnti_en = 0; c->f0 = 0; c->f1 = 0;
    c->mb = 0; c->int_pending = 0; c->pc = 0; c->dbuf = 0;
    c->halted = 0; c->bus_out = 0xff;
}

/* ---- helpers ---- */
static uint8_t rd(mcs48 *c, uint8_t addr) { return c->ram[addr & 0x7f]; }
static void    wr(mcs48 *c, uint8_t addr, uint8_t v) { c->ram[addr & 0x7f] = v; }
static int     rsel(mcs48 *c) { return (c->psw & 0x10) ? 1 : 0; }
static uint8_t *reg(mcs48 *c, int n) { return &c->ram[(rsel(c) * 8) + (n & 7)]; }
static int     cbit(mcs48 *c) { return (c->psw & 0x80) ? 1 : 0; }
static void    setc(mcs48 *c, int v) { if (v) c->psw |= 0x80; else c->psw &= 0x7f; }
static int     abit(mcs48 *c) { return (c->psw & 0x40) ? 1 : 0; }
static void    seta(mcs48 *c, int v) { if (v) c->psw |= 0x40; else c->psw &= ~0x40; }

static void push_pc(mcs48 *c)
{
    int sp = c->psw & 7;
    uint8_t lo = (uint8_t) (c->pc & 0xff);
    uint8_t hi = (uint8_t) ((c->pc >> 8) & 0x0f);

    wr(c, (uint8_t) (8 + sp * 2), lo);
    wr(c, (uint8_t) (8 + sp * 2 + 1),
       (uint8_t) ((rd(c, (uint8_t) (8 + sp * 2 + 1)) & 0xf0) | hi));
    c->psw = (uint8_t) ((c->psw & ~7) | ((sp + 1) & 7));
}

static void pop_pc(mcs48 *c)
{
    int sp = (c->psw - 1) & 7;
    uint8_t lo = rd(c, 8 + sp * 2);
    uint8_t hi = rd(c, 8 + sp * 2 + 1) & 0x0f;

    c->pc = (uint16_t) ((hi << 8) | lo);
    c->psw = (uint8_t) ((c->psw & ~7) | sp);
}

static void update_t0(mcs48 *c)
{
    if (c->h.t0_in) c->t0 = c->h.t0_in(c);
    if (c->h.t1_in) c->t1 = c->h.t1_in(c);
}

static void add_a(mcs48 *c, uint8_t v, int with_carry)
{
    int r = c->a + v + (with_carry ? cbit(c) : 0);

    setc(c, r > 0xff);
    seta(c, ((c->a & 0x0f) + (v & 0x0f) + (with_carry ? cbit(c) : 0)) > 0x0f);
    c->a = (uint8_t) r;
}

/* Runs one instruction; returns cycles used. */
int mcs48_step(mcs48 *c)
{
    uint8_t op, arg;
    int     used = 1, take = 0, target = 0;

    if (c->int_pending) {
        c->int_pending = 0;
        if (c->irq_en) {
            push_pc(c);
            c->pc = 0x003;
            return 2;
        }
    }

    op = c->rom[c->pc & c->rom_mask];
    c->pc = (c->pc + 1) & 0xfff;
    arg = c->rom[c->pc & c->rom_mask];

    switch (op) {
        case 0x00: break;                                    /* NOP */
        case 0x02: c->bus_out = c->a; if (c->h.bus_out) c->h.bus_out(c, c->a); break;
        case 0x03: add_a(c, arg, 0); c->pc++; break;
        case 0x05: c->irq_en = 1; break;
        case 0x07: c->a--; seta(c, (c->a & 0x0f) == 0x0f); break;
        case 0x08: c->a = c->h.bus_in ? c->h.bus_in(c) : 0xff; break;
        case 0x09: c->a = c->h.p1_in ? c->h.p1_in(c) : c->p1; break;
        case 0x0a: c->a = c->h.p2_in ? c->h.p2_in(c) : c->p2; break;
        case 0x10: { uint8_t *p = &c->ram[*reg(c, 0) & 0x7f]; (*p)++; break; }
        case 0x11: { uint8_t *p = &c->ram[*reg(c, 1) & 0x7f]; (*p)++; break; }
        case 0x12: take = (c->a & 1) == 0; break;             /* JB0 */
        case 0x13: add_a(c, arg, 1); c->pc++; break;
        case 0x15: c->irq_en = 0; break;
        case 0x16: take = c->tf; c->tf = 0; break;            /* JTF */
        case 0x17: c->a++; seta(c, (c->a & 0x0f) == 0); break;
        case 0x20: { uint8_t *p = &c->ram[*reg(c, 0) & 0x7f]; uint8_t t = c->a; c->a = *p; *p = t; break; }
        case 0x21: { uint8_t *p = &c->ram[*reg(c, 1) & 0x7f]; uint8_t t = c->a; c->a = *p; *p = t; break; }
        case 0x23: c->a = arg; c->pc++; break;
        case 0x25: c->tcnti_en = 1; break;
        case 0x26: take = c->t0 == 0; break;                  /* JNT0 */
        case 0x27: c->a = 0; break;
        case 0x30: { uint8_t *p = &c->ram[*reg(c, 0) & 0x7f]; uint8_t t = c->a & 0x0f;
                     c->a = (uint8_t) ((c->a & 0xf0) | (*p & 0x0f)); *p = (uint8_t) ((*p & 0xf0) | t); break; }
        case 0x31: { uint8_t *p = &c->ram[*reg(c, 1) & 0x7f]; uint8_t t = c->a & 0x0f;
                     c->a = (uint8_t) ((c->a & 0xf0) | (*p & 0x0f)); *p = (uint8_t) ((*p & 0xf0) | t); break; }
        case 0x32: take = (c->a & 2) == 0; break;             /* JB1 */
        case 0x35: c->tcnti_en = 0; break;
        case 0x36: take = c->t0 != 0; break;                  /* JT0 */
        case 0x37: c->a = (uint8_t) ~c->a; break;
        case 0x39: c->p1 = c->a; if (c->h.p1_out) c->h.p1_out(c, c->a); break;
        case 0x3a: c->p2 = c->a; if (c->h.p2_out) c->h.p2_out(c, c->a); break;
        case 0x40: c->a |= c->ram[*reg(c, 0) & 0x7f]; break;
        case 0x41: c->a |= c->ram[*reg(c, 1) & 0x7f]; break;
        case 0x42: c->a = c->t; break;
        case 0x43: c->a |= arg; c->pc++; break;
        case 0x45: c->cnt_running = 1; c->t_running = 0; break;
        case 0x46: take = c->t1 == 0; break;                  /* JNT1 */
        case 0x47: c->a = (uint8_t) ((c->a >> 4) | (c->a << 4)); break;
        case 0x50: c->a &= c->ram[*reg(c, 0) & 0x7f]; break;
        case 0x51: c->a &= c->ram[*reg(c, 1) & 0x7f]; break;
        case 0x52: take = (c->a & 4) == 0; break;             /* JB2 */
        case 0x53: c->a &= arg; c->pc++; break;
        case 0x55: c->t_running = 1; c->cnt_running = 0; break;
        case 0x56: take = c->t1 != 0; break;                  /* JT1 */
        case 0x57: {                                          /* DA A */
            int lo = c->a & 0x0f, hi = c->a >> 4, cy = cbit(c);
            if (lo > 9) lo += 6;
            if (hi > 9 || cy) { hi += 6; cy = 1; } else cy = 0;
            if (hi > 0x0f) { hi &= 0x0f; cy = 1; }
            c->a = (uint8_t) ((hi << 4) | (lo & 0x0f));
            setc(c, cy);
            break;
        }
        case 0x60: add_a(c, c->ram[*reg(c, 0) & 0x7f], 0); break;
        case 0x61: add_a(c, c->ram[*reg(c, 1) & 0x7f], 0); break;
        case 0x62: c->t = c->a; break;
        case 0x65: c->t_running = 0; c->cnt_running = 0; break;
        case 0x67: {                                          /* RRC A */
            int nc = cbit(c);
            setc(c, c->a & 1);
            c->a = (uint8_t) ((c->a >> 1) | (nc ? 0x80 : 0));
            break;
        }
        case 0x70: add_a(c, c->ram[*reg(c, 0) & 0x7f], 1); break;
        case 0x71: add_a(c, c->ram[*reg(c, 1) & 0x7f], 1); break;
        case 0x72: take = (c->a & 8) == 0; break;             /* JB3 */
        case 0x75: break;                                     /* ENT0 CLK */
        case 0x76: take = c->f1; break;                       /* JF1 */
        case 0x77: c->a = (uint8_t) ((c->a >> 1) | ((c->a & 1) << 7)); break;
        case 0x80: c->a = c->ext[*reg(c, 0)]; break;   /* MOVX A,@R0 */
        case 0x81: c->a = c->ext[*reg(c, 1)]; break;   /* MOVX A,@R1 */
        case 0x83: pop_pc(c); used = 2; break;                /* RET */
        case 0x85: c->f0 = 0; break;
        case 0x86: take = c->int_pending == 0; break;         /* JNI (approx) */
        case 0x88: case 0x89: case 0x8a:                      /* ORL BUS/P1/P2,# */
            if (op == 0x89) { c->p1 |= arg; if (c->h.p1_out) c->h.p1_out(c, c->p1); }
            else if (op == 0x8a) { c->p2 |= arg; if (c->h.p2_out) c->h.p2_out(c, c->p2); }
            else { c->bus_out |= arg; if (c->h.bus_out) c->h.bus_out(c, c->bus_out); }
            c->pc++;
            break;
        case 0x90: c->ext[*reg(c, 0)] = c->a; break;         /* MOVX @R0,A */
        case 0x91: c->ext[*reg(c, 1)] = c->a; break;         /* MOVX @R1,A */
        case 0x92: take = (c->a & 0x10) == 0; break;          /* JB4 */
        case 0x93: pop_pc(c); c->irq_en = 1; used = 2; break; /* RETR */
        case 0x95: c->f0 = !c->f0; break;
        case 0x96: take = c->a != 0; break;                   /* JNZ */
        case 0x97: setc(c, 0); break;
        case 0x98: case 0x99: case 0x9a:                      /* ANL BUS/P1/P2,# */
            if (op == 0x99) { c->p1 &= arg; if (c->h.p1_out) c->h.p1_out(c, c->p1); }
            else if (op == 0x9a) { c->p2 &= arg; if (c->h.p2_out) c->h.p2_out(c, c->p2); }
            else { c->bus_out &= arg; if (c->h.bus_out) c->h.bus_out(c, c->bus_out); }
            c->pc++;
            break;
        case 0xa0: c->ram[*reg(c, 0) & 0x7f] = c->a; break;
        case 0xa1: c->ram[*reg(c, 1) & 0x7f] = c->a; break;
        case 0xa3: c->a = c->rom[((c->pc & 0xf00) | c->a) & c->rom_mask]; used = 2; break;
        case 0xa5: c->f1 = 0; break;
        case 0xa7: setc(c, !cbit(c)); break;
        case 0xb0: c->ram[*reg(c, 0) & 0x7f] = arg; c->pc++; break;
        case 0xb1: c->ram[*reg(c, 1) & 0x7f] = arg; c->pc++; break;
        case 0xb2: take = (c->a & 0x20) == 0; break;          /* JB5 */
        case 0xb3: {                                          /* JMPP @A */
            uint16_t npc = (uint16_t) ((c->pc & 0xf00) | c->a);
            c->pc = c->rom[npc & c->rom_mask];
            used = 2;
            break;
        }
        case 0xb5: c->f1 = !c->f1; break;
        case 0xb6: take = c->f0; break;                       /* JF0 */
        case 0xc5: c->psw &= ~0x10; break;                    /* SEL RB0 */
        case 0xc6: take = c->a == 0; break;                   /* JZ */
        case 0xc7: c->a = c->psw; break;                      /* MOV A,PSW */
        case 0xd0: c->a ^= c->ram[*reg(c, 0) & 0x7f]; break;
        case 0xd1: c->a ^= c->ram[*reg(c, 1) & 0x7f]; break;
        case 0xd2: take = (c->a & 0x40) == 0; break;          /* JB6 */
        case 0xd3: c->a ^= arg; c->pc++; break;
        case 0xd5: c->psw |= 0x10; break;                     /* SEL RB1 */
        case 0xd7: c->psw = c->a; break;                      /* MOV PSW,A */
        case 0xe3: c->a = c->rom[(0x300 | c->a) & c->rom_mask]; used = 2; break;
        case 0xe5: c->mb = 0; break;
        case 0xf5: c->mb = 1; break;
        default:
            /* block forms */
            if ((op & 0x1f) == 0x04) {                        /* JMP addr */
                target = (uint16_t) (((op & 0xe0) << 3) | arg);
                c->pc = target; used = 2;
                break;
            }
            if ((op & 0x1f) == 0x14) {                        /* CALL addr */
                target = (uint16_t) (((op & 0xe0) << 3) | arg);
                push_pc(c);
                c->pc = target; used = 2;
                break;
            }
            switch (op & 0xf0) {
                case 0x10: (*reg(c, op & 7))++; break;        /* INC Rr */
                case 0x20: { uint8_t t = c->a; c->a = *reg(c, op & 7); *reg(c, op & 7) = t; break; } /* XCH A,Rr */
                case 0x40: c->a |= *reg(c, op & 7); break;    /* ORL A,Rr */
                case 0x50: c->a &= *reg(c, op & 7); break;    /* ANL A,Rr */
                case 0x60: add_a(c, *reg(c, op & 7), 0); break;
                case 0x70: add_a(c, *reg(c, op & 7), 1); break;
                case 0xa0: *reg(c, op & 7) = c->a; break;     /* MOV Rr,A */
                case 0xc0: (*reg(c, op & 7))--; break;        /* DEC Rr */
                case 0xd0: c->a ^= *reg(c, op & 7); break;    /* XRL A,Rr */
                case 0xb0: *reg(c, op & 7) = arg; c->pc++; break;  /* MOV Rr,# */
                default: break;
            }
            break;
    }

    if (take) { c->pc = (uint16_t) ((c->pc & 0xf00) | arg); used = 2; }

    /* The timer counts machine cycles divided by 32, and one machine cycle is
       fifteen clock periods. */
    if (c->t_running) {
        c->t_prescale += used;

        while (c->t_prescale >= 32) {
            c->t_prescale -= 32;

            if (++c->t == 0) {
                c->tf = 1;
                if (c->tcnti_en && c->irq_en) c->int_pending = 1;
            }
        }
    }

    c->cycles += used;
    return used;
}

void mcs48_run(mcs48 *c, int cycles)
{
    int target = c->cycles + cycles;

    while (c->cycles < target)
        mcs48_step(c);
}
