#include "psx_bus.c"

static u32 psx_cpu_fetch(struct psx_system *psx)
{
    return psx_bus_read(psx, psx->pc, 4);
}

static void psx_cpu_execute(struct psx_system *psx, u32 ins)
{
    u32 op     = ins >> 26;
    u32 rs     = (ins >> 21) & 0x1F;
    u32 rt     = (ins >> 16) & 0x1F;
    u32 rd     = (ins >> 11) & 0x1F;
    u32 sa     = (ins >>  6) & 0x1F;
    u32 imm    = ins & 0xFFFF;
    u32 imm_s  = (u32)(s32)(s16)imm;  /* sign-extended */
    u32 target = ins & 0x3FFFFFF;

    /* default: sequential next PC (may be overridden by branch/jump) */
    psx->next_pc = psx->pc + 4;

    switch (op) {

    /* ── SPECIAL group (op=0x00) ─────────────────────────────────── */
    case 0x00: {
        u32 fn = ins & 0x3F;
        switch (fn) {
        case 0x00: psx->regs[rd] = psx->regs[rt] << sa; break; /* SLL  */
        case 0x02: psx->regs[rd] = psx->regs[rt] >> sa; break; /* SRL  */
        case 0x03: psx->regs[rd] = (u32)((s32)psx->regs[rt] >> sa); break; /* SRA */
        case 0x04: psx->regs[rd] = psx->regs[rt] << (psx->regs[rs] & 0x1F); break; /* SLLV */
        case 0x06: psx->regs[rd] = psx->regs[rt] >> (psx->regs[rs] & 0x1F); break; /* SRLV */
        case 0x07: psx->regs[rd] = (u32)((s32)psx->regs[rt] >> (psx->regs[rs] & 0x1F)); break; /* SRAV */
        case 0x08: psx->next_pc = psx->regs[rs]; break; /* JR   */
        case 0x09: /* JALR */
            psx->regs[rd]  = psx->pc + 8;
            psx->next_pc   = psx->regs[rs];
            break;
        case 0x0C: /* SYSCALL – trap to exception vector */
        case 0x0D: /* BREAK   – trap to exception vector */
        {
            /* R3000A exception entry:
             * EPC = PC of faulting instruction
             * Cause[6:2] = 8 (SYSCALL) or 9 (BREAK)
             * Status: shift KU/IE bits, enter kernel mode */
            u32 sr = psx->cp0_regs[12];
            psx->cp0_regs[14] = psx->pc; /* EPC */
            psx->cp0_regs[13] = (fn == 0x0C) ? (8u << 2) : (9u << 2); /* Cause */
            /* Shift status bits: old←prev←current; KUc=0, IEc=0 */
            psx->cp0_regs[12] = (sr & ~0x3Fu) | ((sr & 0x0Fu) << 2);
            /* Jump to vector: BEV=0 → 0x80000080, BEV=1 → 0xBFC00180 */
            u32 bev = (sr >> 22) & 1u;
            psx->exception_vec = bev ? 0xBFC00180u : 0x80000080u;
            break;
        }
        case 0x10: psx->regs[rd] = psx->hi; break; /* MFHI */
        case 0x11: psx->hi = psx->regs[rs]; break; /* MTHI */
        case 0x12: psx->regs[rd] = psx->lo; break; /* MFLO */
        case 0x13: psx->lo = psx->regs[rs]; break; /* MTLO */
        case 0x18: { /* MULT */
            s64 r = (s64)(s32)psx->regs[rs] * (s64)(s32)psx->regs[rt];
            psx->lo = (u32)r;  psx->hi = (u32)(r >> 32);
            break;
        }
        case 0x19: { /* MULTU */
            u64 r = (u64)psx->regs[rs] * (u64)psx->regs[rt];
            psx->lo = (u32)r;  psx->hi = (u32)(r >> 32);
            break;
        }
        case 0x1A: { /* DIV */
            s32 n = (s32)psx->regs[rs], d = (s32)psx->regs[rt];
            if (d) { psx->lo = (u32)(n / d); psx->hi = (u32)(n % d); }
            break;
        }
        case 0x1B: { /* DIVU */
            u32 n = psx->regs[rs], d = psx->regs[rt];
            if (d) { psx->lo = n / d; psx->hi = n % d; }
            break;
        }
        case 0x20: /* ADD  (no overflow trap) */
        case 0x21: psx->regs[rd] = psx->regs[rs] + psx->regs[rt]; break; /* ADDU */
        case 0x22: /* SUB  (no overflow trap) */
        case 0x23: psx->regs[rd] = psx->regs[rs] - psx->regs[rt]; break; /* SUBU */
        case 0x24: psx->regs[rd] = psx->regs[rs] & psx->regs[rt]; break; /* AND  */
        case 0x25: psx->regs[rd] = psx->regs[rs] | psx->regs[rt]; break; /* OR   */
        case 0x26: psx->regs[rd] = psx->regs[rs] ^ psx->regs[rt]; break; /* XOR  */
        case 0x27: psx->regs[rd] = ~(psx->regs[rs] | psx->regs[rt]); break; /* NOR */
        case 0x2A: psx->regs[rd] = ((s32)psx->regs[rs] < (s32)psx->regs[rt]) ? 1 : 0; break; /* SLT  */
        case 0x2B: psx->regs[rd] = (psx->regs[rs] < psx->regs[rt]) ? 1 : 0; break; /* SLTU */
        }
        break;
    }



    /* ── Jumps ────────────────────────────────────────────────────── */
    case 0x02: /* J */
        psx->next_pc = (psx->pc & 0xF0000000u) | (target << 2);
        break;
    case 0x03: /* JAL */
        psx->regs[31] = psx->pc + 8;
        psx->next_pc  = (psx->pc & 0xF0000000u) | (target << 2);
        break;

    /* ── Branches ─────────────────────────────────────────────────── */
    case 0x01: { /* REGIMM instructions */
        u32 sub = rt;
        if (sub == 0x00) { if ((s32)psx->regs[rs] < 0) psx->next_pc = psx->pc + 4 + (imm_s << 2); } /* BLTZ */
        if (sub == 0x01) { if ((s32)psx->regs[rs] >= 0) psx->next_pc = psx->pc + 4 + (imm_s << 2); } /* BGEZ */
        if (sub == 0x10) { if ((s32)psx->regs[rs] < 0) { psx->regs[31] = psx->pc + 8; psx->next_pc = psx->pc + 4 + (imm_s << 2); } } /* BLTZAL */
        if (sub == 0x11) { if ((s32)psx->regs[rs] >= 0) { psx->regs[31] = psx->pc + 8; psx->next_pc = psx->pc + 4 + (imm_s << 2); } } /* BGEZAL */
        break;
    }
    case 0x04: /* BEQ  */
        if (psx->regs[rs] == psx->regs[rt])
            psx->next_pc = psx->pc + 4 + (imm_s << 2);
        break;
    case 0x05: /* BNE  */
        if (psx->regs[rs] != psx->regs[rt])
            psx->next_pc = psx->pc + 4 + (imm_s << 2);
        break;
    case 0x06: /* BLEZ */
        if ((s32)psx->regs[rs] <= 0)
            psx->next_pc = psx->pc + 4 + (imm_s << 2);
        break;
    case 0x07: /* BGTZ */
        if ((s32)psx->regs[rs] > 0)
            psx->next_pc = psx->pc + 4 + (imm_s << 2);
        break;

    /* ── Immediate ALU ────────────────────────────────────────────── */
    case 0x08: /* ADDI  (no trap) */
    case 0x09: psx->regs[rt] = psx->regs[rs] + imm_s; break; /* ADDIU */
    case 0x0A: psx->regs[rt] = ((s32)psx->regs[rs] < (s32)imm_s) ? 1 : 0; break; /* SLTI  */
    case 0x0B: psx->regs[rt] = (psx->regs[rs] < imm_s) ? 1 : 0; break; /* SLTIU */
    case 0x0C: psx->regs[rt] = psx->regs[rs] & imm;   break; /* ANDI  */
    case 0x0D: psx->regs[rt] = psx->regs[rs] | imm;   break; /* ORI   */
    case 0x0E: psx->regs[rt] = psx->regs[rs] ^ imm;   break; /* XORI  */
    case 0x0F: psx->regs[rt] = imm << 16;              break; /* LUI   */

    /* ── COP0 ─────────────────────────────────────────────────────── */
    case 0x10:
        if      (rs == 0x00) psx->regs[rt] = psx->cp0_regs[rd]; /* MFC0 */
        else if (rs == 0x04) psx->cp0_regs[rd] = psx->regs[rt]; /* MTC0 */
        else if (rs == 0x10 && (ins & 0x3F) == 0x10) { /* RFE */
            u32 sr = psx->cp0_regs[12];
            psx->cp0_regs[12] = (sr & ~0x3Fu) | ((sr >> 2) & 0x0Fu);
        }
        break;

    /* ── Memory loads ─────────────────────────────────────────────── */
    case 0x20: psx->regs[rt] = (u32)(s32)(s8)psx_bus_read(psx, psx->regs[rs] + imm_s, 1); break; /* LB */
    case 0x21: psx->regs[rt] = (u32)(s32)(s16)psx_bus_read(psx, psx->regs[rs] + imm_s, 2); break; /* LH */
    case 0x23: psx->regs[rt] = psx_bus_read(psx, psx->regs[rs] + imm_s, 4); break; /* LW */
    case 0x24: psx->regs[rt] = psx_bus_read(psx, psx->regs[rs] + imm_s, 1); break; /* LBU */
    case 0x25: psx->regs[rt] = psx_bus_read(psx, psx->regs[rs] + imm_s, 2); break; /* LHU */
    
    case 0x22: { /* LWL – load word left (unaligned) */
        u32 addr = psx->regs[rs] + imm_s;
        u32 mem  = psx_bus_read(psx, addr & ~3u, 4);
        u32 mod  = addr & 3;
        if      (mod == 0) psx->regs[rt] = (psx->regs[rt] & 0x00FFFFFFu) | (mem << 24);
        else if (mod == 1) psx->regs[rt] = (psx->regs[rt] & 0x0000FFFFu) | (mem << 16);
        else if (mod == 2) psx->regs[rt] = (psx->regs[rt] & 0x000000FFu) | (mem << 8);
        else if (mod == 3) psx->regs[rt] = mem;
        break;
    }
    case 0x26: { /* LWR – load word right (unaligned) */
        u32 addr = psx->regs[rs] + imm_s;
        u32 mem  = psx_bus_read(psx, addr & ~3u, 4);
        u32 mod  = addr & 3;
        if      (mod == 0) psx->regs[rt] = mem;
        else if (mod == 1) psx->regs[rt] = (psx->regs[rt] & 0xFF000000u) | (mem >> 8);
        else if (mod == 2) psx->regs[rt] = (psx->regs[rt] & 0xFFFF0000u) | (mem >> 16);
        else if (mod == 3) psx->regs[rt] = (psx->regs[rt] & 0xFFFFFF00u) | (mem >> 24);
        break;
    }

    /* ── Memory stores ────────────────────────────────────────────── */
    case 0x28: psx_bus_write(psx, psx->regs[rs] + imm_s, psx->regs[rt], 1); break; /* SB */
    case 0x29: psx_bus_write(psx, psx->regs[rs] + imm_s, psx->regs[rt], 2); break; /* SH */
    case 0x2A: { /* SWL – store word left (unaligned, Little-Endian) */
        u32 addr = psx->regs[rs] + imm_s;
        u32 mem  = psx_bus_read(psx, addr & ~3u, 4);
        u32 mod  = addr & 3;
        if      (mod == 0) mem = (mem & 0xFFFFFF00u) | (psx->regs[rt] >> 24);
        else if (mod == 1) mem = (mem & 0xFFFF0000u) | (psx->regs[rt] >> 16);
        else if (mod == 2) mem = (mem & 0xFF000000u) | (psx->regs[rt] >> 8);
        else if (mod == 3) mem = psx->regs[rt];
        psx_bus_write(psx, addr & ~3u, mem, 4);
        break;
    }
    case 0x2B: psx_bus_write(psx, psx->regs[rs] + imm_s, psx->regs[rt], 4); break; /* SW */
    case 0x2E: { /* SWR – store word right (unaligned, Little-Endian) */
        u32 addr = psx->regs[rs] + imm_s;
        u32 mem  = psx_bus_read(psx, addr & ~3u, 4);
        u32 mod  = addr & 3;
        if      (mod == 0) mem = psx->regs[rt];
        else if (mod == 1) mem = (mem & 0x000000FFu) | (psx->regs[rt] << 8);
        else if (mod == 2) mem = (mem & 0x0000FFFFu) | (psx->regs[rt] << 16);
        else if (mod == 3) mem = (mem & 0x00FFFFFFu) | (psx->regs[rt] << 24);
        psx_bus_write(psx, addr & ~3u, mem, 4);
        break;
    }

    /* ── COP2 (GTE) – stub: ignore for now ──────────────────────── */
    case 0x12: break;
    case 0x32: break; /* LWC2 */
    case 0x3A: break; /* SWC2 */

    default: break; /* unknown opcode – treat as NOP */
    }

    psx->regs[0] = 0; /* r0 is hardwired zero */
}

/* ── Hardware interrupt check (R3000A) ───────────────────────────── */
static void psx_check_irq(struct psx_system *psx)
{
    u32 sr = psx->cp0_regs[12];

    /* IE (bit 0) must be set; if already in exception (IEc was cleared), skip */
    if (!(sr & 1u)) return;

    /* Hardware interrupt pending? PSX routes all I/O ints through IP[2] (bit 10) */
    if (!(psx->i_stat & psx->i_mask)) return;

    /* CPU must have IP[2] unmasked in Status (bit 10) */
    if (!(sr & (1u << 10))) return;

    /* Take hardware interrupt exception */
    psx->cp0_regs[14] = psx->pc;              /* EPC = interrupted PC */
    psx->cp0_regs[13] = (1u << 10);           /* Cause: IP[2], ExcCode=0 */
    psx->cp0_regs[12] = (sr & ~0x3Fu) | ((sr & 0x0Fu) << 2); /* KU/IE shift */

    u32 bev = (sr >> 22) & 1u;
    psx->exception_vec = bev ? 0xBFC00180u : 0x80000080u;
}

/* ── Single CPU step with MIPS delay-slot semantics ─────────────── */
void psx_cpu_step(struct psx_system *psx)
{
    psx->exception_vec = 0;

    /* Check for hardware interrupt before executing instruction */
    psx_check_irq(psx);
    if (psx->exception_vec) {
        psx->pc = psx->exception_vec;
        return;
    }

    u32 ins = psx_cpu_fetch(psx);
    psx->next_pc = psx->pc + 4;
    psx_cpu_execute(psx, ins);

    if (psx->exception_vec) {
        /* Exception (SYSCALL/BREAK): no delay slot, jump directly */
        psx->pc = psx->exception_vec;
    } else if (psx->next_pc != psx->pc + 4) {
        /* Branch or jump: execute delay slot before jumping */
        u32 target = psx->next_pc;
        psx->pc = psx->pc + 4;
        u32 ds  = psx_cpu_fetch(psx);
        psx->next_pc = psx->pc + 4;
        psx_cpu_execute(psx, ds);
        psx->pc = target;
    } else {
        psx->pc = psx->next_pc;
    }

    psx->regs[0] = 0; /* R0 must always be zero */
}
