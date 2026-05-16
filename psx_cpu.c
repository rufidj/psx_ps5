#include "psx_bus.c"
#include "psx_gte.c"

static int psx_cpu_fetch_is_mapped(u32 addr)
{
    u32 phys = psx_bus_mask(addr);
    if (phys < (2u * 1024u * 1024u)) return 1;
    if (phys >= 0x1FC00000u && phys < 0x1FC80000u) return 1;
    if (phys >= 0x1F800000u && phys < 0x1F800400u) return 1;
    if (phys >= 0x1F801000u && phys < 0x1F802000u) return 1;
    return 0;
}

static u32 psx_cpu_fetch(struct psx_system *psx)
{
    if (psx->exe_entry_pc && psx->pc < 0x00200000u) {
        if (!psx->low_pc_fix_logged) {
            psx->low_pc_fix_logged = 1;
            psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                       "[CPU] CanonPC from=", psx->pc);
        }
        psx->pc |= 0x80000000u;
        if (psx->low_pc_fix_logged == 1u) {
            psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                       "[CPU] CanonPC to=", psx->pc);
            psx->low_pc_fix_logged = 2u;
        }
    }
    if (!psx->bad_fetch_logged && !psx_cpu_fetch_is_mapped(psx->pc)) {
        psx->bad_fetch_logged = 1;
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                   "[CPU] BadFetch PC=", psx->pc);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                   "[CPU] BadFetch RA=", psx->regs[31]);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                   "[CPU] BadFetch SP=", psx->regs[29]);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                   "[CPU] BadFetch GP=", psx->regs[28]);
    }
    return psx_bus_read(psx, psx->pc, 4);
}

static void psx_cpu_log_bad_jump(struct psx_system *psx, u32 ins, u32 target,
                                 u32 rs, u32 rt)
{
    if (psx->bad_jump_logged || psx_cpu_fetch_is_mapped(target)) return;
    psx->bad_jump_logged = 1;
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] BadJump PC=", psx->pc);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] BadJump INS=", ins);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] BadJump TGT=", target);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] BadJump RS=", rs);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] BadJump RT=", rt);
}

static void psx_cpu_log_exception(struct psx_system *psx)
{
    (void)psx; /* EXC logs disabled */
}

static void psx_cpu_log_hot_loop(struct psx_system *psx)
{
    u32 pc = psx->pc;
    u32 ins;
    if (psx->hot_loop_logged) return;
    if (pc < 0x80029774u || pc > 0x8002978Cu) return;
    ins = psx_bus_read(psx, pc, 4);
    psx->hot_loop_logged = 1;
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL PC=", pc);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL INS=", ins);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL istat=", psx->i_stat);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL imask=", psx->i_mask);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL SR=", psx->cp0_regs[12]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL RA=", psx->regs[31]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL S0=", psx->regs[16]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL S1=", psx->regs[17]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL V0=", psx->regs[2]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL A0=", psx->regs[4]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL T0=", psx->regs[8]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
               "[CPU] HL T1=", psx->regs[9]);
    /* Log 8 words around the stuck PC for disassembly */
    for (u32 _i = 0; _i < 8u; _i++) {
        u32 _w = psx_bus_read(psx, pc + _i*4u, 4);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                   "[CPU] HL W=", _w);
    }
}

static void psx_cpu_log_bios_call(struct psx_system *psx)
{
    u32 pc = psx->pc;
    u32 vec = pc & 0x1FFFFFFFu;
    const char *tag = 0;
    u32 fn;
    int interesting = 0;

    return; /* BIOS call logs disabled */
    if (psx->bios_call_logs >= 96u) return;
    fn = psx->regs[9] & 0xFFu;
    if (vec == 0xA0u) {
        tag = "[BIOS] A0 fn=";
        interesting = (fn == 0x44u || fn == 0x45u || fn == 0x96u || fn == 0x97u ||
                       fn == 0x99u || fn == 0x9Cu || fn == 0x9Du || fn == 0xA1u);
    } else if (vec == 0xB0u) {
        tag = "[BIOS] B0 fn=";
        interesting = ((fn >= 0x07u && fn <= 0x0Du) || (fn >= 0x12u && fn <= 0x16u) ||
                       fn == 0x18u || fn == 0x19u ||
                       fn == 0x20u || fn == 0x47u);
    } else if (vec == 0xC0u) {
        tag = "[BIOS] C0 fn=";
        interesting = (fn == 0x07u || fn == 0x08u || fn == 0x09u || fn == 0x0Au ||
                       fn == 0x0Cu || fn == 0x0Du || fn == 0x12u || fn == 0x1Cu);
    } else {
        return;
    }
    if (!interesting) return;

    psx->bios_call_logs++;
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, tag, fn);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[BIOS] A0=", psx->regs[4]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[BIOS] A1=", psx->regs[5]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[BIOS] A2=", psx->regs[6]);
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[BIOS] A3=", psx->regs[7]);
}

static int psx_bios_cd_event_slot_from_spec(u32 spec)
{
    switch (spec) {
    case 0x00000010u: return 0;
    case 0x00000020u: return 1;
    case 0x00000040u: return 2;
    case 0x00000080u: return 3;
    case 0x00008000u: return 4;
    default: return -1;
    }
}

static u32 psx_bios_cd_event_handle_from_slot(int slot)
{
    return 0xF1CD0000u + (u32)slot;
}

static int psx_bios_cd_event_slot_from_handle(u32 handle)
{
    if (handle < 0xF1CD0000u || handle > 0xF1CD0004u) return -1;
    return (int)(handle - 0xF1CD0000u);
}

static u32 psx_bios_event_handle_from_slot(int slot)
{
    return 0xF1000000u + (u32)slot;
}

static int psx_bios_event_slot_from_handle(u32 handle)
{
    if (handle < 0xF1000000u || handle > 0xF100000Fu) return -1;
    return (int)(handle - 0xF1000000u);
}

static int psx_bios_alloc_event_slot(struct psx_system *psx)
{
    for (int slot = 0; slot < 16; slot++) {
        if (!(psx->bios_ev_open & (1u << (u32)slot)))
            return slot;
    }
    return -1;
}

static int psx_cpu_handle_bios_event_call(struct psx_system *psx)
{
    u32 vec = psx->pc & 0x1FFFFFFFu;
    u32 fn = psx->regs[9] & 0xFFu;
    u32 a0 = psx->regs[4];
    u32 a1 = psx->regs[5];
    u32 a2 = psx->regs[6];
    int slot;

    if (vec != 0xB0u) return 0;

    if (fn == 0x12u) { /* InitPAD2(buf1,siz1,buf2,siz2) */
        psx->bios_pad_buf1 = a0;
        psx->bios_pad_siz1 = a1;
        psx->bios_pad_buf2 = a2;
        psx->bios_pad_siz2 = psx->regs[7];
        psx->bios_pad_started = 0;
        psx->bios_pad_use_hidden = 0;
        psx->bios_pad_button_dest = 0;
        for (u32 i = 0; i < a1; i++)
            psx_bus_write(psx, a0 + i, 0u, 1);
        for (u32 i = 0; i < psx->regs[7]; i++)
            psx_bus_write(psx, a2 + i, 0u, 1);
        psx->regs[2] = 2u;
        psx->pc = psx->regs[31];
        return 1;
    }

    if (fn == 0x13u) { /* StartPAD2() — let BIOS ROM install real VBL handler.
                        * The BIOS handler implements the J$/EPC-advance VSync wait
                        * mechanism. We still set bios_pad_started so our own pad
                        * buffer update continues, but we do NOT intercept the call. */
        psx->bios_pad_started = 1u;
        return 0; /* pass-through: BIOS ROM installs its own VBL callback chain */
    }

    if (fn == 0x14u) { /* StopPAD2() */
        psx->bios_pad_started = 0u;
        psx->regs[2] = 1u;
        psx->pc = psx->regs[31];
        return 1;
    }

    if (fn == 0x15u) { /* PAD_init2(type,button_dest,...) */
        if (a0 != 0x20000000u && a0 != 0x20000001u) {
            psx->regs[2] = 0u;
        } else {
            for (u32 i = 0; i < 0x22u; i++) {
                psx->bios_pad_hidden1[i] = 0;
                psx->bios_pad_hidden2[i] = 0;
            }
            psx->bios_pad_use_hidden = 1u;
            psx->bios_pad_started = 1u;
            psx->bios_pad_button_dest = a1;
            psx->regs[2] = 2u;
        }
        psx->pc = psx->regs[31];
        return 1;
    }

    if (fn == 0x16u) { /* PAD_dr() */
        psx->regs[2] = psx_pad_bios_dr(psx);
        psx->pc = psx->regs[31];
        return 1;
    }

    if (fn == 0x08u) { /* OpenEvent */
        if (a0 == 0xF0000003u && a2 == 0x00002000u) {
            slot = psx_bios_cd_event_slot_from_spec(a1);
            if (slot >= 0) {
                u32 bit = 1u << (u32)slot;
                psx->bios_cd_ev_open |= bit;
                psx->bios_cd_ev_enabled &= ~bit;
                psx->bios_cd_ev_ready &= ~bit;
                psx->regs[2] = psx_bios_cd_event_handle_from_slot(slot);
                psx->pc = psx->regs[31];
                return 1;
            }
        }
        slot = psx_bios_alloc_event_slot(psx);
        if (slot >= 0) {
            u32 bit = 1u << (u32)slot;
            psx->bios_ev_open |= bit;
            psx->bios_ev_enabled &= ~bit;
            psx->bios_ev_ready &= ~bit;
            psx->bios_ev_class[slot] = a0;
            psx->bios_ev_spec[slot] = a1;
            psx->bios_ev_mode[slot] = a2;
            psx->bios_ev_func[slot] = psx->regs[7];
            psx->regs[2] = psx_bios_event_handle_from_slot(slot);
            psx->pc = psx->regs[31];
            return 1;
        }
        psx->regs[2] = 0xFFFFFFFFu;
        psx->pc = psx->regs[31];
        return 1;
    }

    if (fn == 0x09u || fn == 0x0Au || fn == 0x0Cu || fn == 0x0Du || fn == 0x0Bu) {
        slot = psx_bios_cd_event_slot_from_handle(a0);
        if (slot >= 0 && (psx->bios_cd_ev_open & (1u << (u32)slot))) {
            u32 bit = 1u << (u32)slot;
            if (fn == 0x09u) { /* CloseEvent */
                psx->bios_cd_ev_open &= ~bit;
                psx->bios_cd_ev_enabled &= ~bit;
                psx->bios_cd_ev_ready &= ~bit;
                psx->regs[2] = 1;
            } else if (fn == 0x0Au) { /* WaitEvent */
                if (!(psx->bios_cd_ev_enabled & bit)) {
                    psx->regs[2] = 0;
                } else if (psx->bios_cd_ev_ready & bit) {
                    psx->bios_cd_ev_ready &= ~bit;
                    psx->regs[2] = 1;
                } else {
                    /* Real BIOS blocks here until the event becomes ready. */
                    return 1;
                }
            } else if (fn == 0x0Cu) { /* EnableEvent */
                psx->bios_cd_ev_enabled |= bit;
                psx->regs[2] = 1;
            } else if (fn == 0x0Du) { /* DisableEvent */
                psx->bios_cd_ev_enabled &= ~bit;
                psx->bios_cd_ev_ready &= ~bit;
                psx->regs[2] = 1;
            } else { /* TestEvent */
                psx->regs[2] = (psx->bios_cd_ev_ready & bit) ? 1u : 0u;
                psx->bios_cd_ev_ready &= ~bit;
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[BIOS] EvTest=", a0);
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[BIOS] EvRet=", psx->regs[2]);
            }
            psx->pc = psx->regs[31];
            return 1;
        }

        slot = psx_bios_event_slot_from_handle(a0);
        if (slot >= 0 && (psx->bios_ev_open & (1u << (u32)slot))) {
            u32 bit = 1u << (u32)slot;
            if (fn == 0x09u) { /* CloseEvent */
                psx->bios_ev_open &= ~bit;
                psx->bios_ev_enabled &= ~bit;
                psx->bios_ev_ready &= ~bit;
                psx->regs[2] = 1;
            } else if (fn == 0x0Au) { /* WaitEvent */
                if (!(psx->bios_ev_enabled & bit)) {
                    psx->regs[2] = 0;
                } else if (psx->bios_ev_ready & bit) {
                    psx->bios_ev_ready &= ~bit;
                    psx->regs[2] = 1;
                    if (psx->bios_call_logs > 64u) psx->bios_call_logs = 64u;
                } else {
                    /* Real BIOS blocks here until the event becomes ready. */
                    return 1;
                }
            } else if (fn == 0x0Cu) { /* EnableEvent */
                psx->bios_ev_enabled |= bit;
                psx->regs[2] = 1;
            } else if (fn == 0x0Du) { /* DisableEvent */
                psx->bios_ev_enabled &= ~bit;
                psx->bios_ev_ready &= ~bit;
                psx->regs[2] = 1;
            } else { /* TestEvent */
                psx->regs[2] = (psx->bios_ev_ready & bit) ? 1u : 0u;
                psx->bios_ev_ready &= ~bit;
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[BIOS] EvTest=", a0);
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[BIOS] EvRet=", psx->regs[2]);
            }
            psx->pc = psx->regs[31];
            return 1;
        }
    }

    if (fn == 0x07u || fn == 0x20u) { /* DeliverEvent / UnDeliverEvent */
        if (a0 == 0xF0000003u) {
            slot = psx_bios_cd_event_slot_from_spec(a1);
            if (slot >= 0 && (psx->bios_cd_ev_open & (1u << (u32)slot))) {
                u32 bit = 1u << (u32)slot;
                if (fn == 0x07u) {
                    if (psx->bios_cd_ev_enabled & bit)
                        psx->bios_cd_ev_ready |= bit;
                } else {
                    psx->bios_cd_ev_ready &= ~bit;
                }
                psx->regs[2] = 1;
                psx->pc = psx->regs[31];
                return 1;
            }
        }
        psx->regs[2] = 1;
        for (slot = 0; slot < 16; slot++) {
            u32 bit = 1u << (u32)slot;
            if (!(psx->bios_ev_open & bit)) continue;
            if (psx->bios_ev_class[slot] != a0 || psx->bios_ev_spec[slot] != a1)
                continue;
            if (fn == 0x07u) {
                if (psx->bios_ev_enabled & bit)
                    psx->bios_ev_ready |= bit;
            } else {
                psx->bios_ev_ready &= ~bit;
            }
        }
        psx->pc = psx->regs[31];
        return 1;
    }

    return 0;
}

static void psx_gte_exec(struct psx_system *psx, u32 cmd)
{
    if (!psx->gte_cmd_logged) {
        psx->gte_cmd_logged = 1;
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[GTE] cmd=", cmd);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[GTE] pc=", psx->pc);
    }
    psx_gte_dispatch(psx, cmd);
}

struct psx_exec_result {
    u32 write_mask;
    u32 load_reg;
    u32 load_value;
    u32 load_scheduled;
};

static void psx_mark_write(struct psx_exec_result *res, u32 reg)
{
    if (reg < 32u) res->write_mask |= (1u << reg);
}

static void psx_schedule_load(struct psx_exec_result *res, u32 reg, u32 val)
{
    if (reg == 0) return;
    res->load_reg = reg;
    res->load_value = val;
    res->load_scheduled = 1u;
}

static void psx_commit_load_delay(struct psx_system *psx, struct psx_exec_result *res)
{
    u32 preg = psx->load_pending_reg;
    if (preg && !(res->write_mask & (1u << preg))) {
        psx->regs[preg] = psx->load_pending_value;
    }

    if (res->load_scheduled) {
        psx->load_pending_reg = res->load_reg;
        psx->load_pending_value = res->load_value;
    } else {
        psx->load_pending_reg = 0;
        psx->load_pending_value = 0;
    }
}

static void psx_cpu_execute(struct psx_system *psx, u32 ins, struct psx_exec_result *res)
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
        case 0x00: psx->regs[rd] = psx->regs[rt] << sa; psx_mark_write(res, rd); break; /* SLL  */
        case 0x02: psx->regs[rd] = psx->regs[rt] >> sa; psx_mark_write(res, rd); break; /* SRL  */
        case 0x03: psx->regs[rd] = (u32)((s32)psx->regs[rt] >> sa); psx_mark_write(res, rd); break; /* SRA */
        case 0x04: psx->regs[rd] = psx->regs[rt] << (psx->regs[rs] & 0x1F); psx_mark_write(res, rd); break; /* SLLV */
        case 0x06: psx->regs[rd] = psx->regs[rt] >> (psx->regs[rs] & 0x1F); psx_mark_write(res, rd); break; /* SRLV */
        case 0x07: psx->regs[rd] = (u32)((s32)psx->regs[rt] >> (psx->regs[rs] & 0x1F)); psx_mark_write(res, rd); break; /* SRAV */
        case 0x08:
            psx->next_pc = psx->regs[rs];
            psx_cpu_log_bad_jump(psx, ins, psx->next_pc, psx->regs[rs], psx->regs[rt]);
            break; /* JR   */
        case 0x09: /* JALR */
            psx->regs[rd]  = psx->pc + 8;
            psx_mark_write(res, rd);
            psx->next_pc   = psx->regs[rs];
            psx_cpu_log_bad_jump(psx, ins, psx->next_pc, psx->regs[rs], psx->regs[rt]);
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
            psx_cpu_log_exception(psx);
            break;
        }
        case 0x10: psx->regs[rd] = psx->hi; psx_mark_write(res, rd); break; /* MFHI */
        case 0x11: psx->hi = psx->regs[rs]; break; /* MTHI */
        case 0x12: psx->regs[rd] = psx->lo; psx_mark_write(res, rd); break; /* MFLO */
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
        case 0x21: psx->regs[rd] = psx->regs[rs] + psx->regs[rt]; psx_mark_write(res, rd); break; /* ADDU */
        case 0x22: /* SUB  (no overflow trap) */
        case 0x23: psx->regs[rd] = psx->regs[rs] - psx->regs[rt]; psx_mark_write(res, rd); break; /* SUBU */
        case 0x24: psx->regs[rd] = psx->regs[rs] & psx->regs[rt]; psx_mark_write(res, rd); break; /* AND  */
        case 0x25: psx->regs[rd] = psx->regs[rs] | psx->regs[rt]; psx_mark_write(res, rd); break; /* OR   */
        case 0x26: psx->regs[rd] = psx->regs[rs] ^ psx->regs[rt]; psx_mark_write(res, rd); break; /* XOR  */
        case 0x27: psx->regs[rd] = ~(psx->regs[rs] | psx->regs[rt]); psx_mark_write(res, rd); break; /* NOR */
        case 0x2A: psx->regs[rd] = ((s32)psx->regs[rs] < (s32)psx->regs[rt]) ? 1 : 0; psx_mark_write(res, rd); break; /* SLT  */
        case 0x2B: psx->regs[rd] = (psx->regs[rs] < psx->regs[rt]) ? 1 : 0; psx_mark_write(res, rd); break; /* SLTU */
        }
        break;
    }



    /* ── Jumps ────────────────────────────────────────────────────── */
    case 0x02: /* J */
        psx->next_pc = (psx->pc & 0xF0000000u) | (target << 2);
        psx_cpu_log_bad_jump(psx, ins, psx->next_pc, psx->regs[rs], psx->regs[rt]);
        break;
    case 0x03: /* JAL */
        psx->regs[31] = psx->pc + 8;
        psx_mark_write(res, 31);
        psx->next_pc  = (psx->pc & 0xF0000000u) | (target << 2);
        psx_cpu_log_bad_jump(psx, ins, psx->next_pc, psx->regs[rs], psx->regs[rt]);
        break;

    /* ── Branches ─────────────────────────────────────────────────── */
    case 0x01: { /* REGIMM instructions */
        u32 sub = rt;
        if (sub == 0x00) { if ((s32)psx->regs[rs] < 0) psx->next_pc = psx->pc + 4 + (imm_s << 2); } /* BLTZ */
        if (sub == 0x01) { if ((s32)psx->regs[rs] >= 0) psx->next_pc = psx->pc + 4 + (imm_s << 2); } /* BGEZ */
        if (sub == 0x10) { if ((s32)psx->regs[rs] < 0) { psx->regs[31] = psx->pc + 8; psx_mark_write(res, 31); psx->next_pc = psx->pc + 4 + (imm_s << 2); } } /* BLTZAL */
        if (sub == 0x11) { if ((s32)psx->regs[rs] >= 0) { psx->regs[31] = psx->pc + 8; psx_mark_write(res, 31); psx->next_pc = psx->pc + 4 + (imm_s << 2); } } /* BGEZAL */
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
    case 0x09: psx->regs[rt] = psx->regs[rs] + imm_s; psx_mark_write(res, rt); break; /* ADDIU */
    case 0x0A: psx->regs[rt] = ((s32)psx->regs[rs] < (s32)imm_s) ? 1 : 0; psx_mark_write(res, rt); break; /* SLTI  */
    case 0x0B: psx->regs[rt] = (psx->regs[rs] < imm_s) ? 1 : 0; psx_mark_write(res, rt); break; /* SLTIU */
    case 0x0C: psx->regs[rt] = psx->regs[rs] & imm;   psx_mark_write(res, rt); break; /* ANDI  */
    case 0x0D: psx->regs[rt] = psx->regs[rs] | imm;   psx_mark_write(res, rt); break; /* ORI   */
    case 0x0E: psx->regs[rt] = psx->regs[rs] ^ imm;   psx_mark_write(res, rt); break; /* XORI  */
    case 0x0F: psx->regs[rt] = imm << 16;             psx_mark_write(res, rt); break; /* LUI   */

    /* ── COP0 ─────────────────────────────────────────────────────── */
    case 0x10:
        if      (rs == 0x00) psx_schedule_load(res, rt, psx->cp0_regs[rd]); /* MFC0 */
        else if (rs == 0x04) psx->cp0_regs[rd] = psx->regs[rt]; /* MTC0 */
        else if (rs == 0x10 && (ins & 0x3F) == 0x10) { /* RFE */
            u32 sr = psx->cp0_regs[12];
            psx->cp0_regs[12] = (sr & ~0x3Fu) | ((sr >> 2) & 0x0Fu);
        }
        break;

    /* ── Memory loads ─────────────────────────────────────────────── */
    case 0x20: psx_schedule_load(res, rt, (u32)(s32)(s8)psx_bus_read(psx, psx->regs[rs] + imm_s, 1)); break; /* LB */
    case 0x21: psx_schedule_load(res, rt, (u32)(s32)(s16)psx_bus_read(psx, psx->regs[rs] + imm_s, 2)); break; /* LH */
    case 0x23: psx_schedule_load(res, rt, psx_bus_read(psx, psx->regs[rs] + imm_s, 4)); break; /* LW */
    case 0x24: psx_schedule_load(res, rt, psx_bus_read(psx, psx->regs[rs] + imm_s, 1)); break; /* LBU */
    case 0x25: psx_schedule_load(res, rt, psx_bus_read(psx, psx->regs[rs] + imm_s, 2)); break; /* LHU */
    
    case 0x22: { /* LWL – load word left (unaligned) */
        u32 addr = psx->regs[rs] + imm_s;
        u32 mem  = psx_bus_read(psx, addr & ~3u, 4);
        u32 mod  = addr & 3;
        u32 cur = psx->regs[rt];
        if      (psx->load_pending_reg == rt) cur = psx->load_pending_value;
        if      (mod == 0) cur = (cur & 0x00FFFFFFu) | (mem << 24);
        else if (mod == 1) cur = (cur & 0x0000FFFFu) | (mem << 16);
        else if (mod == 2) cur = (cur & 0x000000FFu) | (mem << 8);
        else if (mod == 3) cur = mem;
        psx_schedule_load(res, rt, cur);
        break;
    }
    case 0x26: { /* LWR – load word right (unaligned) */
        u32 addr = psx->regs[rs] + imm_s;
        u32 mem  = psx_bus_read(psx, addr & ~3u, 4);
        u32 mod  = addr & 3;
        u32 cur = psx->regs[rt];
        if      (psx->load_pending_reg == rt) cur = psx->load_pending_value;
        if      (mod == 0) cur = mem;
        else if (mod == 1) cur = (cur & 0xFF000000u) | (mem >> 8);
        else if (mod == 2) cur = (cur & 0xFFFF0000u) | (mem >> 16);
        else if (mod == 3) cur = (cur & 0xFFFFFF00u) | (mem >> 24);
        psx_schedule_load(res, rt, cur);
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

    /* ── COP2 (GTE) ─────────────────────────────────────────────── */
    case 0x12:
        if      (rs == 0x00) psx_schedule_load(res, rt, psx_gte_read_data(psx, rd)); /* MFC2 */
        else if (rs == 0x02) psx_schedule_load(res, rt, psx->gte_ctrl[rd & 31u]);    /* CFC2 */
        else if (rs == 0x04) psx_gte_write_data(psx, rd, psx->regs[rt]);              /* MTC2 */
        else if (rs == 0x06) psx->gte_ctrl[rd & 31u] = psx->regs[rt];                 /* CTC2 */
        else if (rs & 0x10u) psx_gte_exec(psx, ins & 0x1FFFFFFu);                     /* GTE cmd */
        break;
    case 0x32: /* LWC2 */
        psx_gte_write_data(psx, rt, psx_bus_read(psx, psx->regs[rs] + imm_s, 4));
        break;
    case 0x3A: /* SWC2 */
        psx_bus_write(psx, psx->regs[rs] + imm_s, psx_gte_read_data(psx, rt), 4);
        break;

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

    /* VSync J$ exit: PSX games use "j $" (self-jump) as a VBL wait primitive.
     * The BIOS VBL callback normally advances EPC past the loop. Simulate that
     * here when VBL is pending and the interrupted instruction is a self-jump. */
    if ((psx->i_stat & psx->i_mask & 1u) && psx->pc >= 0x80000000u
                                          && psx->pc < 0x80200000u) {
        u32 _ins = *(u32 *)(psx->ram + (psx->pc & 0x1FFFFFu));
        if ((_ins >> 26) == 2u) { /* J instruction */
            u32 _tgt = (psx->pc & 0xF0000000u) | ((_ins & 0x3FFFFFFu) << 2);
            if (_tgt == psx->pc)  /* self-jump: skip J$ + delay slot */
                psx->cp0_regs[14] = psx->pc + 8u;
        }
    }

    psx->cp0_regs[13] = (1u << 10);           /* Cause: IP[2], ExcCode=0 */
    psx->cp0_regs[12] = (sr & ~0x3Fu) | ((sr & 0x0Fu) << 2); /* KU/IE shift */

    u32 bev = (sr >> 22) & 1u;
    psx->exception_vec = bev ? 0xBFC00180u : 0x80000080u;
    psx_cpu_log_exception(psx);
}

/* ── Single CPU step with MIPS delay-slot semantics ─────────────── */
static void psx_cpu_run_one(struct psx_system *psx, u32 ins)
{
    struct psx_exec_result res;
    res.write_mask = 0;
    res.load_reg = 0;
    res.load_value = 0;
    res.load_scheduled = 0;

    psx->next_pc = psx->pc + 4;
    psx_cpu_execute(psx, ins, &res);
    psx_commit_load_delay(psx, &res);
    psx->regs[0] = 0;
}

void psx_cpu_step(struct psx_system *psx)
{
    /* Update Timers, CDROM and other timed HW */
    psx_bus_tick(psx, 1u); 

    psx->exception_vec = 0;

    /* Check for hardware interrupt before executing instruction */
    psx_check_irq(psx);
    if (psx->exception_vec) {
        psx->pc = psx->exception_vec;
        return;
    }

    u32 ins = psx_cpu_fetch(psx);
    psx_cpu_log_bios_call(psx);
    psx_cpu_log_hot_loop(psx);
    if (psx_cpu_handle_bios_event_call(psx)) {
        psx->regs[0] = 0;
        return;
    }
    psx_cpu_run_one(psx, ins);

    if (psx->exception_vec) {
        /* Exception (SYSCALL/BREAK): no delay slot, jump directly */
        psx->pc = psx->exception_vec;
    } else if (psx->next_pc != psx->pc + 4) {
        /* Branch or jump: execute delay slot before jumping */
        u32 target = psx->next_pc;
        psx->pc = psx->pc + 4;
        u32 ds  = psx_cpu_fetch(psx);
        psx_cpu_log_bios_call(psx);
        psx_cpu_log_hot_loop(psx);
        if (psx_cpu_handle_bios_event_call(psx)) {
            psx->regs[0] = 0;
            return;
        }
        psx_cpu_run_one(psx, ds);
        if (psx->exception_vec) {
            psx->pc = psx->exception_vec;
        } else {
            psx->pc = target;
        }
    } else {
        psx->pc = psx->next_pc;
    }

    psx->regs[0] = 0; /* R0 must always be zero */
}
