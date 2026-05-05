#ifndef PSX_BUS_C
#define PSX_BUS_C

#include "psx_types.h"

#define RAM_SIZE     (2u * 1024u * 1024u)
#define BIOS_SIZE    (512u * 1024u)
#define SCRATCH_SIZE (1024u)
#define ADDR_SCRATCH  0x1F800000u
#define ADDR_IO       0x1F801000u
#define ADDR_BIOS     0x1FC00000u

/* Forward declarations: gpu_gp0_write / gpu_gp1_write defined in psx_gpu.c */
static void gpu_gp0_write(struct psx_gpu_io *g, u32 val);
static void gpu_gp1_write(struct psx_gpu_io *g, u32 val);

/* kseg0 / kseg1 → physical address */
static u32 psx_bus_mask(u32 addr)
{
    u32 seg = addr >> 29;
    if (seg == 4) return addr & 0x7FFFFFFFu;
    if (seg == 5) return addr & 0x1FFFFFFFu;
    return addr;
}

/* ── DMA channel 2: GPU linked-list (RAM → GP0) ──────────────────── */
static void dma2_exec(struct psx_system *psx)
{
    u32 addr  = psx->dma_madr[2] & 0x1FFFFFu;
    u32 limit = 0x100000u; /* safety: max 1M packets */

    while (limit--) {
        if (addr + 3u >= RAM_SIZE) break;
        u32 hdr   = *(u32 *)(psx->ram + addr);
        u32 count = (hdr >> 24) & 0xFFu;
        u32 next  = hdr & 0xFFFFFFu;

        for (u32 i = 0; i < count; i++) {
            u32 wa = (addr + 4u + i * 4u) & 0x1FFFFFu;
            if (wa + 3u < RAM_SIZE)
                gpu_gp0_write(&psx->gpu_io, *(u32 *)(psx->ram + wa));
        }

        if (next == 0xFFFFFFu || next == addr) break;
        addr = next;
    }
    psx->i_stat |= (1u << 3); /* DMA interrupt */
}

/* ── DMA channel 6: OTC (Ordering Table Clear) ───────────────────── */
static void dma6_exec(struct psx_system *psx)
{
    u32 addr  = psx->dma_madr[6] & 0x1FFFFFu;
    u32 count = psx->dma_bcr[6] & 0xFFFFu;
    if (!count) count = 0x10000u;

    for (u32 i = 0; i < count; i++) {
        if (addr + 3u >= RAM_SIZE) break;
        /* Last entry terminates the list; others point to addr-4 */
        u32 val = (i == count - 1u) ? 0xFFFFFFu : ((addr - 4u) & 0x1FFFFFu);
        *(u32 *)(psx->ram + addr) = val;
        addr = (addr - 4u) & 0x1FFFFFu;
    }
    psx->i_stat |= (1u << 3);
}

/* ─────────────────────────────────────────────────────────────────── */

u32 psx_bus_read(struct psx_system *psx, u32 addr, int width)
{
    u32 phys = psx_bus_mask(addr);

    /* RAM */
    if (phys < RAM_SIZE) {
        u8 *p = psx->ram + phys;
        if (width == 4) return *(u32 *)p;
        if (width == 2) return *(u16 *)p;
        return *p;
    }
    /* BIOS */
    if (phys >= ADDR_BIOS && phys < ADDR_BIOS + BIOS_SIZE) {
        u8 *p = psx->bios + (phys - ADDR_BIOS);
        if (width == 4) return *(u32 *)p;
        if (width == 2) return *(u16 *)p;
        return *p;
    }
    /* Scratch */
    if (phys >= ADDR_SCRATCH && phys < ADDR_SCRATCH + SCRATCH_SIZE) {
        u8 *p = psx->scratch + (phys - ADDR_SCRATCH);
        if (width == 4) return *(u32 *)p;
        if (width == 2) return *(u16 *)p;
        return *p;
    }
    /* I/O */
    if (phys >= ADDR_IO && phys < 0x1F802000u) {
        switch (phys) {
        /* Interrupt */
        case 0x1F801070: return psx->i_stat;
        case 0x1F801074: return psx->i_mask;
        /* GPU */
        case 0x1F801810: return 0;
        case 0x1F801814: return psx->gpu_io.gpustat;
        /* CD-ROM: "no disc" state.
         * Sequence: 0-2 cmds = INT3 shell-open, 3+ cmds = INT5 error
         * INT5 response: [0x11=stat(err+shellOpen), 0x80=errcode(seekErr)]
         * PSX BIOS interprets INT5 as fatal disc error → shows Browser menu */
        case 0x1F801800:
            return 0x18u | (psx->cd_idx & 3u);
        case 0x1F801801:
            if (psx->cd_idx == 1) return 0; /* Data FIFO */
            /* Response FIFO: byte depends on INT type and read position */
            if (psx->cd_irq_pending == 3u) {
                /* INT5 multi-byte: [stat=0x11, errcode=0x80] */
                return (psx->cd_resp_pos++ == 0) ? 0x11u : 0x80u;
            }
            /* INT3 GetStat: shell open first 3 commands, then motor-off */
            return (psx->cd_cmd_cnt < 3u) ? 0x10u : 0x00u;
        case 0x1F801802: return 0;
        case 0x1F801803:
            /* INT flag register (index 1) */
            if (psx->cd_idx == 1) {
                if (psx->cd_irq_pending == 1u) return 0xE0u | 3u; /* INT3 */
                if (psx->cd_irq_pending == 3u) return 0xE0u | 5u; /* INT5 */
                return 0xE0u;
            }
            return 0;
        /* JOY/SIO: bit1 of JOY_STAT must be set (RX FIFO not empty)
         * or the BIOS shell loops forever waiting for controller data */
        case 0x1F801040: return 0xFFFFFFFFu; /* JOY_DATA: 0xFF = all buttons released */
        case 0x1F801044: return 0x00000007u; /* JOY_STAT: bit0=TX ready, bit1=RX ready, bit2=TX empty */
        case 0x1F801048: return 0;
        /* Timers */
        case 0x1F801100: case 0x1F801104: case 0x1F801108:
        case 0x1F801110: case 0x1F801114: case 0x1F801118:
        case 0x1F801120: case 0x1F801124: case 0x1F801128:
            return psx->timer0++;
        /* DMA channel 2 (GPU) */
        case 0x1F8010A0: return psx->dma_madr[2];
        case 0x1F8010A4: return psx->dma_bcr[2];
        case 0x1F8010A8: return 0;
        /* DMA channel 6 (OTC) */
        case 0x1F8010E0: return psx->dma_madr[6];
        case 0x1F8010E4: return psx->dma_bcr[6];
        case 0x1F8010E8: return 0;
        /* DMA control */
        case 0x1F8010F0: return 0x07654321u; /* DPCR – all channels enabled */
        case 0x1F8010F4: return 0;            /* DICR */
        default: return 0;
        }
    }
    return 0;
}

void psx_bus_write(struct psx_system *psx, u32 addr, u32 val, int width)
{
    /* PSX COP0 SR bit 16 = IsC (Isolate Cache).
     * When IsC=1 all stores go to I-cache, NOT RAM.
     * We don't model the cache, so just suppress RAM/scratch writes.
     * I/O writes (hardware registers) are still forwarded. */
    u32 phys = psx_bus_mask(addr);
    if ((psx->cp0_regs[12] & (1u << 16)) &&
        (phys < RAM_SIZE ||
         (phys >= 0x1F800000u && phys < 0x1F800400u))) {
        return; /* cached write suppressed */
    }

    /* RAM */
    if (phys < RAM_SIZE) {
        u8 *p = psx->ram + phys;
        if (width == 4) *(u32 *)p = val;
        else if (width == 2) *(u16 *)p = (u16)val;
        else *p = (u8)val;
        return;
    }
    /* Scratch */
    if (phys >= ADDR_SCRATCH && phys < ADDR_SCRATCH + SCRATCH_SIZE) {
        u8 *p = psx->scratch + (phys - ADDR_SCRATCH);
        if (width == 4) *(u32 *)p = val;
        else if (width == 2) *(u16 *)p = (u16)val;
        else *p = (u8)val;
        return;
    }
    /* I/O */
    if (phys >= ADDR_IO && phys < 0x1F802000u) {
        switch (phys) {
        case 0x1F801070: psx->i_stat &= val; break;
        case 0x1F801074: psx->i_mask  = val; break;
        case 0x1F801810: gpu_gp0_write(&psx->gpu_io, val); break;
        case 0x1F801814: gpu_gp1_write(&psx->gpu_io, val); break;
        /* CD-ROM */
        case 0x1F801800: psx->cd_idx = val & 3u; break;
        case 0x1F801801:
            if (psx->cd_idx == 0) {
                psx->cd_cmd_cnt++;
                psx->cd_resp_pos = 0;
                /* First 3 cmds: INT3 (ack with GetStat); after that: INT5 (error) */
                psx->cd_irq_pending = (psx->cd_cmd_cnt < 3u) ? 1u : 3u;
            }
            break;
        case 0x1F801803:
            /* Acknowledge any INT (INT3 or INT5) → clear */
            if (psx->cd_idx == 1) psx->cd_irq_pending = 0;
            break;
        /* DMA channel 2 (GPU) */
        case 0x1F8010A0: psx->dma_madr[2] = val; break;
        case 0x1F8010A4: psx->dma_bcr[2]  = val; break;
        case 0x1F8010A8:
            if (val & (1u << 24)) {
                psx->gpu_io.dma2_calls++;
                dma2_exec(psx);
            }
            break;
        /* DMA channel 6 (OTC) */
        case 0x1F8010E0: psx->dma_madr[6] = val; break;
        case 0x1F8010E4: psx->dma_bcr[6]  = val; break;
        case 0x1F8010E8:
            if (val & (1u << 24)) dma6_exec(psx);
            break;
        default: break;
        }
    }
}

/* Fire VBLANK interrupt — hardware IRQ check in CPU will take it */
static void psx_bus_vblank(struct psx_system *psx)
{
    psx->i_stat |= 1u;  /* VBLANK (bit 0) */
    if (psx->cd_irq_pending > 0)
        psx->i_stat |= (1u << 2); /* CD interrupt (INT3 or INT2) */
    /* Bit 31 of GPUSTAT = odd/even interlaced field; alternates every VBLANK.
     * The BIOS polls this to synchronize with the 640×480 display. */
    psx->gpu_io.gpustat ^= 0x80000000u;
}

u32  psx_bus_read32 (struct psx_system *p, u32 a)      { return psx_bus_read(p, a, 4); }
void psx_bus_write32(struct psx_system *p, u32 a, u32 v){ psx_bus_write(p, a, v, 4); }

#endif /* PSX_BUS_C */
