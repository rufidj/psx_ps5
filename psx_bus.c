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

/* Helper to convert BCD MM:SS:FF to LBA */
static u32 psx_bcd_to_lba(u32 *bcd) {
    u32 m = ((bcd[0] >> 4) * 10) + (bcd[0] & 0xF);
    u32 s = ((bcd[1] >> 4) * 10) + (bcd[1] & 0xF);
    u32 f = ((bcd[2] >> 4) * 10) + (bcd[2] & 0xF);
    u32 lba = (m * 60 + s) * 75 + f;
    return (lba >= 150) ? (lba - 150) : 0;
}


/* ── DMA channel 2: GPU linked-list (RAM → GP0) ──────────────────── */
static void dma2_exec(struct psx_system *psx, u32 chcr)
{
    u32 mode = (chcr >> 9) & 3u;
    u32 addr = psx->dma_madr[2] & 0x1FFFFFu;

    if (mode == 1) { /* Block Mode: used for VRAM uploads (font, icons) */
        u32 words = psx->dma_bcr[2] & 0xFFFFu;
        if (!words) words = 0x10000u;
        for (u32 i = 0; i < words; i++) {
            if (addr + 3u < RAM_SIZE) {
                gpu_gp0_write(&psx->gpu_io, *(u32 *)(psx->ram + addr));
            }
            addr = (addr + 4) & 0x1FFFFFu;
        }
        psx->dma_madr[2] = addr;
    } 
    else if (mode == 2) { /* Linked List Mode: used for drawing the UI */
        u32 limit = 0x10000u;
        while (limit--) {
            if (addr + 3u >= RAM_SIZE) break;
            u32 hdr   = *(u32 *)(psx->ram + addr);
            u32 count = (hdr >> 24) & 0xFFu;
            u32 next  = hdr & 0xFFFFFFu;

            for (u32 i = 0; i < count; i++) {
                addr = (addr + 4u) & 0x1FFFFCu;
                if (addr + 3u < RAM_SIZE)
                    gpu_gp0_write(&psx->gpu_io, *(u32 *)(psx->ram + addr));
            }
            if (next == 0xFFFFFFu || next == addr) break;
            addr = next & 0x1FFFFFu;
        }
        psx->dma_madr[2] = addr;
    }
    psx->i_stat |= (1u << 3); /* DMA interrupt */
}

/* ── DMA channel 3: CD-ROM ───────────────────────────────────────── */
static void dma3_exec(struct psx_system *psx) {
    u32 addr  = psx->dma_madr[3] & 0x1FFFFCu;
    u32 words = psx->dma_bcr[3] & 0xFFFFu;
    if (!words) words = 0x10000u;
    u32 offset = (psx->cd_mode & 0x20u) ? 0 : 24u; /* Skip header if Data mode */
    for (u32 i = 0; i < words; i++) {
        u32 val = 0;
        /* Read 4 bytes from CD-ROM Data FIFO */
        for (int j = 0; j < 4; j++) {
            u8 b = psx->cd_sector_buf[(psx->cd_data_pos + offset) % 2352u];
            psx->cd_data_pos++;
            val |= ((u32)b << (j * 8));
        }
        if (addr + 3u < RAM_SIZE) {
            *(u32 *)(psx->ram + addr) = val;
        }
        addr = (addr + 4) & 0x1FFFFCu;
    }
    psx->dma_madr[3] = addr;
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
        case 0x1F801800: {
            u32 val = 0x18u | (psx->cd_idx & 3u);
            if (psx->cd_resp_pos < psx->cd_resp_len) val |= 0x20u; /* Response FIFO not empty */
            if (psx->cd_read_active) val |= 0x40u; /* Data FIFO not empty (HACK: assume data ready if reading) */
            return val;
        }
        case 0x1F801801:
            if (psx->cd_idx == 1) { /* Data FIFO: return sector content */
                u8 byte = psx->cd_sector_buf[(psx->cd_data_pos + ((psx->cd_mode & 0x20u) ? 0 : 24u)) % 2352u];
                psx->cd_data_pos++;
                return byte;
            }
            /* Response FIFO: return from buffer */
            if (psx->cd_resp_pos < psx->cd_resp_len) {
                return psx->cd_resp_buf[psx->cd_resp_pos++];
            }
            return (psx->cd_read_active ? 0x22u : 0x02u);
        case 0x1F801802: return 0x20u; /* bit 5 = Data FIFO has data */
        case 0x1F801803:
            /* INT flag register (index 1) */
            if (psx->cd_idx == 1) {
                u8 val = 0xE0u;
                if (psx->cd_irq_pending) val |= (psx->cd_irq_pending & 7u);
                return val;
            }
            /* Index 0: Request register / Status bits */
            return (psx->cd_resp_pos < psx->cd_resp_len) ? 0x20u : 0;
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
            if (psx->cd_idx == 0) { /* Command register */
                psx->cd_cmd_cnt++;
                psx->cd_resp_pos = 0;
                psx->cd_resp_len = 0;
                u32 cmd = val & 0xFFu;
                u8 stat = (psx->cd_read_active ? 0x22u : 0x02u);
                if (cmd == 0x01 || cmd == 0x0A) { /* GetStat / Init */
                    psx->cd_irq_pending = 3u; /* INT3 */
                    psx->cd_resp_buf[0] = stat;
                    psx->cd_resp_len = 1;
                } else if (cmd == 0x1A) { /* GetID */
                    psx->cd_irq_pending = 3u; /* 1st Response: INT3 */
                    psx->cd_resp_buf[0] = stat;
                    psx->cd_resp_len = 1;
                    /* 2nd Response: INT2 (delivered in next vblank) */
                    psx->cd_resp_second[0] = stat;
                    psx->cd_resp_second[1] = 0x40u; /* Licensed */
                    psx->cd_resp_second[2] = 0x00u;
                    psx->cd_resp_second[3] = 0x00u;
                    psx->cd_resp_second[4] = 'S'; psx->cd_resp_second[5] = 'C';
                    psx->cd_resp_second[6] = 'E'; psx->cd_resp_second[7] = 'E';
                    psx->cd_resp_second_len = 8;
                    psx->cd_irq_second = 2u; /* INT2 */
                } else if (cmd == 0x02) { /* SetLoc */
                    psx->cd_sector = psx_bcd_to_lba(psx->cd_bcd);
                    psx->cd_irq_pending = 3u; /* INT3 */
                    psx->cd_resp_buf[0] = stat;
                    psx->cd_resp_len = 1;
                } else if (cmd == 0x06) { /* ReadN */
                    psx->cd_irq_pending = 3u; /* INT3: Ack */
                    psx->cd_resp_buf[0] = stat;
                    psx->cd_resp_len = 1;
                    psx->cd_read_active = 1;
                } else if (cmd == 0x08) { /* Stop */
                    psx->cd_irq_pending = 3u; /* INT3: Ack */
                    psx->cd_resp_buf[0] = stat;
                    psx->cd_resp_len = 1;
                    psx->cd_read_active = 0;
                } else if (cmd == 0x0E) { /* SetMode */
                    if (psx->cd_p_idx > 0) psx->cd_mode = psx->cd_bcd[0];
                    psx->cd_irq_pending = 3u; /* INT3: Ack */
                    psx->cd_resp_buf[0] = stat;
                    psx->cd_resp_len = 1;
                } else if (cmd == 0x19) { /* Test */
                    psx->cd_irq_pending = 3u; /* INT3 */
                    if (psx->cd_bcd[0] == 0x20u) { /* Get Version */
                        psx->cd_resp_buf[0] = 0x94u; /* Year */
                        psx->cd_resp_buf[1] = 0x09u; /* Month */
                        psx->cd_resp_buf[2] = 0x19u; /* Day */
                        psx->cd_resp_buf[3] = 0xC1u; /* Version C1 */
                        psx->cd_resp_len = 4;
                    } else {
                        psx->cd_resp_buf[0] = stat;
                        psx->cd_resp_len = 1;
                    }
                }
                psx->cd_p_idx = 0; /* Reset parameters AFTER use */
                if (psx->cd_irq_pending) psx->i_stat |= (1u << 2);
            }
            break;
        case 0x1F801802:
            if (psx->cd_idx == 0) { /* Parameter FIFO */
                if (psx->cd_p_idx < 8) {
                    psx->cd_bcd[psx->cd_p_idx++] = val & 0xFFu;
                }
            }
            break;
        case 0x1F801803:
            if (psx->cd_idx == 0) { /* Request register */
                /* bit 7 = start data transfer */
            } else if (psx->cd_idx == 1) { /* Interrupt acknowledge */
                psx->cd_irq_pending = 0;
            }
            break;
        /* DMA channel 2 (GPU) */
        case 0x1F8010A0: psx->dma_madr[2] = val; break;
        case 0x1F8010A4: psx->dma_bcr[2]  = val; break;
        case 0x1F8010A8:
            if (val & (1u << 24)) {
                psx->gpu_io.dma2_calls++;
                dma2_exec(psx, val);
            }
            break;
        /* DMA channel 3 (CD-ROM) */
        case 0x1F8010B0: psx->dma_madr[3] = val; break;
        case 0x1F8010B4: psx->dma_bcr[3]  = val; break;
        case 0x1F8010B8:
            if (val & (1u << 24)) dma3_exec(psx);
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

    /* Deliver delayed second response if previous IRQ was acknowledged */
    if (psx->cd_irq_pending == 0 && psx->cd_irq_second > 0) {
        for (u32 i = 0; i < psx->cd_resp_second_len; i++)
            psx->cd_resp_buf[i] = psx->cd_resp_second[i];
        psx->cd_resp_len = psx->cd_resp_second_len;
        psx->cd_resp_pos = 0;
        psx->cd_irq_pending = psx->cd_irq_second;
        psx->cd_irq_second = 0;
        psx->i_stat |= (1u << 2);
    }

    if (psx->cd_read_active && psx->cd_irq_pending == 0) {
        if (psx->cd_fd > 0 && psx->kread_fn && psx->klseek_fn) {
            /* Detect sector size (2352 raw or 2048 data) */
            u32 sec_size = 2352;
            /* Simple LBA check for file size would be better but requires fstat.
             * Assume 2352 for now as it is the most common for .bin files. */
            u64 off = (u64)psx->cd_sector * sec_size;
            NC(psx->G, psx->klseek_fn, (u64)psx->cd_fd, off, 0, 0, 0, 0);
            NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)psx->cd_sector_buf, 2352, 0, 0, 0);
            psx->cd_data_pos = 0;
            psx->cd_sector++;
            psx->cd_irq_pending = 1u; /* INT1: Data Ready */
            psx->i_stat |= (1u << 2);
        }
    }
    if (psx->cd_irq_pending > 0)
        psx->i_stat |= (1u << 2); /* CD interrupt */
    psx->gpu_io.gpustat ^= 0x80000000u;
}


u32  psx_bus_read32 (struct psx_system *p, u32 a)      { return psx_bus_read(p, a, 4); }
void psx_bus_write32(struct psx_system *p, u32 a, u32 v){ psx_bus_write(p, a, v, 4); }

#endif /* PSX_BUS_C */
