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
static void psx_cd_fifo_reset(struct psx_system *psx);
static void psx_cd_fifo_sync_state(struct psx_system *psx);
static void psx_cd_fifo_push(struct psx_system *psx, const u8 *src, u32 size);
static u8 psx_cd_fifo_read_byte(struct psx_system *psx, int *ok);

static void psx_sio_reset(struct psx_system *psx)
{
    psx->joy_mode = 0;
    psx->joy_ctrl = 0;
    psx->joy_baud = 0xDCu;
    psx->joy_rx_data = 0xFFu;
    psx->joy_rx_ready = 0;
    psx->joy_irq = 0;
    psx->joy_selected = 0;
    psx->joy_cmd = 0;
    psx->joy_phase = 0;
    psx->joy_tx_prev = 0;
    psx->joy_mc_sector = 0;
    psx->joy_mc_index = 0;
    psx->joy_mc_checksum = 0;
    psx->joy_mc_flag = 0x08u;
    psx->joy_mc_write = 0;
}

static void psx_memcard_format_blank(struct psx_system *psx)
{
    u32 i, j;
    for (i = 0; i < sizeof(psx->joy_memcard); i++)
        psx->joy_memcard[i] = 0x00u;

    psx->joy_memcard[0] = 'M';
    psx->joy_memcard[1] = 'C';
    psx->joy_memcard[0x7F] = 'M' ^ 'C';

    for (i = 1; i < 16; i++) {
        u8 *sec = &psx->joy_memcard[i * 128u];
        sec[0] = 0xA0u;
        sec[1] = 0x00u;
        sec[2] = 0x00u;
        sec[3] = 0x00u;
        sec[8] = 0xFFu;
        sec[9] = 0xFFu;
        sec[0x7F] = 0;
        for (j = 0; j < 0x7Fu; j++)
            sec[0x7F] ^= sec[j];
    }
}

static void psx_sio_raise_irq(struct psx_system *psx)
{
    psx->joy_irq = 1;
    psx->i_stat |= (1u << 7);
}

static void psx_sio_finish_transfer(struct psx_system *psx, u8 rx, int ack)
{
    psx->joy_rx_data = rx;
    psx->joy_rx_ready = 1;
    if (ack)
        psx_sio_raise_irq(psx);
}

static void psx_sio_deselect(struct psx_system *psx)
{
    psx->joy_selected = 0;
    psx->joy_cmd = 0;
    psx->joy_phase = 0;
    psx->joy_mc_sector = 0;
    psx->joy_mc_index = 0;
    psx->joy_mc_checksum = 0;
    psx->joy_mc_write = 0;
}

static void psx_sio_handle_pad(struct psx_system *psx, u8 tx)
{
    u8 rx = 0xFFu;
    int ack = 1;

    if (psx->joy_phase == 0) {
        psx->joy_cmd = tx;
        if (tx == 0x42u || tx == 0x43u)
            rx = 0x41u;
        else if (tx >= 0x44u && tx <= 0x4Fu)
            rx = 0x41u;
        else
            rx = 0xFFu;
        psx->joy_phase = 1;
    } else if (psx->joy_phase == 1) {
        rx = 0x5Au;
        psx->joy_phase = 2;
    } else if (psx->joy_phase == 2) {
        rx = 0xFFu;
        psx->joy_phase = 3;
    } else if (psx->joy_phase == 3) {
        rx = 0xFFu;
        ack = 0;
        psx_sio_deselect(psx);
    } else {
        ack = 0;
        psx_sio_deselect(psx);
    }

    psx_sio_finish_transfer(psx, rx, ack);
}

static void psx_sio_handle_memcard(struct psx_system *psx, u8 tx)
{
    u8 rx = 0xFFu;
    int ack = 1;

    if (psx->joy_memcard[0] != 'M' || psx->joy_memcard[1] != 'C')
        psx_memcard_format_blank(psx);

    if (psx->joy_phase == 0) {
        psx->joy_cmd = tx;
        psx->joy_phase = 1;
        psx->joy_mc_index = 0;
        psx->joy_mc_checksum = 0;
        psx->joy_mc_write = 0;
        rx = (u8)psx->joy_mc_flag;
        if (tx != 0x52u && tx != 0x53u && tx != 0x57u) {
            ack = 0;
            psx_sio_deselect(psx);
        }
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }

    if (psx->joy_cmd == 0x53u) { /* Get ID */
        switch (psx->joy_phase) {
        case 1: rx = 0x5Au; psx->joy_phase = 2; break;
        case 2: rx = 0x5Du; psx->joy_phase = 3; break;
        case 3: rx = 0x5Cu; psx->joy_phase = 4; break;
        case 4: rx = 0x5Du; psx->joy_phase = 5; break;
        case 5: rx = 0x04u; psx->joy_phase = 6; break;
        case 6: rx = 0x00u; psx->joy_phase = 7; break;
        case 7: rx = 0x00u; psx->joy_phase = 8; break;
        case 8: rx = 0x80u; ack = 0; psx_sio_deselect(psx); break;
        default: ack = 0; psx_sio_deselect(psx); break;
        }
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }

    if (psx->joy_phase == 1) {
        rx = 0x5Au;
        psx->joy_phase = 2;
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }
    if (psx->joy_phase == 2) {
        rx = 0x5Du;
        psx->joy_phase = 3;
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }
    if (psx->joy_phase == 3) {
        psx->joy_mc_sector = ((u32)tx & 0xFFu) << 8;
        psx->joy_mc_checksum = tx;
        rx = 0x00u;
        psx->joy_phase = 4;
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }
    if (psx->joy_phase == 4) {
        psx->joy_mc_sector |= (u32)tx & 0xFFu;
        psx->joy_mc_checksum ^= tx;
        rx = (u8)((psx->joy_mc_sector >> 8) & 0xFFu);
        psx->joy_phase = 5;
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }

    if (psx->joy_cmd == 0x52u) { /* Read sector */
        if (psx->joy_phase == 5) {
            rx = 0x5Cu;
            psx->joy_phase = 6;
        } else if (psx->joy_phase == 6) {
            rx = 0x5Du;
            psx->joy_phase = 7;
        } else if (psx->joy_phase == 7) {
            if (psx->joy_mc_sector > 0x3FFu) {
                rx = 0xFFu;
                psx->joy_phase = 8;
            } else {
                rx = (u8)((psx->joy_mc_sector >> 8) & 0xFFu);
                psx->joy_phase = 8;
            }
        } else if (psx->joy_phase == 8) {
            if (psx->joy_mc_sector > 0x3FFu) {
                rx = 0xFFu;
                ack = 0;
                psx_sio_deselect(psx);
            } else {
                rx = (u8)(psx->joy_mc_sector & 0xFFu);
                psx->joy_phase = 9;
                psx->joy_mc_index = 0;
            }
        } else if (psx->joy_phase >= 9 && psx->joy_phase < (9 + 128)) {
            u32 off = (psx->joy_mc_sector & 0x3FFu) * 128u + psx->joy_mc_index;
            rx = psx->joy_memcard[off];
            psx->joy_mc_checksum ^= rx;
            psx->joy_mc_index++;
            psx->joy_phase++;
        } else if (psx->joy_phase == (9 + 128)) {
            rx = (u8)(psx->joy_mc_checksum & 0xFFu);
            psx->joy_phase++;
        } else if (psx->joy_phase == (10 + 128)) {
            rx = 0x47u;
            ack = 0;
            psx_sio_deselect(psx);
        } else {
            ack = 0;
            psx_sio_deselect(psx);
        }
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }

    if (psx->joy_cmd == 0x57u) { /* Write sector */
        if (psx->joy_phase >= 5 && psx->joy_phase < (5 + 128)) {
            psx->joy_mc_buf[psx->joy_mc_index++] = tx;
            psx->joy_mc_checksum ^= tx;
            rx = psx->joy_tx_prev & 0xFFu;
            psx->joy_phase++;
        } else if (psx->joy_phase == (5 + 128)) {
            u8 expect = (u8)(psx->joy_mc_checksum & 0xFFu);
            rx = psx->joy_tx_prev & 0xFFu;
            psx->joy_phase++;
            psx->joy_mc_write = (tx == expect) ? 1u : 0u;
        } else if (psx->joy_phase == (6 + 128)) {
            rx = 0x5Cu;
            psx->joy_phase++;
        } else if (psx->joy_phase == (7 + 128)) {
            rx = 0x5Du;
            psx->joy_phase++;
        } else if (psx->joy_phase == (8 + 128)) {
            if (psx->joy_mc_sector > 0x3FFu)
                rx = 0xFFu;
            else if (!psx->joy_mc_write)
                rx = 0x4Eu;
            else {
                u32 off = (psx->joy_mc_sector & 0x3FFu) * 128u;
                u32 i;
                for (i = 0; i < 128u; i++)
                    psx->joy_memcard[off + i] = psx->joy_mc_buf[i];
                psx->joy_mc_flag &= ~0x08u;
                rx = 0x47u;
            }
            ack = 0;
            psx_sio_deselect(psx);
        } else {
            ack = 0;
            psx_sio_deselect(psx);
        }
        psx_sio_finish_transfer(psx, rx, ack);
        return;
    }

    ack = 0;
    psx_sio_deselect(psx);
    psx_sio_finish_transfer(psx, 0xFFu, ack);
}

static void psx_sio_write_data(struct psx_system *psx, u8 tx)
{
    int cs = (psx->joy_ctrl & 0x0002u) ? 1 : 0;
    int port = (psx->joy_ctrl & 0x2000u) ? 1 : 0;
    psx->joy_tx_prev = tx;
    psx->joy_rx_ready = 0;

    if (!cs) {
        psx_sio_finish_transfer(psx, 0xFFu, 0);
        return;
    }

    if (port != 0) {
        psx_sio_finish_transfer(psx, 0xFFu, 0);
        return;
    }

    if (psx->joy_selected == 0) {
        if (tx == 0x01u) {
            psx->joy_selected = 1;
            psx->joy_phase = 0;
            psx_sio_finish_transfer(psx, 0xFFu, 1);
            return;
        }
        if (tx == 0x81u) {
            psx->joy_selected = 2;
            psx->joy_phase = 0;
            psx_sio_finish_transfer(psx, 0xFFu, 1);
            return;
        }
        psx_sio_finish_transfer(psx, 0xFFu, 0);
        return;
    }

    if (psx->joy_selected == 1)
        psx_sio_handle_pad(psx, tx);
    else if (psx->joy_selected == 2)
        psx_sio_handle_memcard(psx, tx);
    else
        psx_sio_finish_transfer(psx, 0xFFu, 0);
}

static u32 psx_sio_stat(const struct psx_system *psx)
{
    u32 stat = 0;
    stat |= 1u << 0; /* TX ready */
    stat |= 1u << 2; /* TX done / empty */
    stat |= 1u << 8; /* CTS high */
    if (psx->joy_rx_ready) stat |= 1u << 1;
    if (psx->joy_irq) {
        stat |= 1u << 7; /* DSR/ACK level */
        stat |= 1u << 9; /* interrupt pending */
    }
    return stat;
}

/* ── SPU Helpers ─────────────────────────────────────────────────── */
static const s32 f1[] = { 0, 60, 115, 98, 122 };
static const s32 f2[] = { 0, 0, -52, -55, -60 };

static void psx_spu_decode_block(struct psx_system *psx, int v) {
    u32 addr = psx->spu_voice_addr[v] & 0x7FFFFu;
    u8 head = psx->spu_ram[addr];
    int shift = 12 - (head & 0xFu);
    int filter = (head >> 4) & 0x7u;
    if (filter > 4) filter = 4; /* Clamp to 0..4 */

    s32 b1 = f1[filter];
    s32 b2 = f2[filter];
    s32 s1 = psx->spu_voice_last[v][0];
    s32 s2 = psx->spu_voice_last[v][1];

    for (int i = 0; i < 28; i++) {
        u8 byte = psx->spu_ram[addr + 2 + (i >> 1)];
        int nib = (i & 1) ? (byte >> 4) : (byte & 0xF);
        s32 s = (s32)(s16)((u16)nib << 12) >> shift;
        
        s32 res = s + ((s1 * b1 + s2 * b2) >> 6);
        if (res > 32767) res = 32767; else if (res < -32768) res = -32768;
        
        psx->spu_voice_samples[v][i] = (s16)res;
        s2 = s1;
        s1 = res;
    }
    psx->spu_voice_last[v][0] = (s16)s1;
    psx->spu_voice_last[v][1] = (s16)s2;
}

/* kseg0 / kseg1 → physical address */
static u32 psx_bus_mask(u32 addr)
{
    u32 seg = addr >> 29;
    if (seg == 4) return addr & 0x7FFFFFFFu;
    if (seg == 5) return addr & 0x1FFFFFFFu;
    return addr;
}

/* Helper to convert BCD MM:SS:FF to LBA */
static u32 psx_bcd_to_lba(const u8 *bcd) {
    u32 m = ((bcd[0] >> 4) * 10) + (bcd[0] & 0xF);
    u32 s = ((bcd[1] >> 4) * 10) + (bcd[1] & 0xF);
    u32 f = ((bcd[2] >> 4) * 10) + (bcd[2] & 0xF);
    u32 lba = (m * 60 + s) * 75 + f;
    return (lba >= 150) ? (lba - 150) : 0;
}

static u8 psx_to_bcd(u32 v)
{
    return (u8)(((v / 10u) << 4) | (v % 10u));
}

static void psx_lba_to_msf(u32 lba, u8 *m, u8 *s, u8 *f)
{
    u32 abs = lba + 150u;
    u32 mm = abs / (60u * 75u);
    u32 rem = abs % (60u * 75u);
    u32 ss = rem / 75u;
    u32 ff = rem % 75u;
    *m = psx_to_bcd(mm);
    *s = psx_to_bcd(ss);
    *f = psx_to_bcd(ff);
}

static void psx_cd_get_subheader(const struct psx_system *psx,
                                 u8 *file, u8 *channel, u8 *submode, u8 *coding)
{
    if (psx->cd_sector_size == 2352u && psx->cd_sector_buf[15] == 2u) {
        if (file) *file = psx->cd_sector_buf[16];
        if (channel) *channel = psx->cd_sector_buf[17] & 0x1Fu;
        if (submode) *submode = psx->cd_sector_buf[18];
        if (coding) *coding = psx->cd_sector_buf[19];
    } else {
        if (file) *file = 0;
        if (channel) *channel = 0;
        if (submode) *submode = 0x08u;
        if (coding) *coding = 0x00u;
    }
}

static int psx_cd_is_xa_audio_rt(const struct psx_system *psx)
{
    u8 submode = 0;
    psx_cd_get_subheader(psx, 0, 0, &submode, 0);
    return ((submode & 0x44u) == 0x44u);
}

static u8 psx_cd_stat(struct psx_system *psx)
{
    u8 val = 0x02u; /* Motor on (0x02). 0x10 is Shell Open! */
    if (psx->cd_read_active) val |= 0x20u;  /* Reading data (0x20) */
    if (psx->cd_seek_pending) val |= 0x40u; /* Seeking (0x40) */
    if (psx->cd_shell_open) val |= 0x10u;   /* Tray was opened / media changed */
    /* Note: Double speed is a mode setting, not typically reflected in the base status byte. */
    return val;
}

static void psx_cd_set_resp(struct psx_system *psx, u32 irq, const u8 *data, u32 len)
{
    psx->cd_irq_pending = irq & 7u;
    psx->cd_resp_pos = 0;
    psx->cd_resp_len = (len > 16u) ? 16u : len;
    for (u32 i = 0; i < psx->cd_resp_len; i++) psx->cd_resp_buf[i] = data[i];
    if (psx->cd_irq_pending && (psx->cd_hint_mask & (psx->cd_irq_pending & 7u)))
        psx->i_stat |= (1u << 2);
}

static void psx_cd_set_second_resp(struct psx_system *psx, u32 irq, const u8 *data, u32 len, u32 delay)
{
    psx->cd_irq_second = irq & 7u;
    psx->cd_irq_timer = delay;
    psx->cd_resp_second_len = (len > 16u) ? 16u : len;
    for (u32 i = 0; i < psx->cd_resp_second_len; i++) psx->cd_resp_second[i] = data[i];
}

static u32 psx_cd_hsts(const struct psx_system *psx)
{
    u32 val = psx->cd_idx & 3u;
    if (psx->cd_p_idx == 0) val |= 0x08u; /* PRMEMPT */
    if (psx->cd_p_idx < 16u) val |= 0x10u; /* PRMWRDY */
    if (psx->cd_resp_pos < psx->cd_resp_len) val |= 0x20u; /* RSLRRDY */
    if ((psx->cd_req & 0x80u) && psx->cd_data_ready) val |= 0x40u; /* DRQSTS */
    if (psx->cd_busy_ticks) val |= 0x80u; /* BUSYSTS */
    return val;
}

static void psx_cd_log_cmd(struct psx_system *psx, u32 cmd)
{
    if (!psx->sendto_fn || psx->log_fd < 0) return;
    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[CD] cmd=", cmd & 0xFFu);
    
    /* Log parameters if any */
    if (psx->cd_p_idx > 0) {
        for (u32 i = 0; i < psx->cd_p_idx; i++) {
            psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[CD]  p=", psx->cd_bcd[i]);
        }
    }

    if (!psx->cd_boot_trace &&
        (cmd == 0x06u || cmd == 0x1Bu || cmd == 0x1Au || cmd == 0x19u)) {
        psx->cd_boot_trace = 1u;
        psx_udp_log(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                    "[CD] BIOS/Game solicito acceso real al disco\n");
    }
}

static const u8 *psx_cd_region_id(const struct psx_system *psx)
{
    static const u8 jp[4] = { 'S', 'C', 'E', 'I' };
    static const u8 us[4] = { 'S', 'C', 'E', 'A' };
    static const u8 eu[4] = { 'S', 'C', 'E', 'E' };
    static const u8 none[4] = { 0, 0, 0, 0 };

    switch (psx->cd_disc_region) {
    case 0u: return jp;
    case 1u: return us;
    case 2u: return eu;
    default: return none;
    }
}

static int psx_cd_region_matches_console(const struct psx_system *psx)
{
    if (psx->cd_disc_region >= 3u || psx->cd_console_region >= 3u)
        return 1;
    return (psx->cd_disc_region == psx->cd_console_region);
}

static int psx_cd_test_region_string(const struct psx_system *psx, u8 *out)
{
    static const u8 jp[] = { 'f','o','r',' ','J','a','p','a','n' };
    static const u8 us[] = { 'f','o','r',' ','U','/','C' };
    static const u8 eu[] = { 'f','o','r',' ','E','u','r','o','p','e' };
    const u8 *src = jp;
    u32 n = 9u;

    if (psx->cd_console_region == 1u) {
        src = us;
        n = 7u;
    } else if (psx->cd_console_region == 2u) {
        src = eu;
        n = 10u;
    }
    for (u32 i = 0; i < n; i++)
        out[i] = src[i];
    return (int)n;
}

static void psx_cd_exec_cmd(struct psx_system *psx, u32 cmd)
{
    u8 stat = psx_cd_stat(psx);
    u8 resp[16];

    psx->cd_busy_ticks = 512u;
    psx_cd_log_cmd(psx, cmd);
    psx->cd_resp_pos = 0;
    psx->cd_resp_len = 0;

    if (cmd == 0x00) { /* Sync / invalid */
        resp[0] = 0x11u;
        resp[1] = 0x40u;
        psx_cd_set_resp(psx, 5u, resp, 2);
    } else if (cmd == 0x01) { /* Nop / GetStat */
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_resp(psx, 3u, resp, 1);
        psx->cd_shell_open = 0;
    } else if (cmd == 0x03) { /* Play */
        psx->cd_read_active = 0;
        psx->cd_seek_pending = 0;
        resp[0] = stat | 0x80u;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x04 || cmd == 0x05) { /* Forward / Backward */
        resp[0] = stat;
        if (!(stat & 0x80u))
            psx_cd_set_resp(psx, 5u, resp, 1);
        else
            psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x07) { /* Standby / MotorOn */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_second_resp(psx, 2u, resp, 1, 338688);
    } else if (cmd == 0x0D) { /* SetFilter */
        if (psx->cd_p_idx > 0) psx->cd_filter_file = psx->cd_bcd[0] & 0xFFu;
        if (psx->cd_p_idx > 1) psx->cd_filter_channel = psx->cd_bcd[1] & 0x1Fu;
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x1A) { /* GetID */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        {
            u8 id[8];
            const u8 *rid = psx_cd_region_id(psx);
            id[0] = stat;
            id[1] = psx_cd_region_matches_console(psx) ? 0x00u : 0x80u;
            id[2] = 0x20u;
            id[3] = 0x00u;
            id[4] = rid[0];
            id[5] = rid[1];
            id[6] = rid[2];
            id[7] = rid[3];
            psx_cd_set_second_resp(psx,
                                   psx_cd_region_matches_console(psx) ? 2u : 5u,
                                   id, 8, 338688);
        }
    } else if (cmd == 0x0A) { /* Init */
        psx->cd_mode = 0;
        psx->cd_req = 0;
        psx->cd_read_active = 0;
        psx->cd_pause_pending = 0;
        psx->cd_filter_file = 0;
        psx->cd_filter_channel = 0;
        psx_cd_fifo_reset(psx);
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_second_resp(psx, 2u, resp, 1, 677376);
    } else if (cmd == 0x02) { /* SetLoc */
        psx->cd_seek_lba = psx_bcd_to_lba(psx->cd_bcd);
        psx->cd_setloc_pending = 1;
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x06 || cmd == 0x1Bu || cmd == 0x1Cu) { /* ReadN / ReadS */
        psx->cd_read_cmd = cmd;
        psx_cd_fifo_reset(psx);
        if (psx->cd_setloc_pending) {
            psx->cd_sector = psx->cd_seek_lba;
            psx->cd_seek_pending = 1;
            psx->cd_read_after_seek = 1;
            psx->cd_read_active = 0;
            psx->cd_setloc_pending = 0;
        } else {
            psx->cd_read_active = 1;
            psx->cd_seek_pending = 0;
        }
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x08) { /* Stop */
        psx->cd_read_cmd = 0;
        psx->cd_read_active = 0;
        psx->cd_read_after_seek = 0;
        psx->cd_pause_pending = 0;
        psx->cd_setloc_pending = 0;
        psx_cd_fifo_reset(psx);
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_second_resp(psx, 2u, resp, 1, 0);
    } else if (cmd == 0x09) { /* Pause */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        if (psx->cd_read_active || psx->cd_read_after_seek) {
            /* Games commonly issue ReadN immediately followed by Pause to fetch
             * exactly one sector. Hardware keeps the in-flight read alive until
             * the next INT1 sector delivery, then transitions to paused state. */
            psx->cd_pause_pending = 1;
        } else {
            psx->cd_read_cmd = 0;
            psx->cd_read_active = 0;
            psx->cd_read_after_seek = 0;
            psx->cd_pause_pending = 0;
            psx_cd_fifo_reset(psx);
            resp[0] = psx_cd_stat(psx);
            psx_cd_set_second_resp(psx, 2u, resp, 1, 0);
        }
    } else if (cmd == 0x0B || cmd == 0x0C) { /* Mute / Demute */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x0E) { /* SetMode */
        if (psx->cd_p_idx > 0) psx->cd_mode = psx->cd_bcd[0];
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x0F) { /* GetParam */
        resp[0] = stat;
        resp[1] = psx->cd_mode;
        resp[2] = 0x00u;
        resp[3] = psx->cd_filter_file & 0xFFu;
        resp[4] = psx->cd_filter_channel & 0xFFu;
        psx_cd_set_resp(psx, 3u, resp, 5);
    } else if (cmd == 0x10) { /* GetLocL */
        u8 m, s, f, file, channel, submode, coding;
        psx_lba_to_msf(psx->cd_sector, &m, &s, &f);
        psx_cd_get_subheader(psx, &file, &channel, &submode, &coding);
        resp[0] = m; resp[1] = s; resp[2] = f;
        resp[3] = 0x02u;
        resp[4] = file;
        resp[5] = channel;
        resp[6] = submode;
        resp[7] = coding;
        psx_cd_set_resp(psx, 3u, resp, 8);
    } else if (cmd == 0x11) { /* GetLocP */
        u8 m, s, f;
        psx_lba_to_msf(psx->cd_sector, &m, &s, &f);
        resp[0] = 0x01; resp[1] = 0x01;
        resp[2] = m; resp[3] = s; resp[4] = f;
        resp[5] = m; resp[6] = s; resp[7] = f;
        psx_cd_set_resp(psx, 3u, resp, 8);
    } else if (cmd == 0x12) { /* SetSession */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_second_resp(psx, 2u, resp, 1, 338688);
    } else if (cmd == 0x13) { /* GetTN */
        resp[0] = stat;
        resp[1] = 0x01u;
        resp[2] = 0x01u;
        psx_cd_set_resp(psx, 3u, resp, 3);
    } else if (cmd == 0x14) { /* GetTD */
        u32 track = (psx->cd_p_idx > 0) ? (psx->cd_bcd[0] & 0xFFu) : 1u;
        u8 m, s, f;
        u32 lba = 0;
        if (track == 0u && psx->cd_total_sectors > 0) lba = psx->cd_total_sectors;
        psx_lba_to_msf(lba, &m, &s, &f);
        resp[0] = stat;
        resp[1] = m;
        resp[2] = s;
        psx_cd_set_resp(psx, 3u, resp, 3);
    } else if (cmd == 0x15 || cmd == 0x16) { /* SeekL / SeekP */
        psx->cd_read_cmd = 0;
        psx->cd_sector = psx->cd_seek_lba;
        psx->cd_seek_pending = 1;
        psx->cd_setloc_pending = 0;
        psx->cd_read_active = 0;
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_second_resp(psx, 2u, resp, 1, 0);
    } else if (cmd == 0x17) { /* SetClock */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else if (cmd == 0x18) { /* GetClock */
        resp[0] = stat;
        resp[1] = 0x00u;
        resp[2] = 0x00u;
        resp[3] = 0x00u;
        psx_cd_set_resp(psx, 3u, resp, 4);
    } else if (cmd == 0x19) { /* Test */
        if (psx->cd_bcd[0] == 0x20u) {
            resp[0] = 0x95u;
            resp[1] = 0x07u;
            resp[2] = 0x24u;
            resp[3] = 0xC1u;
            psx_cd_set_resp(psx, 3u, resp, 4);
        } else if (psx->cd_bcd[0] == 0x22u) {
            int n = psx_cd_test_region_string(psx, resp);
            psx_cd_set_resp(psx, 3u, resp, (u32)n);
        } else {
            resp[0] = stat;
            psx_cd_set_resp(psx, 3u, resp, 1);
        }
    } else if (cmd == 0x1E) { /* ReadTOC */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        resp[0] = psx_cd_stat(psx);
        psx_cd_set_second_resp(psx, 2u, resp, 1, 0);
    } else if (cmd == 0x1D) { /* GetQ */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
        {
            u8 q[11];
            u32 i;
            for (i = 0; i < 11u; i++) q[i] = 0;
            q[0] = 0x41u;
            psx_cd_set_second_resp(psx, 2u, q, 11, 0);
        }
    } else if (cmd == 0x1F) { /* VideoCD / unused */
        resp[0] = stat;
        psx_cd_set_resp(psx, 3u, resp, 1);
    } else {
        resp[0] = stat;
        psx_cd_set_resp(psx, 5u, resp, 1);
    }
}

static u32 psx_cd_data_skip(const struct psx_system *psx)
{
    /* SetMode.bit5 selects sector size:
     *   0 = 0x800 data-only  -> payload starts at +24
     *   1 = 0x924 whole sector except 12-byte sync -> starts at +12
     * The BIOS/game sequence uses SetMode A0h, so honoring bit5 matters. */
    if (psx->cd_sector_size == 2352) {
        if (psx->cd_mode & 0x20u) return 12u;
        return 24u;
    }
    return 0u;
}

static u32 psx_cd_data_limit(const struct psx_system *psx)
{
    if (psx->cd_sector_size == 2352) {
        if (psx->cd_mode & 0x20u) return 2340u;
        return 2048u;
    }
    return (psx->cd_sector_size == 2352) ? 2352u : 2048u;
}

static void psx_cd_fifo_reset(struct psx_system *psx)
{
    psx->cd_fifo_r = 0;
    psx->cd_fifo_w = 0;
    psx->cd_fifo_count = 0;
    psx->cd_data_ready = 0;
    psx->cd_data_pos = 0;
}

static void psx_cd_fifo_sync_state(struct psx_system *psx)
{
    if (psx->cd_fifo_count == 0) {
        psx->cd_data_ready = 0;
        psx->cd_data_pos = 0;
        return;
    }

    psx->cd_data_ready = 1;
    psx->cd_data_pos = psx->cd_fifo_pos[psx->cd_fifo_r & 7u];
}

static void psx_cd_fifo_push(struct psx_system *psx, const u8 *src, u32 size)
{
    u32 idx;

    if (!src || size == 0u)
        return;
    if (size > 2340u)
        size = 2340u;

    if (psx->cd_fifo_count >= 8u) {
        psx->cd_fifo_r = (psx->cd_fifo_r + 1u) & 7u;
        psx->cd_fifo_count--;
    }

    idx = psx->cd_fifo_w & 7u;
    for (u32 i = 0; i < size; i++)
        psx->cd_fifo_data[idx][i] = src[i];
    psx->cd_fifo_size[idx] = (u16)size;
    psx->cd_fifo_pos[idx] = 0;
    psx->cd_fifo_w = (psx->cd_fifo_w + 1u) & 7u;
    psx->cd_fifo_count++;
    psx_cd_fifo_sync_state(psx);
}

static void psx_cd_fifo_replace_single(struct psx_system *psx, const u8 *src, u32 size)
{
    psx_cd_fifo_reset(psx);
    psx_cd_fifo_push(psx, src, size);
}

static u8 psx_cd_fifo_read_byte(struct psx_system *psx, int *ok)
{
    u32 idx;
    u8 value;

    if (ok)
        *ok = 0;
    if (psx->cd_fifo_count == 0u) {
        psx_cd_fifo_sync_state(psx);
        return 0;
    }

    idx = psx->cd_fifo_r & 7u;
    if (psx->cd_fifo_pos[idx] >= psx->cd_fifo_size[idx]) {
        psx->cd_fifo_r = (psx->cd_fifo_r + 1u) & 7u;
        psx->cd_fifo_count--;
        psx_cd_fifo_sync_state(psx);
        if (psx->cd_fifo_count == 0u)
            return 0;
        idx = psx->cd_fifo_r & 7u;
    }

    value = psx->cd_fifo_data[idx][psx->cd_fifo_pos[idx]++];
    if (ok)
        *ok = 1;

    if (psx->cd_fifo_pos[idx] >= psx->cd_fifo_size[idx]) {
        psx->cd_fifo_r = (psx->cd_fifo_r + 1u) & 7u;
        psx->cd_fifo_count--;
    }
    psx_cd_fifo_sync_state(psx);
    return value;
}

static void psx_dma_recalc_dicr(struct psx_system *psx)
{
    u32 dicr = psx->dma_dicr & 0x7FFFFFFFu;
    if ((dicr & 0x00008000u) || ((dicr & 0x00800000u) && (dicr & 0x7F000000u)))
        dicr |= 0x80000000u;
    else
        dicr &= ~0x80000000u;
    psx->dma_dicr = dicr;
}

static void psx_dma_complete_channel(struct psx_system *psx, u32 ch)
{
    u32 old_master = psx->dma_dicr & 0x80000000u;
    psx->dma_chcr[ch & 7u] &= ~(1u << 24);
    if (psx->dma_dicr & (1u << (16u + (ch & 7u))))
        psx->dma_dicr |= (1u << (24u + (ch & 7u)));
    psx_dma_recalc_dicr(psx);
    if (!old_master && (psx->dma_dicr & 0x80000000u))
        psx->i_stat |= (1u << 3);
}

static void psx_spu_refresh_stat(struct psx_system *psx, int transfer_busy)
{
    u16 cnt = psx->spu_regs[0xD5];
    u16 stat = cnt & 0x003Fu;
    u16 mode = (cnt >> 4) & 3u;
    if (transfer_busy) stat |= 0x0400u;
    if (mode == 2u) stat |= 0x0180u; /* DMA write request */
    else if (mode == 3u) stat |= 0x0280u; /* DMA read request */
    if (psx->spu_regs[0xD3] & 0x0040u) stat |= 0x0040u; /* IRQ9 flag */
    psx->spu_regs[0xD3] = stat;
}

static void psx_spu_raise_irq9(struct psx_system *psx)
{
    if (!(psx->spu_regs[0xD5] & 0x8000u)) return;
    if (!(psx->spu_regs[0xD5] & 0x0040u)) return;
    psx->spu_regs[0xD3] |= 0x0040u;
    psx_spu_refresh_stat(psx, 0);
    psx->i_stat |= (1u << 9);
}

static void psx_spu_check_transfer_irq(struct psx_system *psx, u32 start_addr, u32 bytes)
{
    u32 irq_addr = ((u32)psx->spu_regs[0xD2] << 3) & 0x7FFFFu;
    if (!bytes) return;
    if (!(psx->spu_regs[0xD5] & 0x0040u) || !(psx->spu_regs[0xD5] & 0x8000u))
        return;

    for (u32 i = 0; i < bytes; i++) {
        if (((start_addr + i) & 0x7FFFFu) == irq_addr) {
            psx_spu_raise_irq9(psx);
            break;
        }
    }
}

static const u8 psx_mdec_zigzag[64] = {
    0, 8, 1, 2, 9, 16, 24, 17,
    10, 3, 4, 11, 18, 25, 32, 40,
    33, 26, 19, 12, 5, 6, 13, 20,
    27, 34, 41, 48, 56, 49, 42, 35,
    28, 21, 14, 7, 15, 22, 29, 36,
    43, 50, 57, 58, 51, 44, 37, 30,
    23, 31, 38, 45, 52, 59, 60, 53,
    46, 39, 47, 54, 61, 62, 55, 63
};

static const s16 psx_mdec_default_scale[64] = {
    0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82,
    0x7D8A, 0x6A6D, 0x471C, 0x18F8, (s16)0xE707, (s16)0xB8E3, (s16)0x9592, (s16)0x8275,
    0x7641, 0x30FB, (s16)0xCF04, (s16)0x89BE, (s16)0x89BE, (s16)0xCF04, 0x30FB, 0x7641,
    0x6A6D, (s16)0xE707, (s16)0x8275, (s16)0xB8E3, 0x471C, 0x7D8A, 0x18F8, (s16)0x9592,
    0x5A82, (s16)0xA57D, (s16)0xA57D, 0x5A82, 0x5A82, (s16)0xA57D, (s16)0xA57D, 0x5A82,
    0x471C, (s16)0x8275, 0x18F8, 0x6A6D, (s16)0x9592, (s16)0xE707, 0x7D8A, (s16)0xB8E3,
    0x30FB, (s16)0x89BE, 0x7641, (s16)0xCF04, (s16)0xCF04, 0x7641, (s16)0x89BE, 0x30FB,
    0x18F8, (s16)0xB8E3, 0x6A6D, (s16)0x8275, 0x7D8A, (s16)0x9592, 0x471C, (s16)0xE707
};

static s32 psx_mdec_clamp(s32 v, s32 lo, s32 hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static s32 psx_mdec_signed10(u32 v)
{
    v &= 0x3FFu;
    if (v & 0x200u)
        return (s32)v - 0x400;
    return (s32)v;
}

static s32 psx_mdec_signextend9(s32 v)
{
    v &= 0x1FF;
    if (v & 0x100)
        v -= 0x200;
    return v;
}

static void psx_mdec_write_u32_ram(struct psx_system *psx, u32 *addr, u32 val)
{
    u32 a = *addr & 0x1FFFFCu;
    if (a + 3u < RAM_SIZE)
        *(u32 *)(psx->ram + a) = val;
    *addr = (a + 4u) & 0x1FFFFCu;
}

static void psx_mdec_init_defaults(struct psx_system *psx)
{
    for (u32 i = 0; i < 64u; i++) {
        psx->mdec_iq_y[i] = 1u;
        psx->mdec_iq_uv[i] = 1u;
        psx->mdec_scale[i] = psx_mdec_default_scale[i];
    }
}

static void psx_mdec_update_status(struct psx_system *psx)
{
    u32 st = 0;
    u32 remaining = psx->mdec_words_remaining;

    if (!psx->mdec_output_ready)
        st |= 1u << 31; /* data-out fifo empty */
    if (remaining == 0)
        st |= 1u << 30; /* data-in full/last word received */
    if (remaining != 0)
        st |= 1u << 29; /* busy receiving parameters */
    if (psx->mdec_in_enabled)
        st |= 1u << 28;
    if (psx->mdec_out_enabled && psx->mdec_output_ready)
        st |= 1u << 27;

    /* STAT.26..23 mirror CMD.28..25, shifted down by two bits. */
    st |= (psx->mdec_cmd >> 2) & 0x07800000u;

    if (remaining) {
        st |= (remaining - 1u) & 0xFFFFu;
    } else {
        st |= 0xFFFFu;
    }

    psx->mdec_status = st;
}

static void psx_mdec_reset(struct psx_system *psx)
{
    psx->mdec_ctrl = 0;
    psx->mdec_cmd = 0;
    psx->mdec_words_remaining = 0;
    psx->mdec_in_enabled = 0;
    psx->mdec_out_enabled = 0;
    psx->mdec_output_ready = 0;
    psx->mdec_output_words = 0;
    psx->mdec_data_read_pos = 0;
    psx->mdec_in_word_count = 0;
    psx->mdec_in_half_pos = 0;
    psx->mdec_log_flags = 0;
    psx_mdec_init_defaults(psx);
    psx->mdec_status = 0x80040000u;
}

static void psx_mdec_write_command(struct psx_system *psx, u32 val)
{
    if (psx->mdec_words_remaining == 0) {
        u32 cmd = val >> 29;
        psx->mdec_cmd = val;
        if (cmd == 1u) {
            /* Decode macroblock(s): low 16 bits are parameter words. */
            psx->mdec_words_remaining = val & 0xFFFFu;
        } else if (cmd == 2u) {
            /* Set quant table: 64 bytes luma, +64 bytes chroma if bit0=1. */
            psx->mdec_words_remaining = (val & 1u) ? 32u : 16u;
        } else if (cmd == 3u) {
            /* Set scale table: 64 halfwords = 32 words. */
            psx->mdec_words_remaining = 32u;
        } else {
            psx->mdec_words_remaining = 0;
        }
        psx->mdec_in_word_count = 0;
        psx->mdec_in_half_pos = 0;
        psx->mdec_output_ready = 0;
        psx->mdec_output_words = 0;
        psx->mdec_data_read_pos = 0;
        if (!(psx->mdec_log_flags & 1u) && psx->sendto_fn && psx->log_fd >= 0) {
            psx->mdec_log_flags |= 1u;
            psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] cmd=", val);
            psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] words=", psx->mdec_words_remaining);
        }
    } else {
        psx->mdec_words_remaining--;
        if (psx->mdec_words_remaining == 0 && ((psx->mdec_cmd >> 29) == 1u)) {
            /* Minimal stub: decoded data becomes available as black blocks. */
            psx->mdec_output_ready = 1;
            psx->mdec_data_read_pos = 0;
        }
    }
    psx_mdec_update_status(psx);
}

static void psx_mdec_commit_iq_table(struct psx_system *psx)
{
    u8 *src = (u8 *)psx->mdec_in_words;
    u32 words = psx->mdec_in_word_count;
    u32 bytes = words * 4u;

    if (bytes >= 64u) {
        for (u32 i = 0; i < 64u; i++)
            psx->mdec_iq_y[i] = src[i];
    }
    if ((psx->mdec_cmd & 1u) && bytes >= 128u) {
        for (u32 i = 0; i < 64u; i++)
            psx->mdec_iq_uv[i] = src[64u + i];
    }
}

static void psx_mdec_commit_scale_table(struct psx_system *psx)
{
    s16 *src = (s16 *)psx->mdec_in_words;
    u32 halfwords = psx->mdec_in_word_count * 2u;

    if (halfwords < 64u)
        return;
    for (u32 y = 0; y < 8u; y++) {
        for (u32 x = 0; x < 8u; x++)
            psx->mdec_scale[y * 8u + x] = src[x * 8u + y];
    }
}

static s32 psx_mdec_idct_row_s32(const s32 *blk, const s16 *idct)
{
    s64 sum = 0;
    for (u32 i = 0; i < 8u; i++)
        sum += (s64)blk[i] * (s64)idct[i];
    return (s32)((sum + 0x20000) >> 18);
}

static void psx_mdec_idct_block(struct psx_system *psx, s32 *blk)
{
    s32 tmp[64];
    s32 col[8];

    for (u32 x = 0; x < 8u; x++) {
        for (u32 y = 0; y < 8u; y++)
            tmp[y * 8u + x] = psx_mdec_idct_row_s32(&blk[x * 8u], &psx->mdec_scale[y * 8u]);
    }

    for (u32 x = 0; x < 8u; x++) {
        for (u32 i = 0; i < 8u; i++)
            col[i] = tmp[x * 8u + i];
        for (u32 y = 0; y < 8u; y++) {
            s32 sum = psx_mdec_idct_row_s32(col, &psx->mdec_scale[y * 8u]);
            blk[x * 8u + y] = psx_mdec_clamp(psx_mdec_signextend9(sum), -128, 127);
        }
    }
}

static int psx_mdec_rl_decode_block(struct psx_system *psx, s32 *blk,
                                    const u16 *src, u32 halfwords, u32 *pos,
                                    const u8 *qt)
{
    u32 k = 0;
    u32 n;
    s32 val;
    u32 q_scale;

    for (u32 i = 0; i < 64u; i++)
        blk[i] = 0;

    for (;;) {
        if (*pos >= halfwords)
            return 0;
        n = src[(*pos)++];
        if (n != 0xFE00u)
            break;
    }

    q_scale = (n >> 10) & 0x3Fu;
    val = psx_mdec_signed10(n);
    if (q_scale == 0u)
        val <<= 5;
    else
        val = ((val * (s32)qt[0]) << 4) + (val ? ((val < 0) ? 8 : -8) : 0);

    for (;;) {
        s32 store = val;
        store = psx_mdec_clamp(store, -0x4000, 0x3FFF);
        blk[psx_mdec_zigzag[k]] = store;

        if (*pos >= halfwords)
            break;

        n = src[(*pos)++];
        k += ((n >> 10) & 0x3Fu) + 1u;
        if (k > 63u)
            break;
        val = psx_mdec_signed10(n);
        {
            s32 scq = (s32)q_scale * (s32)qt[k];
            if (scq == 0)
                val <<= 5;
            else
                val = (((val * scq) >> 3) << 4) + (val ? ((val < 0) ? 8 : -8) : 0);
        }
    }

    psx_mdec_idct_block(psx, blk);
    return 1;
}

static u8 psx_mdec_pack_component(s32 c, u32 signed_output)
{
    c = psx_mdec_clamp(c, -128, 127);
    if (signed_output)
        return (u8)(c & 0xFF);
    return (u8)(c + 128);
}

static void psx_mdec_decode_colored_macroblock(struct psx_system *psx, u8 *out, u32 *out_bytes)
{
    const u16 *src = (const u16 *)psx->mdec_in_words;
    u32 halfwords = psx->mdec_in_word_count * 2u;
    u32 pos = psx->mdec_in_half_pos;
    s32 blk[64 * 6];
    u32 depth = (psx->mdec_cmd >> 27) & 3u;
    u32 signed_output = (psx->mdec_cmd >> 26) & 1u;
    u32 bit15 = (psx->mdec_cmd >> 25) & 1u;
    s32 *crblk = blk;
    s32 *cbblk = blk + 64;
    s32 *yblk = blk + 128;
    u32 p = 0;

    if (!psx_mdec_rl_decode_block(psx, crblk, src, halfwords, &pos, psx->mdec_iq_uv) ||
        !psx_mdec_rl_decode_block(psx, cbblk, src, halfwords, &pos, psx->mdec_iq_uv) ||
        !psx_mdec_rl_decode_block(psx, yblk + 0, src, halfwords, &pos, psx->mdec_iq_y) ||
        !psx_mdec_rl_decode_block(psx, yblk + 64, src, halfwords, &pos, psx->mdec_iq_y) ||
        !psx_mdec_rl_decode_block(psx, yblk + 128, src, halfwords, &pos, psx->mdec_iq_y) ||
        !psx_mdec_rl_decode_block(psx, yblk + 192, src, halfwords, &pos, psx->mdec_iq_y)) {
        *out_bytes = 0;
        return;
    }

    psx->mdec_in_half_pos = pos;

    for (u32 py = 0; py < 16u; py++) {
        for (u32 px = 0; px < 16u; px++) {
            u32 yblock = (py >= 8u ? 2u : 0u) + (px >= 8u ? 1u : 0u);
            u32 yi = (py & 7u) * 8u + (px & 7u);
            s32 yv = yblk[yblock * 64u + yi];
            s32 cr = crblk[(py >> 1) * 8u + (px >> 1)];
            s32 cb = cbblk[(py >> 1) * 8u + (px >> 1)];
            s32 r = yv + (((359 * cr) + 0x80) >> 8);
            s32 g = yv + ((((-88 * cb) & ~0x1F) + ((-183 * cr) & ~0x07) + 0x80) >> 8);
            s32 b = yv + (((454 * cb) + 0x80) >> 8);
            u8 pr = psx_mdec_pack_component(r, signed_output);
            u8 pg = psx_mdec_pack_component(g, signed_output);
            u8 pb = psx_mdec_pack_component(b, signed_output);

            if (depth == 3u) {
                u16 pix = (u16)(((pr >> 3) & 0x1Fu) |
                                (((pg >> 3) & 0x1Fu) << 5) |
                                (((pb >> 3) & 0x1Fu) << 10) |
                                (bit15 ? 0x8000u : 0u));
                out[p++] = (u8)(pix & 0xFFu);
                out[p++] = (u8)(pix >> 8);
            } else {
                out[p++] = pr;
                out[p++] = pg;
                out[p++] = pb;
            }
        }
    }

    *out_bytes = p;
}

static void psx_mdec_decode_monochrome_macroblock(struct psx_system *psx, u8 *out, u32 *out_bytes)
{
    const u16 *src = (const u16 *)psx->mdec_in_words;
    u32 halfwords = psx->mdec_in_word_count * 2u;
    u32 pos = psx->mdec_in_half_pos;
    s32 blk[64];
    u32 depth = (psx->mdec_cmd >> 27) & 3u;
    u32 signed_output = (psx->mdec_cmd >> 26) & 1u;
    u32 p = 0;

    if (!psx_mdec_rl_decode_block(psx, blk, src, halfwords, &pos, psx->mdec_iq_y)) {
        *out_bytes = 0;
        return;
    }

    psx->mdec_in_half_pos = pos;

    if (depth == 0u) {
        for (u32 i = 0; i < 64u; i += 2u) {
            s32 ya = psx_mdec_clamp(psx_mdec_signextend9(blk[i]), -128, 127);
            s32 yb = psx_mdec_clamp(psx_mdec_signextend9(blk[i + 1u]), -128, 127);
            u8 a = psx_mdec_pack_component(ya, signed_output) >> 4;
            u8 b = psx_mdec_pack_component(yb, signed_output) >> 4;
            out[p++] = (u8)(a | (b << 4));
        }
    } else {
        for (u32 i = 0; i < 64u; i++) {
            s32 y = psx_mdec_clamp(psx_mdec_signextend9(blk[i]), -128, 127);
            out[p++] = psx_mdec_pack_component(y, signed_output);
        }
    }

    *out_bytes = p;
}

static void psx_mdec_compact_input(struct psx_system *psx)
{
    u16 *buf = (u16 *)psx->mdec_in_words;
    u32 total_halfwords = psx->mdec_in_word_count * 2u;
    u32 used_halfwords = psx->mdec_in_half_pos;

    if (used_halfwords == 0u)
        return;
    if (used_halfwords >= total_halfwords) {
        psx->mdec_in_word_count = 0;
        psx->mdec_in_half_pos = 0;
        return;
    }

    {
        u32 remaining_halfwords = total_halfwords - used_halfwords;
        for (u32 i = 0; i < remaining_halfwords; i++)
            buf[i] = buf[used_halfwords + i];
        if (remaining_halfwords & 1u)
            buf[remaining_halfwords] = 0;
        psx->mdec_in_word_count = (remaining_halfwords + 1u) >> 1;
        psx->mdec_in_half_pos = 0;
    }
}

static void dma0_exec(struct psx_system *psx)
{
    u32 addr = psx->dma_madr[0] & 0x1FFFFCu;
    u32 words_per_block = psx->dma_bcr[0] & 0xFFFFu;
    u32 block_count = (psx->dma_bcr[0] >> 16) & 0xFFFFu;
    u32 words = words_per_block;

    if (!words_per_block)
        words_per_block = 0x10000u;
    if (!block_count)
        block_count = 1u;
    words = words_per_block * block_count;

    if (!(psx->mdec_log_flags & 2u) && psx->sendto_fn && psx->log_fd >= 0) {
        psx->mdec_log_flags |= 2u;
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] DMA0 bcr=", psx->dma_bcr[0]);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] DMA0 chcr=", psx->dma_chcr[0]);
    }

    for (u32 i = 0; i < words; i++) {
        if (addr + 3u < RAM_SIZE) {
            u32 data = *(u32 *)(psx->ram + addr);
            u32 is_command_word = (psx->mdec_words_remaining == 0u);
            if (!is_command_word && psx->mdec_in_word_count < 0x10000u)
                psx->mdec_in_words[psx->mdec_in_word_count++] = data;
            psx_mdec_write_command(psx, data);
        }
        addr = (addr + 4u) & 0x1FFFFFu;
    }

    if ((psx->mdec_cmd >> 29) == 2u)
        psx_mdec_commit_iq_table(psx);
    else if ((psx->mdec_cmd >> 29) == 3u)
        psx_mdec_commit_scale_table(psx);

    psx->dma_madr[0] = addr;
    psx_dma_complete_channel(psx, 0u);
}

static void dma1_exec(struct psx_system *psx)
{
    u32 addr = psx->dma_madr[1] & 0x1FFFFCu;
    u32 words_per_block = psx->dma_bcr[1] & 0xFFFFu;
    u32 block_count = (psx->dma_bcr[1] >> 16) & 0xFFFFu;
    u32 words = words_per_block;

    if (!words_per_block)
        words_per_block = 0x10000u;
    if (!block_count)
        block_count = 1u;
    words = words_per_block * block_count;

    if (!(psx->mdec_log_flags & 4u) && psx->sendto_fn && psx->log_fd >= 0) {
        psx->mdec_log_flags |= 4u;
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] DMA1 bcr=", psx->dma_bcr[1]);
        psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] DMA1 chcr=", psx->dma_chcr[1]);
    }

    if ((psx->mdec_cmd >> 29) == 1u) {
        u32 remaining_words = words;
        while (remaining_words) {
            u8 out[768];
            u32 out_bytes = 0;
            u32 out_words;
            u32 depth = (psx->mdec_cmd >> 27) & 3u;

            if (depth == 2u || depth == 3u)
                psx_mdec_decode_colored_macroblock(psx, out, &out_bytes);
            else
                psx_mdec_decode_monochrome_macroblock(psx, out, &out_bytes);

            if (!out_bytes)
                break;

            out_words = (out_bytes + 3u) >> 2;
            for (u32 i = 0; i < out_words && remaining_words; i++) {
                u32 v = 0;
                u32 base = i * 4u;
                if (base + 0u < out_bytes) v |= (u32)out[base + 0u];
                if (base + 1u < out_bytes) v |= (u32)out[base + 1u] << 8;
                if (base + 2u < out_bytes) v |= (u32)out[base + 2u] << 16;
                if (base + 3u < out_bytes) v |= (u32)out[base + 3u] << 24;
                psx_mdec_write_u32_ram(psx, &addr, v);
                remaining_words--;
            }

            if (out_words == 0u)
                break;
        }
        psx_mdec_compact_input(psx);
    } else {
        for (u32 i = 0; i < words; i++)
            psx_mdec_write_u32_ram(psx, &addr, 0u);
    }

    psx->mdec_output_ready = 0;
    psx->mdec_output_words = 0;
    psx->mdec_data_read_pos = 0;
    psx->dma_madr[1] = addr;
    psx_mdec_update_status(psx);
    psx_dma_complete_channel(psx, 1u);
}


/* ── DMA channel 2: GPU linked-list (RAM → GP0) ──────────────────── */
static void dma2_exec(struct psx_system *psx, u32 chcr)
{
    u32 mode = (chcr >> 9) & 3u;
    u32 addr = psx->dma_madr[2] & 0x1FFFFFu;
    u32 words_per_block = psx->dma_bcr[2] & 0xFFFFu;
    u32 block_count = (psx->dma_bcr[2] >> 16) & 0xFFFFu;
    u32 words = words_per_block;

    if (mode == 0 || mode == 1) {
        if (!words_per_block)
            words_per_block = 0x10000u;
        if (mode == 1) {
            if (!block_count)
                block_count = 0x10000u;
            words = words_per_block * block_count;
        } else {
            words = words_per_block;
        }
    }

    if (mode == 0 || mode == 1) { /* Linear RAM -> GP0 transfer */
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
    psx_dma_complete_channel(psx, 2u);
}

/* ── DMA channel 3: CD-ROM ───────────────────────────────────────── */
static void dma3_exec(struct psx_system *psx) {
    u32 addr  = psx->dma_madr[3] & 0x1FFFFCu;
    u32 bcr   = psx->dma_bcr[3];
    u32 words = bcr & 0xFFFFu;
    u32 blocks = (bcr >> 16) & 0xFFFFu;
    if (!words) words = 0x10000u;
    if (!blocks) blocks = 1u;
    u32 total_words = words * blocks;
    for (u32 i = 0; i < total_words; i++) {
        u32 val = 0;
        /* Read 4 bytes from CD-ROM Data FIFO */
        for (int j = 0; j < 4; j++) {
            u8 b = 0;
            int ok = 0;
            b = psx_cd_fifo_read_byte(psx, &ok);
            val |= ((u32)b << (j * 8));
        }
        if (addr + 3u < RAM_SIZE) {
            *(u32 *)(psx->ram + addr) = val;
        }
        addr = (addr + 4) & 0x1FFFFCu;
    }
    psx->dma_madr[3] = addr;
    psx_dma_complete_channel(psx, 3u);
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
    psx_dma_complete_channel(psx, 6u);
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
        /* MDEC */
        case 0x1F801820:
            if (psx->mdec_output_ready) {
                psx->mdec_data_read_pos++;
                if (psx->mdec_output_words && psx->mdec_data_read_pos >= psx->mdec_output_words) {
                    psx->mdec_output_ready = 0;
                    psx->mdec_data_read_pos = 0;
                    psx_mdec_update_status(psx);
                }
            }
            return 0;
        case 0x1F801824:
            psx_mdec_update_status(psx);
            return psx->mdec_status;
        /* GPU */
        case 0x1F801810: return psx->gpu_io.gpuread;
        case 0x1F801814: return psx->gpu_io.gpustat;
        /* CD-ROM: 1F801800 - 1F801803 */
        case 0x1F801800: return psx_cd_hsts(psx);
        case 0x1F801801:
            /* Response FIFO: return from buffer (all indices) */
            if (psx->cd_resp_pos < psx->cd_resp_len) {
                return psx->cd_resp_buf[psx->cd_resp_pos++];
            }
            return 0;
        case 0x1F801802:
            /* Data FIFO: return sector content (all indices) */
            if (!(psx->cd_req & 0x80u) || !psx->cd_data_ready) return 0;
            return psx_cd_fifo_read_byte(psx, 0);
        case 0x1F801803:
            /* Interrupt Enable (index 0, 2) / Interrupt Flag (index 1, 3) */
            if (psx->cd_idx == 1 || psx->cd_idx == 3) {
                u8 val = 0xE0u;
                if (psx->cd_irq_pending) val |= (psx->cd_irq_pending & 7u);
                return val;
            }
            if (psx->cd_idx == 0 || psx->cd_idx == 2)
                return 0xE0u | (psx->cd_hint_mask & 0x1Fu);
            return 0;
        /* JOY/SIO0 */
        case 0x1F801040: {
            u32 v = psx->joy_rx_ready ? (psx->joy_rx_data & 0xFFu) : 0xFFu;
            psx->joy_rx_ready = 0;
            return v;
        }
        case 0x1F801044: return psx_sio_stat(psx);
        case 0x1F801048: return psx->joy_mode & 0xFFFFu;
        case 0x1F80104A: return psx->joy_ctrl & 0xFFFFu;
        case 0x1F80104E: return psx->joy_baud & 0xFFFFu;
        /* DMA channel 0 (MDEC in) */
        case 0x1F801080: return psx->dma_madr[0];
        case 0x1F801084: return psx->dma_bcr[0];
        case 0x1F801088: return psx->dma_chcr[0];
        /* DMA channel 1 (MDEC out) */
        case 0x1F801090: return psx->dma_madr[1];
        case 0x1F801094: return psx->dma_bcr[1];
        case 0x1F801098: return psx->dma_chcr[1];
        /* Timers: 16-bit counters */
        case 0x1F801100: return psx->timer0 & 0xFFFFu;
        case 0x1F801110: return psx->timer1 & 0xFFFFu;
        case 0x1F801120: return psx->timer2 & 0xFFFFu;
        case 0x1F801104: case 0x1F801114: case 0x1F801124: return 0; /* Mode */
        case 0x1F801108: case 0x1F801118: case 0x1F801128: return 0; /* Target */
        /* DMA channel 2 (GPU) */
        case 0x1F8010A0: return psx->dma_madr[2];
        case 0x1F8010A4: return psx->dma_bcr[2];
        case 0x1F8010A8: return psx->dma_chcr[2];
        /* DMA channel 3 (CD-ROM) */
        case 0x1F8010B0: return psx->dma_madr[3];
        case 0x1F8010B4: return psx->dma_bcr[3];
        case 0x1F8010B8: return psx->dma_chcr[3];
        /* DMA channel 4 (SPU) */
        case 0x1F8010C0: return psx->dma_madr[4];
        case 0x1F8010C4: return psx->dma_bcr[4];
        case 0x1F8010C8: return psx->dma_chcr[4];
        /* DMA channel 6 (OTC) */
        case 0x1F8010E0: return psx->dma_madr[6];
        case 0x1F8010E4: return psx->dma_bcr[6];
        case 0x1F8010E8: return psx->dma_chcr[6];
        /* DMA control */
        case 0x1F8010F0: return 0x07654321u; /* DPCR – all channels enabled */
        case 0x1F8010F4: return psx->dma_dicr;
        /* SPU */
        case 0x1F801DA6: return psx->spu_regs[0xD3]; /* SPUSTAT */
        case 0x1F801DAA: return psx->spu_regs[0xD5]; /* SPUCNT */
        default:
            if (phys >= 0x1F801C00 && phys < 0x1F801E00) {
                return psx->spu_regs[(phys - 0x1F801C00) >> 1];
            }
            return 0;
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
        case 0x1F801820:
            psx_mdec_write_command(psx, val);
            break;
        case 0x1F801824:
            if (val & 0x80000000u) {
                psx_mdec_reset(psx);
            } else {
                psx->mdec_ctrl = val;
                psx->mdec_in_enabled = (val >> 30) & 1u;
                psx->mdec_out_enabled = (val >> 29) & 1u;
                if (!(psx->mdec_log_flags & 8u) && psx->sendto_fn && psx->log_fd >= 0) {
                    psx->mdec_log_flags |= 8u;
                    psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr, "[MDEC] ctrl=", val);
                }
                psx_mdec_update_status(psx);
            }
            break;
        case 0x1F801810: gpu_gp0_write(&psx->gpu_io, val); break;
        case 0x1F801814: gpu_gp1_write(&psx->gpu_io, val); break;
        case 0x1F801040:
            psx_sio_write_data(psx, (u8)(val & 0xFFu));
            break;
        case 0x1F801048:
            psx->joy_mode = val & 0xFFFFu;
            break;
        case 0x1F80104A: {
            u32 old_ctrl = psx->joy_ctrl;
            psx->joy_ctrl = val & 0xFFFFu;
            if (psx->joy_ctrl & 0x0040u) {
                psx_sio_reset(psx);
            } else {
                if (psx->joy_ctrl & 0x0010u) {
                    psx->joy_irq = 0;
                    psx->i_stat &= ~(1u << 7);
                }
                if ((old_ctrl & 0x0002u) && !(psx->joy_ctrl & 0x0002u))
                    psx_sio_deselect(psx);
            }
            break;
        }
        case 0x1F80104E:
            psx->joy_baud = val & 0xFFFFu;
            break;
        /* DMA channel 0 (MDEC in) */
        case 0x1F801080: psx->dma_madr[0] = val; break;
        case 0x1F801084: psx->dma_bcr[0]  = val; break;
        case 0x1F801088:
            psx->dma_chcr[0] = val;
            if (val & (1u << 24))
                dma0_exec(psx);
            break;
        /* DMA channel 1 (MDEC out) */
        case 0x1F801090: psx->dma_madr[1] = val; break;
        case 0x1F801094: psx->dma_bcr[1]  = val; break;
        case 0x1F801098:
            psx->dma_chcr[1] = val;
            if (val & (1u << 24)) {
                psx->mdec_output_words = (psx->dma_bcr[1] & 0xFFFFu) *
                                         (((psx->dma_bcr[1] >> 16) & 0xFFFFu) ? ((psx->dma_bcr[1] >> 16) & 0xFFFFu) : 1u);
                if (!psx->mdec_output_words)
                    psx->mdec_output_words = 0x10000u;
                psx->mdec_output_ready = 1;
                psx_mdec_update_status(psx);
                dma1_exec(psx);
            }
            break;
        /* CD-ROM */
        case 0x1F801800: psx->cd_idx = val & 3u; break;
        case 0x1F801801:
            if (psx->cd_idx == 0) { /* Command register */
                psx->cd_cmd_cnt++;
                u32 cmd = val & 0xFFu;
                if (psx->cd_irq_pending) {
                    psx->cd_cmd_pending = 1;
                    psx->cd_cmd_pending_code = (u8)cmd;
                    psx->cd_cmd_pending_n = psx->cd_p_idx;
                    for (u32 i = 0; i < psx->cd_p_idx && i < 16u; i++)
                        psx->cd_cmd_pending_params[i] = psx->cd_bcd[i];
                } else {
                    psx_cd_exec_cmd(psx, cmd);
                }
                psx->cd_p_idx = 0; /* Reset parameters AFTER use */
            }
            break;
        case 0x1F801802:
            if (psx->cd_idx == 0) { /* Parameter FIFO */
                if (psx->cd_p_idx < 16)
                    psx->cd_bcd[psx->cd_p_idx++] = (u8)(val & 0xFFu);
            } else if (psx->cd_idx == 1) { /* HINTMSK */
                psx->cd_hint_mask = val & 0x1Fu;
                if (psx->cd_irq_pending && (psx->cd_hint_mask & (psx->cd_irq_pending & 7u)))
                    psx->i_stat |= (1u << 2);
                else
                    psx->i_stat &= ~(1u << 2);
            }
            break;
        case 0x1F801803:
            if (psx->cd_idx == 0) { /* HCHPCTL / request register */
                psx->cd_req = val & 0xFFu;
            } else if (psx->cd_idx == 1) { /* HCLRCTL */
                psx->cd_irq_pending &= ~(val & 7u);
                if (val & 0x1Fu) {
                    psx->cd_resp_pos = 0;
                    psx->cd_resp_len = 0;
                }
                if (val & 0x40u) psx->cd_p_idx = 0; /* CLRPRM */
                if (val & 0x20u) { /* clear result fifo */
                    psx->cd_resp_pos = 0;
                    psx->cd_resp_len = 0;
                }
                if (val & 0x80u) { /* CHPRST */
                    psx->cd_req = 0;
                    psx->cd_read_active = 0;
                    psx->cd_pause_pending = 0;
                    psx_cd_fifo_reset(psx);
                }
                if (psx->cd_irq_pending && (psx->cd_hint_mask & (psx->cd_irq_pending & 7u)))
                    psx->i_stat |= (1u << 2);
                else
                    psx->i_stat &= ~(1u << 2);
                if (psx->cd_irq_pending == 0 && psx->cd_cmd_pending) {
                    psx->cd_cmd_pending = 0;
                    psx->cd_p_idx = psx->cd_cmd_pending_n;
                    for (u32 i = 0; i < psx->cd_p_idx && i < 16u; i++)
                        psx->cd_bcd[i] = psx->cd_cmd_pending_params[i];
                    psx_cd_exec_cmd(psx, psx->cd_cmd_pending_code);
                    psx->cd_p_idx = 0;
                }
            }
            break;
        /* DMA channel 2 (GPU) */
        case 0x1F8010A0: psx->dma_madr[2] = val; break;
        case 0x1F8010A4: psx->dma_bcr[2]  = val; break;
        case 0x1F8010A8:
            psx->dma_chcr[2] = val;
            if (val & (1u << 24)) {
                psx->gpu_io.dma2_calls++;
                dma2_exec(psx, val);
            }
            break;
        /* DMA channel 3 (CD-ROM) */
        case 0x1F8010B0: psx->dma_madr[3] = val; break;
        case 0x1F8010B4: psx->dma_bcr[3]  = val; break;
        case 0x1F8010B8:
            psx->dma_chcr[3] = val;
            if (val & (1u << 24)) dma3_exec(psx);
            break;
        /* DMA channel 4 (SPU) */
        case 0x1F8010C0: psx->dma_madr[4] = val; break;
        case 0x1F8010C4: psx->dma_bcr[4]  = val; break;
        case 0x1F8010C8:
            psx->dma_chcr[4] = val;
            if (val & (1u << 24)) {
                u32 addr = psx->dma_madr[4] & 0x1FFFFCu;
                u32 words = psx->dma_bcr[4] & 0xFFFFu;
                u32 blocks = (psx->dma_bcr[4] >> 16) & 0xFFFFu;
                if (!words) words = 0x10000u;
                if (!blocks) blocks = 1u;
                words *= blocks;
                u32 spu_addr = psx->spu_addr_internal & 0x7FFFFu;
                u32 spu_start = spu_addr;
                psx_spu_refresh_stat(psx, 1);
                for (u32 i = 0; i < words; i++) {
                    if (addr + 3u < RAM_SIZE) {
                        u32 data = *(u32 *)(psx->ram + addr);
                        psx->spu_ram[spu_addr] = data & 0xFF;
                        psx->spu_ram[spu_addr+1] = (data >> 8) & 0xFF;
                        psx->spu_ram[spu_addr+2] = (data >> 16) & 0xFF;
                        psx->spu_ram[spu_addr+3] = (data >> 24) & 0xFF;
                        spu_addr = (spu_addr + 4) & 0x7FFFFu;
                    }
                    addr = (addr + 4) & 0x1FFFFCu;
                }
                psx->spu_addr_internal = spu_addr;
                psx->spu_regs[0xD1] = (u16)(spu_addr >> 3);
                psx_spu_check_transfer_irq(psx, spu_start, words * 4u);
                psx_spu_refresh_stat(psx, 0);
                psx_dma_complete_channel(psx, 4u);
            }
            break;
        /* DMA channel 6 (OTC) */
        case 0x1F8010E0: psx->dma_madr[6] = val; break;
        case 0x1F8010E4: psx->dma_bcr[6]  = val; break;
        case 0x1F8010E8:
            psx->dma_chcr[6] = val;
            if (val & (1u << 24)) dma6_exec(psx);
            break;
        case 0x1F8010F4:
            psx->dma_dicr &= ~(val & 0x7F000000u); /* write-1-to-ack flags */
            psx->dma_dicr = (psx->dma_dicr & 0xFF00FFFFu) | (val & 0x00FF8000u);
            psx_dma_recalc_dicr(psx);
            break;
        case 0x1F801D88: /* Key On (0-15) */
            psx->spu_voice_on |= (val & 0xFFFFu);
            for (int v = 0; v < 16; v++) {
                if (val & (1u << v)) {
                    psx->spu_voice_addr[v] = (psx->spu_regs[v*8 + 3] << 3) & 0x7FFFFu;
                    psx->spu_voice_idx[v] = 0;
                    psx->spu_voice_phase[v] = 0;
                    psx->spu_voice_last[v][0] = 0;
                    psx->spu_voice_last[v][1] = 0;
                    psx_spu_decode_block(psx, v);
                }
            }
            break;
        case 0x1F801D8A: /* Key On (16-23) */
            psx->spu_voice_on |= ((val & 0xFFu) << 16);
            for (int v = 0; v < 8; v++) {
                if (val & (1u << v)) {
                    psx->spu_voice_addr[v+16] = (psx->spu_regs[(v+16)*8 + 3] << 3) & 0x7FFFFu;
                    psx->spu_voice_idx[v+16] = 0;
                    psx->spu_voice_phase[v+16] = 0;
                    psx->spu_voice_last[v+16][0] = 0;
                    psx->spu_voice_last[v+16][1] = 0;
                    psx_spu_decode_block(psx, v+16);
                }
            }
            break;
        case 0x1F801D8C: /* Key Off (0-15) */
            psx->spu_voice_on &= ~(val & 0xFFFFu);
            break;
        case 0x1F801D8E: /* Key Off (16-23) */
            psx->spu_voice_on &= ~((val & 0xFFu) << 16);
            break;
        case 0x1F801DA2: /* SPU Transfer Address */
            psx->spu_regs[0xD1] = (u16)val;
            psx->spu_addr_internal = (u32)val << 3;
            break;
        case 0x1F801DA8: { /* SPU RAM Transfer */
            u32 addr = psx->spu_addr_internal & 0x7FFFFu;
            psx->spu_ram[addr] = val & 0xFFu;
            psx->spu_ram[addr+1] = (val >> 8) & 0xFFu;
            psx->spu_addr_internal = (addr + 2) & 0x7FFFFu;
            psx->spu_regs[0xD1] = (u16)(psx->spu_addr_internal >> 3);
            break;
        }
        case 0x1F801DAA: /* SPUCNT */
            psx->spu_regs[0xD5] = (u16)val;
            if (!(val & 0x0040u))
                psx->spu_regs[0xD3] &= ~0x0040u; /* bit6=0 also acknowledges IRQ9 */
            psx_spu_refresh_stat(psx, 0);
            break;
        default:
            if (phys >= 0x1F801C00 && phys < 0x1F801E00) {
                psx->spu_regs[(phys - 0x1F801C00) >> 1] = (u16)val;
            }
            break;
        }
    }
}



/* Cycle-accurate hardware updates */
void psx_bus_tick(struct psx_system *psx, u32 cycles)
{
    if (psx->cd_busy_ticks > cycles) psx->cd_busy_ticks -= cycles;
    else psx->cd_busy_ticks = 0;

    /* 0. Update Timers (16-bit) */
    /* Timer 0: ~8MHz (div 4) */
    /* Timer 1: ~15.7kHz (div 2153) */
    /* Timer 2: ~4.2MHz (div 8) */
    psx->timer0_acc += cycles;
    if (psx->timer0_acc >= 4) {
        psx->timer0 = (psx->timer0 + psx->timer0_acc / 4) & 0xFFFFu;
        psx->timer0_acc %= 4;
    }
    psx->timer1_acc += cycles;
    if (psx->timer1_acc >= 2153) {
        psx->timer1 = (psx->timer1 + psx->timer1_acc / 2153) & 0xFFFFu;
        psx->timer1_acc %= 2153;
    }
    psx->timer2_acc += cycles;
    if (psx->timer2_acc >= 8) {
        psx->timer2 = (psx->timer2 + psx->timer2_acc / 8) & 0xFFFFu;
        psx->timer2_acc %= 8;
    }

    /* 1. Update CD-ROM second response timer */
    if (psx->cd_irq_second > 0 && psx->cd_irq_timer > 0) {
        if (psx->cd_irq_timer > cycles) psx->cd_irq_timer -= cycles;
        else psx->cd_irq_timer = 0;
    }

    /* Deliver delayed second response if timer reached 0 and no IRQ is pending */
    if (psx->cd_irq_pending == 0 && psx->cd_irq_second > 0 && psx->cd_irq_timer == 0) {
        for (u32 i = 0; i < psx->cd_resp_second_len; i++)
            psx->cd_resp_buf[i] = psx->cd_resp_second[i];
        psx->cd_resp_len = psx->cd_resp_second_len;
        psx->cd_resp_pos = 0;
        psx->cd_irq_pending = psx->cd_irq_second;
        psx->cd_irq_second = 0;
        psx->cd_seek_pending = 0;
        psx->i_stat |= (1u << 2);
    }

    if (psx->cd_seek_pending && psx->cd_irq_pending == 0 && psx->cd_read_after_seek) {
        psx->cd_tick_acc += cycles;
        if (psx->cd_tick_acc >= 33868u) {
            psx->cd_tick_acc = 0;
            psx->cd_seek_pending = 0;
            psx->cd_read_after_seek = 0;
            psx->cd_read_active = 1;
        }
    }

    /* 2. CD reading: deliver sectors at 75Hz or 150Hz */
    if (psx->cd_read_active && !psx->cd_seek_pending && psx->cd_irq_pending == 0) {
        psx->cd_tick_acc += cycles;
        /* 33.8688 MHz / 75 Hz = 451584 cycles per sector (Single Speed) */
        /* 33.8688 MHz / 150 Hz = 225792 cycles per sector (Double Speed) */
        u32 threshold = (psx->cd_mode & 0x80u) ? 225792u : 451584u;
        
        if (psx->cd_tick_acc >= threshold) {
            psx->cd_tick_acc = 0;
            if (psx->cd_fd > 0 && psx->kread_fn && psx->klseek_fn) {
                u32 lba = psx->cd_sector;
                u32 sec_size = psx->cd_sector_size;
                u64 off = (u64)lba * (u64)sec_size;
                NC(psx->G, psx->klseek_fn, (u64)psx->cd_fd, off, 0, 0, 0, 0);
                
                u8 *buf = psx->cd_sector_buf;
                if (sec_size == 2048) {
                    /* ISO image: Synthesize RAW header */
                    buf[0] = 0x00; for(int k=1; k<11; k++) buf[k] = 0xFF; buf[11] = 0x00;
                    u8 mm, ss, ff;
                    psx_lba_to_msf(lba, &mm, &ss, &ff);
                    buf[12] = mm; buf[13] = ss; buf[14] = ff;
                    buf[15] = 0x02;
                    for(int k=16; k<24; k++) buf[k] = 0;
                    NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)(buf + 24), 2048, 0, 0, 0);
                    for(int k=2072; k<2352; k++) buf[k] = 0;
                } else {
                    NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)buf, 2352, 0, 0, 0);
                }

                {
                    u8 file = 0, channel = 0, submode = 0, coding = 0;
                    psx_cd_get_subheader(psx, &file, &channel, &submode, &coding);
                    psx->cd_last_file = file;
                    psx->cd_last_channel = channel;
                    psx->cd_last_submode = submode;
                    psx->cd_last_coding = coding;
                }

                psx->cd_sector++;

                /* XA audio sectors inside interleaved STR/XA streams must not be
                 * fed to the CPU data FIFO. Real hardware routes them to the XA
                 * decoder/SPU path when XA is enabled. Delivering them as plain
                 * data corrupts FMV/MDEC streams that are multiplexed 7/8 video
                 * + 1/8 audio, as in Ridge Racer Type 4's R4.STR. */
                if ((psx->cd_mode & 0x40u) && psx_cd_is_xa_audio_rt(psx)) {
                    psx_cd_fifo_sync_state(psx);
                } else {
                    u32 skip = psx_cd_data_skip(psx);
                    u32 limit = psx_cd_data_limit(psx);
                    if (psx->cd_read_cmd == 0x1Bu)
                        psx_cd_fifo_push(psx, buf + skip, limit);
                    else
                        psx_cd_fifo_replace_single(psx, buf + skip, limit);
                }

                /* A streamed XA audio sector still completes the in-flight read
                 * and raises INT1/Drive Ready status even if no CPU-visible data
                 * is queued from it. Games that interleave video and XA audio
                 * rely on the sector cadence itself, not only on the payload. */
                {
                    u8 rstat = psx_cd_stat(psx);
                    psx_cd_set_resp(psx, 1u, &rstat, 1);
                    if (psx->cd_pause_pending) {
                        psx->cd_pause_pending = 0;
                        psx->cd_read_cmd = 0;
                        psx->cd_read_active = 0;
                        psx->cd_read_after_seek = 0;
                        rstat = psx_cd_stat(psx);
                        psx_cd_set_second_resp(psx, 2u, &rstat, 1, 0);
                    }
                }
            }
        }
    }
}

/* Fire VBLANK interrupt — hardware IRQ check in CPU will take it */
static void psx_bus_vblank(struct psx_system *psx)
{
    psx->i_stat |= 1u;  /* VBLANK (bit 0) */

    /* Update SPU status bit */
    psx->spu_regs[0xD3] |= 0x400; 

    psx->gpu_io.gpustat ^= 0x80000000u;
}


u32  psx_bus_read32 (struct psx_system *p, u32 a)      { return psx_bus_read(p, a, 4); }
void psx_bus_write32(struct psx_system *p, u32 a, u32 v){ psx_bus_write(p, a, v, 4); }


void psx_spu_render(struct psx_system *psx, s16 *out, u32 count) {
    u16 spucnt = psx->spu_regs[0xD5];
    /* Master enable bit 15 (1=On), Mute bit 14 (0=Mute, 1=Unmute) */
    if (!(spucnt & 0x8000) || !(spucnt & 0x4000)) {
        if (!out) return;
        for (u32 i = 0; i < count * 2; i++) out[i] = 0;
        return;
    }

    s32 master_l = (s16)psx->spu_regs[0xC0];
    s32 master_r = (s16)psx->spu_regs[0xC1];

    for (u32 i = 0; i < count; i++) {
        s32 mix_l = 0, mix_r = 0;
        for (int v = 0; v < 24; v++) {
            if (!(psx->spu_voice_on & (1u << v))) continue;
            
            u32 idx = psx->spu_voice_idx[v];
            s32 sample = psx->spu_voice_samples[v][idx];
            
            s32 vol_l = (s16)psx->spu_regs[v*8 + 0];
            s32 vol_r = (s16)psx->spu_regs[v*8 + 1];
            
            mix_l += (sample * vol_l) >> 15;
            mix_r += (sample * vol_r) >> 15;

            /* Advance phase */
            u32 pitch = psx->spu_regs[v*8 + 2];
            u32 step = pitch << 4; /* pitch 0x1000 = 1.0 = step 0x10000 */
            psx->spu_voice_phase[v] += step;
            
            while (psx->spu_voice_phase[v] >= 0x10000) {
                psx->spu_voice_phase[v] -= 0x10000;
                psx->spu_voice_idx[v]++;
                if (psx->spu_voice_idx[v] >= 28) {
                    psx->spu_voice_idx[v] = 0;
                    u8 flags = psx->spu_ram[(psx->spu_voice_addr[v] + 1) & 0x7FFFFu];
                    if (flags & 1) { /* End bit */
                        psx->spu_voice_on &= ~(1u << v);
                        break;
                    }
                    psx->spu_voice_addr[v] = (psx->spu_voice_addr[v] + 16) & 0x7FFFFu;
                    psx_spu_decode_block(psx, v);
                }
            }
        }
        
        /* Apply master volume, then a small gain boost for debugging audio */
        s32 final_l = ((mix_l * master_l) >> 15);
        s32 final_r = ((mix_r * master_r) >> 15);
        final_l = (final_l * 0x3FFF) >> 12;
        final_r = (final_r * 0x3FFF) >> 12;

        if (final_l > 32767) final_l = 32767; else if (final_l < -32768) final_l = -32768;
        if (final_r > 32767) final_r = 32767; else if (final_r < -32768) final_r = -32768;
        
        if (out) {
            out[i*2 + 0] = (s16)final_l;
            out[i*2 + 1] = (s16)final_r;
        }
    }
}

#endif /* PSX_BUS_C */
