#ifndef PSX_GTE_C
#define PSX_GTE_C

/* ════════════════════════════════════════════════════════════════════
 * PSX GTE (COP2) – Implementation
 * ════════════════════════════════════════════════════════════════════ */

/* ── Clamp helpers ─────────────────────────────────────────────────── */
static s32 gte_clamp_s32(s64 v) {
    if (v >  0x7FFFFFFFll) return  0x7FFFFFFF;
    if (v < -0x80000000ll) return (s32)0x80000000u;
    return (s32)v;
}
static s32 gte_clamp_ir(s64 v, int lm) {
    s32 lo = lm ? 0 : -32768;
    if (v > 32767) return 32767;
    if (v < lo)    return lo;
    return (s32)v;
}
static u32 gte_clamp_sz(s64 v) {
    if (v < 0)      return 0;
    if (v > 0xFFFF) return 0xFFFF;
    return (u32)v;
}
static s32 gte_clamp_sx(s64 v) {
    if (v >  1023) return  1023;
    if (v < -1024) return -1024;
    return (s32)v;
}
static s32 gte_clamp_ir0(s64 v) {
    if (v > 4095) return 4095;
    if (v < 0)    return 0;
    return (s32)v;
}
static u32 gte_clamp_col(s64 v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (u32)v;
}

/* ── Control register accessors ─────────────────────────────────────── */
static s32 gte_clo(struct psx_system *psx, u32 r) { return (s32)(s16)(psx->gte_ctrl[r] & 0xFFFFu); }
static s32 gte_chi(struct psx_system *psx, u32 r) { return (s32)(s16)(psx->gte_ctrl[r] >> 16); }
static s32 gte_cs(struct psx_system *psx, u32 r)  { return (s32)psx->gte_ctrl[r]; }
static s32 gte_m11(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 0u] & 0xFFFFu); }
static s32 gte_m12(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 0u] >> 16); }
static s32 gte_m13(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 1u] & 0xFFFFu); }
static s32 gte_m21(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 1u] >> 16); }
static s32 gte_m22(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 2u] & 0xFFFFu); }
static s32 gte_m23(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 2u] >> 16); }
static s32 gte_m31(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 3u] & 0xFFFFu); }
static s32 gte_m32(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 3u] >> 16); }
static s32 gte_m33(struct psx_system *psx, u32 b) { return (s32)(s16)(psx->gte_ctrl[b + 4u] & 0xFFFFu); }

/* ── Data register accessors ─────────────────────────────────────────── */
static s32 gte_vx(struct psx_system *psx, u32 v) { return (s32)(s16)(psx->gte_data[v*2u] & 0xFFFFu); }
static s32 gte_vy(struct psx_system *psx, u32 v) { return (s32)(s16)(psx->gte_data[v*2u] >> 16); }
static s32 gte_vz(struct psx_system *psx, u32 v) { return (s32)(s16)(psx->gte_data[v*2u+1u] & 0xFFFFu); }

static void gte_fifo_rgb(struct psx_system *psx, u32 rgb)
{
    psx->gte_data[20] = psx->gte_data[21];
    psx->gte_data[21] = psx->gte_data[22];
    psx->gte_data[22] = rgb;
}

static void gte_set_mac_ir(struct psx_system *psx, s64 m1, s64 m2, s64 m3, int sf, int lm)
{
    int sh = sf ? 12 : 0;
    psx->gte_data[25] = (u32)gte_clamp_s32(m1 >> sh);
    psx->gte_data[26] = (u32)gte_clamp_s32(m2 >> sh);
    psx->gte_data[27] = (u32)gte_clamp_s32(m3 >> sh);
    psx->gte_data[9]  = (u32)gte_clamp_ir(m1 >> 12, lm);
    psx->gte_data[10] = (u32)gte_clamp_ir(m2 >> 12, lm);
    psx->gte_data[11] = (u32)gte_clamp_ir(m3 >> 12, lm);
}

static void gte_mul_mat_vec(struct psx_system *psx, u32 base, s32 x, s32 y, s32 z,
                            s64 addx, s64 addy, s64 addz, int sf, int lm)
{
    s64 m1 = addx + (s64)gte_m11(psx, base) * x + (s64)gte_m12(psx, base) * y + (s64)gte_m13(psx, base) * z;
    s64 m2 = addy + (s64)gte_m21(psx, base) * x + (s64)gte_m22(psx, base) * y + (s64)gte_m23(psx, base) * z;
    s64 m3 = addz + (s64)gte_m31(psx, base) * x + (s64)gte_m32(psx, base) * y + (s64)gte_m33(psx, base) * z;
    gte_set_mac_ir(psx, m1, m2, m3, sf, lm);
}

static void gte_compute_light_color(struct psx_system *psx, s32 vx, s32 vy, s32 vz,
                                    int sf, int lm, u32 *out_rgb)
{
    s32 bk_r = gte_cs(psx, 13);
    s32 bk_g = gte_cs(psx, 14);
    s32 bk_b = gte_cs(psx, 15);
    gte_mul_mat_vec(psx, 8u, vx, vy, vz, 0, 0, 0, sf, lm);
    gte_mul_mat_vec(psx, 16u,
                    (s32)(s16)psx->gte_data[9],
                    (s32)(s16)psx->gte_data[10],
                    (s32)(s16)psx->gte_data[11],
                    (s64)bk_r << 12, (s64)bk_g << 12, (s64)bk_b << 12, sf, lm);
    *out_rgb =
        gte_clamp_col((s32)(s16)psx->gte_data[9] >> 4) |
        (gte_clamp_col((s32)(s16)psx->gte_data[10] >> 4) << 8) |
        (gte_clamp_col((s32)(s16)psx->gte_data[11] >> 4) << 16) |
        (psx->gte_data[6] & 0xFF000000u);
}

static u32 gte_modulate_rgb(struct psx_system *psx, u32 rgb)
{
    u32 cr = (psx->gte_data[6] >> 0) & 0xFFu;
    u32 cg = (psx->gte_data[6] >> 8) & 0xFFu;
    u32 cb = (psx->gte_data[6] >> 16) & 0xFFu;
    u32 r = (rgb >> 0) & 0xFFu;
    u32 g = (rgb >> 8) & 0xFFu;
    u32 b = (rgb >> 16) & 0xFFu;
    r = (r * cr + 127u) / 255u;
    g = (g * cg + 127u) / 255u;
    b = (b * cb + 127u) / 255u;
    return r | (g << 8) | (b << 16) | (psx->gte_data[6] & 0xFF000000u);
}

static u32 gte_depth_cue_rgb(struct psx_system *psx, u32 rgb)
{
    s32 r = (s32)((rgb >> 0) & 0xFFu);
    s32 g = (s32)((rgb >> 8) & 0xFFu);
    s32 b = (s32)((rgb >> 16) & 0xFFu);
    s32 fr = gte_cs(psx, 21);
    s32 fg = gte_cs(psx, 22);
    s32 fb = gte_cs(psx, 23);
    s32 ir0 = (s32)(s16)psx->gte_data[8];
    s64 m1 = ((s64)r << 12) + (s64)ir0 * (fr - r);
    s64 m2 = ((s64)g << 12) + (s64)ir0 * (fg - g);
    s64 m3 = ((s64)b << 12) + (s64)ir0 * (fb - b);
    return gte_clamp_col(m1 >> 12) |
           (gte_clamp_col(m2 >> 12) << 8) |
           (gte_clamp_col(m3 >> 12) << 16) |
           (psx->gte_data[6] & 0xFF000000u);
}

/* ── FIFO push helpers ──────────────────────────────────────────────── */
static void gte_push_sz(struct psx_system *psx, u32 v) {
    psx->gte_data[16] = psx->gte_data[17];
    psx->gte_data[17] = psx->gte_data[18];
    psx->gte_data[18] = psx->gte_data[19];
    psx->gte_data[19] = v & 0xFFFFu;
}
static void gte_push_sxy(struct psx_system *psx, s32 sx, s32 sy) {
    psx->gte_data[12] = psx->gte_data[13];
    psx->gte_data[13] = psx->gte_data[14];
    u32 packed = (u32)(u16)(u32)sx | ((u32)(u16)(u32)sy << 16);
    psx->gte_data[14] = packed;
    psx->gte_data[15] = packed;
}

/* ── Hardware-approximate H/SZ divide ──────────────────────────────── */
static u32 gte_divide(u32 h, u32 sz) {
    if (sz == 0u) return 0x1FFFFu;
    if (sz <= (h / 2u)) return 0x1FFFFu;
    u64 r = ((u64)h << 16) / (u64)sz;
    return r > 0x1FFFFu ? 0x1FFFFu : (u32)r;
}

/* ── Register accessors (External) ─────────────────────────────────── */
u32 psx_gte_read_data(struct psx_system *psx, u32 reg) {
    reg &= 31u;
    if (reg == 29u) return psx->gte_data[28];
    if (reg == 31u) {
        u32 lzcs = psx->gte_data[30];
        u32 n = 0;
        if (lzcs & 0x80000000u) lzcs = ~lzcs;
        while (n < 32u && !(lzcs & 0x80000000u)) {
            lzcs <<= 1; n++;
        }
        psx->gte_data[31] = n;
    }
    return psx->gte_data[reg];
}
void psx_gte_write_data(struct psx_system *psx, u32 reg, u32 val) {
    reg &= 31u;
    if (reg == 15u) {
        psx->gte_data[12] = psx->gte_data[13];
        psx->gte_data[13] = psx->gte_data[14];
        psx->gte_data[14] = val; psx->gte_data[15] = val;
        return;
    }
    if (reg == 28u || reg == 29u) {
        psx->gte_data[28] = val & 0x7FFFu;
        psx->gte_data[29] = psx->gte_data[28];
        return;
    }
    psx->gte_data[reg] = val;
}

/* ── RTPS Logic ────────────────────────────────────────────────────── */
static void gte_rtps_v(struct psx_system *psx, u32 v, int sf, int lm, int last) {
    s32 r11=gte_clo(psx,0), r12=gte_chi(psx,0), r13=gte_clo(psx,1);
    s32 r21=gte_chi(psx,1), r22=gte_clo(psx,2), r23=gte_chi(psx,2);
    s32 r31=gte_clo(psx,3), r32=gte_chi(psx,3), r33=gte_clo(psx,4);
    s32 trx=gte_cs(psx,5), try_=gte_cs(psx,6), trz=gte_cs(psx,7);
    s32 vx=gte_vx(psx,v), vy=gte_vy(psx,v), vz=gte_vz(psx,v);

    s64 m1 = ((s64)trx<<12) + (s64)r11*vx + (s64)r12*vy + (s64)r13*vz;
    s64 m2 = ((s64)try_<<12) + (s64)r21*vx + (s64)r22*vy + (s64)r23*vz;
    s64 m3 = ((s64)trz<<12) + (s64)r31*vx + (s64)r32*vy + (s64)r33*vz;

    int sh = sf ? 12 : 0;
    psx->gte_data[25] = (u32)gte_clamp_s32(m1 >> sh);
    psx->gte_data[26] = (u32)gte_clamp_s32(m2 >> sh);
    psx->gte_data[27] = (u32)gte_clamp_s32(m3 >> sh);
    psx->gte_data[9]  = (u32)gte_clamp_ir(m1 >> 12, lm);
    psx->gte_data[10] = (u32)gte_clamp_ir(m2 >> 12, lm);
    psx->gte_data[11] = (u32)gte_clamp_ir(m3 >> 12, lm);

    u32 sz3 = gte_clamp_sz(m3 >> 12);
    gte_push_sz(psx, sz3);

    u32 h   = (u16)psx->gte_ctrl[26];
    u32 div = gte_divide(h, sz3);
    s32 sx = gte_clamp_sx((gte_cs(psx, 24) + (s64)(s32)psx->gte_data[9] * div) >> 16);
    s32 sy = gte_clamp_sx((gte_cs(psx, 25) + (s64)(s32)psx->gte_data[10] * div) >> 16);
    gte_push_sxy(psx, sx, sy);

    if (last) {
        s64 mac0 = gte_cs(psx, 28) + (s64)(s16)psx->gte_ctrl[27] * (s32)div;
        psx->gte_data[24] = (u32)gte_clamp_s32(mac0);
        psx->gte_data[8]  = (u32)gte_clamp_ir0(mac0 >> 12);
    }
}

/* ── Other Opcodes ─────────────────────────────────────────────────── */
static void gte_nclip(struct psx_system *psx) {
    s32 sx0=(s32)(s16)psx->gte_data[12], sy0=(s32)(s16)(psx->gte_data[12]>>16);
    s32 sx1=(s32)(s16)psx->gte_data[13], sy1=(s32)(s16)(psx->gte_data[13]>>16);
    s32 sx2=(s32)(s16)psx->gte_data[14], sy2=(s32)(s16)(psx->gte_data[14]>>16);
    s64 mac0 = (s64)sx0*(sy1-sy2) + (s64)sx1*(sy2-sy0) + (s64)sx2*(sy0-sy1);
    psx->gte_data[24] = (u32)gte_clamp_s32(mac0);
}

static void gte_avsz3(struct psx_system *psx) {
    s32 zsf3 = (s16)(psx->gte_ctrl[29] & 0xFFFFu);
    s64 mac0 = (s64)zsf3 * ((s32)psx->gte_data[17] + (s32)psx->gte_data[18] + (s32)psx->gte_data[19]);
    psx->gte_data[24] = (u32)gte_clamp_s32(mac0);
    psx->gte_data[7]  = gte_clamp_sz(mac0 >> 12);
}

static void gte_avsz4(struct psx_system *psx) {
    s32 zsf4 = (s16)(psx->gte_ctrl[30] & 0xFFFFu);
    s64 mac0 = (s64)zsf4 * ((s32)psx->gte_data[16] + (s32)psx->gte_data[17] + (s32)psx->gte_data[18] + (s32)psx->gte_data[19]);
    psx->gte_data[24] = (u32)gte_clamp_s32(mac0);
    psx->gte_data[7]  = gte_clamp_sz(mac0 >> 12);
}

static void gte_mvmva(struct psx_system *psx, u32 cmd) {
    int sf = (cmd >> 19) & 1, lm = (cmd >> 10) & 1;
    u32 mx = (cmd >> 17) & 3u;
    u32 v_sel = (cmd >> 15) & 3u;
    u32 cv = (cmd >> 13) & 3u;
    u32 base = 0u;
    s32 vx, vy, vz;
    s64 addx = 0, addy = 0, addz = 0;
    s64 m1, m2, m3;

    switch (mx) {
    case 0: base = 0u;  break; /* Rotation */
    case 1: base = 8u;  break; /* Light */
    case 2: base = 16u; break; /* Color */
    default: base = 0u; break; /* Reserved -> treat as rotation */
    }

    if (v_sel < 3u) {
        vx = gte_vx(psx, v_sel);
        vy = gte_vy(psx, v_sel);
        vz = gte_vz(psx, v_sel);
    } else {
        vx = (s32)(s16)psx->gte_data[9];
        vy = (s32)(s16)psx->gte_data[10];
        vz = (s32)(s16)psx->gte_data[11];
    }

    switch (cv) {
    case 0:
        addx = (s64)gte_cs(psx, 5) << 12;
        addy = (s64)gte_cs(psx, 6) << 12;
        addz = (s64)gte_cs(psx, 7) << 12;
        break;
    case 1:
        addx = (s64)gte_cs(psx, 13) << 12;
        addy = (s64)gte_cs(psx, 14) << 12;
        addz = (s64)gte_cs(psx, 15) << 12;
        break;
    case 2:
        /* Hardware bug: FC is not added correctly in MVMVA. */
        m1 = (s64)gte_m13(psx, base) * vz;
        m2 = (s64)gte_m23(psx, base) * vz;
        m3 = (s64)gte_m33(psx, base) * vz;
        gte_set_mac_ir(psx, m1, m2, m3, sf, lm);
        return;
    default:
        break; /* None */
    }

    m1 = addx + (s64)gte_m11(psx, base) * vx + (s64)gte_m12(psx, base) * vy + (s64)gte_m13(psx, base) * vz;
    m2 = addy + (s64)gte_m21(psx, base) * vx + (s64)gte_m22(psx, base) * vy + (s64)gte_m23(psx, base) * vz;
    m3 = addz + (s64)gte_m31(psx, base) * vx + (s64)gte_m32(psx, base) * vy + (s64)gte_m33(psx, base) * vz;
    gte_set_mac_ir(psx, m1, m2, m3, sf, lm);
}

static void gte_dpcs(struct psx_system *psx, int sf, int lm) {
    /* Depth Cue Color Single: RGB2 = RGBC + IR0 * (FC - RGBC) */
    s32 r=(psx->gte_data[6]>>0)&0xFF, g=(psx->gte_data[6]>>8)&0xFF, b=(psx->gte_data[6]>>16)&0xFF;
    s32 br=gte_cs(psx,21), bg=gte_cs(psx,22), bb=gte_cs(psx,23);
    s32 ir0=(s32)(s16)psx->gte_data[8];
    s64 m1=((s64)r<<12)+(s64)ir0*(br-r);
    s64 m2=((s64)g<<12)+(s64)ir0*(bg-g);
    s64 m3=((s64)b<<12)+(s64)ir0*(bb-b);
    int sh = sf ? 12 : 0;
    psx->gte_data[25]=gte_clamp_s32(m1>>sh); psx->gte_data[26]=gte_clamp_s32(m2>>sh); psx->gte_data[27]=gte_clamp_s32(m3>>sh);
    psx->gte_data[9]=gte_clamp_ir(m1>>12,lm); psx->gte_data[10]=gte_clamp_ir(m2>>12,lm); psx->gte_data[11]=gte_clamp_ir(m3>>12,lm);
    gte_fifo_rgb(psx,
        gte_clamp_col(m1>>12)|(gte_clamp_col(m2>>12)<<8)|(gte_clamp_col(m3>>12)<<16)|(psx->gte_data[6]&0xFF000000u));
}

static void gte_dpct(struct psx_system *psx, int sf, int lm) {
    /* Triple version: apply depth cue to RGB FIFO 0..2 in order. */
    for (int i = 0; i < 3; i++) {
        u32 src = psx->gte_data[20 + (u32)i];
        s32 r = (src >> 0) & 0xFFu;
        s32 g = (src >> 8) & 0xFFu;
        s32 b = (src >> 16) & 0xFFu;
        s32 br = gte_cs(psx, 21), bg = gte_cs(psx, 22), bb = gte_cs(psx, 23);
        s32 ir0 = (s32)(s16)psx->gte_data[8];
        s64 m1 = ((s64)r << 12) + (s64)ir0 * (br - r);
        s64 m2 = ((s64)g << 12) + (s64)ir0 * (bg - g);
        s64 m3 = ((s64)b << 12) + (s64)ir0 * (bb - b);
        int sh = sf ? 12 : 0;
        psx->gte_data[25] = gte_clamp_s32(m1 >> sh);
        psx->gte_data[26] = gte_clamp_s32(m2 >> sh);
        psx->gte_data[27] = gte_clamp_s32(m3 >> sh);
        psx->gte_data[9] = gte_clamp_ir(m1 >> 12, lm);
        psx->gte_data[10] = gte_clamp_ir(m2 >> 12, lm);
        psx->gte_data[11] = gte_clamp_ir(m3 >> 12, lm);
        gte_fifo_rgb(psx,
            gte_clamp_col(m1 >> 12) |
            (gte_clamp_col(m2 >> 12) << 8) |
            (gte_clamp_col(m3 >> 12) << 16) |
            (src & 0xFF000000u));
    }
}

static void gte_ncs(struct psx_system *psx, int sf, int lm) {
    u32 rgb;
    gte_compute_light_color(psx, gte_vx(psx, 0), gte_vy(psx, 0), gte_vz(psx, 0), sf, lm, &rgb);
    gte_fifo_rgb(psx, rgb);
}

static void gte_nct(struct psx_system *psx, int sf, int lm) {
    for (u32 i = 0; i < 3u; i++) {
        u32 rgb;
        gte_compute_light_color(psx, gte_vx(psx, i), gte_vy(psx, i), gte_vz(psx, i), sf, lm, &rgb);
        gte_fifo_rgb(psx, rgb);
    }
}

static void gte_nccs(struct psx_system *psx, int sf, int lm)
{
    u32 rgb;
    gte_compute_light_color(psx, gte_vx(psx, 0), gte_vy(psx, 0), gte_vz(psx, 0), sf, lm, &rgb);
    gte_fifo_rgb(psx, gte_modulate_rgb(psx, rgb));
}

static void gte_ncct(struct psx_system *psx, int sf, int lm)
{
    for (u32 i = 0; i < 3u; i++) {
        u32 rgb;
        gte_compute_light_color(psx, gte_vx(psx, i), gte_vy(psx, i), gte_vz(psx, i), sf, lm, &rgb);
        gte_fifo_rgb(psx, gte_modulate_rgb(psx, rgb));
    }
}

static void gte_ncds(struct psx_system *psx, int sf, int lm)
{
    u32 rgb;
    gte_compute_light_color(psx, gte_vx(psx, 0), gte_vy(psx, 0), gte_vz(psx, 0), sf, lm, &rgb);
    gte_fifo_rgb(psx, gte_depth_cue_rgb(psx, gte_modulate_rgb(psx, rgb)));
}

static void gte_ncdt(struct psx_system *psx, int sf, int lm)
{
    for (u32 i = 0; i < 3u; i++) {
        u32 rgb;
        gte_compute_light_color(psx, gte_vx(psx, i), gte_vy(psx, i), gte_vz(psx, i), sf, lm, &rgb);
        gte_fifo_rgb(psx, gte_depth_cue_rgb(psx, gte_modulate_rgb(psx, rgb)));
    }
}

static void gte_cc(struct psx_system *psx, int sf, int lm)
{
    u32 rgb;
    gte_compute_light_color(psx,
                            (s32)(s16)psx->gte_data[9],
                            (s32)(s16)psx->gte_data[10],
                            (s32)(s16)psx->gte_data[11],
                            sf, lm, &rgb);
    gte_fifo_rgb(psx, gte_modulate_rgb(psx, rgb));
}

static void gte_intpl(struct psx_system *psx, int sf, int lm)
{
    u32 rgb =
        gte_clamp_col((s32)(s16)psx->gte_data[9] >> 4) |
        (gte_clamp_col((s32)(s16)psx->gte_data[10] >> 4) << 8) |
        (gte_clamp_col((s32)(s16)psx->gte_data[11] >> 4) << 16) |
        (psx->gte_data[6] & 0xFF000000u);
    (void)sf; (void)lm;
    gte_fifo_rgb(psx, gte_depth_cue_rgb(psx, rgb));
}

static void gte_cdp(struct psx_system *psx, int sf, int lm)
{
    u32 rgb;
    gte_compute_light_color(psx,
                            (s32)(s16)psx->gte_data[9],
                            (s32)(s16)psx->gte_data[10],
                            (s32)(s16)psx->gte_data[11],
                            sf, lm, &rgb);
    gte_fifo_rgb(psx, gte_depth_cue_rgb(psx, gte_modulate_rgb(psx, rgb)));
}

static void gte_dcpl(struct psx_system *psx, int sf, int lm)
{
    u32 rgb =
        gte_clamp_col((s32)(s16)psx->gte_data[9] >> 4) |
        (gte_clamp_col((s32)(s16)psx->gte_data[10] >> 4) << 8) |
        (gte_clamp_col((s32)(s16)psx->gte_data[11] >> 4) << 16) |
        (psx->gte_data[6] & 0xFF000000u);
    gte_fifo_rgb(psx, gte_depth_cue_rgb(psx, gte_modulate_rgb(psx, rgb)));
    (void)sf; (void)lm;
}

static void gte_sqr(struct psx_system *psx, int sf, int lm)
{
    s64 ir1 = (s32)(s16)psx->gte_data[9];
    s64 ir2 = (s32)(s16)psx->gte_data[10];
    s64 ir3 = (s32)(s16)psx->gte_data[11];
    gte_set_mac_ir(psx, ir1 * ir1, ir2 * ir2, ir3 * ir3, sf, lm);
}

static void gte_op(struct psx_system *psx, int sf, int lm)
{
    s32 d1 = gte_m11(psx, 0);
    s32 d2 = gte_m22(psx, 0);
    s32 d3 = gte_m33(psx, 0);
    s32 ir1 = (s32)(s16)psx->gte_data[9];
    s32 ir2 = (s32)(s16)psx->gte_data[10];
    s32 ir3 = (s32)(s16)psx->gte_data[11];
    s64 m1 = (s64)d2 * ir3 - (s64)d3 * ir2;
    s64 m2 = (s64)d3 * ir1 - (s64)d1 * ir3;
    s64 m3 = (s64)d1 * ir2 - (s64)d2 * ir1;
    gte_set_mac_ir(psx, m1, m2, m3, sf, lm);
}

static void gte_gpf(struct psx_system *psx, int sf, int lm)
{
    s64 ir0 = (s32)(s16)psx->gte_data[8];
    s64 ir1 = (s32)(s16)psx->gte_data[9];
    s64 ir2 = (s32)(s16)psx->gte_data[10];
    s64 ir3 = (s32)(s16)psx->gte_data[11];
    gte_set_mac_ir(psx, ir1 * ir0, ir2 * ir0, ir3 * ir0, sf, lm);
    gte_fifo_rgb(psx,
        gte_clamp_col((s32)(s16)psx->gte_data[9] >> 4) |
        (gte_clamp_col((s32)(s16)psx->gte_data[10] >> 4) << 8) |
        (gte_clamp_col((s32)(s16)psx->gte_data[11] >> 4) << 16) |
        (psx->gte_data[6] & 0xFF000000u));
}

static void gte_gpl(struct psx_system *psx, int sf, int lm)
{
    s64 ir0 = (s32)(s16)psx->gte_data[8];
    s64 ir1 = (s32)(s16)psx->gte_data[9];
    s64 ir2 = (s32)(s16)psx->gte_data[10];
    s64 ir3 = (s32)(s16)psx->gte_data[11];
    s64 mac1 = (s32)psx->gte_data[25];
    s64 mac2 = (s32)psx->gte_data[26];
    s64 mac3 = (s32)psx->gte_data[27];
    gte_set_mac_ir(psx, (mac1 << (sf ? 12 : 0)) + ir1 * ir0,
                        (mac2 << (sf ? 12 : 0)) + ir2 * ir0,
                        (mac3 << (sf ? 12 : 0)) + ir3 * ir0, sf, lm);
    gte_fifo_rgb(psx,
        gte_clamp_col((s32)(s16)psx->gte_data[9] >> 4) |
        (gte_clamp_col((s32)(s16)psx->gte_data[10] >> 4) << 8) |
        (gte_clamp_col((s32)(s16)psx->gte_data[11] >> 4) << 16) |
        (psx->gte_data[6] & 0xFF000000u));
}

void psx_gte_dispatch(struct psx_system *psx, u32 cmd) {
    int sf = (cmd >> 19) & 1, lm = (cmd >> 10) & 1;
    u32 fn = cmd & 0x3Fu;
    switch(fn) {
        case 0x01: gte_rtps_v(psx, 0, sf, lm, 1); break;
        case 0x06: gte_nclip(psx); break;
        case 0x0C: gte_op(psx, sf, lm); break;
        case 0x11: gte_intpl(psx, sf, lm); break;
        case 0x10: gte_dpcs(psx, sf, lm); break;
        case 0x14: gte_cdp(psx, sf, lm); break;
        case 0x2A: gte_dpct(psx, sf, lm); break;
        case 0x13: gte_ncds(psx, sf, lm); break;
        case 0x16: gte_ncdt(psx, sf, lm); break;
        case 0x1B: gte_nccs(psx, sf, lm); break;
        case 0x1C: gte_cc(psx, sf, lm); break;
        case 0x1E: gte_ncs(psx, sf, lm); break;
        case 0x20: gte_nct(psx, sf, lm); break;
        case 0x28: gte_sqr(psx, sf, lm); break;
        case 0x29: gte_dcpl(psx, sf, lm); break;
        case 0x12: gte_mvmva(psx, cmd); break;
        case 0x2D: gte_avsz3(psx); break;
        case 0x2E: gte_avsz4(psx); break;
        case 0x30: gte_rtps_v(psx,0,sf,lm,0); gte_rtps_v(psx,1,sf,lm,0); gte_rtps_v(psx,2,sf,lm,1); break;
        case 0x3D: gte_gpf(psx, sf, lm); break;
        case 0x3E: gte_gpl(psx, sf, lm); break;
        case 0x3F: gte_ncct(psx, sf, lm); break;
        default: break;
    }
}

#endif
