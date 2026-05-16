#include "psx_types.h"

/* ── PSX SPU Constants ────────────────────────────────────────── */
static const s32 f1[] = { 0, 60, 115, 98, 122 };
static const s32 f2[] = { 0, 0, -52, -55, -60 };

static const s16 psx_spu_gauss[512] = {
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x001,
    0x001, 0x001, 0x001, 0x002, 0x002, 0x002, 0x003, 0x003,
    0x003, 0x004, 0x004, 0x005, 0x005, 0x006, 0x007, 0x007,
    0x008, 0x009, 0x009, 0x00A, 0x00B, 0x00C, 0x00D, 0x00E,
    0x00F, 0x010, 0x011, 0x012, 0x013, 0x015, 0x016, 0x018,
    0x019, 0x01B, 0x01C, 0x01E, 0x020, 0x021, 0x023, 0x025,
    0x027, 0x029, 0x02C, 0x02E, 0x030, 0x033, 0x035, 0x038,
    0x03A, 0x03D, 0x040, 0x043, 0x046, 0x049, 0x04D, 0x050,
    0x054, 0x057, 0x05B, 0x05F, 0x063, 0x067, 0x06B, 0x06F,
    0x074, 0x078, 0x07D, 0x082, 0x087, 0x08C, 0x091, 0x096,
    0x09C, 0x0A1, 0x0A7, 0x0AD, 0x0B3, 0x0BA, 0x0C0, 0x0C7,
    0x0CD, 0x0D4, 0x0DB, 0x0E3, 0x0EA, 0x0F2, 0x0FA, 0x101,
    0x10A, 0x112, 0x11B, 0x123, 0x12C, 0x135, 0x13F, 0x148,
    0x152, 0x15C, 0x166, 0x171, 0x17B, 0x186, 0x191, 0x19C,
    0x1A8, 0x1B4, 0x1C0, 0x1CC, 0x1D9, 0x1E5, 0x1F2, 0x200,
    0x20D, 0x21B, 0x229, 0x237, 0x246, 0x255, 0x264, 0x273,
    0x283, 0x293, 0x2A3, 0x2B4, 0x2C4, 0x2D6, 0x2E7, 0x2F9,
    0x30B, 0x31D, 0x330, 0x343, 0x356, 0x36A, 0x37E, 0x392,
    0x3A7, 0x3BC, 0x3D1, 0x3E7, 0x3FC, 0x413, 0x42A, 0x441,
    0x458, 0x470, 0x488, 0x4A0, 0x4B9, 0x4D2, 0x4EC, 0x506,
    0x520, 0x53B, 0x556, 0x572, 0x58E, 0x5AA, 0x5C7, 0x5E4,
    0x601, 0x61F, 0x63E, 0x65C, 0x67C, 0x69B, 0x6BB, 0x6DC,
    0x6FD, 0x71E, 0x740, 0x762, 0x784, 0x7A7, 0x7CB, 0x7EF,
    0x813, 0x838, 0x85D, 0x883, 0x8A9, 0x8D0, 0x8F7, 0x91E,
    0x946, 0x96F, 0x998, 0x9C1, 0x9EB, 0xA16, 0xA40, 0xA6C,
    0xA98, 0xAC4, 0xAF1, 0xB1E, 0xB4C, 0xB7A, 0xBA9, 0xBD8,
    0xC07, 0xC38, 0xC68, 0xC99, 0xCCB, 0xCFD, 0xD30, 0xD63,
    0xD97, 0xDCB, 0xE00, 0xE35, 0xE6B, 0xEA1, 0xED7, 0xF0F,
    0xF46, 0xF7F, 0xFB7, 0xFF1, 0x102A, 0x1065, 0x109F, 0x10DB,
    0x1116, 0x1153, 0x118F, 0x11CD, 0x120B, 0x1249, 0x1288, 0x12C7,
    0x1307, 0x1347, 0x1388, 0x13C9, 0x140B, 0x144D, 0x1490, 0x14D4,
    0x1517, 0x155C, 0x15A0, 0x15E6, 0x162C, 0x1672, 0x16B9, 0x1700,
    0x1747, 0x1790, 0x17D8, 0x1821, 0x186B, 0x18B5, 0x1900, 0x194B,
    0x1996, 0x19E2, 0x1A2E, 0x1A7B, 0x1AC8, 0x1B16, 0x1B64, 0x1BB3,
    0x1C02, 0x1C51, 0x1CA1, 0x1CF1, 0x1D42, 0x1D93, 0x1DE5, 0x1E37,
    0x1E89, 0x1EDC, 0x1F2F, 0x1F82, 0x1FD6, 0x202A, 0x207F, 0x20D4,
    0x2129, 0x217F, 0x21D5, 0x222C, 0x2282, 0x22DA, 0x2331, 0x2389,
    0x23E1, 0x2439, 0x2492, 0x24EB, 0x2545, 0x259E, 0x25F8, 0x2653,
    0x26AD, 0x2708, 0x2763, 0x27BE, 0x281A, 0x2876, 0x28D2, 0x292E,
    0x298B, 0x29E7, 0x2A44, 0x2AA1, 0x2AFF, 0x2B5C, 0x2BBA, 0x2C18,
    0x2C76, 0x2CD4, 0x2D33, 0x2D91, 0x2DF0, 0x2E4F, 0x2EAE, 0x2F0D,
    0x2F6C, 0x2FCC, 0x302B, 0x308B, 0x30EA, 0x314A, 0x31AA, 0x3209,
    0x3269, 0x32C9, 0x3329, 0x3389, 0x33E9, 0x3449, 0x34A9, 0x3509,
    0x3569, 0x35C9, 0x3629, 0x3689, 0x36E8, 0x3748, 0x37A8, 0x3807,
    0x3867, 0x38C6, 0x3926, 0x3985, 0x39E4, 0x3A43, 0x3AA2, 0x3B00,
    0x3B5F, 0x3BBD, 0x3C1B, 0x3C79, 0x3CD7, 0x3D35, 0x3D92, 0x3DEF,
    0x3E4C, 0x3EA9, 0x3F05, 0x3F62, 0x3FBD, 0x4019, 0x4074, 0x40D0,
    0x412A, 0x4185, 0x41DF, 0x4239, 0x4292, 0x42EB, 0x4344, 0x439C,
    0x43F4, 0x444C, 0x44A3, 0x44FA, 0x4550, 0x45A6, 0x45FC, 0x4651,
    0x46A6, 0x46FA, 0x474E, 0x47A1, 0x47F4, 0x4846, 0x4898, 0x48E9,
    0x493A, 0x498A, 0x49D9, 0x4A29, 0x4A77, 0x4AC5, 0x4B13, 0x4B5F,
    0x4BAC, 0x4BF7, 0x4C42, 0x4C8D, 0x4CD7, 0x4D20, 0x4D68, 0x4DB0,
    0x4DF7, 0x4E3E, 0x4E84, 0x4EC9, 0x4F0E, 0x4F52, 0x4F95, 0x4FD7,
    0x5019, 0x505A, 0x509A, 0x50DA, 0x5118, 0x5156, 0x5194, 0x51D0,
    0x520C, 0x5247, 0x5281, 0x52BA, 0x52F3, 0x532A, 0x5361, 0x5397,
    0x53CC, 0x5401, 0x5434, 0x5467, 0x5499, 0x54CA, 0x54FA, 0x5529,
    0x5558, 0x5585, 0x55B2, 0x55DE, 0x5609, 0x5632, 0x565B, 0x5684,
    0x56AB, 0x56D1, 0x56F6, 0x571B, 0x573E, 0x5761, 0x5782, 0x57A3,
    0x57C3, 0x57E2, 0x57FF, 0x581C, 0x5838, 0x5853, 0x586D, 0x5886,
    0x589E, 0x58B5, 0x58CB, 0x58E0, 0x58F4, 0x5907, 0x5919, 0x592A,
    0x593A, 0x5949, 0x5958, 0x5965, 0x5971, 0x597C, 0x5986, 0x598F,
    0x5997, 0x599E, 0x59A4, 0x59A9, 0x59AD, 0x59B0, 0x59B2, 0x59B3
};

static void spu_decode_block(struct psx_system *psx, int v) {
    u32 addr = psx->spu_voice_addr[v] & 0x7FFFFu;
    u8 head = psx->spu_ram[addr];
    int raw_shift = (int)(head & 0xFu);
    int shift = (raw_shift > 12) ? 9 : (12 - raw_shift);
    int filter = (head >> 4) & 0x3u;

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
        s2 = s1; s1 = res;
    }
    psx->spu_voice_last[v][0] = (s16)s1;
    psx->spu_voice_last[v][1] = (s16)s2;
}

static s32 spu_fixed_volume(u16 reg)
{
    u32 raw = reg & 0x7FFFu;
    return (s32)(((s32)(raw << 17)) >> 16);
}

static void spu_envelope_step(s32 *level, u32 *counter, u32 shift, u32 step,
                              int exponential, int decreasing,
                              int phase_negative, int decay_release_mode)
{
    s32 env = *level;
    u32 cnt = *counter & 0x7FFFu;
    s32 env_step = 7 - (s32)(step & 3u);
    u32 counter_inc;
    int all_bits;

    if (decreasing ^ phase_negative)
        env_step = ~env_step;
    if (shift < 11u)
        env_step <<= (11u - shift);

    counter_inc = (shift > 11u) ? (0x8000u >> (shift - 11u)) : 0x8000u;

    if (exponential && !decreasing && env > 0x6000) {
        if (shift < 10u) {
            env_step >>= 2;
        } else if (shift >= 11u) {
            counter_inc >>= 2;
        } else {
            env_step >>= 2;
            counter_inc >>= 2;
        }
    } else if (exponential && decreasing) {
        env_step = (env_step * env) / 0x8000;
    }

    all_bits = decay_release_mode ? (shift == 0x1Fu)
                                  : (((step & 3u) | (shift << 2)) == 0x7Fu);
    if (!all_bits && counter_inc == 0u)
        counter_inc = 1u;

    cnt += counter_inc;
    if ((cnt & 0x8000u) == 0u) {
        *counter = cnt;
        return;
    }

    cnt &= 0x7FFFu;
    env += env_step;

    if (!decreasing) {
        if (env < -0x8000)
            env = -0x8000;
        else if (env > 0x7FFF)
            env = 0x7FFF;
    } else if (phase_negative) {
        if (env < -0x8000)
            env = -0x8000;
        else if (env > 0)
            env = 0;
    } else if (env < 0) {
        env = 0;
    }

    *level = env;
    *counter = cnt;
}

static void spu_volume_tick(u16 reg, s32 *level, u32 *counter)
{
    if (!(reg & 0x8000u)) {
        *level = spu_fixed_volume(reg);
        *counter = 0;
        return;
    }

    spu_envelope_step(level, counter,
                      (reg >> 2) & 0x1Fu,
                      reg & 0x3u,
                      (reg >> 14) & 1u,
                      (reg >> 13) & 1u,
                      (reg >> 12) & 1u,
                      0);
}

static s32 psx_spu_sample_gauss(const struct psx_system *psx, int v)
{
    u32 idx = psx->spu_voice_idx[v];
    u32 frac = (psx->spu_voice_phase[v] >> 8) & 0xFFu;
    s32 oldest, older, old, current;

    if (idx == 0u) {
        oldest = psx->spu_voice_prev[v][0];
        older  = psx->spu_voice_prev[v][1];
        old    = psx->spu_voice_prev[v][2];
        current = psx->spu_voice_samples[v][0];
    } else if (idx == 1u) {
        oldest = psx->spu_voice_prev[v][1];
        older  = psx->spu_voice_prev[v][2];
        old    = psx->spu_voice_samples[v][0];
        current = psx->spu_voice_samples[v][1];
    } else if (idx == 2u) {
        oldest = psx->spu_voice_prev[v][2];
        older  = psx->spu_voice_samples[v][0];
        old    = psx->spu_voice_samples[v][1];
        current = psx->spu_voice_samples[v][2];
    } else {
        oldest = psx->spu_voice_samples[v][idx - 3u];
        older  = psx->spu_voice_samples[v][idx - 2u];
        old    = psx->spu_voice_samples[v][idx - 1u];
        current = psx->spu_voice_samples[v][idx];
    }

    return ((psx_spu_gauss[0x0FFu - frac] * oldest) >> 15) +
           ((psx_spu_gauss[0x1FFu - frac] * older)  >> 15) +
           ((psx_spu_gauss[0x100u + frac] * old)    >> 15) +
           ((psx_spu_gauss[0x000u + frac] * current) >> 15);
}

static void spu_adsr_tick(struct psx_system *psx, int v) {
    u16 lo = psx->spu_regs[v*8 + 4];
    u16 hi = psx->spu_regs[v*8 + 5];
    u8  phase = psx->spu_adsr_phase[v];
    s32 vol = psx->spu_adsr_vol[v];
    s32 sustain_level = (s32)(((lo & 0xFu) + 1u) << 11);

    if (phase == 0) {
        spu_envelope_step(&vol, &psx->spu_adsr_cnt[v],
                          ((u32)lo >> 10) & 0x1Fu,
                          ((u32)lo >> 8) & 0x3u,
                          ((u32)lo >> 15) & 1u,
                          0, 0, 0);
        if (vol >= 0x7FFF) {
            vol = 0x7FFF;
            phase = 1;
            psx->spu_adsr_cnt[v] = 0;
        }
    } else if (phase == 1) {
        spu_envelope_step(&vol, &psx->spu_adsr_cnt[v],
                          ((u32)lo >> 4) & 0xFu,
                          0u, 1, 1, 0, 1);
        if (vol <= sustain_level) {
            vol = sustain_level;
            phase = 2;
            psx->spu_adsr_cnt[v] = 0;
        }
    } else if (phase == 2) {
        spu_envelope_step(&vol, &psx->spu_adsr_cnt[v],
                          ((u32)hi >> 8) & 0x1Fu,
                          ((u32)hi >> 6) & 0x3u,
                          ((u32)hi >> 15) & 1u,
                          ((u32)hi >> 14) & 1u,
                          0, 0);
        if (vol < 0)
            vol = 0;
        else if (vol > 0x7FFF)
            vol = 0x7FFF;
    } else {
        spu_envelope_step(&vol, &psx->spu_adsr_cnt[v],
                          (u32)hi & 0x1Fu,
                          0u,
                          ((u32)hi >> 5) & 1u,
                          1, 0, 1);
        if (vol <= 0) {
            vol = 0;
            psx->spu_voice_on &= ~(1u << v);
        }
    }

    psx->spu_adsr_vol[v] = vol;
    psx->spu_adsr_phase[v] = phase;
    psx->spu_regs[v*8 + 6] = (u16)vol;
}

void psx_spu_render(struct psx_system *psx, s16 *out, u32 count) {
    u16 spucnt = psx->spu_regs[SPU_REG_CTRL];
    if (!(spucnt & 0x8000u) || !(spucnt & 0x4000u)) {
        if (out) for (u32 i = 0; i < count * 2u; i++) out[i] = 0;
        return;
    }

    for (u32 i = 0; i < count; i++) {
        s64 mix_l = 0, mix_r = 0;

        if (psx->spu_kon_latch) {
            u32 mask = psx->spu_kon_latch;
            psx->spu_kon_latch = 0;
            for (int v = 0; v < 24; v++) {
                if (mask & (1u << v)) {
                    psx->spu_voice_on |= (1u << v);
                    psx->spu_voice_addr[v] = (u32)psx->spu_regs[v*8 + 3] << 3;
                    psx->spu_voice_idx[v] = 0;
                    psx->spu_voice_phase[v] = 0;
                    psx->spu_adsr_vol[v] = 0;
                    psx->spu_adsr_phase[v] = 0;
                    psx->spu_adsr_cnt[v] = 0;
                    psx->spu_voice_prev[v][0] = 0;
                    psx->spu_voice_prev[v][1] = 0;
                    psx->spu_voice_prev[v][2] = 0;
                    spu_decode_block(psx, v);
                }
            }
        }
        if (psx->spu_koff_latch) {
            u32 mask = psx->spu_koff_latch;
            psx->spu_koff_latch = 0;
            for (int v = 0; v < 24; v++) {
                if (mask & (1u << v))
                    psx->spu_adsr_phase[v] = 3;
            }
        }

        spu_volume_tick(psx->spu_regs[0xC0], &psx->spu_main_vol_cur[0], &psx->spu_main_vol_cnt[0]);
        spu_volume_tick(psx->spu_regs[0xC1], &psx->spu_main_vol_cur[1], &psx->spu_main_vol_cnt[1]);

        for (int v = 0; v < 24; v++) {
            if (!(psx->spu_voice_on & (1u << v))) continue;

            s32 sample = psx_spu_sample_gauss(psx, v);
            spu_adsr_tick(psx, v);
            s32 adsr = psx->spu_adsr_vol[v];
            s32 ampl = (sample * adsr) >> 15;

            spu_volume_tick(psx->spu_regs[v*8 + 0],
                            &psx->spu_voice_vol_cur[v][0],
                            &psx->spu_voice_vol_cnt[v][0]);
            spu_volume_tick(psx->spu_regs[v*8 + 1],
                            &psx->spu_voice_vol_cur[v][1],
                            &psx->spu_voice_vol_cnt[v][1]);
            mix_l += ((s64)ampl * (s64)psx->spu_voice_vol_cur[v][0]) >> 15;
            mix_r += ((s64)ampl * (s64)psx->spu_voice_vol_cur[v][1]) >> 15;

            {
                u32 pitch = (u32)psx->spu_regs[v*8 + 2];
                if (pitch > 0x4000u)
                    pitch = 0x4000u;
                psx->spu_voice_phase[v] += ((pitch << 4) * 44100u) / 48000u;
            }

            while (psx->spu_voice_phase[v] >= 0x10000u) {
                psx->spu_voice_phase[v] -= 0x10000u;
                psx->spu_voice_idx[v]++;
                if (psx->spu_voice_idx[v] >= 28u) {
                    psx->spu_voice_idx[v] = 0;
                    psx->spu_voice_prev[v][0] = psx->spu_voice_samples[v][25];
                    psx->spu_voice_prev[v][1] = psx->spu_voice_samples[v][26];
                    psx->spu_voice_prev[v][2] = psx->spu_voice_samples[v][27];
                    {
                        u8 flags = psx->spu_ram[(psx->spu_voice_addr[v] + 1u) & 0x7FFFFu];
                        if (flags & 4u)
                            psx->spu_regs[v*8 + 7] = (u16)(psx->spu_voice_addr[v] >> 3);
                        if (flags & 1u) {
                            psx->spu_endx |= (1u << v);
                            if (flags & 2u) {
                                psx->spu_voice_addr[v] =
                                    ((u32)psx->spu_regs[v*8 + 7] << 3) & 0x7FFFFu;
                            } else {
                                psx->spu_adsr_vol[v] = 0;
                                psx->spu_regs[v*8 + 6] = 0;
                                psx->spu_voice_on &= ~(1u << v);
                                break;
                            }
                        } else {
                            psx->spu_voice_addr[v] =
                                (psx->spu_voice_addr[v] + 16u) & 0x7FFFFu;
                        }
                    }
                    spu_decode_block(psx, v);
                }
            }
        }

        {
            s64 final_l = (mix_l * (s64)psx->spu_main_vol_cur[0]) >> 12;
            s64 final_r = (mix_r * (s64)psx->spu_main_vol_cur[1]) >> 12;
            if (final_l > 32767) final_l = 32767; else if (final_l < -32768) final_l = -32768;
            if (final_r > 32767) final_r = 32767; else if (final_r < -32768) final_r = -32768;
            psx->spu_lpf_l = ((s32)final_l + psx->spu_lpf_l) >> 1;
            psx->spu_lpf_r = ((s32)final_r + psx->spu_lpf_r) >> 1;
            if (out) {
                out[i*2 + 0] = (s16)psx->spu_lpf_l;
                out[i*2 + 1] = (s16)psx->spu_lpf_r;
            }
        }
    }
}

/* ── SPU Worker Thread ────────────────────────────────────────── */
void psx_spu_worker(void *arg) {
    struct psx_system *psx = (struct psx_system *)arg;
    s16 buffer[512]; /* 256 stereo samples */

    while (psx->spu_thread_run) {
        psx_spu_render(psx, buffer, 256);

        if (psx->audio_out_fn && psx->audio_h >= 0) {
            s32 ret = (s32)NC(psx->G, psx->audio_out_fn, (u64)psx->audio_h,
                              (u64)buffer, 0, 0, 0, 0);
            if (!psx->audio_phase && (buffer[0] != 0 || buffer[1] != 0)) {
                psx->audio_phase = 1;
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[SPU] W out ret=", (u32)ret);
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[SPU] W L=", (u32)(u16)buffer[0]);
                psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
                           "[SPU] W R=", (u32)(u16)buffer[1]);
            }
        }
    }
}
