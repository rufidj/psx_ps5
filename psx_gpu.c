#include "psx_types.h"

/* ── PS5 output struct (double-buffer handles) ───────────────────── */
struct psx_gpu {
    u32 *fb[2];
    u32  fb_idx;
};

/* Forward: psx_local_zero is in psx_main.c (same TU) */
static void psx_local_zero(u8 *p, u32 n);
static u16 gpu_modulate_texel(u16 texel, u32 color24);
static void gpu_apply_texwin(const struct psx_gpu_io *g, u32 *u, u32 *v);
static s32 gpu_clamp_i32(s32 v, s32 lo, s32 hi);
static u16 gpu_blend_15(u16 dst, u16 src, u32 mode);
static u16 gpu_write_pixel(struct psx_gpu_io *g, u32 ux, u32 uy, u16 src, int semi);

/* GP0 command length – pure computation, no static data */
static u32 gpu_gp0_cmd_len(u8 cmd)
{
    if (cmd == 0x02)           return 3;   /* Fill rect */
    if (cmd >= 0xE0)           return 1;   /* Draw settings */
    if (cmd >= 0xC0)           return 3;   /* VRAM→CPU */
    if (cmd >= 0xA0)           return 3;   /* CPU→VRAM (header only) */
    if (cmd >= 0x80)           return 4;   /* VRAM→VRAM */
    if (cmd >= 0x7C)           return 3;   /* Textured 16x16 sprite */
    if (cmd >= 0x78)           return 2;   /* Mono 16x16 sprite */
    if (cmd >= 0x74)           return 3;   /* Textured 8x8 sprite */
    if (cmd >= 0x70)           return 2;   /* Mono 8x8 sprite */
    if (cmd >= 0x6C)           return 3;   /* Textured 1x1 sprite */
    if (cmd >= 0x68)           return 2;   /* Mono 1x1 sprite */
    if (cmd >= 0x64)           return 4;   /* Textured rect variable */
    if (cmd >= 0x60)           return 3;   /* Mono rect variable */
    if (cmd >= 0x50)           return 4;   /* Gouraud line */
    if (cmd >= 0x40)           return 3;   /* Mono line */
    if (cmd >= 0x3C)           return 12;  /* Gouraud textured quad */
    if (cmd >= 0x38)           return 8;   /* Gouraud quad */
    if (cmd >= 0x34)           return 9;   /* Gouraud textured tri */
    if (cmd >= 0x30)           return 6;   /* Gouraud tri */
    if (cmd >= 0x2C)           return 9;   /* Textured quad */
    if (cmd >= 0x28)           return 5;   /* Mono quad */
    if (cmd >= 0x24)           return 7;   /* Textured tri */
    if (cmd >= 0x20)           return 4;   /* Mono tri */
    return 1;
}

static int gpu_is_polyline_cmd(u8 cmd)
{
    return ((cmd >= 0x48 && cmd <= 0x4Fu) || (cmd >= 0x58 && cmd <= 0x5Fu));
}

static int gpu_polyline_terminator(u32 word)
{
    return ((word & 0xF000F000u) == 0x50005000u);
}

static void gpu_apply_texwin(const struct psx_gpu_io *g, u32 *u, u32 *v)
{
    u32 mask_x = (g->texwin & 0x1Fu) << 3;
    u32 mask_y = ((g->texwin >> 5) & 0x1Fu) << 3;
    u32 off_x = ((g->texwin >> 10) & 0x1Fu) << 3;
    u32 off_y = ((g->texwin >> 15) & 0x1Fu) << 3;

    *u = (*u & ~mask_x) | (off_x & mask_x);
    *v = (*v & ~mask_y) | (off_y & mask_y);
}

static s32 gpu_clamp_i32(s32 v, s32 lo, s32 hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static u16 gpu_rgb24_to_15_dither(u32 c, s32 x, s32 y, int enable)
{
    static const s8 dither4x4[16] = {
        -4,  0, -3,  1,
         2, -2,  3, -1,
        -3,  1, -4,  0,
         3, -1,  2, -2
    };
    s32 r = (s32)(c & 0xFFu);
    s32 g = (s32)((c >> 8) & 0xFFu);
    s32 b = (s32)((c >> 16) & 0xFFu);
    if (enable) {
        s32 d = dither4x4[((y & 3) << 2) | (x & 3)];
        r = gpu_clamp_i32(r + d, 0, 255);
        g = gpu_clamp_i32(g + d, 0, 255);
        b = gpu_clamp_i32(b + d, 0, 255);
    }
    return (u16)((r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10));
}

/* ─── helpers ─────────────────────────────────────────────────────── */

/* Textured triangle with UV interpolation.
 * cl_x, cl_y = CLUT position in VRAM (for 4-bit/8-bit indexed textures).
 * Uses current g->texpage for texture page and depth. */
static void gpu_draw_tri_tex_uv(struct psx_gpu_io *g,
    s32 x0, s32 y0, s32 u0, s32 v0,
    s32 x1, s32 y1, s32 u1, s32 v1,
    s32 x2, s32 y2, s32 u2, s32 v2,
    u32 cl_x, u32 cl_y,
    u32 c0, u32 c1, u32 c2,
    int raw_tex, int semi)
{
    if (!g->vram) return;
    x0 += g->off_x; y0 += g->off_y;
    x1 += g->off_x; y1 += g->off_y;
    x2 += g->off_x; y2 += g->off_y;

    /* PSX hardware rejects polygons with |Δx|>1023 or |Δy|>511 */
    {
        s32 xmin = x0 < x1 ? x0 : x1; if (x2 < xmin) xmin = x2;
        s32 xmax = x0 > x1 ? x0 : x1; if (x2 > xmax) xmax = x2;
        s32 ymin = y0 < y1 ? y0 : y1; if (y2 < ymin) ymin = y2;
        s32 ymax = y0 > y1 ? y0 : y1; if (y2 > ymax) ymax = y2;
        if ((xmax - xmin) > 1023 || (ymax - ymin) > 511) return;
    }

    /* Sort by Y ascending */
#define SWPVT(ax,ay,au,av,bx,by,bu,bv) do { \
    s32 _tx=(ax),_ty=(ay),_tu=(au),_tv=(av); \
    (ax)=(bx);(ay)=(by);(au)=(bu);(av)=(bv); \
    (bx)=_tx;(by)=_ty;(bu)=_tu;(bv)=_tv; \
} while(0)
    if (y1 < y0) SWPVT(x0,y0,u0,v0, x1,y1,u1,v1);
    if (y2 < y0) SWPVT(x0,y0,u0,v0, x2,y2,u2,v2);
    if (y2 < y1) SWPVT(x1,y1,u1,v1, x2,y2,u2,v2);
#undef SWPVT

    s32 total_h = y2 - y0; if (total_h == 0) return;
    u32 tp_x  = (g->texpage & 0xFu) * 64u;
    u32 tp_y  = (((g->texpage >> 4) & 1u) * 256u) + (((g->texpage >> 11) & 1u) * 512u);
    s32 r0 = (s32)(c0 & 0xFFu), g0 = (s32)((c0 >> 8) & 0xFFu), b0 = (s32)((c0 >> 16) & 0xFFu);
    s32 r1 = (s32)(c1 & 0xFFu), g1 = (s32)((c1 >> 8) & 0xFFu), b1 = (s32)((c1 >> 16) & 0xFFu);
    s32 r2 = (s32)(c2 & 0xFFu), g2 = (s32)((c2 >> 8) & 0xFFu), b2 = (s32)((c2 >> 16) & 0xFFu);

    for (s32 y = y0; y <= y2; y++) {
        /* Scissor clipping */
        if (y < g->draw_y1 || y > g->draw_y2) continue;

        /* Hardware Y-wrap */
        u32 uy = (u32)y & 0x1FFu;

        s32 t_ab = ((y - y0) << 16) / total_h;
        s32 xA = x0 + (((x2 - x0) * t_ab) >> 16);
        s32 uA = u0 + (((u2 - u0) * t_ab) >> 16);
        s32 vA = v0 + (((v2 - v0) * t_ab) >> 16);
        s32 rA = r0 + (((r2 - r0) * t_ab) >> 16);
        s32 gA = g0 + (((g2 - g0) * t_ab) >> 16);
        s32 bA = b0 + (((b2 - b0) * t_ab) >> 16);
        
        s32 xB, uB, vB, rB, gB, bB;
        if (y <= y1) {
            s32 h01 = y1 - y0; if (!h01) h01 = 1;
            s32 t = ((y - y0) << 16) / h01;
            xB = x0 + (((x1 - x0) * t) >> 16);
            uB = u0 + (((u1 - u0) * t) >> 16);
            vB = v0 + (((v1 - v0) * t) >> 16);
            rB = r0 + (((r1 - r0) * t) >> 16);
            gB = g0 + (((g1 - g0) * t) >> 16);
            bB = b0 + (((b1 - b0) * t) >> 16);
        } else {
            s32 h12 = y2 - y1; if (!h12) h12 = 1;
            s32 t = ((y - y1) << 16) / h12;
            xB = x1 + (((x2 - x1) * t) >> 16);
            uB = u1 + (((u2 - u1) * t) >> 16);
            vB = v1 + (((v2 - v1) * t) >> 16);
            rB = r1 + (((r2 - r1) * t) >> 16);
            gB = g1 + (((g2 - g1) * t) >> 16);
            bB = b1 + (((b2 - b1) * t) >> 16);
        }

        if (xA > xB) {
            s32 tx=xA; xA=xB; xB=tx;
            s32 tu=uA; uA=uB; uB=tu;
            s32 tv=vA; vA=vB; vB=tv;
            s32 tr=rA; rA=rB; rB=tr;
            s32 tg=gA; gA=gB; gB=tg;
            s32 tb=bA; bA=bB; bB=tb;
        }

        s32 span = xB - xA; if (span < 1) span = 1;
        s32 du = ((uB - uA) << 16) / span;
        s32 dv = ((vB - vA) << 16) / span;
        s32 dr = ((rB - rA) << 16) / span;
        s32 dg = ((gB - gA) << 16) / span;
        s32 db = ((bB - bA) << 16) / span;
        s32 cur_u = uA << 16;
        s32 cur_v = vA << 16;
        s32 cur_r = rA << 16;
        s32 cur_g = gA << 16;
        s32 cur_b = bA << 16;

        for (s32 x = xA; x <= xB; x++) {
            if (x < g->draw_x1 || x > g->draw_x2) {
                cur_u += du; cur_v += dv;
                cur_r += dr; cur_g += dg; cur_b += db;
                continue;
            }
            u32 ux = (u32)x & 0x3FFu;
            u32 u  = (u32)(cur_u >> 16) & 0xFFu;
            u32 v  = (u32)(cur_v >> 16) & 0xFFu;
            gpu_apply_texwin(g, &u, &v);

            u16 texel = 0;
            u32 depth = (g->texpage >> 7) & 3u;
            if (depth == 0) { /* 4-bit */
                u32 tx2 = (tp_x + (u >> 2)) & 0x3FFu;
                u32 ty2 = (tp_y + v) & 0x1FFu;
                u16 pk  = g->vram[ty2 * 1024u + tx2];
                u32 nib = (pk >> ((u & 3u) * 4u)) & 0xFu;
                texel = g->vram[(cl_y & 0x1FFu) * 1024u + ((cl_x + nib) & 0x3FFu)];
            } else if (depth == 1) { /* 8-bit */
                u32 tx2 = (tp_x + (u >> 1)) & 0x3FFu;
                u32 ty2 = (tp_y + v) & 0x1FFu;
                u16 pk  = g->vram[ty2 * 1024u + tx2];
                u32 idx = (pk >> ((u & 1u) * 8u)) & 0xFFu;
                texel = g->vram[(cl_y & 0x1FFu) * 1024u + ((cl_x + idx) & 0x3FFu)];
            } else { /* 16-bit */
                texel = g->vram[((tp_y + v) & 0x1FFu) * 1024u + ((tp_x + u) & 0x3FFu)];
            }

            if (!texel) { cur_u += du; cur_v += dv; cur_r += dr; cur_g += dg; cur_b += db; continue; }
            u16 out = texel;
            int pix_semi = semi && (texel & 0x8000u);
            if (!raw_tex) {
                u32 mod =
                    ((u32)((cur_r >> 16) & 0xFFu)) |
                    ((u32)((cur_g >> 16) & 0xFFu) << 8) |
                    ((u32)((cur_b >> 16) & 0xFFu) << 16);
                out = gpu_modulate_texel(texel, mod);
            }
            gpu_write_pixel(g, ux, uy, out, pix_semi);
            cur_u += du; cur_v += dv;
            cur_r += dr; cur_g += dg; cur_b += db;
        }
    }
}

/* Gouraud-shaded triangle rasterizer.
 * Vertices: (x0,y0,c0), (x1,y1,c1), (x2,y2,c2)
 * Colors are 24-bit 0x00BBGGRR format (PSX convention). */
static void gpu_draw_tri_gouraud(struct psx_gpu_io *g,
    s32 x0, s32 y0, u32 c0,
    s32 x1, s32 y1, u32 c1,
    s32 x2, s32 y2, u32 c2,
    int semi)
{
    x0 += g->off_x; y0 += g->off_y;
    x1 += g->off_x; y1 += g->off_y;
    x2 += g->off_x; y2 += g->off_y;

    /* PSX hardware rejects polygons with |Δx|>1023 or |Δy|>511 */
    {
        s32 xmin = x0 < x1 ? x0 : x1; if (x2 < xmin) xmin = x2;
        s32 xmax = x0 > x1 ? x0 : x1; if (x2 > xmax) xmax = x2;
        s32 ymin = y0 < y1 ? y0 : y1; if (y2 < ymin) ymin = y2;
        s32 ymax = y0 > y1 ? y0 : y1; if (y2 > ymax) ymax = y2;
        if ((xmax - xmin) > 1023 || (ymax - ymin) > 511) return;
    }

    /* Sort by Y ascending */
#define SWPV(ax,ay,ac,bx,by,bc) do { s32 _x=(ax),_y=(ay); u32 _c=(ac); (ax)=(bx);(ay)=(by);(ac)=(bc);(bx)=_x;(by)=_y;(bc)=_c; } while(0)
    if (y1 < y0) SWPV(x0,y0,c0,x1,y1,c1);
    if (y2 < y0) SWPV(x0,y0,c0,x2,y2,c2);
    if (y2 < y1) SWPV(x1,y1,c1,x2,y2,c2);
#undef SWPV

    s32 r0=c0&0xFF, g0=(c0>>8)&0xFF, b0=(c0>>16)&0xFF;
    s32 r1=c1&0xFF, g1=(c1>>8)&0xFF, b1=(c1>>16)&0xFF;
    s32 r2=c2&0xFF, g2=(c2>>8)&0xFF, b2=(c2>>16)&0xFF;

    s32 total_h = y2 - y0;
    if (total_h == 0) return;

    for (s32 y = y0; y <= y2; y++) {
        if (y < g->draw_y1 || y > g->draw_y2) continue;
        u32 uy = (u32)y & 0x1FFu;

        s32 t_ab = ((y - y0) << 16) / total_h;
        s32 xA = x0 + (((x2 - x0) * t_ab) >> 16);
        s32 rA = r0 + (((r2 - r0) * t_ab) >> 16);
        s32 gA = g0 + (((g2 - g0) * t_ab) >> 16);
        s32 bA = b0 + (((b2 - b0) * t_ab) >> 16);

        s32 xB, rB, gB, bB;
        if (y <= y1) {
            s32 h01 = y1 - y0; if (!h01) h01 = 1;
            s32 t = ((y - y0) << 16) / h01;
            xB = x0 + (((x1 - x0) * t) >> 16);
            rB = r0 + (((r1 - r0) * t) >> 16);
            gB = g0 + (((g1 - g0) * t) >> 16);
            bB = b0 + (((b1 - b0) * t) >> 16);
        } else {
            s32 h12 = y2 - y1; if (!h12) h12 = 1;
            s32 t = ((y - y1) << 16) / h12;
            xB = x1 + (((x2 - x1) * t) >> 16);
            rB = r1 + (((r2 - r1) * t) >> 16);
            gB = g1 + (((g2 - g1) * t) >> 16);
            bB = b1 + (((b2 - b1) * t) >> 16);
        }

        if (xA > xB) {
            s32 tx=xA; xA=xB; xB=tx;
            s32 tr=rA; rA=rB; rB=tr;
            s32 tg=gA; gA=gB; gB=tg;
            s32 tb=bA; bA=bB; bB=tb;
        }

        s32 span = xB - xA; if (span < 1) span = 1;
        {
            s32 dr = ((rB - rA) << 16) / span;
            s32 dg = ((gB - gA) << 16) / span;
            s32 db = ((bB - bA) << 16) / span;
            s32 cur_r = rA << 16;
            s32 cur_g = gA << 16;
            s32 cur_b = bA << 16;
            for (s32 x = xA; x <= xB; x++) {
                if (x >= g->draw_x1 && x <= g->draw_x2) {
                    u32 ux = (u32)x & 0x3FFu;
                    u32 r  = (u32)(cur_r >> 16) & 0xFFu;
                    u32 gg = (u32)(cur_g >> 16) & 0xFFu;
                    u32 b  = (u32)(cur_b >> 16) & 0xFFu;
                    gpu_write_pixel(g, ux, uy,
                        gpu_rgb24_to_15_dither(r | (gg << 8) | (b << 16), x, y, g->dither_enable),
                        semi);
                }
                cur_r += dr; cur_g += dg; cur_b += db;
            }
        }
    }
}

static u16 gpu_rgb24_to_15(u32 c)
{
    return gpu_rgb24_to_15_dither(c, 0, 0, 0);
}

static u32 gpu_rgb15_to_bgra8(u16 v)
{
    u32 r = ((v >>  0) & 0x1F) << 3;
    u32 g = ((v >>  5) & 0x1F) << 3;
    u32 b = ((v >> 10) & 0x1F) << 3;
    return 0xFF000000u | (r << 16) | (g << 8) | b;
}

static u16 gpu_modulate_texel(u16 texel, u32 color24)
{
    u32 tr = texel & 0x1Fu;
    u32 tg = (texel >> 5) & 0x1Fu;
    u32 tb = (texel >> 10) & 0x1Fu;
    u32 cr = color24 & 0xFFu;
    u32 cg = (color24 >> 8) & 0xFFu;
    u32 cb = (color24 >> 16) & 0xFFu;

    tr = (tr * cr) >> 7;
    tg = (tg * cg) >> 7;
    tb = (tb * cb) >> 7;
    if (tr > 0x1Fu) tr = 0x1Fu;
    if (tg > 0x1Fu) tg = 0x1Fu;
    if (tb > 0x1Fu) tb = 0x1Fu;
    return (u16)(tr | (tg << 5) | (tb << 10) | (texel & 0x8000u));
}

static u16 gpu_blend_15(u16 dst, u16 src, u32 mode)
{
    s32 dr = dst & 0x1Fu;
    s32 dg = (dst >> 5) & 0x1Fu;
    s32 db = (dst >> 10) & 0x1Fu;
    s32 sr = src & 0x1Fu;
    s32 sg = (src >> 5) & 0x1Fu;
    s32 sb = (src >> 10) & 0x1Fu;
    s32 r = sr, g = sg, b = sb;

    switch (mode & 3u) {
    case 0: r = (dr + sr) >> 1; g = (dg + sg) >> 1; b = (db + sb) >> 1; break;
    case 1: r = dr + sr; g = dg + sg; b = db + sb; break;
    case 2: r = dr - sr; g = dg - sg; b = db - sb; break;
    case 3: r = dr + (sr >> 2); g = dg + (sg >> 2); b = db + (sb >> 2); break;
    }
    r = gpu_clamp_i32(r, 0, 0x1F);
    g = gpu_clamp_i32(g, 0, 0x1F);
    b = gpu_clamp_i32(b, 0, 0x1F);
    return (u16)(r | (g << 5) | (b << 10) | (src & 0x8000u));
}

static u16 gpu_write_pixel(struct psx_gpu_io *g, u32 ux, u32 uy, u16 src, int semi)
{
    u16 dst = g->vram[uy * 1024u + ux];
    if (g->mask_check && (dst & 0x8000u))
        return dst;
    if (semi)
        src = gpu_blend_15(dst, src, (g->texpage >> 5) & 3u);
    if (g->mask_set)
        src |= 0x8000u;
    g->vram[uy * 1024u + ux] = src;
    return src;
}

static void gpu_vram_xfer_write(struct psx_gpu_io *g, u32 ux, u32 uy, u16 src)
{
    u16 dst = g->vram[uy * 1024u + ux];
    if (g->mask_check && (dst & 0x8000u))
        return;
    if (g->mask_set)
        src |= 0x8000u;
    g->vram[uy * 1024u + ux] = src;
}

static void gpu_vram_put(struct psx_gpu_io *g, s32 x, s32 y, u16 col);

static void gpu_draw_line_shaded(struct psx_gpu_io *g,
    s32 x0, s32 y0, u32 c0,
    s32 x1, s32 y1, u32 c1,
    int semi)
{
    x0 += g->off_x; y0 += g->off_y;
    x1 += g->off_x; y1 += g->off_y;

    s32 dx = x1 - x0;
    s32 dy = y1 - y0;
    s32 adx = (dx < 0) ? -dx : dx;
    s32 ady = (dy < 0) ? -dy : dy;
    s32 steps = (adx > ady) ? adx : ady;
    if (steps <= 0) {
        if (x0 >= g->draw_x1 && x0 <= g->draw_x2 && y0 >= g->draw_y1 && y0 <= g->draw_y2)
            gpu_write_pixel(g, (u32)x0 & 0x3FFu, (u32)y0 & 0x1FFu,
                            gpu_rgb24_to_15_dither(c0, x0, y0, g->dither_enable), semi);
        return;
    }

    s32 r0 = (s32)(c0 & 0xFFu), g0 = (s32)((c0 >> 8) & 0xFFu), b0 = (s32)((c0 >> 16) & 0xFFu);
    s32 r1 = (s32)(c1 & 0xFFu), g1 = (s32)((c1 >> 8) & 0xFFu), b1 = (s32)((c1 >> 16) & 0xFFu);
    s32 xfp = x0 << 16;
    s32 yfp = y0 << 16;
    s32 xstep = (dx << 16) / steps;
    s32 ystep = (dy << 16) / steps;

    for (s32 i = 0; i <= steps; i++) {
        s32 t = (i << 16) / steps;
        u32 r = (u32)(r0 + (((r1 - r0) * t) >> 16));
        u32 gg = (u32)(g0 + (((g1 - g0) * t) >> 16));
        u32 b = (u32)(b0 + (((b1 - b0) * t) >> 16));
        {
            s32 px = xfp >> 16;
            s32 py = yfp >> 16;
            u16 col = gpu_rgb24_to_15_dither(r | (gg << 8) | (b << 16), px, py, g->dither_enable);
            if (px >= g->draw_x1 && px <= g->draw_x2 && py >= g->draw_y1 && py <= g->draw_y2)
                gpu_write_pixel(g, (u32)px & 0x3FFu, (u32)py & 0x1FFu, col, semi);
        }
        xfp += xstep;
        yfp += ystep;
    }
}

/* Write a pixel to VRAM, respecting drawing area */
static void gpu_vram_put(struct psx_gpu_io *g, s32 x, s32 y, u16 col)
{
    /* Hardware wraps coordinates within 1024x512 VRAM */
    u32 ux = (u32)x & 0x3FFu;
    u32 uy = (u32)y & 0x1FFu;

    /* Scissor/Drawing Area clipping still applies */
    if (x < g->draw_x1 || x > g->draw_x2) return;
    if (y < g->draw_y1 || y > g->draw_y2) return;

    gpu_write_pixel(g, ux, uy, col, 0);
}

/* ─── GP0 command executor ────────────────────────────────────────── */

static void gpu_exec_fill_rect(struct psx_gpu_io *g)
{
    u16 col = gpu_rgb24_to_15(g->fifo[0] & 0xFFFFFFu);
    s32 x   = (s32)(g->fifo[1] & 0x3FFu);
    s32 y   = (s32)((g->fifo[1] >> 16) & 0x1FFu);
    s32 w   = (s32)(g->fifo[2] & 0x3FFu);
    s32 h   = (s32)((g->fifo[2] >> 16) & 0x3FFu);
    /* FILL ignores drawing area – writes directly to VRAM */
    for (s32 fy = 0; fy < h; fy++) {
        for (s32 fx = 0; fx < w; fx++) {
            u32 vx = (u32)(x + fx) & 0x3FFu;
            u32 vy = (u32)(y + fy) & 0x1FFu;
            g->vram[vy * 1024u + vx] = col;
        }
    }
}

/* Mono rectangle (GP0 0x60..0x6F): word0=cmd|color, word1=xy, word2=wh */
static void gpu_exec_mono_rect(struct psx_gpu_io *g, s32 w, s32 h, int semi)
{
    u16  col = gpu_rgb24_to_15(g->fifo[0] & 0xFFFFFFu);
    s32  x   = (s32)(s16)(g->fifo[1] & 0xFFFFu) + g->off_x;
    s32  y   = (s32)(s16)((g->fifo[1] >> 16) & 0xFFFFu) + g->off_y;
    /* variable size: read from fifo[2] */
    if (w < 0) { w = (s32)(g->fifo[2] & 0xFFFFu); }
    if (h < 0) { h = (s32)((g->fifo[2] >> 16) & 0xFFFFu); }
    for (s32 dy = 0; dy < h; dy++)
        for (s32 dx = 0; dx < w; dx++)
            if ((x + dx) >= g->draw_x1 && (x + dx) <= g->draw_x2 &&
                (y + dy) >= g->draw_y1 && (y + dy) <= g->draw_y2)
                gpu_write_pixel(g, (u32)(x + dx) & 0x3FFu, (u32)(y + dy) & 0x1FFu, col, semi);
}

/* Textured sprite helper – handles 4-bit/8-bit/15-bit textures + CLUT */
static void gpu_draw_tex_sprite(struct psx_gpu_io *g,
    s32 sx, s32 sy, s32 w, s32 h, u8 u0, u8 v0, u16 clut_word, u32 color24,
    int raw_tex, int semi)
{
    u32 depth  = (g->texpage >> 7) & 3u;
    u32 tp_x   = (g->texpage & 0xFu) * 64u;
    u32 tp_y   = (((g->texpage >> 4) & 1u) * 256u) + (((g->texpage >> 11) & 1u) * 512u);
    u32 cl_x   = (clut_word & 0x3Fu) * 16u;
    u32 cl_y   = (clut_word >> 6) & 0x1FFu;

    for (s32 dy = 0; dy < h; dy++) {
        for (s32 dx = 0; dx < w; dx++) {
            s32 sdx = g->rect_flip_x ? (w - 1 - dx) : dx;
            s32 sdy = g->rect_flip_y ? (h - 1 - dy) : dy;
            u8  u = (u8)(u0 + sdx);
            u8  v = (u8)(v0 + sdy);
            s32 px = sx + dx + g->off_x;
            s32 py = sy + dy + g->off_y;
            u32 uu = u;
            u32 vv = v;
            gpu_apply_texwin(g, &uu, &vv);

            u16 texel;
            if (depth == 0) { /* 4-bit indexed */
                u32 tx  = (tp_x + (uu >> 2)) & 0x3FFu;
                u32 ty  = (tp_y + vv) & 0x1FFu;
                u16 pk  = g->vram[ty * 1024u + tx];
                u32 nib = (pk >> ((uu & 3u) * 4u)) & 0xFu;
                texel = g->vram[(cl_y & 0x1FFu) * 1024u + ((cl_x + nib) & 0x3FFu)];
            } else if (depth == 1) { /* 8-bit indexed */
                u32 tx  = (tp_x + (uu >> 1)) & 0x3FFu;
                u32 ty  = (tp_y + vv) & 0x1FFu;
                u16 pk  = g->vram[ty * 1024u + tx];
                u32 idx = (pk >> ((uu & 1u) * 8u)) & 0xFFu;
                texel = g->vram[(cl_y & 0x1FFu) * 1024u + ((cl_x + idx) & 0x3FFu)];
            } else { /* 15-bit direct */
                u32 tx = (tp_x + uu) & 0x3FFu;
                u32 ty = (tp_y + vv) & 0x1FFu;
                texel  = g->vram[ty * 1024u + tx];
            }
            if (!texel) continue;
            if (px < g->draw_x1 || px > g->draw_x2 || py < g->draw_y1 || py > g->draw_y2)
                continue;
            {
                u16 out = raw_tex ? texel : gpu_modulate_texel(texel, color24);
                int pix_semi = semi && (texel & 0x8000u);
                gpu_write_pixel(g, (u32)px & 0x3FFu, (u32)py & 0x1FFu, out, pix_semi);
            }
        }
    }
}

static void gpu_exec_polyline(struct psx_gpu_io *g)
{
    u32 cmd = g->fifo[0] >> 24;
    if (cmd >= 0x48 && cmd <= 0x4Fu) {
        u32 col = g->fifo[0] & 0xFFFFFFu;
        s32 x0 = (s32)(s16)(g->fifo[1] & 0xFFFFu);
        s32 y0 = (s32)(s16)(g->fifo[1] >> 16);
        for (u32 i = 2; i < g->fifo_len; i++) {
            if (gpu_polyline_terminator(g->fifo[i]))
                break;
            s32 x1 = (s32)(s16)(g->fifo[i] & 0xFFFFu);
            s32 y1 = (s32)(s16)(g->fifo[i] >> 16);
            gpu_draw_line_shaded(g, x0, y0, col, x1, y1, col, (cmd & 2u) ? 1 : 0);
            x0 = x1;
            y0 = y1;
        }
    } else if (cmd >= 0x58 && cmd <= 0x5Fu) {
        u32 c0 = g->fifo[0] & 0xFFFFFFu;
        s32 x0 = (s32)(s16)(g->fifo[1] & 0xFFFFu);
        s32 y0 = (s32)(s16)(g->fifo[1] >> 16);
        for (u32 i = 2; (i + 1) < g->fifo_len; i += 2) {
            if (gpu_polyline_terminator(g->fifo[i]))
                break;
            u32 c1 = g->fifo[i] & 0xFFFFFFu;
            s32 x1 = (s32)(s16)(g->fifo[i + 1] & 0xFFFFu);
            s32 y1 = (s32)(s16)(g->fifo[i + 1] >> 16);
            gpu_draw_line_shaded(g, x0, y0, c0, x1, y1, c1, (cmd & 2u) ? 1 : 0);
            x0 = x1;
            y0 = y1;
            c0 = c1;
        }
    }
}

/* Execute a fully-buffered GP0 command */
static void gpu_gp0_exec(struct psx_gpu_io *g)
{
    if (!g->vram) return;  /* VRAM not allocated – skip all rendering */
    u32 cmd = g->fifo[0] >> 24;
    g->last_cmd = cmd;
    if      (cmd == 0x02)                    g->fill_cnt++;
    else if (cmd >= 0x20 && cmd <= 0x5F)     g->poly_cnt++;
    else if (cmd >= 0x60 && cmd <= 0x7F)     g->rect_cnt++;
    switch (cmd) {
    case 0x00: break; /* NOP */
    case 0x01: break; /* Clear cache */

    case 0x02: /* Fill rectangle */
        gpu_exec_fill_rect(g);
        break;

    case 0x80: { /* VRAM→VRAM copy */
        u32 sx = g->fifo[1] & 0x3FFu;
        u32 sy = (g->fifo[1] >> 16) & 0x1FFu;
        u32 dx = g->fifo[2] & 0x3FFu;
        u32 dy = (g->fifo[2] >> 16) & 0x1FFu;
        u32 w  = g->fifo[3] & 0xFFFFu;
        u32 h  = (g->fifo[3] >> 16) & 0xFFFFu;
        for (u32 y = 0; y < h; y++) {
            for (u32 x = 0; x < w; x++) {
                u16 pix = g->vram[((sy + y) & 0x1FFu) * 1024u + ((sx + x) & 0x3FFu)];
                gpu_vram_xfer_write(g, (dx + x) & 0x3FFu, (dy + y) & 0x1FFu, pix);
            }
        }
        break;
    } /* ── Polygons ────────────────────────────────────────────────── */
#define XV(w) ((s32)(s16)((g->fifo[w]) & 0xFFFFu))
#define YV(w) ((s32)(s16)((g->fifo[w]) >> 16))

    /* Flat mono triangle: cmd+col, xy0, xy1, xy2 */
    case 0x20: case 0x21: case 0x22: case 0x23: {
        u32 col = g->fifo[0] & 0xFFFFFFu;
        gpu_draw_tri_gouraud(g, XV(1),YV(1),col, XV(2),YV(2),col, XV(3),YV(3),col,
                             (cmd & 2u) ? 1 : 0);
        break;
    }
    /* Flat mono quad: cmd+col, xy0..xy3 → 2 triangles */
    case 0x28: case 0x29: case 0x2A: case 0x2B: {
        u32 col = g->fifo[0] & 0xFFFFFFu;
        gpu_draw_tri_gouraud(g, XV(1),YV(1),col, XV(2),YV(2),col, XV(3),YV(3),col,
                             (cmd & 2u) ? 1 : 0);
        gpu_draw_tri_gouraud(g, XV(2),YV(2),col, XV(3),YV(3),col, XV(4),YV(4),col,
                             (cmd & 2u) ? 1 : 0);
        break;
    }
    /* Gouraud triangle: c0,xy0, c1,xy1, c2,xy2 */
    case 0x30: case 0x31: case 0x32: case 0x33: {
        u32 c0 = g->fifo[0]&0xFFFFFFu, c1 = g->fifo[2]&0xFFFFFFu, c2 = g->fifo[4]&0xFFFFFFu;
        gpu_draw_tri_gouraud(g, XV(1),YV(1),c0, XV(3),YV(3),c1, XV(5),YV(5),c2,
                             (cmd & 2u) ? 1 : 0);
        break;
    }
    /* Gouraud quad: c0,xy0, c1,xy1, c2,xy2, c3,xy3 */
    case 0x38: case 0x39: case 0x3A: case 0x3B: {
        u32 c0=g->fifo[0]&0xFFFFFFu, c1=g->fifo[2]&0xFFFFFFu;
        u32 c2=g->fifo[4]&0xFFFFFFu, c3=g->fifo[6]&0xFFFFFFu;
        gpu_draw_tri_gouraud(g, XV(1),YV(1),c0, XV(3),YV(3),c1, XV(5),YV(5),c2,
                             (cmd & 2u) ? 1 : 0);
        gpu_draw_tri_gouraud(g, XV(3),YV(3),c1, XV(5),YV(5),c2, XV(7),YV(7),c3,
                             (cmd & 2u) ? 1 : 0);
        break;
    }
    /* Flat textured triangle: cmd+col, xy0,uv0+clut, xy1,uv1+texpage, xy2,uv2 */
#define UV_U(w) ((s32)((g->fifo[w]) & 0xFFu))
#define UV_V(w) ((s32)(((g->fifo[w]) >> 8) & 0xFFu))
#define CL_WORD(w) ((u16)((g->fifo[w]) >> 16))
    case 0x24: case 0x25: case 0x26: case 0x27: {
        u16 cl = CL_WORD(2);
        u32 cl_x = (u32)(cl & 0x3Fu) * 16u;
        u32 cl_y = (u32)(cl >> 6) & 0x1FFu;
        u16 new_tp = CL_WORD(4);
        int semi = (cmd & 2u) ? 1 : 0;
        int raw_tex = (cmd & 1u) ? 1 : 0;
        /* Always apply embedded texpage (even if 0 = page 0 4-bit) */
        g->texpage = new_tp & 0x09FFu;
        g->gpustat = (g->gpustat & ~0x87FFu) | (new_tp & 0x07FFu) | ((new_tp & 0x0800u) << 4);
        g->clut_x = cl_x; g->clut_y = cl_y;
        {
            u32 col = g->fifo[0] & 0xFFFFFFu;
            gpu_draw_tri_tex_uv(g, XV(1),YV(1),UV_U(2),UV_V(2),
                                   XV(3),YV(3),UV_U(4),UV_V(4),
                                   XV(5),YV(5),UV_U(6),UV_V(6), cl_x, cl_y,
                                   col, col, col, raw_tex, semi);
        }
        break;
    }
    /* Flat textured quad: cmd+col, xy0,uv0+clut, xy1,uv1+texpage, xy2,uv2, xy3,uv3 */
    case 0x2C: case 0x2D: case 0x2E: case 0x2F: {
        for (int _i=0;_i<9;_i++) g->last_quad[_i] = g->fifo[_i]; /* capture for debug */
        u16 cl = CL_WORD(2);
        u32 cl_x = (u32)(cl & 0x3Fu) * 16u;
        u32 cl_y = (u32)(cl >> 6) & 0x1FFu;
        u16 new_tp = CL_WORD(4);
        int semi = (cmd & 2u) ? 1 : 0;
        int raw_tex = (cmd & 1u) ? 1 : 0;
        /* Always apply embedded texpage (even if 0 = page 0 4-bit) */
        g->texpage = new_tp & 0x09FFu;
        g->gpustat = (g->gpustat & ~0x87FFu) | (new_tp & 0x07FFu) | ((new_tp & 0x0800u) << 4);
        g->clut_x = cl_x; g->clut_y = cl_y;
        {
            u32 col = g->fifo[0] & 0xFFFFFFu;
            gpu_draw_tri_tex_uv(g, XV(1),YV(1),UV_U(2),UV_V(2),
                                   XV(3),YV(3),UV_U(4),UV_V(4),
                                   XV(5),YV(5),UV_U(6),UV_V(6), cl_x, cl_y,
                                   col, col, col, raw_tex, semi);
            gpu_draw_tri_tex_uv(g, XV(5),YV(5),UV_U(6),UV_V(6),
                                   XV(3),YV(3),UV_U(4),UV_V(4),
                                   XV(7),YV(7),UV_U(8),UV_V(8), cl_x, cl_y,
                                   col, col, col, raw_tex, semi);
        }
        break;
    }
#undef UV_U
#undef UV_V
#undef CL_WORD
    /* Gouraud textured triangle: c0,xy0,uv0+clut, c1,xy1,uv1+tp, c2,xy2,uv2 */
    case 0x34: case 0x35: case 0x36: case 0x37: {
        u16 cl = (u16)((g->fifo[2] >> 16) & 0xFFFFu);
        u32 cl_x = (u32)(cl & 0x3Fu) * 16u;
        u32 cl_y = (u32)(cl >> 6) & 0x1FFu;
        u16 new_tp = (u16)((g->fifo[5] >> 16) & 0xFFFFu);
        int semi = (cmd & 2u) ? 1 : 0;
        int raw_tex = (cmd & 1u) ? 1 : 0;
        g->texpage = new_tp & 0x09FFu;
        g->gpustat = (g->gpustat & ~0x87FFu) | (new_tp & 0x07FFu) | ((new_tp & 0x0800u) << 4);
        g->clut_x = cl_x;
        g->clut_y = cl_y;
        gpu_draw_tri_tex_uv(g, XV(1),YV(1), (s32)(g->fifo[2] & 0xFFu), (s32)((g->fifo[2] >> 8) & 0xFFu),
                               XV(4),YV(4), (s32)(g->fifo[5] & 0xFFu), (s32)((g->fifo[5] >> 8) & 0xFFu),
                               XV(7),YV(7), (s32)(g->fifo[8] & 0xFFu), (s32)((g->fifo[8] >> 8) & 0xFFu),
                               cl_x, cl_y,
                               g->fifo[0] & 0xFFFFFFu, g->fifo[3] & 0xFFFFFFu, g->fifo[6] & 0xFFFFFFu,
                               raw_tex, semi);
        break;
    }
    /* Gouraud textured quad */
    case 0x3C: case 0x3D: case 0x3E: case 0x3F: {
        u16 cl = (u16)((g->fifo[2] >> 16) & 0xFFFFu);
        u32 cl_x = (u32)(cl & 0x3Fu) * 16u;
        u32 cl_y = (u32)(cl >> 6) & 0x1FFu;
        u16 new_tp = (u16)((g->fifo[5] >> 16) & 0xFFFFu);
        int semi = (cmd & 2u) ? 1 : 0;
        int raw_tex = (cmd & 1u) ? 1 : 0;
        g->texpage = new_tp & 0x09FFu;
        g->gpustat = (g->gpustat & ~0x87FFu) | (new_tp & 0x07FFu) | ((new_tp & 0x0800u) << 4);
        g->clut_x = cl_x;
        g->clut_y = cl_y;
        gpu_draw_tri_tex_uv(g, XV(1),YV(1), (s32)(g->fifo[2] & 0xFFu), (s32)((g->fifo[2] >> 8) & 0xFFu),
                               XV(4),YV(4), (s32)(g->fifo[5] & 0xFFu), (s32)((g->fifo[5] >> 8) & 0xFFu),
                               XV(7),YV(7), (s32)(g->fifo[8] & 0xFFu), (s32)((g->fifo[8] >> 8) & 0xFFu),
                               cl_x, cl_y,
                               g->fifo[0] & 0xFFFFFFu, g->fifo[3] & 0xFFFFFFu, g->fifo[6] & 0xFFFFFFu,
                               raw_tex, semi);
        gpu_draw_tri_tex_uv(g, XV(7),YV(7), (s32)(g->fifo[8] & 0xFFu), (s32)((g->fifo[8] >> 8) & 0xFFu),
                               XV(4),YV(4), (s32)(g->fifo[5] & 0xFFu), (s32)((g->fifo[5] >> 8) & 0xFFu),
                               XV(10),YV(10), (s32)(g->fifo[11] & 0xFFu), (s32)((g->fifo[11] >> 8) & 0xFFu),
                               cl_x, cl_y,
                               g->fifo[6] & 0xFFFFFFu, g->fifo[3] & 0xFFFFFFu, g->fifo[9] & 0xFFFFFFu,
                               raw_tex, semi);
        break;
    }
    /* Lines – stub (ignore for now) */
    case 0x40: case 0x41: case 0x42: case 0x43:
    case 0x44: case 0x45: case 0x46: case 0x47:
    case 0x48: case 0x49: case 0x4A: case 0x4B:
    case 0x4C: case 0x4D: case 0x4E: case 0x4F:
        gpu_draw_line_shaded(g, XV(1), YV(1), g->fifo[0] & 0xFFFFFFu,
                                XV(2), YV(2), g->fifo[0] & 0xFFFFFFu, (cmd & 2u) ? 1 : 0);
        break;
    case 0x50: case 0x51: case 0x52: case 0x53:
    case 0x54: case 0x55: case 0x56: case 0x57:
    case 0x58: case 0x59: case 0x5A: case 0x5B:
    case 0x5C: case 0x5D: case 0x5E: case 0x5F:
        gpu_draw_line_shaded(g, XV(1), YV(1), g->fifo[0] & 0xFFFFFFu,
                                XV(3), YV(3), g->fifo[2] & 0xFFFFFFu, (cmd & 2u) ? 1 : 0);
        break;
#undef XV
#undef YV

    /* Mono rectangles – variable, 8x8, 16x16 */
    case 0x60: case 0x61: case 0x62: case 0x63:
        gpu_exec_mono_rect(g, -1, -1, (cmd & 2u) ? 1 : 0);
        break;
    case 0x68: case 0x69: case 0x6A: case 0x6B:
        gpu_exec_mono_rect(g, 1, 1, (cmd & 2u) ? 1 : 0);
        break;
    case 0x70: case 0x71: case 0x72: case 0x73:
        gpu_exec_mono_rect(g, 8, 8, (cmd & 2u) ? 1 : 0);
        break;
    case 0x78: case 0x79: case 0x7A: case 0x7B:
        gpu_exec_mono_rect(g, 16, 16, (cmd & 2u) ? 1 : 0);
        break;

    /* Textured sprites */
    case 0x64: case 0x65: case 0x66: case 0x67: { /* variable */
        s32  x  = (s32)(s16)(g->fifo[1] & 0xFFFFu);
        s32  y  = (s32)(s16)((g->fifo[1] >> 16) & 0xFFFFu);
        u8   u0 = (u8)(g->fifo[2] & 0xFFu);
        u8   v0 = (u8)((g->fifo[2] >> 8) & 0xFFu);
        u16  cl = (u16)((g->fifo[2] >> 16) & 0xFFFFu);
        s32  w  = (s32)(g->fifo[3] & 0xFFFFu);
        s32  h  = (s32)((g->fifo[3] >> 16) & 0xFFFFu);
        g->clut_x = (u32)(cl & 0x3Fu) * 16u;
        g->clut_y = (u32)(cl >> 6) & 0x1FFu;
        gpu_draw_tex_sprite(g, x, y, w, h, u0, v0, cl, g->fifo[0] & 0xFFFFFFu,
                            (cmd & 1u) ? 1 : 0, (cmd & 2u) ? 1 : 0);
        break;
    }
    case 0x74: case 0x75: case 0x76: case 0x77: { /* 8×8 */
        g->last_spr[0]=g->fifo[0]; g->last_spr[1]=g->fifo[1]; g->last_spr[2]=g->fifo[2];
        s32  x  = (s32)(s16)(g->fifo[1] & 0xFFFFu);
        s32  y  = (s32)(s16)((g->fifo[1] >> 16) & 0xFFFFu);
        u8   u0 = (u8)(g->fifo[2] & 0xFFu);
        u8   v0 = (u8)((g->fifo[2] >> 8) & 0xFFu);
        u16  cl = (u16)((g->fifo[2] >> 16) & 0xFFFFu);
        g->clut_x = (u32)(cl & 0x3Fu) * 16u;
        g->clut_y = (u32)(cl >> 6) & 0x1FFu;
        gpu_draw_tex_sprite(g, x, y, 8, 8, u0, v0, cl, g->fifo[0] & 0xFFFFFFu,
                            (cmd & 1u) ? 1 : 0, (cmd & 2u) ? 1 : 0);
        break;
    }
    case 0x6C: case 0x6D: case 0x6E: case 0x6F: { /* 1×1 */
        s32  x  = (s32)(s16)(g->fifo[1] & 0xFFFFu);
        s32  y  = (s32)(s16)((g->fifo[1] >> 16) & 0xFFFFu);
        u8   u0 = (u8)(g->fifo[2] & 0xFFu);
        u8   v0 = (u8)((g->fifo[2] >> 8) & 0xFFu);
        u16  cl = (u16)((g->fifo[2] >> 16) & 0xFFFFu);
        g->clut_x = (u32)(cl & 0x3Fu) * 16u;
        g->clut_y = (u32)(cl >> 6) & 0x1FFu;
        gpu_draw_tex_sprite(g, x, y, 1, 1, u0, v0, cl, g->fifo[0] & 0xFFFFFFu,
                            (cmd & 1u) ? 1 : 0, (cmd & 2u) ? 1 : 0);
        break;
    }
    case 0x7C: case 0x7D: case 0x7E: case 0x7F: { /* 16×16 */
        s32  x  = (s32)(s16)(g->fifo[1] & 0xFFFFu);
        s32  y  = (s32)(s16)((g->fifo[1] >> 16) & 0xFFFFu);
        u8   u0 = (u8)(g->fifo[2] & 0xFFu);
        u8   v0 = (u8)((g->fifo[2] >> 8) & 0xFFu);
        u16  cl = (u16)((g->fifo[2] >> 16) & 0xFFFFu);
        g->clut_x = (u32)(cl & 0x3Fu) * 16u;
        g->clut_y = (u32)(cl >> 6) & 0x1FFu;
        gpu_draw_tex_sprite(g, x, y, 16, 16, u0, v0, cl, g->fifo[0] & 0xFFFFFFu,
                            (cmd & 1u) ? 1 : 0, (cmd & 2u) ? 1 : 0);
        break;
    }

    /* CPU → VRAM copy: set up transfer; data arrives in subsequent GP0 writes */
    case 0xA0: case 0xA1: {
        g->cpuvram_x  = g->fifo[1] & 0x3FFu;
        g->cpuvram_y  = (g->fifo[1] >> 16) & 0x1FFu;
        g->cpuvram_w  = g->fifo[2] & 0xFFFFu;
        g->cpuvram_h  = (g->fifo[2] >> 16) & 0xFFFFu;
        g->cpuvram_cx = g->cpuvram_x;
        g->cpuvram_cy = g->cpuvram_y;
        g->cpuvram_byte_x = (g->cpuvram_x * 2u) & 0x7FFu;
        /* Track destination for diagnostic */
        {
            u32 _p = g->vram_dest_pos & 7u;
            g->vram_dests[_p] = (g->cpuvram_x << 16) | g->cpuvram_y;
            g->vram_sizes[_p] = (g->cpuvram_w << 16) | g->cpuvram_h;
            g->vram_dest_pos++;
        }
        if (g->cpuvram_w && g->cpuvram_h)
            g->cpuvram_active = 1;
        break;
    }

    /* VRAM → CPU copy */
    case 0xC0: case 0xC1:
        g->vramcpu_x  = g->fifo[1] & 0x3FFu;
        g->vramcpu_y  = (g->fifo[1] >> 16) & 0x1FFu;
        g->vramcpu_w  = g->fifo[2] & 0xFFFFu;
        g->vramcpu_h  = (g->fifo[2] >> 16) & 0xFFFFu;
        if (g->vramcpu_w == 0) g->vramcpu_w = 0x400u;
        if (g->vramcpu_h == 0) g->vramcpu_h = 0x200u;
        g->vramcpu_cx = g->vramcpu_x;
        g->vramcpu_cy = g->vramcpu_y;
        g->vramcpu_active = 1;
        break;

    /* Drawing settings */
    case 0xE1: /* Draw mode */
        g->draw_mode_word = g->fifo[0];
        g->texpage = g->fifo[0] & 0x09FFu;
        g->dither_enable = (g->fifo[0] >> 9) & 1u;
        g->draw_to_display = (g->fifo[0] >> 10) & 1u;
        g->rect_flip_x = (g->fifo[0] >> 12) & 1u;
        g->rect_flip_y = (g->fifo[0] >> 13) & 1u;
        /* Also update GPUSTAT bits 10:0 from texpage word */
        g->gpustat = (g->gpustat & ~0x87FFu) | (g->fifo[0] & 0x07FFu) | ((g->fifo[0] & 0x0800u) << 4);
        break;
    case 0xE2:
        g->texwin = g->fifo[0] & 0x000FFFFFu;
        break;
    case 0xE3: /* Drawing area top-left */
        g->draw_x1 = (s32)(g->fifo[0] & 0x3FFu);
        g->draw_y1 = (s32)((g->fifo[0] >> 10) & 0x3FFu);
        break;
    case 0xE4: /* Drawing area bottom-right */
        g->draw_x2 = (s32)(g->fifo[0] & 0x3FFu);
        g->draw_y2 = (s32)((g->fifo[0] >> 10) & 0x3FFu);
        break;
    case 0xE5: /* Drawing offset (signed 11-bit) */
        g->off_x = (s32)((g->fifo[0] & 0x7FFu) << 21) >> 21;
        g->off_y = (s32)(((g->fifo[0] >> 11) & 0x7FFu) << 21) >> 21;
        break;
    case 0xE6:
        g->mask_set = g->fifo[0] & 1u;
        g->mask_check = (g->fifo[0] >> 1) & 1u;
        break;
    default:   break;
    }
}

/* ─── GP0 write (called from psx_bus_write) ──────────────────────── */
static void gpu_gp0_write(struct psx_gpu_io *g, u32 val)
{
    /* Handle active CPU→VRAM copy: data arrives as pixel pairs */
    if (g->cpuvram_active) {
        if (g->disp_24bit) {
            u8 *vram8 = (u8 *)g->vram;
            u32 row_bytes = g->cpuvram_w * 3u;
            u32 row_start = (g->cpuvram_cy & 0x1FFu) * 2048u + (g->cpuvram_x * 2u);
            u32 cur_byte = g->cpuvram_byte_x;
            u8 bytes[4] = {
                (u8)(val & 0xFFu),
                (u8)((val >> 8) & 0xFFu),
                (u8)((val >> 16) & 0xFFu),
                (u8)((val >> 24) & 0xFFu)
            };
            for (int i = 0; i < 4; i++) {
                vram8[(row_start + cur_byte) & 0xFFFFFu] = bytes[i];
                cur_byte++;
                if (cur_byte >= row_bytes) {
                    cur_byte = 0;
                    g->cpuvram_cy++;
                    if (g->cpuvram_cy >= g->cpuvram_y + g->cpuvram_h) {
                        g->cpuvram_active = 0;
                        break;
                    }
                    row_start = (g->cpuvram_cy & 0x1FFu) * 2048u + (g->cpuvram_x * 2u);
                }
            }
            g->cpuvram_byte_x = cur_byte;
            return;
        }

        u16 p[2];
        p[0] = (u16)(val & 0xFFFFu);
        p[1] = (u16)(val >> 16);
        for (int i = 0; i < 2; i++) {
            u32 vx = g->cpuvram_cx & 0x3FFu;
            u32 vy = g->cpuvram_cy & 0x1FFu;
            
            /* capture CLUT for debug */
            if (vy == 384u && vx == 640u) g->clut_wr_dest = p[i];

            gpu_vram_xfer_write(g, vx, vy, p[i]);
            
            g->cpuvram_cx++;
            if (g->cpuvram_cx >= g->cpuvram_x + g->cpuvram_w) {
                g->cpuvram_cx = g->cpuvram_x;
                g->cpuvram_cy++;
                if (g->cpuvram_cy >= g->cpuvram_y + g->cpuvram_h) {
                    g->cpuvram_active = 0;
                    break;
                }
            }
        }
        return;
    }

    /* Add to FIFO */
    g->gp0_total++;  /* count every word received */
    if (g->fifo_len == 0) {
        u8 cmd = (u8)(val >> 24);
        g->fifo_need = gpu_is_polyline_cmd(cmd) ? 0xFFFFFFFFu : gpu_gp0_cmd_len(cmd);
    }
    if (g->fifo_len < 16u)
        g->fifo[g->fifo_len++] = val;

    if (g->fifo_need == 0xFFFFFFFFu) {
        u8 cmd = (u8)(g->fifo[0] >> 24);
        u32 min_words = (cmd >= 0x58 && cmd <= 0x5Fu) ? 4u : 3u;
        if (g->fifo_len >= min_words) {
            u32 term_index = 0xFFFFFFFFu;
            u32 start = (cmd >= 0x58 && cmd <= 0x5Fu) ? 2u : 2u;
            for (u32 i = start; i < g->fifo_len; i += ((cmd >= 0x58 && cmd <= 0x5Fu) ? 2u : 1u)) {
                if (gpu_polyline_terminator(g->fifo[i])) {
                    term_index = i;
                    break;
                }
            }
            if (term_index != 0xFFFFFFFFu) {
                g->fifo_len = term_index;
                gpu_exec_polyline(g);
                g->fifo_len = 0;
                g->fifo_need = 0;
            }
        }
    } else if (g->fifo_len >= g->fifo_need && g->fifo_need > 0) {
        gpu_gp0_exec(g);
        g->fifo_len  = 0;
        g->fifo_need = 0;
    }
}

/* ─── GP1 write (called from psx_bus_write) ──────────────────────── */
static void gpu_gp1_write(struct psx_gpu_io *g, u32 val)
{
    u32 cmd = val >> 24;
    switch (cmd) {
    case 0x00: /* Reset GPU */
        g->gpustat      = 0x1C000000u; /* bits 28,27,26=ready */
        g->gpustat     |= 0x02000000u; /* bit 25 = DMA ready */
        g->gpustat     |= 0x08000000u; /* bit 27 = IDLE */
        g->gpuread      = 0;
        g->texwin       = 0;
        g->fifo_len     = 0;
        g->fifo_need    = 0;
        g->cpuvram_active = 0;
        g->vramcpu_active = 0;
        g->disp_en      = 0;  /* display on by default */
        g->disp_vram_x  = 0;
        g->disp_vram_y  = 0;
        g->disp_w       = 320;
        g->disp_h       = 240;
        g->disp_24bit   = 0;
        g->disp_mode    = 0;
        g->disp_x1      = 0x200u;
        g->disp_x2      = 0x200u + 256u * 10u;
        g->disp_y1      = 0x010u;
        g->disp_y2      = 0x010u + 240u;
        g->draw_x1      = 0;  g->draw_y1 = 0;
        g->draw_x2      = 1023; g->draw_y2 = 511;
        g->off_x        = 0;  g->off_y   = 0;
        g->mask_set     = 0;
        g->mask_check   = 0;
        g->texpage      = 0;
        g->draw_mode_word = 0;
        g->rect_flip_x  = 0;
        g->rect_flip_y  = 0;
        g->dither_enable = 0;
        g->draw_to_display = 0;
        g->pal = 0;
        break;
    case 0x01: /* Reset command buffer */
        g->fifo_len  = 0;
        g->fifo_need = 0;
        break;
    case 0x02: /* Acknowledge IRQ */ break;
    case 0x03: /* Display enable: 0=on, 1=off */
        g->disp_en = val & 1u;
        if (g->disp_en) g->gpustat |=  (1u << 23);
        else            g->gpustat &= ~(1u << 23);
        break;
    case 0x04: /* DMA direction */
        g->gpustat = (g->gpustat & ~(3u << 29)) | ((val & 3u) << 29);
        break;
    case 0x05: /* Display start in VRAM */
        g->disp_vram_x = val & 0x3FFu;
        g->disp_vram_y = (val >> 10) & 0x1FFu;
        break;
    case 0x06:
        g->disp_x1 = val & 0xFFFu;
        g->disp_x2 = (val >> 12) & 0xFFFu;
        break;
    case 0x07:
        g->disp_y1 = val & 0x3FFu;
        g->disp_y2 = (val >> 10) & 0x3FFu;
        break;
    case 0x08: { /* Display mode – no static arrays to avoid rodata issues */
        /* bit[6]=1 → 368; bits[1:0]: 0=256, 1=320, 2=512, 3=640 */
        u32 h368 = (val >> 6) & 1u;
        u32 hw;
        if (h368) {
            hw = 368u;
        } else {
            switch (val & 3u) {
                case 1:  hw = 320u; break;
                case 2:  hw = 512u; break;
                case 3:  hw = 640u; break;
                default: hw = 256u; break;
            }
        }
        g->disp_mode = val & 0x7Fu;
        g->disp_w = hw;
        g->disp_24bit = (val >> 4) & 1u;
        g->pal = (val >> 3) & 1u;
        g->disp_h = ((val >> 2) & 1u) ? 480u : 240u;
        g->gpustat = (g->gpustat & ~0x7F800u) | ((val & 0x3Fu) << 17);
        break;
    }
    case 0x09:
    case 0x20:
        /* Set VRAM size (v2) – mostly for GPU detection; keep value observable. */
        g->gpustat = (g->gpustat & ~(3u << 15)) | ((val & 3u) << 15);
        break;
    case 0x10:
    case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
    case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E: case 0x1F: {
        u32 idx = val & 0xFFFFFFu;
        switch (idx & 0xFu) {
        case 0x02: g->gpuread = g->texwin; break;
        case 0x03: g->gpuread = ((u32)g->draw_y1 << 10) | ((u32)g->draw_x1 & 0x3FFu); break;
        case 0x04: g->gpuread = ((u32)g->draw_y2 << 10) | ((u32)g->draw_x2 & 0x3FFu); break;
        case 0x05: g->gpuread = (((u32)g->off_y & 0x7FFu) << 11) | ((u32)g->off_x & 0x7FFu); break;
        case 0x07: g->gpuread = 0x00000002u; break;
        case 0x08: g->gpuread = 0x00000000u; break;
        default: break;
        }
        break;
    }
    default:   break;
    }
}

/* ─── GPU init ────────────────────────────────────────────────────── */
static void psx_gpu_init(struct psx_system *psx, struct psx_gpu *gpu,
                         void *G, void *mmap_fn, u32 *fb0, u32 *fb1)
{
    struct psx_gpu_io *g = &psx->gpu_io;

    /* Allocate 1024×512×2 bytes of VRAM */
    g->vram = (u16 *)NC(G, mmap_fn, 0, 1024u * 512u * 2u, 3, 0x1002, (u64)-1, 0);
    if (!g->vram || (s64)g->vram == -1) { g->vram = 0; return; }
    psx_local_zero((u8 *)g->vram, 1024u * 512u * 2u);

    /* Reset GPU state */
    gpu_gp1_write(g, 0x00000000u); /* GP1 Reset */

    /* PS5 output */
    gpu->fb[0]  = fb0;
    gpu->fb[1]  = fb1;
    gpu->fb_idx = 0;
}

/* ─── Sync PSX VRAM → PS5 framebuffer ────────────────────────────── */
static void psx_gpu_sync_to_ps5(struct psx_system *psx, u32 *target_fb)
{
    struct psx_gpu_io *g = &psx->gpu_io;
    if (!g->vram || !target_fb) return;

    /* Always clear PS5 FB to black */
    for (u32 i = 0; i < 1920u * 1080u; i++) target_fb[i] = 0xFF000000u;

    u32 dw = g->disp_w  ? g->disp_w  : 320u;
    u32 dh = g->disp_h  ? g->disp_h  : 240u;
    u32 vx = g->disp_vram_x;
    u32 vy = g->disp_vram_y;

    /* PSX always outputs 4:3 on a CRT TV regardless of pixel resolution.
     * Both NTSC (320x240) and PAL (320x288) fill the same 4:3 screen area.
     * Fix: always pillarbox at 1440x1080 within 1920x1080 PS5 framebuffer. */
    u32 dst_w = 1440u;
    u32 dst_h = 1080u;
    u32 ox = 240u;
    u32 oy = 0u;

    u32 x_step = (dw << 16) / dst_w;
    u32 y_step = (dh << 16) / dst_h;

    u32 src_y_fp = 0;
    if (g->disp_24bit) {
        u8 *vram8 = (u8 *)g->vram;
        for (u32 py = 0; py < dst_h; py++, src_y_fp += y_step) {
            u32 src_y = vy + (src_y_fp >> 16);
            u32 row_base = (src_y & 0x1FFu) * 2048u + (vx * 2u);
            u32 *dst_row = target_fb + (oy + py) * 1920u + ox;
            u32 src_x_fp = 0;
            for (u32 px = 0; px < dst_w; px++, src_x_fp += x_step) {
                u32 pixel = (src_x_fp >> 16);
                u32 byte_off = row_base + pixel * 3u;
                u8 r = vram8[byte_off & 0xFFFFFu];
                u8 g8 = vram8[(byte_off + 1u) & 0xFFFFFu];
                u8 b = vram8[(byte_off + 2u) & 0xFFFFFu];
                dst_row[px] = 0xFF000000u | ((u32)r << 16) | ((u32)g8 << 8) | (u32)b;
            }
        }
    } else {
        for (u32 py = 0; py < dst_h; py++, src_y_fp += y_step) {
            u32 src_y = vy + (src_y_fp >> 16);
            const u16 *src_row = g->vram + (src_y & 0x1FFu) * 1024u;
            u32 *dst_row = target_fb + (oy + py) * 1920u + ox;
            u32 src_x_fp = vx << 16;
            for (u32 px = 0; px < dst_w; px++, src_x_fp += x_step) {
                dst_row[px] = gpu_rgb15_to_bgra8(src_row[(src_x_fp >> 16) & 0x3FFu]);
            }
        }
    }
}
