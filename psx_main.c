/* psx_main.c – PSX emulator payload for PS5 (fw 13.00)
 * GADGET_OFFSET / NC / SYM come from ps5_sdk_types.h (0x31AA9, fw13).
 */
#include "ps5_sdk/ps5_sdk_types.h"
#include "ps5_sdk/ps5_sdk_core.h"
#include "ps5_sdk/ps5_sdk_kernel.h"
#include "example_common.h"

#include "psx_bus.c"
#include "psx_cpu.c"
#include "psx_gpu.c"

/* ── utilities ────────────────────────────────────────────────────── */
static void psx_local_zero(u8 *p, u32 n) { while (n--) *p++ = 0; }

static void psx_udp_log(void *G, void *sendto_fn, s32 fd,
                        u8 *sa, const char *msg)
{
    if (fd < 0 || !sendto_fn || !msg) return;
    u32 len = 0; while (msg[len]) len++;
    NC(G, sendto_fn, (u64)fd, (u64)msg, (u64)len, 0, (u64)sa, 16);
}

/* Log "prefix XXXXXXXX\n" without any static data (avoids rodata issues) */
static void psx_log_pc(void *G, void *sfn, s32 fd, u8 *sa,
                       const char *pfx, u32 val)
{
    char buf[32];
    int i = 0;
    while (pfx[i] && i < 16) { buf[i] = pfx[i]; i++; }
    for (int j = 28; j >= 0; j -= 4) {
        int n = (int)((val >> j) & 0xFu);
        buf[i++] = (char)(n < 10 ? ('0' + n) : ('A' + n - 10));
    }
    buf[i++] = '\n'; buf[i] = '\0';
    psx_udp_log(G, sfn, fd, sa, buf);
}

static s32 psx_vid_open(void *G, void *vid_open)
{
    s32 types[4] = { 0xFF, 0, 1, 2 };
    for (int i = 0; i < 4; i++) {
        s32 h = (s32)NC(G, vid_open, (u64)types[i], 0, 0, 0, 0, 0);
        if (h >= 0) return h;
    }
    return -1;
}

/* ── framebuffer layout ───────────────────────────────────────────── */
enum {
    PSX_FB_W       = 1920,
    PSX_FB_H       = 1080,
    PSX_FB_SIZE    = PSX_FB_W * PSX_FB_H * 4,
    PSX_FB_ALIGNED = (PSX_FB_SIZE + 0x1FFFFF) & ~0x1FFFFF,
    PSX_FB_TOTAL   = PSX_FB_ALIGNED * 2,
};

/* ── combined emulator state ──────────────────────────────────────── */
struct psx_state {
    struct psx_system sys;
    struct psx_gpu    gpu;
};

/* ── entry point ──────────────────────────────────────────────────── */
__attribute__((section(".text._start")))
void _start(u64 eboot_base, u64 dlsym_addr, struct ext_args *ext)
{
    if (!ext) return;
    ext->step = 1;

    void *G = (void *)(eboot_base + GADGET_OFFSET);
    void *D = (void *)dlsym_addr;

    void *sendto_fn  = SYM(G, D, LIBKERNEL_HANDLE, "sendto");
    void *usleep_fn  = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelUsleep);
    void *mmap_fn    = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_mmap);
    void *munmap_fn  = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_munmap);
    void *kopen_fn   = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelOpen);
    void *kread_fn   = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelRead);
    void *kclose_fn  = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelClose);
    void *load_mod   = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelLoadStartModule);
    void *alloc_dm   = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelAllocateDirectMemory);
    void *map_dm     = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelMapDirectMemory);
    void *rel_dm     = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelReleaseDirectMemory);
    void *cancel_fn  = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_scePthreadCancel);

    if (!mmap_fn || !usleep_fn) return;

#define LOG(m) psx_udp_log(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, m)

    LOG("[PSX] Init\n");

    /* ── VideoOut ─────────────────────────────────────────────────── */
    if (load_mod) NC(G, load_mod, (u64)"libSceVideoOut.sprx", 0, 0, 0, 0, 0);
    void *vid_open  = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutOpen");
    void *vid_close = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutClose");
    void *vid_reg   = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutRegisterBuffers");
    void *vid_flip  = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutSubmitFlip");
    void *vid_rate  = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutSetFlipRate");

    s32 emu_vid   = *(s32 *)(eboot_base + EBOOT_VIDOUT);
    u64 gs_thread = *(u64 *)(eboot_base + EBOOT_GS_THREAD);

    s32 video_h = vid_open ? psx_vid_open(G, vid_open) : -1;
    if (video_h < 0) {
        if (cancel_fn && gs_thread) NC(G, cancel_fn, gs_thread, 0, 0, 0, 0, 0);
        NC(G, usleep_fn, 300000, 0, 0, 0, 0, 0);
        if (vid_close && emu_vid >= 0)
            NC(G, vid_close, (u64)emu_vid, 0, 0, 0, 0, 0);
        NC(G, usleep_fn, 100000, 0, 0, 0, 0, 0);
        video_h = vid_open ? psx_vid_open(G, vid_open) : -1;
    }
    if (video_h < 0) { LOG("[PSX] Error VideoOut\n"); ext->status=-1; return; }
    LOG("[PSX] VideoOut OK\n");

    /* ── Framebuffers ─────────────────────────────────────────────── */
    u64   phys = 0;
    void *vmem = 0;
    if (alloc_dm)
        NC(G, alloc_dm, 0, 0x300000000ULL, PSX_FB_TOTAL, 0x200000, 3, (u64)&phys);
    if (phys && map_dm)
        NC(G, map_dm, (u64)&vmem, PSX_FB_TOTAL, 0x33, 0, phys, 0x200000);
    if (!vmem) { LOG("[PSX] Error FB\n"); ext->status=-2; goto cleanup_vid; }

    {
        u8 attr[64];
        psx_local_zero(attr, sizeof(attr));
        *(u32 *)(attr +  0) = 0x80000000u;
        *(u32 *)(attr +  4) = 1;
        *(u32 *)(attr + 12) = PSX_FB_W;
        *(u32 *)(attr + 16) = PSX_FB_H;
        *(u32 *)(attr + 20) = PSX_FB_W;
        void *fbs[2] = { vmem, (u8 *)vmem + PSX_FB_ALIGNED };

        if ((s32)NC(G, vid_reg, (u64)video_h, 0, (u64)fbs, 2, (u64)attr, 0) != 0) {
            LOG("[PSX] Error vid_reg\n"); ext->status=-3; goto cleanup_fb;
        }
        if (vid_rate) NC(G, vid_rate, (u64)video_h, 0, 0, 0, 0, 0);
        LOG("[PSX] FB OK\n");

        /* ── PSX state ────────────────────────────────────────────── */
        struct psx_state *st = (struct psx_state *)NC(
            G, mmap_fn, 0, sizeof(struct psx_state), 3, 0x1002, (u64)-1, 0);
        if (!st || (s64)st == -1) {
            LOG("[PSX] Error: state alloc\n"); ext->status=-4; goto cleanup_fb;
        }
        psx_local_zero((u8 *)st, sizeof(struct psx_state));

        st->sys.ram     = (u8 *)NC(G, mmap_fn, 0, 0x200000, 3, 0x1002, (u64)-1, 0);
        st->sys.bios    = (u8 *)NC(G, mmap_fn, 0, 0x80000,  3, 0x1002, (u64)-1, 0);
        st->sys.scratch = (u8 *)NC(G, mmap_fn, 0, 0x400,    3, 0x1002, (u64)-1, 0);
        if (!st->sys.ram     || (s64)st->sys.ram     == -1 ||
            !st->sys.bios    || (s64)st->sys.bios    == -1 ||
            !st->sys.scratch || (s64)st->sys.scratch == -1) {
            LOG("[PSX] Error: mem\n"); ext->status=-5; goto cleanup_fb;
        }

        /* ── Load BIOS ────────────────────────────────────────────── */
        const char *bpaths[2] = {
            "/data/psx/bios/SCPH7502.bin",
            "/temp0/psx/bios/SCPH7502.bin",
        };
        s32 fd = -1;
        for (int i = 0; i < 2; i++) {
            fd = (s32)NC(G, kopen_fn, (u64)bpaths[i], 0, 0, 0, 0, 0);
            if (fd > 0) break;
        }
        if (fd <= 0) { LOG("[PSX] Error: BIOS\n"); ext->status=-6; goto cleanup_fb; }
        NC(G, kread_fn,  (u64)fd, (u64)st->sys.bios, 0x80000, 0, 0, 0);
        NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
        LOG("[PSX] BIOS cargada\n");

        /* ── CPU reset ────────────────────────────────────────────── */
        st->sys.pc      = 0xBFC00000u;
        st->sys.next_pc = 0xBFC00004u;
        st->sys.cp0_regs[12] = 0x10900000u; /* Status: BEV=1, IE=0 */

        /* ── GPU init ─────────────────────────────────────────────── */
        psx_gpu_init(&st->sys, &st->gpu, G, mmap_fn,
                     (u32 *)fbs[0], (u32 *)fbs[1]);
        if (!st->sys.gpu_io.vram)
            LOG("[PSX] WARN: VRAM null\n");
        LOG("[PSX] Emulando\n");

#define LOGPC(pfx) psx_log_pc(G, sendto_fn, ext->log_fd, \
                              (u8*)ext->log_addr, pfx, st->sys.pc)

        /* ── Main loop ────────────────────────────────────────────── */
        /* PSX: 33.868 MHz / 60 Hz = 564,480 cycles per VBLANK */
        u32 vblank = 0;
        for (;;) {
            psx_bus_vblank(&st->sys);

            /* Inject CLUT BEFORE CPU runs: BIOS zeroes CLUT during execution
             * (no disc → no real colors), so we must pre-populate each frame.
             * CLUT protection in psx_gpu.c blocks the subsequent zero-writes. */
            if (vblank > 300u && st->sys.gpu_io.vram) {
                u16 *cv = st->sys.gpu_io.vram + 384u*1024u + 640u;
                int _any = 0;
                for (int _ci = 1; _ci < 16; _ci++) if (cv[_ci]) { _any = 1; break; }
                if (!_any) {
                    cv[0]  = 0x0000u; cv[1]  = 0x7FFFu; cv[2]  = 0x6318u; cv[3]  = 0x4210u;
                    cv[4]  = 0x001Fu; cv[5]  = 0x03E0u; cv[6]  = 0x7C00u; cv[7]  = 0x7FE0u;
                    cv[8]  = 0x5294u; cv[9]  = 0x3DEFu; cv[10] = 0x2108u; cv[11] = 0x7FFFu;
                    cv[12] = 0x7FFFu; cv[13] = 0x7FFFu; cv[14] = 0x7FFFu; cv[15] = 0x7FFFu;
                }
            }

            for (u32 i = 0; i < 564480u; i++)
                psx_cpu_step(&st->sys);

            /* Log every 60 VBLANKs (~1 second) */
            if ((vblank % 60u) == 0) {
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "vbl=", vblank);
                LOGPC("PC=");
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "gp0=", st->sys.gpu_io.gp0_total);
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "rec=", st->sys.gpu_io.rect_cnt);
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "lc=",  st->sys.gpu_io.last_cmd);
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dw=",  st->sys.gpu_io.disp_w);
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dh=",  st->sys.gpu_io.disp_h);
                /* VRAM: framebuffer first non-zero + texture area */
                if (st->sys.gpu_io.vram) {
                    u16 *_v = st->sys.gpu_io.vram;
                    u32 _fb=0;
                    for (u32 _r=0; _r<480u && !_fb; _r++)
                        for (u32 _c=0; _c<640u && !_fb; _c++)
                            if (_v[_r*1024u+_c]) _fb=(u32)_v[_r*1024u+_c]|((_r<<10)+_c);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "fb=", _fb);
                    /* Texture+CLUT area: first non-zero in rows 0-239, cols 640-767 */
                    u32 _tx=0;
                    for (u32 _r=0; _r<240u && !_tx; _r++)
                        for (u32 _c=640u; _c<768u && !_tx; _c++)
                            if (_v[_r*1024u+_c]) _tx=(u32)_v[_r*1024u+_c]|((_r<<10)+(_c-640u));
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "tx=", _tx);
                    /* CLUT at row 384, cols 640-655: dump all 16 entries as 8 pairs */
                    {
                        u16 *cv = _v + 384u*1024u + 640u;
                        psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "c01=", ((u32)cv[1]<<16)|cv[0]);
                        psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "c23=", ((u32)cv[3]<<16)|cv[2]);
                        psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "cEF=", ((u32)cv[15]<<16)|cv[14]);
                    }
                    /* Drawing config: display Y, full draw bounds, offset */
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dvy=", st->sys.gpu_io.disp_vram_y);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dy1=", (u32)st->sys.gpu_io.draw_y1);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dy2=", (u32)st->sys.gpu_io.draw_y2);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dx1=", (u32)st->sys.gpu_io.draw_x1);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "dx2=", (u32)st->sys.gpu_io.draw_x2);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "oy=",  (u32)(s32)st->sys.gpu_io.off_y);
                    /* Sample 4 pixels in text-quad area (rows 226-239) to see if it rendered */
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "t226=", _v[226u*1024u+73u]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "t230=", _v[230u*1024u+73u]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "t235=", _v[235u*1024u+73u]);
                }
                /* Last 8 CPU→VRAM transfer destinations (find the CLUT upload) */
                for (u32 _i = 0; _i < 8u; _i++) {
                    u32 _di = (st->sys.gpu_io.vram_dest_pos - 8u + _i) & 7u;
                    if (st->sys.gpu_io.vram_dests[_di]) {
                        psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr,
                            "vd=", st->sys.gpu_io.vram_dests[_di]);
                        psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr,
                            "vs=", st->sys.gpu_io.vram_sizes[_di]);
                    }
                }
                /* Last textured quad (0x2C): xy0, uv0+clut, xy1, texpage, xy3, uv3 */
                {
                    u32 *q = st->sys.gpu_io.last_quad;
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "qxy0=", q[1]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "quv0=", q[2]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "qxy1=", q[3]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "qtp=", (u32)(q[4]>>16));
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "qxy3=", q[7]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "quv3=", q[8]);
                }
                /* Last sprite (0x74): xy, uv+clut */
                {
                    u32 *s = st->sys.gpu_io.last_spr;
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "sxy=", s[1]);
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "suv=", s[2]);
                }
                /* CLUT write capture: destination and first word written to row 384 */
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "cwrd=", st->sys.gpu_io.clut_wr_dest);
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8*)ext->log_addr, "cwsm=", st->sys.gpu_io.clut_wr_sample);
            }

            /* CLUT injection: only if ALL 16 entries are zero (index 0 being
             * transparent is normal; BIOS may legitimately write colors 1-15). */
            u32 idx = st->gpu.fb_idx & 1u;
            psx_gpu_sync_to_ps5(&st->sys, (u32 *)fbs[idx]);
            NC(G, vid_flip, (u64)video_h, (u64)idx, 1, 0, 0, 0);
            st->gpu.fb_idx++;
            vblank++;
        }
#undef LOGPC
    }

cleanup_fb:
    if (vmem && munmap_fn) NC(G, munmap_fn, (u64)vmem, PSX_FB_TOTAL, 0, 0, 0, 0);
    if (phys && rel_dm)    NC(G, rel_dm, phys, PSX_FB_TOTAL, 0, 0, 0, 0);
cleanup_vid:
    if (vid_close && video_h >= 0)
        NC(G, vid_close, (u64)video_h, 0, 0, 0, 0, 0);
#undef LOG
}
