/* psx_main.c – PSX emulator payload for PS5 (fw 13.00)
 * GADGET_OFFSET / NC / SYM come from ps5_sdk_types.h (0x31AA9, fw13).
 */
#include "example_common.h"
#include "ps5_sdk/ps5_sdk_core.h"
#include "ps5_sdk/ps5_sdk_kernel.h"
#include "ps5_sdk/ps5_sdk_types.h"

#include "psx_bus.c"
#include "psx_cpu.c"
#include "psx_gpu.c"

/* ── utilities ────────────────────────────────────────────────────── */
static void psx_local_zero(u8 *p, u32 n) {
  while (n--)
    *p++ = 0;
}

static void psx_udp_log(void *G, void *sendto_fn, s32 fd, u8 *sa,
                        const char *msg) {
  if (fd < 0 || !sendto_fn || !msg)
    return;
  u32 len = 0;
  while (msg[len])
    len++;
  NC(G, sendto_fn, (u64)fd, (u64)msg, (u64)len, 0, (u64)sa, 16);
}

/* Log "prefix XXXXXXXX\n" without any static data (avoids rodata issues) */
static void psx_log_pc(void *G, void *sfn, s32 fd, u8 *sa, const char *pfx,
                       u32 val) {
  char buf[32];
  int i = 0;
  while (pfx[i] && i < 16) {
    buf[i] = pfx[i];
    i++;
  }
  for (int j = 28; j >= 0; j -= 4) {
    int n = (int)((val >> j) & 0xFu);
    buf[i++] = (char)(n < 10 ? ('0' + n) : ('A' + n - 10));
  }
  buf[i++] = '\n';
  buf[i] = '\0';
  psx_udp_log(G, sfn, fd, sa, buf);
}

static s32 psx_vid_open(void *G, void *vid_open) {
  s32 types[4] = {0xFF, 0, 1, 2};
  for (int i = 0; i < 4; i++) {
    s32 h = (s32)NC(G, vid_open, (u64)types[i], 0, 0, 0, 0, 0);
    if (h >= 0)
      return h;
  }
  return -1;
}

/* ── framebuffer layout ───────────────────────────────────────────── */
enum {
  PSX_FB_W = 1920,
  PSX_FB_H = 1080,
  PSX_FB_SIZE = PSX_FB_W * PSX_FB_H * 4,
  PSX_FB_ALIGNED = (PSX_FB_SIZE + 0x1FFFFF) & ~0x1FFFFF,
  PSX_FB_TOTAL = PSX_FB_ALIGNED * 2,
};

/* ── combined emulator state ──────────────────────────────────────── */
struct psx_state {
  struct psx_system sys;
  struct psx_gpu gpu;
};

/* ── entry point ──────────────────────────────────────────────────── */
__attribute__((section(".text._start"))) void
_start(u64 eboot_base, u64 dlsym_addr, struct ext_args *ext) {
  if (!ext)
    return;
  ext->step = 1;

  void *G = (void *)(eboot_base + GADGET_OFFSET);
  void *D = (void *)dlsym_addr;

  void *sendto_fn = SYM(G, D, LIBKERNEL_HANDLE, "sendto");
  void *usleep_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelUsleep);
  void *mmap_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_mmap);
  void *munmap_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_munmap);
  void *kopen_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelOpen);
  void *kread_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelRead);
  void *klseek_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelLseek);
  void *kclose_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelClose);
  void *load_mod = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelLoadStartModule);
  void *alloc_dm =
      SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelAllocateDirectMemory);
  void *map_dm = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelMapDirectMemory);
  void *rel_dm =
      SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelReleaseDirectMemory);
  void *cancel_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_scePthreadCancel);

  if (!mmap_fn || !usleep_fn)
    return;

#define LOG(m) psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, m)

  LOG("[PSX] Init\n");

  /* ── VideoOut ─────────────────────────────────────────────────── */
  if (load_mod)
    NC(G, load_mod, (u64) "libSceVideoOut.sprx", 0, 0, 0, 0, 0);
  void *vid_open = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutOpen");
  void *vid_close = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutClose");
  void *vid_reg = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutRegisterBuffers");
  void *vid_flip = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutSubmitFlip");
  void *vid_rate = SYM(G, D, VIDEOOUT_HANDLE, "sceVideoOutSetFlipRate");

  s32 emu_vid = *(s32 *)(eboot_base + EBOOT_VIDOUT);
  u64 gs_thread = *(u64 *)(eboot_base + EBOOT_GS_THREAD);

  s32 video_h = vid_open ? psx_vid_open(G, vid_open) : -1;
  if (video_h < 0) {
    if (cancel_fn && gs_thread)
      NC(G, cancel_fn, gs_thread, 0, 0, 0, 0, 0);
    NC(G, usleep_fn, 300000, 0, 0, 0, 0, 0);
    if (vid_close && emu_vid >= 0)
      NC(G, vid_close, (u64)emu_vid, 0, 0, 0, 0, 0);
    NC(G, usleep_fn, 100000, 0, 0, 0, 0, 0);
    video_h = vid_open ? psx_vid_open(G, vid_open) : -1;
  }
  if (video_h < 0) {
    LOG("[PSX] Error VideoOut\n");
    ext->status = -1;
    return;
  }
  LOG("[PSX] VideoOut OK\n");

  /* ── Framebuffers ─────────────────────────────────────────────── */
  u64 phys = 0;
  void *vmem = 0;
  if (alloc_dm)
    NC(G, alloc_dm, 0, 0x300000000ULL, PSX_FB_TOTAL, 0x200000, 3, (u64)&phys);
  if (phys && map_dm)
    NC(G, map_dm, (u64)&vmem, PSX_FB_TOTAL, 0x33, 0, phys, 0x200000);
  if (!vmem) {
    LOG("[PSX] Error FB\n");
    ext->status = -2;
    goto cleanup_vid;
  }

  void *fbs[2] = {vmem, (u8 *)vmem + PSX_FB_ALIGNED};
  {
    u8 attr[64];
    psx_local_zero(attr, sizeof(attr));
    *(u32 *)(attr + 0) = 0x80000000u;
    *(u32 *)(attr + 4) = 1;
    *(u32 *)(attr + 12) = PSX_FB_W;
    *(u32 *)(attr + 16) = PSX_FB_H;
    *(u32 *)(attr + 20) = PSX_FB_W;

    if ((s32)NC(G, vid_reg, (u64)video_h, 0, (u64)fbs, 2, (u64)attr, 0) != 0) {
      LOG("[PSX] Error vid_reg\n");
      ext->status = -3;
      goto cleanup_fb;
    }
    if (vid_rate)
      NC(G, vid_rate, (u64)video_h, 0, 0, 0, 0, 0);
    LOG("[PSX] FB OK\n");
  }

  /* ── PSX state ────────────────────────────────────────────── */
  struct psx_state *st = (struct psx_state *)NC(
      G, mmap_fn, 0, sizeof(struct psx_state), 3, 0x1002, (u64)-1, 0);
  if (!st || (s64)st == -1) {
    LOG("[PSX] Error: state alloc\n");
    ext->status = -4;
    goto cleanup_fb;
  }
  psx_local_zero((u8 *)st, sizeof(struct psx_state));

  st->sys.ram = (u8 *)NC(G, mmap_fn, 0, 0x200000, 3, 0x1002, (u64)-1, 0);
  st->sys.bios = (u8 *)NC(G, mmap_fn, 0, 0x80000, 3, 0x1002, (u64)-1, 0);
  st->sys.scratch = (u8 *)NC(G, mmap_fn, 0, 0x400, 3, 0x1002, (u64)-1, 0);
  if (!st->sys.ram || (s64)st->sys.ram == -1 || !st->sys.bios ||
      (s64)st->sys.bios == -1 || !st->sys.scratch ||
      (s64)st->sys.scratch == -1) {
    LOG("[PSX] Error: mem\n");
    ext->status = -5;
    goto cleanup_fb;
  }

  /* ── Load BIOS ────────────────────────────────────────────── */
  const char *bpaths[2] = {
      "/data/psx/bios/SCPH7502.bin",
      "/temp0/psx/bios/SCPH7502.bin",
  };
  s32 fd = -1;
  for (int i = 0; i < 2; i++) {
    fd = (s32)NC(G, kopen_fn, (u64)bpaths[i], 0, 0, 0, 0, 0);
    if (fd > 0)
      break;
  }
  if (fd <= 0) {
    LOG("[PSX] Error: BIOS\n");
    ext->status = -6;
    goto cleanup_fb;
  }
  NC(G, kread_fn, (u64)fd, (u64)st->sys.bios, 0x80000, 0, 0, 0);
  NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
  LOG("[PSX] BIOS cargada\n");

  /* ── CPU reset ────────────────────────────────────────────── */
  st->sys.pc = 0xBFC00000u;
  st->sys.next_pc = 0xBFC00004u;
  st->sys.cp0_regs[12] = 0x10900000u; /* Status: BEV=1, IE=0 */
  st->sys.G = G;
  st->sys.kopen_fn = kopen_fn;
  st->sys.klseek_fn = klseek_fn;
  st->sys.kread_fn = kread_fn;

  /* ── CD-ROM Game load ─────────────────────────────────────────── */
  st->sys.cd_fd =
      (s32)NC(G, kopen_fn, (u64) "/temp0/psx/games/game.bin", 0, 0, 0, 0, 0);
  if (st->sys.cd_fd > 0)
    LOG("[PSX] Game.bin abierto\n");
  else
    LOG("[PSX] WARN: game.bin no encontrado\n");

  /* ── GPU init ─────────────────────────────────────────────── */
  psx_gpu_init(&st->sys, &st->gpu, G, mmap_fn, (u32 *)fbs[0], (u32 *)fbs[1]);
  if (!st->sys.gpu_io.vram)
    LOG("[PSX] WARN: VRAM null\n");
  LOG("[PSX] Emulando\n");

#define LOGPC(pfx)                                                             \
  psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, pfx, st->sys.pc)

  /* ── Main loop ────────────────────────────────────────────── */
  u32 vblank = 0;
  for (;;) {
    psx_bus_vblank(&st->sys);
    vblank++;

    /* Re-inject CLUT: BIOS shell needs this for text visibility */
    if (st->sys.gpu_io.vram) {
      u16 *cv = st->sys.gpu_io.vram + 384u * 1024u + 640u;
      for (int k = 1; k < 16; k++)
        cv[k] = 0x7FFFu;
    }

    for (u32 i = 0; i < 564480u; i++)
      psx_cpu_step(&st->sys);

    /* ── GPU sync & Flip ────────────────────────────────────── */
    u32 idx = st->gpu.fb_idx & 1u;
    psx_gpu_sync_to_ps5(&st->sys, (u32 *)fbs[idx]);
    NC(G, vid_flip, (u64)video_h, (u64)idx, 1, 0, 0, 0);
    st->gpu.fb_idx++;

    /* Log every 30 VBLANKs (~0.5 seconds) */
    if ((vblank % 30u) == 0) {
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "vbl=", vblank);
      LOGPC("PC=");
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "gp0=", st->sys.gpu_io.gp0_total);
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "lc=", st->sys.gpu_io.last_cmd);
      /* Sample VRAM for text rendering detection */
      if (st->sys.gpu_io.vram) {
        u16 *_v = st->sys.gpu_io.vram;
        psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                   "t230=", _v[230u * 1024u + 73u]);
      }
    }
  }
#undef LOGPC

cleanup_fb:
  if (vmem && munmap_fn)
    NC(G, munmap_fn, (u64)vmem, PSX_FB_TOTAL, 0, 0, 0, 0);
  if (phys && rel_dm)
    NC(G, rel_dm, phys, PSX_FB_TOTAL, 0, 0, 0, 0);
cleanup_vid:
  if (vid_close && video_h >= 0)
    NC(G, vid_close, (u64)video_h, 0, 0, 0, 0, 0);
#undef LOG
}
