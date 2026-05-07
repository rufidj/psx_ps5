/* psx_main.c – PSX emulator payload for PS5 (fw 13.00)
 * GADGET_OFFSET / NC / SYM come from ps5_sdk_types.h (0x31AA9, fw13).
 */
#include "example_common.h"
#include "ps5_sdk/ps5_sdk_core.h"
#include "ps5_sdk/ps5_sdk_kernel.h"
#include "ps5_sdk/ps5_sdk_types.h"

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
  char buf[64];
  int i = 0;
  while (pfx[i] && i < 32) {
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

#include "psx_bus.c"
#include "psx_cpu.c"
#include "psx_gpu.c"

static u32 psx_le32(const u8 *p) {
  return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static char psx_ascii_upper(char c) {
  if (c >= 'a' && c <= 'z')
    return (char)(c - 32);
  return c;
}

static int psx_name_eq(const char *a, const u8 *b, u32 blen) {
  u32 i = 0;
  for (; i < blen && a[i]; i++) {
    if (psx_ascii_upper(a[i]) != psx_ascii_upper((char)b[i]))
      return 0;
  }
  return (i == blen && a[i] == '\0');
}

static int psx_read_logical_sector(struct psx_system *psx, u32 lba, u8 *dst) {
  if (!psx->kread_fn || !psx->klseek_fn || psx->cd_fd <= 0)
    return 0;

  u64 off = (u64)lba * (u64)psx->cd_sector_size;
  if (psx->cd_sector_size == 2352)
    off += 24u;

  NC(psx->G, psx->klseek_fn, (u64)psx->cd_fd, off, 0, 0, 0, 0);
  return ((s32)NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)dst, 2048, 0, 0,
                  0) == 2048);
}

static int psx_read_file_bytes(struct psx_system *psx, u32 file_lba,
                               u32 file_off, u8 *dst, u32 len) {
  u8 sec[2048];
  u32 done = 0;

  while (done < len) {
    u32 lba = file_lba + ((file_off + done) >> 11);
    u32 sec_off = (file_off + done) & 2047u;
    u32 chunk;

    if (!psx_read_logical_sector(psx, lba, sec))
      return 0;

    chunk = 2048u - sec_off;
    if (chunk > (len - done))
      chunk = len - done;

    for (u32 i = 0; i < chunk; i++)
      dst[done + i] = sec[sec_off + i];
    done += chunk;
  }

  return 1;
}

static int psx_iso_find_entry(struct psx_system *psx, u32 dir_lba, u32 dir_size,
                              const char *name, u32 *out_lba, u32 *out_size,
                              u32 *out_flags) {
  u8 sec[2048];
  for (u32 ofs = 0; ofs < dir_size; ofs += 2048u) {
    if (!psx_read_logical_sector(psx, dir_lba + (ofs >> 11), sec))
      return 0;
    u32 pos = 0;
    while (pos < 2048u) {
      u32 len = sec[pos];
      if (!len)
        break;
      if (pos + len > 2048u)
        break;
      u32 name_len = sec[pos + 32];
      const u8 *entry_name = sec + pos + 33;
      if (name_len > 0 && entry_name[0] > 1) {
        if (psx_name_eq(name, entry_name, name_len)) {
          if (out_lba)
            *out_lba = psx_le32(sec + pos + 2);
          if (out_size)
            *out_size = psx_le32(sec + pos + 10);
          if (out_flags)
            *out_flags = sec[pos + 25];
          return 1;
        }
      }
      pos += len;
    }
  }
  return 0;
}

static int psx_iso_lookup_path(struct psx_system *psx, const char *path,
                               u32 *out_lba, u32 *out_size) {
  u8 pvd[2048];
  u32 cur_lba, cur_size, cur_flags = 2;
  char part[64];
  u32 part_len = 0;
  u32 idx = 0;

  if (!psx_read_logical_sector(psx, 16, pvd))
    return 0;
  if (pvd[0] != 1 || pvd[1] != 'C' || pvd[2] != 'D' || pvd[3] != '0' ||
      pvd[4] != '0' || pvd[5] != '1')
    return 0;

  cur_lba = psx_le32(pvd + 158);
  cur_size = psx_le32(pvd + 166);

  while (path[idx]) {
    char c = path[idx++];
    if (c == '\\' || c == '/') {
      if (!part_len)
        continue;
      part[part_len] = '\0';
      if (!(cur_flags & 2u))
        return 0;
      if (!psx_iso_find_entry(psx, cur_lba, cur_size, part, &cur_lba, &cur_size,
                              &cur_flags))
        return 0;
      part_len = 0;
      continue;
    }
    if (part_len + 1u < sizeof(part))
      part[part_len++] = psx_ascii_upper(c);
  }

  if (part_len) {
    part[part_len] = '\0';
    if (!(cur_flags & 2u))
      return 0;
    if (!psx_iso_find_entry(psx, cur_lba, cur_size, part, &cur_lba, &cur_size,
                            &cur_flags))
      return 0;
  }

  if (out_lba)
    *out_lba = cur_lba;
  if (out_size)
    *out_size = cur_size;
  return 1;
}

static int psx_extract_boot_path(const u8 *buf, u32 len, char *out,
                                 u32 out_cap) {
  for (u32 i = 0; i + 4 < len; i++) {
    if (psx_ascii_upper((char)buf[i]) != 'B' ||
        psx_ascii_upper((char)buf[i + 1]) != 'O' ||
        psx_ascii_upper((char)buf[i + 2]) != 'O' ||
        psx_ascii_upper((char)buf[i + 3]) != 'T')
      continue;

    u32 p = i + 4;
    while (p < len && buf[p] != '=' && buf[p] != '\n' && buf[p] != '\r')
      p++;
    if (p >= len || buf[p] != '=')
      continue;
    p++;
    while (p < len && (buf[p] == ' ' || buf[p] == '\t'))
      p++;
    if (p + 6 >= len)
      continue;

    if (psx_ascii_upper((char)buf[p]) == 'C' &&
        psx_ascii_upper((char)buf[p + 1]) == 'D' &&
        psx_ascii_upper((char)buf[p + 2]) == 'R' &&
        psx_ascii_upper((char)buf[p + 3]) == 'O' &&
        psx_ascii_upper((char)buf[p + 4]) == 'M') {
      while (p < len && buf[p] != '\\' && buf[p] != '/')
        p++;
    }

    u32 o = 0;
    while (p < len && buf[p] != '\n' && buf[p] != '\r' && buf[p] != ';') {
      if (o + 2u >= out_cap)
        break;
      out[o++] = (buf[p] == '/') ? '\\' : psx_ascii_upper((char)buf[p]);
      p++;
    }
    if (p < len && buf[p] == ';' && o + 3u < out_cap) {
      out[o++] = ';';
      p++;
      while (p < len && buf[p] >= '0' && buf[p] <= '9' && o + 1u < out_cap)
        out[o++] = (char)buf[p++];
    }
    out[o] = '\0';
    return (o > 0);
  }
  return 0;
}

static u32 psx_region_from_serial_char(char c)
{
  c = psx_ascii_upper(c);
  if (c == 'U')
    return 1u;
  if (c == 'E')
    return 2u;
  if (c == 'P')
    return 0u;
  return 3u;
}

static u32 psx_region_from_boot_path(const char *boot_path)
{
  u32 i;
  if (!boot_path)
    return 3u;
  for (i = 0; boot_path[i]; i++) {
    if (boot_path[i] == 'S' || boot_path[i] == 's') {
      char c0 = psx_ascii_upper(boot_path[i + 0]);
      char c1 = psx_ascii_upper(boot_path[i + 1]);
      char c2 = psx_ascii_upper(boot_path[i + 2]);
      char c3 = psx_ascii_upper(boot_path[i + 3]);
      if (c0 == 'S' && c1 == 'L' && c2 == 'U' && c3 == 'S') return 1u;
      if (c0 == 'S' && c1 == 'C' && c2 == 'U' && c3 == 'S') return 1u;
      if (c0 == 'S' && c1 == 'L' && c2 == 'E' && c3 == 'S') return 2u;
      if (c0 == 'S' && c1 == 'C' && c2 == 'E' && c3 == 'S') return 2u;
      if (c0 == 'S' && c1 == 'L' && c2 == 'P' && c3 == 'S') return 0u;
      if (c0 == 'S' && c1 == 'C' && c2 == 'P' && c3 == 'S') return 0u;
    }
  }
  return 3u;
}

static void psx_detect_disc_boot_info(struct psx_system *psx)
{
  u8 sec[2048];
  u32 cnf_lba, cnf_size;

  psx->cd_disc_region = 3u;
  psx->cd_boot_path[0] = '\0';

  if (!psx_iso_lookup_path(psx, "SYSTEM.CNF;1", &cnf_lba, &cnf_size))
    return;
  if (!psx_read_logical_sector(psx, cnf_lba, sec))
    return;
  if (!psx_extract_boot_path(sec, (cnf_size < sizeof(sec)) ? cnf_size : sizeof(sec),
                             psx->cd_boot_path, sizeof(psx->cd_boot_path)))
    return;

  psx->cd_disc_region = psx_region_from_boot_path(psx->cd_boot_path);
}

static int psx_direct_boot_game(struct psx_system *psx) {
  u8 sec[2048];
  char boot_path[96];
  u32 cnf_lba, cnf_size, exe_lba, exe_size;

  if (!psx_iso_lookup_path(psx, "SYSTEM.CNF;1", &cnf_lba, &cnf_size))
    return 0;
  if (!psx_read_logical_sector(psx, cnf_lba, sec))
    return 0;
  if (!psx_extract_boot_path(sec, (cnf_size < sizeof(sec)) ? cnf_size : sizeof(sec),
                             boot_path, sizeof(boot_path)))
    return 0;
  if (!psx_iso_lookup_path(psx, boot_path, &exe_lba, &exe_size))
    return 0;
  if (!psx_read_logical_sector(psx, exe_lba, sec))
    return 0;

  if (sec[0] != 'P' || sec[1] != 'S' || sec[2] != '-' || sec[3] != 'X')
    return 0;

  u32 pc0 = psx_le32(sec + 0x10);
  u32 gp0 = psx_le32(sec + 0x14);
  u32 text_addr = psx_le32(sec + 0x18);
  u32 text_size = psx_le32(sec + 0x1C);
  u32 bss_addr = psx_le32(sec + 0x28);
  u32 bss_size = psx_le32(sec + 0x2C);
  u32 sp_base = psx_le32(sec + 0x30);
  u32 sp_size = psx_le32(sec + 0x34);
  u32 sp = sp_base ? (sp_base + sp_size) : 0x801FFF00u;
  u32 text_phys = psx_bus_mask(text_addr);

  if (gp0 == 0u && text_size >= 0x48u)
    gp0 = text_addr + text_size - 0x48u;

  if (!text_size || text_phys >= 0x200000u)
    return 0;
  if (text_phys + text_size > 0x200000u)
    return 0;

  if (!psx_read_file_bytes(psx, exe_lba, 0x800u, psx->ram + text_phys, text_size))
    return 0;

  if (bss_size) {
    u32 bss_phys = psx_bus_mask(bss_addr);
    if (bss_phys < 0x200000u && bss_phys + bss_size <= 0x200000u)
      psx_local_zero(psx->ram + bss_phys, bss_size);
  }

  for (u32 i = 0; i < 32u; i++)
    psx->regs[i] = 0;
  psx->hi = 0;
  psx->lo = 0;
  psx->pc = pc0;
  psx->next_pc = pc0 + 4u;
  psx->regs[28] = gp0;
  psx->regs[29] = sp;
  psx->regs[30] = sp;
  psx->load_pending_reg = 0;
  psx->load_pending_value = 0;
  psx->bad_fetch_logged = 0;
  psx->bad_jump_logged = 0;
  psx->low_pc_fix_logged = 0;
  psx->exc_logged = 0;
  psx->launch_window_logged = 0;
  psx->i_stat = 0;
  psx->cp0_regs[13] = 0;
  psx->cp0_regs[12] &= ~1u; /* Disable IE on handoff; game will enable IRQs when ready */
  psx->cd_boot_trace = 1u;
  psx->exe_entry_pc = pc0;
  psx->exe_text_addr = text_addr;
  psx->exe_text_size = text_size;
  psx->direct_boot_state = 1u;
  psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
             "[PSX] EXE gp=", gp0);
  psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
             "[PSX] EXE taddr=", text_addr);
  psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
             "[PSX] EXE tsize=", text_size);
  psx_log_pc(psx->G, psx->sendto_fn, psx->log_fd, psx->log_addr,
             "[PSX] EXE sp=", sp);
  return 1;
}

static int psx_bios_lowmem_ready(struct psx_system *psx) {
  if (!psx->ram)
    return 0;
  if (*(u32 *)(psx->ram + 0x80) == 0)
    return 0;
  if (*(u32 *)(psx->ram + 0xA0) == 0)
    return 0;
  if (*(u32 *)(psx->ram + 0xB0) == 0)
    return 0;
  if (*(u32 *)(psx->ram + 0xC0) == 0)
    return 0;
  return 1;
}

static int psx_bios_runtime_ready(struct psx_system *psx) {
  if (!psx_bios_lowmem_ready(psx))
    return 0;
  if (psx->cp0_regs[12] & (1u << 22))
    return 0; /* BEV still set: exceptions still routed to ROM */
  if (psx->pc < 0x80030000u || psx->pc >= 0x80200000u)
    return 0; /* Wait until BIOS is executing from RAM, not low stubs/ROM */
  return 1;
}

static int psx_bios_shell_ready(struct psx_system *psx) {
  if (!psx_bios_runtime_ready(psx))
    return 0;
  if (psx->gpu_io.gp0_total < 0x1000u)
    return 0; /* BIOS has not visibly started drawing yet */
  return 1;
}

static int psx_bios_launch_window(struct psx_system *psx) {
  u32 pc = psx->pc;
  if (!psx_bios_shell_ready(psx))
    return 0;
  if (pc >= 0x80067800u && pc < 0x80067900u)
    return 0; /* Too late: BIOS shell/menu loop already reached */
  if (pc >= 0x80050000u && pc < 0x80067800u)
    return 1; /* BIOS active before stable menu loop */
  return 0;
}

static void psx_inject_bios_clut(struct psx_system *psx) {
  if (!psx->gpu_io.vram)
    return;
  u16 *cv = psx->gpu_io.vram + 384u * 1024u + 640u;
  for (int k = 1; k < 16; k++)
    cv[k] = 0x7FFFu;
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

  /* ── AudioOut ─────────────────────────────────────────────────── */
  if (load_mod)
    NC(G, load_mod, (u64) "libSceAudioOut.sprx", 0, 0, 0, 0, 0);
  void *audio_init = SYM(G, D, AUDIOOUT_HANDLE, "sceAudioOutInit");
  void *audio_open = SYM(G, D, AUDIOOUT_HANDLE, "sceAudioOutOpen");
  void *audio_out  = SYM(G, D, AUDIOOUT_HANDLE, "sceAudioOutOutput");
  
  if (audio_init) NC(G, audio_init, 0, 0, 0, 0, 0, 0);
  
  /* Open audio: match example_common.h / ps5_sdk_audio_open_default */
  s32 audio_h = audio_open ? (s32)NC(G, audio_open, 0xFF, 0, 0, 256, 48000, 1) : -1;
  
  psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, "[PSX] AudioOpen h=", (u32)audio_h);

  if (audio_h >= 0) {
      LOG("[PSX] AudioOut OK\n");
  } else {
      LOG("[PSX] WARN: AudioOut open failed\n");
  }

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
  st->sys.spu_regs[0xC0] = 0x3FFF; /* Initial Master Vol L */
  st->sys.spu_regs[0xC1] = 0x3FFF; /* Initial Master Vol R */
  st->sys.joy_baud = 0xDCu;
  st->sys.joy_mc_flag = 0x08u;
  psx_mdec_reset(&st->sys);

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
  psx_local_zero(st->sys.ram, 0x200000);
  psx_local_zero(st->sys.scratch, 0x400);
  st->sys.audio_vbuf = (s16 *)NC(G, mmap_fn, 0, 0x1000, 3, 0x1002, (u64)-1, 0);
  if (!st->sys.audio_vbuf || (s64)st->sys.audio_vbuf == -1) {
    st->sys.audio_vbuf = 0;
  } else {
    psx_local_zero((u8 *)st->sys.audio_vbuf, 0x1000);
  }
  st->sys.audio_h = audio_h;
  st->sys.audio_out_fn = audio_out;

  /* ── CD-ROM Game load ─────────────────────────────────────────── */
  st->sys.G = G;
  st->sys.sendto_fn = sendto_fn;
  st->sys.log_fd = ext->log_fd;
  for(int k=0; k<16; k++) st->sys.log_addr[k] = ext->log_addr[k];
  st->sys.kopen_fn = kopen_fn;
  st->sys.klseek_fn = klseek_fn;
  st->sys.kread_fn = kread_fn;
  st->sys.cd_fd =
      (s32)NC(G, kopen_fn, (u64) "/temp0/psx/games/game.bin", 0, 0, 0, 0, 0);
  if (st->sys.cd_fd > 0) {
    LOG("[PSX] Game.bin abierto\n");
    st->sys.cd_shell_open = 1;
    /* Detect sector size: check for RAW sync pattern (00 FF FF FF FF FF FF FF FF FF FF 00) */
    u8 sync[12];
    NC(G, kread_fn, (u64)st->sys.cd_fd, (u64)sync, 12, 0, 0, 0);
    if (sync[0] == 0 && sync[1] == 0xFF && sync[10] == 0xFF && sync[11] == 0) {
        st->sys.cd_sector_size = 2352;
        LOG("[PSX] Formato RAW (2352 bytes/sector)\n");
    } else {
        st->sys.cd_sector_size = 2048;
        LOG("[PSX] Formato ISO (2048 bytes/sector)\n");
    }
    {
      s64 fsz = (s64)NC(G, klseek_fn, (u64)st->sys.cd_fd, 0, 2, 0, 0, 0);
      if (fsz > 0)
        st->sys.cd_total_sectors = (u32)((u64)fsz / st->sys.cd_sector_size);
    }
    NC(G, klseek_fn, (u64)st->sys.cd_fd, 0, 0, 0, 0, 0); /* reset to start */
    psx_detect_disc_boot_info(&st->sys);
    if (st->sys.cd_boot_path[0]) {
      psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                  "[PSX] SYSTEM.CNF detectado\n");
    }
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] DiscRegion=", st->sys.cd_disc_region);
  } else {
    LOG("[PSX] WARN: game.bin no encontrado\n");
    st->sys.cd_sector_size = 2352;
  }

  /* ── Load BIOS ────────────────────────────────────────────── */
  {
    s32 fd = -1;
    st->sys.cd_console_region = 3u;
#define TRY_BIOS(path, region_id)                                              \
    do {                                                                       \
      if (fd <= 0) {                                                           \
        fd = (s32)NC(G, kopen_fn, (u64)(path), 0, 0, 0, 0, 0);                \
        if (fd > 0)                                                            \
          st->sys.cd_console_region = (region_id);                             \
      }                                                                        \
    } while (0)
    if (st->sys.cd_disc_region == 1u) {
      TRY_BIOS("/data/psx/bios/SCPH1001.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH5501.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH7001.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH7501.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph1001.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph5501.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph7001.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph7501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH1001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH5501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH7001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH7501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph1001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph5501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph7001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph7501.bin", 1u);
    } else if (st->sys.cd_disc_region == 2u) {
      TRY_BIOS("/data/psx/bios/SCPH1002.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH5502.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH7002.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH7502.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph1002.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph5502.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph7002.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph7502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH1002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH5502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH7002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH7502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph1002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph5502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph7002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph7502.bin", 2u);
    } else if (st->sys.cd_disc_region == 0u) {
      TRY_BIOS("/data/psx/bios/SCPH1000.bin", 0u);
      TRY_BIOS("/data/psx/bios/SCPH5500.bin", 0u);
      TRY_BIOS("/data/psx/bios/SCPH7000.bin", 0u);
      TRY_BIOS("/data/psx/bios/SCPH7500.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph1000.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph5500.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph7000.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph7500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH1000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH5500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH7000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH7500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph1000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph5500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph7000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph7500.bin", 0u);
    }
    if (fd <= 0) {
      TRY_BIOS("/data/psx/bios/SCPH1001.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH5501.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH7001.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH7501.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph1001.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph5501.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph7001.bin", 1u);
      TRY_BIOS("/data/psx/bios/scph7501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH1001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH5501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH7001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/SCPH7501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph1001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph5501.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph7001.bin", 1u);
      TRY_BIOS("/temp0/psx/bios/scph7501.bin", 1u);
      TRY_BIOS("/data/psx/bios/SCPH1002.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH5502.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH7002.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH7502.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph1002.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph5502.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph7002.bin", 2u);
      TRY_BIOS("/data/psx/bios/scph7502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH1002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH5502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH7002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/SCPH7502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph1002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph5502.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph7002.bin", 2u);
      TRY_BIOS("/temp0/psx/bios/scph7502.bin", 2u);
      TRY_BIOS("/data/psx/bios/SCPH1000.bin", 0u);
      TRY_BIOS("/data/psx/bios/SCPH5500.bin", 0u);
      TRY_BIOS("/data/psx/bios/SCPH7000.bin", 0u);
      TRY_BIOS("/data/psx/bios/SCPH7500.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph1000.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph5500.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph7000.bin", 0u);
      TRY_BIOS("/data/psx/bios/scph7500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH1000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH5500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH7000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/SCPH7500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph1000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph5500.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph7000.bin", 0u);
      TRY_BIOS("/temp0/psx/bios/scph7500.bin", 0u);
    }
#undef TRY_BIOS
    if (fd <= 0) {
      LOG("[PSX] Error: BIOS\n");
      ext->status = -6;
      goto cleanup_fb;
    }
    NC(G, kread_fn, (u64)fd, (u64)st->sys.bios, 0x80000, 0, 0, 0);
    NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
    LOG("[PSX] BIOS cargada\n");
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] AutoBIOS region=", st->sys.cd_console_region);
  }

  /* ── CPU reset ────────────────────────────────────────────── */
  st->sys.pc = 0xBFC00000u;
  st->sys.next_pc = 0xBFC00004u;
  st->sys.cp0_regs[12] = 0x10900000u; /* Status: BEV=1, IE=0 */

  /* ── GPU init ─────────────────────────────────────────────── */
  psx_gpu_init(&st->sys, &st->gpu, G, mmap_fn, (u32 *)fbs[0], (u32 *)fbs[1]);
  if (!st->sys.gpu_io.vram)
    LOG("[PSX] WARN: VRAM null\n");

  if (st->sys.cd_fd > 0) {
    u32 warm_steps = 0;
    u32 warm_frames = 0;
    u32 warm_pc = 0;
    int launch_ready = 0;
    for (; warm_frames < 240u && !launch_ready; warm_frames++) {
      psx_bus_vblank(&st->sys);
      psx_inject_bios_clut(&st->sys);
      for (u32 i = 0; i < 564480u; i++) {
        psx_cpu_step(&st->sys);
        warm_steps++;
        if ((i & 0x1FFFu) == 0) {
          warm_pc = st->sys.pc;
          if (!st->sys.launch_window_logged &&
              psx_bios_launch_window(&st->sys)) {
            st->sys.launch_window_logged = 1;
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[PSX] LaunchWin PC=", warm_pc);
          }
          if (psx_bios_launch_window(&st->sys)) {
            launch_ready = 1;
            break;
          }
        }
      }
      warm_pc = st->sys.pc;
      {
        u32 idx = st->gpu.fb_idx & 1u;
        psx_gpu_sync_to_ps5(&st->sys, (u32 *)fbs[idx]);
        NC(G, vid_flip, (u64)video_h, (u64)idx, 1, 0, 0, 0);
        st->gpu.fb_idx++;
      }
    }
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] Warmup steps=", warm_steps);
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] Warmup frm=", warm_frames);
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] Warmup PC=", warm_pc);
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] StubA0=", *(u32 *)(st->sys.ram + 0xA0));
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] Warmup SR=", st->sys.cp0_regs[12]);
    if (!launch_ready && psx_bios_runtime_ready(&st->sys)) {
      launch_ready = 1;
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "[PSX] LaunchFallback PC=", st->sys.pc);
    }
    if (launch_ready && psx_direct_boot_game(&st->sys)) {
      LOG("[PSX] Direct boot activado\n");
    } else {
      st->sys.direct_boot_state = 3u;
      LOG("[PSX] Direct boot no disponible\n");
    }
  }

  LOG("[PSX] Emulando\n");

#define LOGPC(pfx)                                                             \
  psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, pfx, st->sys.pc)

  /* ── Main loop ────────────────────────────────────────────── */
  u32 vblank = 0;
  for (;;) {
    psx_bus_vblank(&st->sys);
    vblank++;

    /* Re-inject CLUT: BIOS shell needs this for text visibility */
    psx_inject_bios_clut(&st->sys);

    for (u32 i = 0; i < 564480u; i++)
      psx_cpu_step(&st->sys);

    /* ── GPU sync & Flip ────────────────────────────────────── */
    u32 idx = st->gpu.fb_idx & 1u;
    psx_gpu_sync_to_ps5(&st->sys, (u32 *)fbs[idx]);
    NC(G, vid_flip, (u64)video_h, (u64)idx, 1, 0, 0, 0);
    st->gpu.fb_idx++;

    /* ── Audio Output ────────────────────────────────────────── */
    /* Real SPU mixing */
    if (st->sys.audio_h >= 0 && st->sys.audio_out_fn && st->sys.audio_vbuf) {
        void psx_spu_render(struct psx_system *psx, s16 *out, u32 count);
        for (int c = 0; c < 3; c++) {
            if (st->sys.audio_h >= 0 && st->sys.audio_vbuf) {
                psx_spu_render(&st->sys, st->sys.audio_vbuf, 256);
                s32 aret = (s32)NC(G, st->sys.audio_out_fn, (u64)st->sys.audio_h, (u64)st->sys.audio_vbuf, 0, 0, 0, 0);
                if (aret < 0 && (vblank % 60 == 0)) {
                    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, "[PSX] AudioOut err=", (u32)aret);
                }
            } else {
                /* If no audio, just advance SPU state without outputting */
                psx_spu_render(&st->sys, 0, 256);
            }
        }
    }

    /* Log every 30 VBLANKs (~0.5 seconds) */
    if ((vblank % 30u) == 0) {
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, "vbl=", vblank);
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
