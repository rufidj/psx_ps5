#ifndef PSX_TYPES_H
#define PSX_TYPES_H

/* ── PSX GPU I/O state (embedded in psx_system, accessed from bus) ── */
struct psx_gpu_io {
    u32  gpustat;          /* GPUSTAT register (0x1F801814 read) */

    /* GP0 command FIFO */
    u32  fifo[16];
    u32  fifo_len;         /* words currently buffered */
    u32  fifo_need;        /* words needed for current cmd (0 = idle) */

    /* CPU→VRAM copy state (GP0 0xA0) */
    u32  cpuvram_active;
    u32  cpuvram_x, cpuvram_y;   /* destination base in VRAM */
    u32  cpuvram_w, cpuvram_h;   /* copy dimensions */
    u32  cpuvram_cx, cpuvram_cy; /* current write cursor */

    /* Current draw mode (from GP0 E1) */
    u32  texpage;          /* texpage word: tpx, tpy, semi-trans, depth */
    u32  clut_x, clut_y;   /* CLUT base in VRAM (updated per-command) */

    /* Drawing area / offset (from GP0 E3/E4/E5) */
    s32  draw_x1, draw_y1;
    s32  draw_x2, draw_y2;
    s32  off_x,   off_y;

    /* Display config (from GP1) */
    u32  disp_en;          /* 0 = display on, 1 = display off */
    u32  disp_vram_x, disp_vram_y;
    u32  disp_w, disp_h;   /* display area size */

    u16 *vram;             /* 1024×512 × u16 */
    u32  gp0_total;        /* diagnostic: total GP0 words received */
    u32  dma2_calls;       /* diagnostic: DMA2 executions */
    u32  fill_cnt;         /* 0x02 fill rects */
    u32  poly_cnt;         /* 0x20-0x5F polygons/lines */
    u32  rect_cnt;         /* 0x60-0x7F sprites/rects */
    u32  last_cmd;         /* last GP0 command byte executed */
    u32  last_quad[9];     /* last GP0(0x2C) textured quad fifo data */
    u32  vram_dests[8];    /* last 8 CPU→VRAM destinations: (x<<16)|y */
    u32  vram_sizes[8];    /* corresponding (w<<16)|h */
    u32  vram_dest_pos;    /* ring buffer write position */
    u32  clut_wr_sample;   /* first data word written when dest y=384 */
    u32  clut_wr_dest;     /* destination (x<<16)|y of that write */
    u32  last_spr[3];      /* last GP0(0x74) textured sprite fifo data */
};

/* ── PSX CPU + bus state ─────────────────────────────────────────── */
struct psx_system {
    u8  *ram;      /* 2 MB  @ 0x00000000 */
    u8  *bios;     /* 512 KB @ 0x1FC00000 */
    u8  *scratch;  /* 1 KB  @ 0x1F800000 */

    u32  pc, next_pc;
    u32  regs[32];
    u32  hi, lo;
    u32  cp0_regs[32];

    /* I/O state — in struct to avoid static BSS init issues */
    u32  i_stat;
    u32  i_mask;
    u32  timer0;

    /* CD-ROM minimal state */
    u32  cd_idx;          /* register index (0–3) */
    u32  cd_cmd_cnt;      /* total commands received */
    u32  cd_irq_pending;  /* 0=none 1=INT3 2=INT2 3=INT5(error) */
    u32  cd_resp_pos;     /* byte position within multi-byte response */

    /* DMA channels – only MADR and BCR stored (CHCR triggers execution) */
    u32  dma_madr[8];
    u32  dma_bcr[8];

    /* Exception handling */
    u32  exception_vec;  /* non-zero = exception taken, skip delay slot, jump here */

    struct psx_gpu_io gpu_io;
};

#endif /* PSX_TYPES_H */
