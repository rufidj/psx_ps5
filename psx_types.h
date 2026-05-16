#ifndef PSX_TYPES_H
#define PSX_TYPES_H

/* ── PSX GPU I/O state (embedded in psx_system, accessed from bus) ── */
struct psx_gpu_io {
    u32  gpustat;          /* GPUSTAT register (0x1F801814 read) */
    u32  gpuread;          /* GPUREAD register (0x1F801810 read) */
    u32  texwin;           /* GP0(E2h) latched texture window */

    /* GP0 command FIFO */
    u32  fifo[16];
    u32  fifo_len;         /* words currently buffered */
    u32  fifo_need;        /* words needed for current cmd (0 = idle) */

    /* CPU→VRAM copy state (GP0 0xA0) */
    u32  cpuvram_active;
    u32  cpuvram_x, cpuvram_y;   /* destination base in VRAM */
    u32  cpuvram_w, cpuvram_h;   /* copy dimensions */
    u32  cpuvram_cx, cpuvram_cy; /* current write cursor */
    u32  cpuvram_byte_x;        /* byte cursor within current row for 24-bit writes */
    u32  vramcpu_active;
    u32  vramcpu_x, vramcpu_y;   /* source base in VRAM */
    u32  vramcpu_w, vramcpu_h;   /* copy dimensions */
    u32  vramcpu_cx, vramcpu_cy; /* current read cursor */

    /* Current draw mode (from GP0 E1) */
    u32  texpage;          /* texpage word: tpx, tpy, semi-trans, depth */
    u32  draw_mode_word;   /* full GP0(E1) word */
    u32  clut_x, clut_y;   /* CLUT base in VRAM (updated per-command) */
    u32  mask_set;         /* GP0(E6).bit0 */
    u32  mask_check;       /* GP0(E6).bit1 */
    u32  rect_flip_x;      /* GP0(E1).bit12 */
    u32  rect_flip_y;      /* GP0(E1).bit13 */
    u32  dither_enable;    /* GP0(E1).bit9 */
    u32  draw_to_display;  /* GP0(E1).bit10 */

    /* Drawing area / offset (from GP0 E3/E4/E5) */
    s32  draw_x1, draw_y1;
    s32  draw_x2, draw_y2;
    s32  off_x,   off_y;

    /* Display config (from GP1) */
    u32  disp_en;          /* 0 = display on, 1 = display off */
    u32  disp_vram_x, disp_vram_y;
    u32  disp_w, disp_h;   /* display area size */
    u32  disp_24bit;
    u32  disp_mode;
    u32  disp_x1, disp_x2;
    u32  disp_y1, disp_y2;
    u32  pal;              /* 1 = PAL, 0 = NTSC (from GP1(08).bit3) */

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
    u32  load_pending_reg;   /* 0 = none, otherwise register index */
    u32  load_pending_value; /* value to commit after one instruction */
    u32  bad_fetch_logged;   /* one-shot log for first unmapped instruction fetch */
    u32  bad_jump_logged;    /* one-shot log for first unmapped jump target */
    u32  low_pc_fix_logged;  /* one-shot log when canonicalizing low RAM PC */
    u32  exc_logged;         /* one-shot log for first exception taken */
    u32  launch_window_logged; /* one-shot log for first launch-window hit */
    u32  gte_cmd_logged;     /* one-shot log for first GTE command observed */
    u32  hot_loop_logged;    /* one-shot log for current EXE hotspot */
    u32  bios_call_logs;     /* capped BIOS A0/B0/C0 call trace count */
    u32  bios_cd_ev_open;    /* bitmask of intercepted BIOS CD event slots */
    u32  bios_cd_ev_enabled; /* bitmask of enabled intercepted BIOS CD events */
    u32  bios_cd_ev_ready;   /* bitmask of ready intercepted BIOS CD events */
    u32  bios_ev_open;       /* bitmask of generic BIOS event slots */
    u32  bios_ev_enabled;    /* bitmask of enabled generic BIOS event slots */
    u32  bios_ev_ready;      /* bitmask of ready generic BIOS event slots */
    u32  bios_ev_class[16];
    u32  bios_ev_spec[16];
    u32  bios_ev_mode[16];
    u32  bios_ev_func[16];
    u32  bios_pad_buf1;
    u32  bios_pad_buf2;
    u32  bios_pad_siz1;
    u32  bios_pad_siz2;
    u32  bios_pad_started;
    u32  bios_pad_use_hidden;
    u32  bios_pad_button_dest;
    u8   bios_pad_hidden1[0x22];
    u8   bios_pad_hidden2[0x22];

    /* I/O state — in struct to avoid static BSS init issues */
    u32  i_stat;
    u32  i_mask;
    u32  timer0, timer1, timer2;
    u32  timer_div;
    u32  timer0_acc, timer1_acc, timer2_acc;
    u32  timer0_mode, timer1_mode, timer2_mode; /* mode regs with overflow/irq flags */

    /* SIO0 / JOY state */
    u32  joy_mode;
    u32  joy_ctrl;
    u32  joy_baud;
    u32  joy_rx_data;
    u32  joy_rx_ready;
    u32  joy_irq;
    u32  joy_selected;   /* 0=none, 1=pad1, 2=memcard1 */
    u32  joy_cmd;
    u32  joy_phase;
    u32  joy_tx_prev;
    u32  joy_mc_sector;
    u32  joy_mc_index;
    u32  joy_mc_checksum;
    u32  joy_mc_flag;
    u32  joy_mc_write;
    u32  joy_pad_config;
    u32  joy_pad_mode;
    u32  joy_pad_lock;
    u32  joy_pad_analog;
    u8   joy_mc_buf[128];
    u8   joy_memcard[128 * 1024];
    u32  pad_buttons_psx; /* PSX digital pad state, active low bits in 16-bit word */

    /* CD-ROM minimal state */
    u32  cd_idx;          /* register index (0–3) */
    u32  cd_cmd_cnt;      /* total commands received */
    u32  cd_cmd_pending;  /* pending host command waiting for IRQ ack */
    u32  cd_cmd_pending_n;/* pending parameter count */
    u32  cd_irq_pending;  /* 0=none 1=INT3 2=INT2 3=INT5(error) */
    u32  cd_resp_pos;     /* byte position within multi-byte response */
    s32  cd_fd;           /* game.bin file descriptor */
    u32  cd_sector;       /* target sector (LBA) */
    u32  cd_sector_size;  /* 2352 (RAW) or 2048 (Data/ISO) */
    u32  cd_data_ready;   /* 1 if sector_buf contains fresh data */
    u32  cd_data_pos;     /* position within current sector being read */
    u32  cd_total_sectors;/* total sectors in mounted image */
    u32  cd_req;          /* request register (data FIFO enable, etc.) */
    u32  cd_hint_mask;    /* host interrupt mask (HINTMSK) */
    u32  cd_shell_open;   /* shell-open latch, cleared by Nop when tray is closed */
    u32  cd_disc_region;  /* 0=JP, 1=US, 2=EU, 3=other/unknown */
    u32  cd_console_region; /* 0=JP, 1=US, 2=EU, 3=other/unknown */
    u32  cd_seek_lba;     /* latched target from SetLoc */
    u32  cd_seek_pending; /* seek completion pending */
    u32  cd_setloc_pending; /* SetLoc latched and not yet consumed by read/seek */
    u32  cd_read_after_seek; /* Read command waiting for implicit seek */
    u32  cd_read_cmd;     /* last read command (ReadN/ReadS) */
    u32  cd_pause_pending; /* Pause requested while a read is still in flight */
    u32  cd_boot_trace;   /* one-shot trace once BIOS asks for real disc data */
    u8   cd_sector_buf[2352]; /* buffer for one RAW sector */
    u8   cd_fifo_data[32][2340]; /* queued CPU-visible sectors for ReadS/FMVs */
    u16  cd_fifo_size[32];
    u16  cd_fifo_pos[32];
    u32  cd_fifo_r;
    u32  cd_fifo_w;
    u32  cd_fifo_count;
    u32  cd_p_idx;        /* parameter index */
    u8   cd_bcd[16];      /* parameter buffer */
    u8   cd_cmd_pending_code;
    u8   cd_cmd_pending_params[16];
    u8   cd_resp_buf[16]; /* response FIFO */
    u32  cd_resp_len;     /* total bytes in response */
    u8   cd_resp_second[16]; /* delayed second response */
    u32  cd_resp_second_len;
    u32  cd_irq_second;   /* IRQ code for second response (e.g. 2=INT2) */
    u32  cd_irq_timer;
    u32  cd_mode;         /* bit 5: 1=RAW, 0=Data */
    u32  cd_read_active;  /* 1 = drive is reading sectors */
    u32  cd_q_lba;        /* LBA of last sector whose Q-subchannel was captured */
    /* LibCrypt: up to 16 protected sectors with Q-subchannel override */
    u32  lc_count;
    u32  lc_snap_lba;   /* 0 = no snap; else hold cd_q_lba here until drive reaches it */
    u32  lc_snap_idx;   /* index of the entry currently snapped (for marking) */
    u32  lc_snapped[2]; /* 64-bit bitmask: entry i snapped (skip it on next SetLoc) */
    u32  lc_log_cnt;    /* capped [LC] log counter, reset each game start */
    u32  lc_lba[64];
    u8   lc_q[64][10]; /* Q subchannel bytes (ADR/CTL + TNO+IDX+MSF+AMSFQ) */
    u32  cd_tick_acc;     /* cycle accumulator for sector delivery */
    u32  cd_busy_ticks;   /* brief BUSYSTS window after command writes */
    u32  cd_filter_file;  /* SetFilter file number */
    u32  cd_filter_channel; /* SetFilter channel number */
    u32  cd_last_file;    /* last sector subheader file number */
    u32  cd_last_channel; /* last sector subheader channel number */
    u32  cd_last_submode; /* last sector subheader submode */
    u32  cd_last_coding;  /* last sector subheader coding info */
    u32  exe_entry_pc;    /* direct-boot entrypoint, if used */
    u32  exe_text_addr;   /* loaded EXE text base */
    u32  exe_text_size;   /* loaded EXE text size */
    u32  direct_boot_state; /* 0=pending, 1=launched, 2=deferred/failed */
    char cd_boot_path[96]; /* cached BOOT path from SYSTEM.CNF */

    /* DMA channels – only MADR and BCR stored (CHCR triggers execution) */
    u32  dma_madr[8];
    u32  dma_bcr[8];
    u32  dma_chcr[8];
    u32  dma_dicr;

    /* MDEC minimal state */
    u32  mdec_ctrl;
    u32  mdec_status;
    u32  mdec_cmd;
    u32  mdec_words_remaining;
    u32  mdec_in_enabled;
    u32  mdec_out_enabled;
    u32  mdec_output_ready;
    u32  mdec_output_words;
    u32  mdec_data_read_pos;
    u32  mdec_in_word_count;
    u32  mdec_in_half_pos;
    u32  mdec_log_flags;
    u8   mdec_iq_y[64];
    u8   mdec_iq_uv[64];
    s16  mdec_scale[64];
    u32  mdec_in_words[0x10000];

    /* Exception handling */
    u32  exception_vec;  /* non-zero = exception taken, skip delay slot, jump here */

    /* COP2 / GTE state */
    u32  gte_data[32];
    u32  gte_ctrl[32];

    struct psx_gpu_io gpu_io;
    u16  spu_regs[256]; /* SPU registers (1F801C00 - 1F801DFF) */
    u32  spu_addr_internal; /* Internal SPU RAM transfer address in bytes */
    void *G; /* eboot base + gadget offset */
    void *sendto_fn;
    s32   log_fd;
    u8    log_addr[16];
    void *kopen_fn, *klseek_fn, *kread_fn, *kpread_fn;

    /* AudioOut state */
    s32   audio_h;
    void *audio_out_fn;
    s16  *audio_vbuf; /* 16-bit stereo samples (must be in mmap-ed mem) */
    u32   audio_phase;
    u32   audio_chunk_accum;

    u8    spu_ram[512 * 1024]; /* 512KB SPU RAM */
    u32   spu_voice_on;        /* bitmask of active voices */
    u32   spu_endx;            /* bitmask of voices that reached LOOP-END */
    u32   spu_voice_addr[24];  /* current RAM address of the BLOCK */
    u32   spu_voice_idx[24];   /* current sample index within block (0-27) */
    u32   spu_voice_phase[24]; /* fractional phase for pitch (16.16 fixed point) */
    s16   spu_voice_samples[24][28]; /* cached decoded samples for the current block */
    s16   spu_voice_last[24][2]; /* ADPCM history for decoding */
    s16   spu_voice_prev[24][3]; /* last three decoded samples of the previous block */
    s32   spu_voice_vol_cur[24][2]; /* current L/R volume after sweep/fixed decode */
    u32   spu_voice_vol_cnt[24][2]; /* sweep envelope counters for L/R volume */
    s32   spu_main_vol_cur[2]; /* current master L/R volume after sweep/fixed decode */
    u32   spu_main_vol_cnt[2]; /* sweep envelope counters for master L/R volume */
    s32   spu_lpf_l, spu_lpf_r; /* IIR lowpass filter state for crackling reduction */

    /* ADSR envelope state per voice */
    s32   spu_adsr_vol[24];   /* current ADSR volume 0..0x7FFF */
    u8    spu_adsr_phase[24]; /* 0=attack 1=decay 2=sustain 3=release */
    u32   spu_adsr_cnt[24];   /* 15-bit envelope counters */
};

#endif /* PSX_TYPES_H */
