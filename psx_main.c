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
  u8 raw[2352];
  if (!psx->kread_fn || !psx->klseek_fn || psx->cd_fd <= 0)
    return 0;

  if (psx->cd_sector_size == 2048u) {
    u64 off = (u64)lba * 2048ull;
    NC(psx->G, psx->klseek_fn, (u64)psx->cd_fd, off, 0, 0, 0, 0);
    return ((s32)NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)dst, 2048, 0, 0,
                    0) == 2048);
  }

  if (psx->cd_sector_size == 2352u) {
    u64 off = (u64)lba * 2352ull;
    NC(psx->G, psx->klseek_fn, (u64)psx->cd_fd, off, 0, 0, 0, 0);
    if ((s32)NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)raw, 2352, 0, 0, 0) != 2352)
      return 0;
    {
      u32 data_off = (raw[15] == 0x01u) ? 16u : 24u;
      for (u32 i = 0; i < 2048u; i++)
        dst[i] = raw[data_off + i];
    }
    return 1;
  }

  if (psx->cd_sector_size == 2336u) {
    u64 off = (u64)lba * 2336ull;
    NC(psx->G, psx->klseek_fn, (u64)psx->cd_fd, off, 0, 0, 0, 0);
    if ((s32)NC(psx->G, psx->kread_fn, (u64)psx->cd_fd, (u64)raw, 2336, 0, 0, 0) != 2336)
      return 0;
    for (u32 i = 0; i < 2048u; i++)
      dst[i] = raw[8u + i];
    return 1;
  }

  return 0;
}

static int psx_sync_raw_sector(const u8 *buf) {
  if (!buf || buf[0] != 0x00 || buf[11] != 0x00)
    return 0;
  for (u32 i = 1; i <= 10; i++) {
    if (buf[i] != 0xFF)
      return 0;
  }
  return 1;
}

static int psx_read_bytes_at(void *G, void *klseek_fn, void *kread_fn, s32 fd,
                             u64 off, u8 *dst, u32 len) {
  if (!G || !klseek_fn || !kread_fn || fd <= 0 || !dst || !len)
    return 0;
  NC(G, klseek_fn, (u64)fd, off, 0, 0, 0, 0);
  return ((s32)NC(G, kread_fn, (u64)fd, (u64)dst, len, 0, 0, 0) == (s32)len);
}

static int psx_pvd_valid(const u8 *buf) {
  return buf && buf[0] == 1 && buf[1] == 'C' && buf[2] == 'D' &&
         buf[3] == '0' && buf[4] == '0' && buf[5] == '1';
}

static int psx_path_has_ext(const char *path, const char *ext)
{
  u32 plen = 0, elen = 0;
  if (!path || !ext)
    return 0;
  while (path[plen])
    plen++;
  while (ext[elen])
    elen++;
  if (plen < elen)
    return 0;
  while (elen) {
    char a = psx_ascii_upper(path[plen - elen]);
    char b = psx_ascii_upper(ext[0]);
    if (a != b)
      return 0;
    ext++;
    elen--;
  }
  return 1;
}

static u32 psx_detect_image_sector_size(void *G, void *klseek_fn, void *kread_fn,
                                        s32 fd, u64 fsz, const char *path) {
  u8 hdr[12];
  u8 pvd[2048];
  u64 off2352_16 = 16ull * 2352ull;
  u64 off2336_16 = 16ull * 2336ull;
  u64 off2048_16 = 16ull * 2048ull;
  int raw_mod = (fsz > 0 && (fsz % 2352u) == 0);
  int raw2336_mod = (fsz > 0 && (fsz % 2336u) == 0);
  int iso_mod = (fsz > 0 && (fsz % 2048u) == 0);
  int raw_sync0 = 0, raw_sync16 = 0, raw_pvd_m2 = 0, raw_pvd_m1 = 0, raw2336_pvd = 0, iso_pvd = 0;
  int prefer_raw_ext = psx_path_has_ext(path, ".bin") || psx_path_has_ext(path, ".img");

  psx_local_zero(hdr, sizeof(hdr));
  psx_local_zero(pvd, sizeof(pvd));

  if (psx_read_bytes_at(G, klseek_fn, kread_fn, fd, 0, hdr, 12u))
    raw_sync0 = psx_sync_raw_sector(hdr);

  if (!fsz || (off2352_16 + 24ull + 2048ull) <= fsz) {
    if (psx_read_bytes_at(G, klseek_fn, kread_fn, fd, off2352_16, hdr, 12u))
      raw_sync16 = psx_sync_raw_sector(hdr);
    if (psx_read_bytes_at(G, klseek_fn, kread_fn, fd, off2352_16 + 24ull, pvd, 2048u))
      raw_pvd_m2 = psx_pvd_valid(pvd);
    if (psx_read_bytes_at(G, klseek_fn, kread_fn, fd, off2352_16 + 16ull, pvd, 2048u))
      raw_pvd_m1 = psx_pvd_valid(pvd);
  }

  if (!fsz || (off2336_16 + 8ull + 2048ull) <= fsz) {
    if (psx_read_bytes_at(G, klseek_fn, kread_fn, fd, off2336_16 + 8ull, pvd, 2048u))
      raw2336_pvd = psx_pvd_valid(pvd);
  }

  if (!fsz || (off2048_16 + 2048ull) <= fsz) {
    if (psx_read_bytes_at(G, klseek_fn, kread_fn, fd, off2048_16, pvd, 2048u))
      iso_pvd = psx_pvd_valid(pvd);
  }

  NC(G, klseek_fn, (u64)fd, 0, 0, 0, 0, 0);

  if (prefer_raw_ext && (raw_pvd_m2 || raw_pvd_m1 || raw_sync16 || raw_sync0))
    return 2352u;
  if (prefer_raw_ext && raw2336_mod)
    return 2336u;
  if (raw_pvd_m2 || raw_pvd_m1)
    return 2352u;
  if (raw2336_pvd)
    return 2336u;
  if (prefer_raw_ext && (raw_sync0 || raw_sync16 || raw_mod))
    return 2352u;
  if (iso_pvd)
    return 2048u;
  if (raw_sync0 && raw_mod)
    return 2352u;
  if (raw_sync16 && raw_mod)
    return 2352u;
  if (raw_mod && !iso_mod)
    return 2352u;
  if (raw2336_mod && !iso_mod)
    return 2336u;
  if (prefer_raw_ext && raw_mod)
    return 2352u;
  return 2048u;
}

static u32 psx_region_from_filename(const char *path)
{
  u32 i = 0;
  if (!path)
    return 3u;
  while (path[i]) {
    char c0 = psx_ascii_upper(path[i + 0]);
    char c1 = psx_ascii_upper(path[i + 1]);
    char c2 = psx_ascii_upper(path[i + 2]);
    char c3 = psx_ascii_upper(path[i + 3]);
    if (c0 == '[' && c1 == 'E' && c2 == 'U' && c3 == ']') return 2u;
    if (c0 == '[' && c1 == 'U' && c2 == 'S' && c3 == 'A') return 1u;
    if (c0 == '(' && c1 == 'E' && c2 == ')') return 2u;
    if (c0 == '(' && c1 == 'U' && c2 == ')') return 1u;
    if (c0 == '(' && c1 == 'J' && c2 == ')') return 0u;
    if (c0 == 'E' && c1 == 'U' && c2 == 'R' && (c3 == ']' || c3 == ')' || c3 == ' ')) return 2u;
    if (c0 == 'U' && c1 == 'S' && c2 == 'A' && (c3 == ']' || c3 == ')' || c3 == ' ')) return 1u;
    if (c0 == 'J' && c1 == 'P' && (c2 == ']' || c2 == ')' || c2 == ' ')) return 0u;
    i++;
  }
  return 3u;
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

static u8 psx_edc_lut_init_done;
static u8 psx_ecc_f_lut[256];
static u8 psx_ecc_b_lut[256];
static u32 psx_edc_lut[256];

static void psx_ecm_tables_init(void)
{
  if (psx_edc_lut_init_done)
    return;
  for (u32 i = 0; i < 256u; i++) {
    u32 j = (i << 1) ^ ((i & 0x80u) ? 0x11Du : 0u);
    u32 edc = i;
    psx_ecc_f_lut[i] = (u8)j;
    psx_ecc_b_lut[i ^ j] = (u8)i;
    for (u32 k = 0; k < 8u; k++)
      edc = (edc >> 1) ^ ((edc & 1u) ? 0xD8018001u : 0u);
    psx_edc_lut[i] = edc;
  }
  psx_edc_lut_init_done = 1u;
}

static u32 psx_ecm_edc_partial(u32 edc, const u8 *src, u32 size)
{
  while (size--)
    edc = (edc >> 8) ^ psx_edc_lut[(edc ^ (*src++)) & 0xFFu];
  return edc;
}

static void psx_ecm_edc_compute(const u8 *src, u32 size, u8 *dest)
{
  u32 edc = psx_ecm_edc_partial(0u, src, size);
  dest[0] = (u8)(edc >> 0);
  dest[1] = (u8)(edc >> 8);
  dest[2] = (u8)(edc >> 16);
  dest[3] = (u8)(edc >> 24);
}

static void psx_ecm_ecc_compute(u8 *src, u32 major_count, u32 minor_count,
                                u32 major_mult, u32 minor_inc, u8 *dest)
{
  u32 size = major_count * minor_count;
  for (u32 major = 0; major < major_count; major++) {
    u32 index = (major >> 1) * major_mult + (major & 1u);
    u8 ecc_a = 0, ecc_b = 0;
    for (u32 minor = 0; minor < minor_count; minor++) {
      u8 temp = src[index];
      index += minor_inc;
      if (index >= size)
        index -= size;
      ecc_a ^= temp;
      ecc_b ^= temp;
      ecc_a = psx_ecc_f_lut[ecc_a];
    }
    ecc_a = psx_ecc_b_lut[psx_ecc_f_lut[ecc_a] ^ ecc_b];
    dest[major] = ecc_a;
    dest[major + major_count] = ecc_a ^ ecc_b;
  }
}

static void psx_ecm_ecc_generate(u8 *sector, int zeroaddress)
{
  u8 address[4];
  if (zeroaddress) {
    for (u32 i = 0; i < 4u; i++) {
      address[i] = sector[12u + i];
      sector[12u + i] = 0;
    }
  }
  psx_ecm_ecc_compute(sector + 0xCu, 86u, 24u, 2u, 86u, sector + 0x81Cu);
  psx_ecm_ecc_compute(sector + 0xCu, 52u, 43u, 86u, 88u, sector + 0x8C8u);
  if (zeroaddress) {
    for (u32 i = 0; i < 4u; i++)
      sector[12u + i] = address[i];
  }
}

static void psx_ecm_eccedc_generate(u8 *sector, int type)
{
  switch (type) {
  case 1:
    psx_ecm_edc_compute(sector + 0x00, 0x810u, sector + 0x810u);
    for (u32 i = 0; i < 8u; i++)
      sector[0x814u + i] = 0;
    psx_ecm_ecc_generate(sector, 0);
    break;
  case 2:
    psx_ecm_edc_compute(sector + 0x10, 0x808u, sector + 0x818u);
    psx_ecm_ecc_generate(sector, 1);
    break;
  case 3:
    psx_ecm_edc_compute(sector + 0x10, 0x91Cu, sector + 0x92Cu);
    break;
  }
}

static int psx_read_byte(void *G, void *kread_fn, s32 fd, u8 *out)
{
  return ((s32)NC(G, kread_fn, (u64)fd, (u64)out, 1, 0, 0, 0) == 1);
}

static int psx_write_bytes(void *G, void *kwrite_fn, s32 fd, const u8 *src, u32 len)
{
  return ((s32)NC(G, kwrite_fn, (u64)fd, (u64)src, len, 0, 0, 0) == (s32)len);
}

static int psx_decode_ecm_to_file(void *G, void *kread_fn, void *kwrite_fn, s32 in_fd, s32 out_fd)
{
  u8 sector[2352];
  u8 b;
  u32 checkedc = 0;
  psx_ecm_tables_init();

  if (!psx_read_byte(G, kread_fn, in_fd, &b) || b != 'E') return 0;
  if (!psx_read_byte(G, kread_fn, in_fd, &b) || b != 'C') return 0;
  if (!psx_read_byte(G, kread_fn, in_fd, &b) || b != 'M') return 0;
  if (!psx_read_byte(G, kread_fn, in_fd, &b) || b != 0x00u) return 0;

  for (;;) {
    u32 type, num, bits = 5u;
    int c;
    if (!psx_read_byte(G, kread_fn, in_fd, &b)) return 0;
    c = b;
    type = (u32)c & 3u;
    num = ((u32)c >> 2) & 0x1Fu;
    while (c & 0x80) {
      if (!psx_read_byte(G, kread_fn, in_fd, &b)) return 0;
      c = b;
      num |= ((u32)(c & 0x7F)) << bits;
      bits += 7u;
    }
    if (num == 0xFFFFFFFFu)
      break;
    num++;
    if (!type) {
      while (num) {
        u32 chunk = (num > sizeof(sector)) ? sizeof(sector) : num;
        if ((s32)NC(G, kread_fn, (u64)in_fd, (u64)sector, chunk, 0, 0, 0) != (s32)chunk)
          return 0;
        checkedc = psx_ecm_edc_partial(checkedc, sector, chunk);
        if (!psx_write_bytes(G, kwrite_fn, out_fd, sector, chunk))
          return 0;
        num -= chunk;
      }
    } else {
      while (num--) {
        for (u32 i = 0; i < sizeof(sector); i++)
          sector[i] = 0;
        for (u32 i = 1; i < 11u; i++)
          sector[i] = 0xFFu;
        if (type == 1u) {
          sector[0x0F] = 0x01u;
          if ((s32)NC(G, kread_fn, (u64)in_fd, (u64)(sector + 0x00C), 0x003u, 0, 0, 0) != 0x003)
            return 0;
          if ((s32)NC(G, kread_fn, (u64)in_fd, (u64)(sector + 0x010), 0x800u, 0, 0, 0) != 0x800)
            return 0;
          psx_ecm_eccedc_generate(sector, 1);
          checkedc = psx_ecm_edc_partial(checkedc, sector, 2352u);
          if (!psx_write_bytes(G, kwrite_fn, out_fd, sector, 2352u))
            return 0;
        } else if (type == 2u) {
          sector[0x0F] = 0x02u;
          if ((s32)NC(G, kread_fn, (u64)in_fd, (u64)(sector + 0x014), 0x804u, 0, 0, 0) != 0x804)
            return 0;
          sector[0x10] = sector[0x14];
          sector[0x11] = sector[0x15];
          sector[0x12] = sector[0x16];
          sector[0x13] = sector[0x17];
          psx_ecm_eccedc_generate(sector, 2);
          checkedc = psx_ecm_edc_partial(checkedc, sector + 0x10, 2336u);
          if (!psx_write_bytes(G, kwrite_fn, out_fd, sector + 0x10, 2336u))
            return 0;
        } else if (type == 3u) {
          sector[0x0F] = 0x02u;
          if ((s32)NC(G, kread_fn, (u64)in_fd, (u64)(sector + 0x014), 0x918u, 0, 0, 0) != 0x918)
            return 0;
          sector[0x10] = sector[0x14];
          sector[0x11] = sector[0x15];
          sector[0x12] = sector[0x16];
          sector[0x13] = sector[0x17];
          psx_ecm_eccedc_generate(sector, 3);
          checkedc = psx_ecm_edc_partial(checkedc, sector + 0x10, 2336u);
          if (!psx_write_bytes(G, kwrite_fn, out_fd, sector + 0x10, 2336u))
            return 0;
        } else {
          return 0;
        }
      }
    }
  }

  if ((s32)NC(G, kread_fn, (u64)in_fd, (u64)sector, 4u, 0, 0, 0) != 4)
    return 0;
  if (sector[0] != (u8)(checkedc >> 0) ||
      sector[1] != (u8)(checkedc >> 8) ||
      sector[2] != (u8)(checkedc >> 16) ||
      sector[3] != (u8)(checkedc >> 24))
    return 0;
  return 1;
}

static u32 psx_detect_region_from_sector_text(struct psx_system *psx)
{
  u8 sec[2048];
  for (u32 lba = 0; lba < 64u; lba++) {
    if (!psx_read_logical_sector(psx, lba, sec))
      break;
    for (u32 i = 0; i + 4u <= sizeof(sec); i++) {
      u32 r = psx_region_from_boot_path((const char *)&sec[i]);
      if (r != 3u)
        return r;
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
  {
    psx->cd_disc_region = psx_detect_region_from_sector_text(psx);
    return;
  }
  if (!psx_read_logical_sector(psx, cnf_lba, sec))
    return;
  if (!psx_extract_boot_path(sec, (cnf_size < sizeof(sec)) ? cnf_size : sizeof(sec),
                             psx->cd_boot_path, sizeof(psx->cd_boot_path)))
  {
    psx->cd_disc_region = psx_detect_region_from_sector_text(psx);
    return;
  }

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

#define O_RDONLY 0x0000
#define O_WRONLY 0x0001
#define O_CREAT  0x0200
#define O_TRUNC  0x0400
#define DT_DIR   4
#define DT_REG   8

#ifndef PS5SDK_PAD_BTN_L3
#define PS5SDK_PAD_BTN_L3       0x00000100u
#endif
#ifndef PS5SDK_PAD_BTN_R3
#define PS5SDK_PAD_BTN_R3       0x00000200u
#endif
#ifndef PS5SDK_PAD_BTN_TOUCHPAD
#define PS5SDK_PAD_BTN_TOUCHPAD 0x00020000u
#endif

enum {
  PSX_MENU_MAX_GAMES = 128,
  PSX_MENU_LIST_STEP = 38,
  PSX_MENU_TITLE_Y = 36,
  PSX_MENU_LIST_Y = 184,
  PSX_MENU_LIST_H = 780,
};

struct psx_menu_entry {
  char name[96];
  char label[12];
  char path[256];
  u32 launch_bios;
  u32 bios_file;
};

struct psx_menu_state {
  struct psx_menu_entry entries[PSX_MENU_MAX_GAMES];
  s32 count;
  s32 cursor;
  s32 scroll;
  u32 bios_browser;
};

static void psx_str_copy(char *dst, u32 cap, const char *src) {
  u32 i = 0;
  if (!dst || !cap)
    return;
  if (!src) {
    dst[0] = 0;
    return;
  }
  while (src[i] && (i + 1u) < cap) {
    dst[i] = src[i];
    i++;
  }
  dst[i] = 0;
}

static void psx_str_cat(char *dst, u32 cap, const char *src) {
  u32 i = 0;
  if (!dst || !cap || !src)
    return;
  while (dst[i] && i < cap)
    i++;
  while (*src && (i + 1u) < cap)
    dst[i++] = *src++;
  dst[i] = 0;
}

static int psx_str_eq(const char *a, const char *b) {
  if (!a || !b)
    return 0;
  while (*a && *b) {
    if (*a != *b)
      return 0;
    a++;
    b++;
  }
  return (*a == 0 && *b == 0);
}

static int psx_str_cmp(const char *a, const char *b) {
  while (*a && *b && *a == *b) {
    a++;
    b++;
  }
  return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

static int psx_has_disc_extension(const char *name) {
  u32 len = 0;
  while (name && name[len])
    len++;
  if (len < 4)
    return 0;
  if (psx_ascii_upper(name[len - 4]) == '.' &&
      psx_ascii_upper(name[len - 3]) == 'B' &&
      psx_ascii_upper(name[len - 2]) == 'I' &&
      psx_ascii_upper(name[len - 1]) == 'N')
    return 1;
  if (psx_ascii_upper(name[len - 4]) == '.' &&
      psx_ascii_upper(name[len - 3]) == 'I' &&
      psx_ascii_upper(name[len - 2]) == 'S' &&
      psx_ascii_upper(name[len - 1]) == 'O')
    return 1;
  if (len >= 4 &&
      psx_ascii_upper(name[len - 4]) == '.' &&
      psx_ascii_upper(name[len - 3]) == 'I' &&
      psx_ascii_upper(name[len - 2]) == 'M' &&
      psx_ascii_upper(name[len - 1]) == 'G')
    return 1;
  return 0;
}

static int psx_has_bios_extension(const char *name) {
  u32 len = 0;
  while (name && name[len])
    len++;
  if (len < 4)
    return 0;
  return (psx_ascii_upper(name[len - 4]) == '.' &&
          psx_ascii_upper(name[len - 3]) == 'B' &&
          psx_ascii_upper(name[len - 2]) == 'I' &&
          psx_ascii_upper(name[len - 1]) == 'N');
}

static u32 psx_region_from_bios_name(const char *name)
{
  u32 i = 0;
  if (!name)
    return 3u;
  while (name[i]) {
    char c0 = psx_ascii_upper(name[i + 0]);
    char c1 = psx_ascii_upper(name[i + 1]);
    char c2 = psx_ascii_upper(name[i + 2]);
    char c3 = psx_ascii_upper(name[i + 3]);
    char c4 = name[i + 4];
    char c5 = name[i + 5];
    char c6 = name[i + 6];
    char c7 = name[i + 7];
    if (c0 == 'S' && c1 == 'C' && c2 == 'P' && c3 == 'H' &&
        c4 >= '0' && c4 <= '9' &&
        c5 >= '0' && c5 <= '9' &&
        c6 >= '0' && c6 <= '9' &&
        c7 >= '0' && c7 <= '9') {
      if (c7 == '0') return 0u;
      if (c7 == '1') return 1u;
      if (c7 == '2') return 2u;
    }
    i++;
  }
  return 3u;
}

static void psx_menu_set_region_label(char *dst, u32 cap, u32 region)
{
  if (!dst || !cap)
    return;
  if (region == 0u)
    psx_str_copy(dst, cap, "JPN");
  else if (region == 1u)
    psx_str_copy(dst, cap, "USA");
  else if (region == 2u)
    psx_str_copy(dst, cap, "EUR");
  else
    psx_str_copy(dst, cap, "BIOS");
}

static void psx_menu_make_title(char *dst, u32 cap, const char *src)
{
  u32 len = 0, cut = 0;
  if (!dst || !cap) return;
  dst[0] = 0;
  if (!src) return;
  while (src[len]) len++;
  cut = len;
  if (len > 4 && src[len - 4] == '.')
    cut = len - 4;
  for (u32 i = 0; i < cut && (i + 1u) < cap; i++)
    dst[i] = src[i];
  dst[(cut < cap) ? cut : (cap - 1u)] = 0;
}

static void psx_menu_make_label(char *dst, u32 cap, const char *src)
{
  u32 len = 0;
  if (!dst || !cap) return;
  dst[0] = 0;
  if (!src) return;
  while (src[len]) len++;
  if (len > 4 && src[len - 4] == '.') {
    dst[0] = psx_ascii_upper(src[len - 3]);
    dst[1] = psx_ascii_upper(src[len - 2]);
    dst[2] = psx_ascii_upper(src[len - 1]);
    dst[3] = 0;
  }
}

static void psx_menu_sort(struct psx_menu_state *ms) {
  s32 start = (ms->bios_browser || (ms->count > 0 && ms->entries[0].launch_bios)) ? 1 : 0;
  for (s32 i = start; i < ms->count - 1; i++) {
    for (s32 j = start + 1; j < ms->count - (i - start); j++) {
      if (psx_str_cmp(ms->entries[j - 1].name, ms->entries[j].name) > 0) {
        struct psx_menu_entry tmp = ms->entries[j - 1];
        ms->entries[j - 1] = ms->entries[j];
        ms->entries[j] = tmp;
      }
    }
  }
}

static void psx_menu_load(struct psx_menu_state *ms, void *G, void *mmap_fn,
                          void *munmap_fn, void *kopen_fn, void *getdents_fn,
                          void *kclose_fn) {
  s32 fd;
  u8 *db;
  ms->count = 0;
  ms->cursor = 0;
  ms->scroll = 0;
  ms->bios_browser = 0u;

  psx_str_copy(ms->entries[ms->count].name, sizeof(ms->entries[ms->count].name),
               "Launch BIOS");
  psx_str_copy(ms->entries[ms->count].label, sizeof(ms->entries[ms->count].label),
               "BIOS");
  ms->entries[ms->count].path[0] = 0;
  ms->entries[ms->count].launch_bios = 1u;
  ms->entries[ms->count].bios_file = 0u;
  ms->count++;

  if (!G || !mmap_fn || !munmap_fn || !kopen_fn || !getdents_fn || !kclose_fn)
    return;

  fd = (s32)NC(G, kopen_fn, (u64)"/temp0/psx/games", O_RDONLY, 0, 0, 0, 0);
  if (fd < 0)
    return;

  db = (u8 *)NC(G, mmap_fn, 0, 0x8000, 3, 0x1002, (u64)-1, 0);
  if (!db || (s64)db == -1) {
    NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
    return;
  }

  for (;;) {
    s32 n = (s32)NC(G, getdents_fn, (u64)fd, (u64)db, 0x8000, 0, 0, 0);
    if (n <= 0)
      break;
    for (s32 off = 0; off + 8 <= n && ms->count < PSX_MENU_MAX_GAMES;) {
      u16 reclen = *(u16 *)(db + off + 4);
      u8 type = *(u8 *)(db + off + 6);
      char *name = (char *)(db + off + 8);
      if (reclen < 8 || off + reclen > n)
        break;
      if (type == DT_REG && name[0] && !psx_str_eq(name, ".") &&
          !psx_str_eq(name, "..") && psx_has_disc_extension(name)) {
        psx_menu_make_title(ms->entries[ms->count].name,
                            sizeof(ms->entries[ms->count].name), name);
        psx_menu_make_label(ms->entries[ms->count].label,
                            sizeof(ms->entries[ms->count].label), name);
        psx_str_copy(ms->entries[ms->count].path,
                     sizeof(ms->entries[ms->count].path),
                     "/temp0/psx/games/");
        psx_str_cat(ms->entries[ms->count].path,
                    sizeof(ms->entries[ms->count].path), name);
        ms->entries[ms->count].launch_bios = 0u;
        ms->entries[ms->count].bios_file = 0u;
        ms->count++;
      }
      off += reclen;
    }
  }

  NC(G, munmap_fn, (u64)db, 0x8000, 0, 0, 0, 0);
  NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
  psx_menu_sort(ms);
}

static void psx_menu_load_bios(struct psx_menu_state *ms, void *G, void *mmap_fn,
                               void *munmap_fn, void *kopen_fn, void *getdents_fn,
                               void *kclose_fn) {
  s32 fd;
  u8 *db;
  ms->count = 0;
  ms->cursor = 0;
  ms->scroll = 0;
  ms->bios_browser = 1u;

  psx_str_copy(ms->entries[ms->count].name, sizeof(ms->entries[ms->count].name),
               "Back");
  psx_str_copy(ms->entries[ms->count].label, sizeof(ms->entries[ms->count].label),
               "MENU");
  ms->entries[ms->count].path[0] = 0;
  ms->entries[ms->count].launch_bios = 0u;
  ms->entries[ms->count].bios_file = 0u;
  ms->count++;

  if (!G || !mmap_fn || !munmap_fn || !kopen_fn || !getdents_fn || !kclose_fn)
    return;

  fd = (s32)NC(G, kopen_fn, (u64)"/temp0/psx/bios", O_RDONLY, 0, 0, 0, 0);
  if (fd < 0)
    return;

  db = (u8 *)NC(G, mmap_fn, 0, 0x8000, 3, 0x1002, (u64)-1, 0);
  if (!db || (s64)db == -1) {
    NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
    return;
  }

  for (;;) {
    s32 n = (s32)NC(G, getdents_fn, (u64)fd, (u64)db, 0x8000, 0, 0, 0);
    if (n <= 0)
      break;
    for (s32 off = 0; off + 8 <= n && ms->count < PSX_MENU_MAX_GAMES;) {
      u16 reclen = *(u16 *)(db + off + 4);
      u8 type = *(u8 *)(db + off + 6);
      char *name = (char *)(db + off + 8);
      if (reclen < 8 || off + reclen > n)
        break;
      if (type == DT_REG && name[0] && !psx_str_eq(name, ".") &&
          !psx_str_eq(name, "..") && psx_has_bios_extension(name)) {
        u32 region = psx_region_from_bios_name(name);
        psx_menu_make_title(ms->entries[ms->count].name,
                            sizeof(ms->entries[ms->count].name), name);
        psx_menu_set_region_label(ms->entries[ms->count].label,
                                  sizeof(ms->entries[ms->count].label), region);
        psx_str_copy(ms->entries[ms->count].path,
                     sizeof(ms->entries[ms->count].path),
                     "/temp0/psx/bios/");
        psx_str_cat(ms->entries[ms->count].path,
                    sizeof(ms->entries[ms->count].path), name);
        ms->entries[ms->count].launch_bios = 0u;
        ms->entries[ms->count].bios_file = 1u;
        ms->count++;
      }
      off += reclen;
    }
  }

  NC(G, munmap_fn, (u64)db, 0x8000, 0, 0, 0, 0);
  NC(G, kclose_fn, (u64)fd, 0, 0, 0, 0, 0);
  psx_menu_sort(ms);
}

static void psx_menu_draw(u32 *fb, const struct psx_menu_state *ms) {
  s32 max_visible = PSX_MENU_LIST_H / PSX_MENU_LIST_STEP;
  ps5sdk_fb_fill(fb, PSX_FB_W, PSX_FB_H, 0x000E1017);
  ps5sdk_fb_rect(fb, PSX_FB_W, PSX_FB_H, 0, 0, PSX_FB_W, 132, 0x00151B26);
  ps5sdk_fb_rect(fb, PSX_FB_W, PSX_FB_H, 0, 132, PSX_FB_W / 2, 4, 0x00D93A2F);
  ps5sdk_fb_rect(fb, PSX_FB_W, PSX_FB_H, PSX_FB_W / 2, 132, PSX_FB_W / 2, 4,
                 0x0009A8FF);
  ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, 48, PSX_MENU_TITLE_Y,
                "PSX - PS5 EMULATOR", 0x00F5F7FB, 0, 4, 1);
  if (ms->bios_browser) {
    ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, 50, 92,
                  "TEMP0/PSX/BIOS  |  CROSS: choose BIOS  CIRCLE: back",
                  0x0097A3B7, 0, 2, 1);
  } else {
    ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, 50, 92,
                  "TEMP0/PSX/GAMES  |  CROSS: launch  UP/DOWN: move",
                  0x0097A3B7, 0, 2, 1);
  }
  ps5sdk_fb_rect(fb, PSX_FB_W, PSX_FB_H, 40, 164, PSX_FB_W - 80,
                 PSX_MENU_LIST_H, 0x00101319);
  ps5sdk_fb_rect(fb, PSX_FB_W, PSX_FB_H, 0, PSX_FB_H - 54, PSX_FB_W, 54,
                 0x00151B26);
  if (ms->bios_browser) {
    ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, 44, PSX_FB_H - 36,
                  "CIRCLE: back  TRIANGLE: reload BIOS list",
                  0x008D98AA, 0, 2, 1);
  } else {
    ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, 44, PSX_FB_H - 36,
                  "CROSS on 'Launch BIOS': choose BIOS  TRIANGLE: reload list",
                  0x008D98AA, 0, 2, 1);
  }

  for (s32 i = 0; i < max_visible && (ms->scroll + i) < ms->count; i++) {
    s32 idx = ms->scroll + i;
    s32 y = PSX_MENU_LIST_Y + i * PSX_MENU_LIST_STEP;
    u32 color = ms->entries[idx].launch_bios ? 0x00FFD166u : 0x00C7D2E3u;
    if (idx == ms->cursor) {
      ps5sdk_fb_rect(fb, PSX_FB_W, PSX_FB_H, 48, y - 4, PSX_FB_W - 96, 32,
                     0x00263042);
      color = 0x00FFFFFFu;
    }
    ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, 64, y, ms->entries[idx].name, color,
                  0, 3, 1);
    ps5sdk_fb_str(fb, PSX_FB_W, PSX_FB_H, PSX_FB_W - 180, y,
                  ms->entries[idx].label,
                  (idx == ms->cursor) ? 0x00FFD166u : 0x008D98AAu, 0, 2, 1);
  }
}

static int psx_menu_run(void *G, void *usleep_fn, void *mmap_fn, void *munmap_fn,
                        void *kopen_fn, void *getdents_fn, void *kclose_fn,
                        void *pad_read_fn, s32 pad_h, u8 *pad_buf, s32 video_h,
                        void *vid_flip, void **fbs, char *out_game_path,
                        u32 out_game_cap, char *out_bios_path,
                        u32 out_bios_cap) {
  struct psx_menu_state ms;
  u32 prev_btns = 0;
  u32 frame = 0;
  s32 max_visible = PSX_MENU_LIST_H / PSX_MENU_LIST_STEP;
  psx_menu_load(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn, kclose_fn);

  for (;;) {
    u32 raw = 0, btns = 0;
    u32 pressed;
    s32 idx;

    if (pad_read_fn && pad_h >= 0 && pad_buf)
      ps5_sdk_pad_read_buttons(G, pad_read_fn, pad_h, pad_buf, &raw, &btns);
    pressed = btns & ~prev_btns;
    prev_btns = btns;

    if (pressed & PS5SDK_PAD_BTN_TRIANGLE) {
      if (ms.bios_browser)
        psx_menu_load_bios(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                           kclose_fn);
      else
        psx_menu_load(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                      kclose_fn);
    }
    if ((pressed & PS5SDK_PAD_BTN_CIRCLE) && ms.bios_browser) {
      psx_menu_load(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                    kclose_fn);
    }
    if (pressed & PS5SDK_PAD_BTN_OPTIONS) {
      psx_menu_load_bios(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                         kclose_fn);
    }
    if (pressed & PS5SDK_PAD_BTN_DOWN) {
      if (ms.cursor + 1 < ms.count)
        ms.cursor++;
    }
    if (pressed & PS5SDK_PAD_BTN_UP) {
      if (ms.cursor > 0)
        ms.cursor--;
    }
    if (ms.cursor < ms.scroll)
      ms.scroll = ms.cursor;
    if (ms.cursor >= ms.scroll + max_visible)
      ms.scroll = ms.cursor - max_visible + 1;

    if (pressed & PS5SDK_PAD_BTN_CROSS) {
      if (ms.entries[ms.cursor].launch_bios) {
        psx_menu_load_bios(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                           kclose_fn);
      } else if (ms.bios_browser) {
        if (ms.entries[ms.cursor].bios_file) {
          if (out_game_path && out_game_cap)
            out_game_path[0] = 0;
          if (out_bios_path && out_bios_cap)
            psx_str_copy(out_bios_path, out_bios_cap, ms.entries[ms.cursor].path);
          return 1;
        }
        psx_menu_load(&ms, G, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                      kclose_fn);
      } else if (out_game_path && out_game_cap) {
        psx_str_copy(out_game_path, out_game_cap, ms.entries[ms.cursor].path);
        if (out_bios_path && out_bios_cap)
          out_bios_path[0] = 0;
        return 1;
      }
    }

    idx = (s32)(frame & 1u);
    psx_menu_draw((u32 *)fbs[idx], &ms);
    NC(G, vid_flip, (u64)video_h, (u64)idx, 1, 0, 0, 0);
    if (usleep_fn)
      NC(G, usleep_fn, 16000, 0, 0, 0, 0, 0);
    frame++;
  }
}

static u16 psx_map_ps5_to_psx_pad(u32 ps5_btns) {
  u16 pad = 0xFFFFu;
  if (ps5_btns & PS5SDK_PAD_BTN_TOUCHPAD)
    pad &= (u16)~0x0001u; /* Select */
  if (ps5_btns & PS5SDK_PAD_BTN_OPTIONS)  pad &= (u16)~0x0008u; /* Start */
  if (ps5_btns & PS5SDK_PAD_BTN_UP)       pad &= (u16)~0x0010u;
  if (ps5_btns & PS5SDK_PAD_BTN_RIGHT)    pad &= (u16)~0x0020u;
  if (ps5_btns & PS5SDK_PAD_BTN_DOWN)     pad &= (u16)~0x0040u;
  if (ps5_btns & PS5SDK_PAD_BTN_LEFT)     pad &= (u16)~0x0080u;
  if (ps5_btns & PS5SDK_PAD_BTN_L3)       pad &= (u16)~0x0100u; /* L2 */
  if (ps5_btns & PS5SDK_PAD_BTN_R3)       pad &= (u16)~0x0200u; /* R2 */
  if (ps5_btns & PS5SDK_PAD_BTN_L1)       pad &= (u16)~0x0400u;
  if (ps5_btns & PS5SDK_PAD_BTN_R1)       pad &= (u16)~0x0800u;
  if (ps5_btns & PS5SDK_PAD_BTN_TRIANGLE) pad &= (u16)~0x1000u;
  if (ps5_btns & PS5SDK_PAD_BTN_CIRCLE)   pad &= (u16)~0x2000u;
  if (ps5_btns & PS5SDK_PAD_BTN_CROSS)    pad &= (u16)~0x4000u;
  if (ps5_btns & PS5SDK_PAD_BTN_SQUARE)   pad &= (u16)~0x8000u;
  return pad;
}

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
  void *getdents_fn = SYM(G, D, LIBKERNEL_HANDLE, KERNEL_sceKernelGetdents);
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

  if (load_mod)
    NC(G, load_mod, (u64)"libScePad.sprx", 0, 0, 0, 0, 0);
  void *pad_init_fn = SYM(G, D, PAD_HANDLE, "scePadInit");
  void *pad_geth_fn = SYM(G, D, PAD_HANDLE, "scePadGetHandle");
  void *pad_read_fn = SYM(G, D, PAD_HANDLE, "scePadRead");
  if (pad_init_fn)
    NC(G, pad_init_fn, 0, 0, 0, 0, 0, 0);
  s32 pad_h = pad_geth_fn ? (s32)NC(G, pad_geth_fn, (u64)ext->dbg[3], 0, 0, 0, 0, 0) : -1;
  u8 *pad_buf = (u8 *)NC(G, mmap_fn, 0, 0x1000, 3, 0x1002, (u64)-1, 0);
  if (!pad_buf || (s64)pad_buf == -1)
    pad_buf = 0;

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
  
  /* sceAudioOutOpen: userId=0xFF, portType=0 (Main), grain=256, rate=48000Hz */
  s32 audio_h = audio_open ? (s32)NC(G, audio_open, 0xFF, 0, 0, 256, 48000, 1) : -1;
  
  psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr, "[PSX] AudioOpen h=", (u32)audio_h);

  if (audio_h >= 0) {
      /* The Main port on PS5 defaults to volume=0; must set it explicitly */
      void *audio_vol = SYM(G, D, AUDIOOUT_HANDLE, "sceAudioOutSetVolume");
      if (audio_vol) {
          /* 8-element array covering all channel slots the SDK may expect */
          s32 vol[8] = { 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF,
                         0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF };
          /* Bit mask 0x3F = channels L,R,C,LFE,SL,SR */
          s32 vret = (s32)NC(G, audio_vol, (u64)audio_h, 0x3F,
                             (u64)vol, 0, 0, 0);
          psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                     "[PSX] VolSet ret=", (u32)vret);
      }
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
  char selected_game[256];
  char selected_bios[256];
  selected_game[0] = 0;
  selected_bios[0] = 0;
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
  {
    if (!psx_menu_run(G, usleep_fn, mmap_fn, munmap_fn, kopen_fn, getdents_fn,
                      kclose_fn, pad_read_fn, pad_h, pad_buf, video_h, vid_flip,
                      fbs, selected_game, sizeof(selected_game),
                      selected_bios, sizeof(selected_bios))) {
      ext->status = -7;
      goto cleanup_fb;
    }
    if (selected_game[0])
      psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                  "[PSX] Menu selecciono juego\n");
    else if (selected_bios[0])
      psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                  "[PSX] Menu selecciono BIOS explicita\n");
    else
      psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                  "[PSX] Menu selecciono BIOS\n");
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
  /* Pre-load the interpolated volume level so the first render tick is audible */
  st->sys.spu_main_vol_cur[0] = 0x3FFF;
  st->sys.spu_main_vol_cur[1] = 0x3FFF;
  st->sys.joy_baud = 0xDCu;
  st->sys.joy_mc_flag = 0x08u;
  st->sys.pad_buttons_psx = 0xFFFFu;
  st->sys.cd_disc_region = 3u;
  st->sys.cd_console_region = 3u;
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
  st->sys.cd_fd = -1;
  if (selected_game[0]) {
    st->sys.cd_fd =
        (s32)NC(G, kopen_fn, (u64)selected_game, O_RDONLY, 0, 0, 0, 0);
  }
  if (st->sys.cd_fd > 0) {
    LOG("[PSX] Juego abierto\n");
    st->sys.cd_shell_open = 1;
    {
      s64 fsz = (s64)NC(G, klseek_fn, (u64)st->sys.cd_fd, 0, 2, 0, 0, 0);
      st->sys.cd_sector_size =
          psx_detect_image_sector_size(G, klseek_fn, kread_fn, st->sys.cd_fd,
                                       (fsz > 0) ? (u64)fsz : 0u,
                                       selected_game);
      if (st->sys.cd_sector_size == 2352u)
        LOG("[PSX] Formato RAW (2352 bytes/sector)\n");
      else if (st->sys.cd_sector_size == 2336u)
        LOG("[PSX] Formato RAW/M2 (2336 bytes/sector)\n");
      else
        LOG("[PSX] Formato ISO (2048 bytes/sector)\n");
      if (fsz > 0)
        st->sys.cd_total_sectors = (u32)((u64)fsz / st->sys.cd_sector_size);
      if (psx_path_has_ext(selected_game, ".bin") && st->sys.cd_sector_size == 2048u) {
        psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                   "[PSX] DetectFileSize=", (u32)((fsz > 0) ? (u64)fsz : 0u));
      }
    }
    NC(G, klseek_fn, (u64)st->sys.cd_fd, 0, 0, 0, 0, 0); /* reset to start */
    psx_detect_disc_boot_info(&st->sys);
    if (st->sys.cd_disc_region == 3u) {
      u32 file_region = psx_region_from_filename(selected_game);
      if (file_region != 3u)
        st->sys.cd_disc_region = file_region;
    }
    if (st->sys.cd_boot_path[0]) {
      psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                  "[PSX] SYSTEM.CNF detectado\n");
    } else {
      psx_udp_log(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                  "[PSX] SYSTEM.CNF no detectado\n");
    }
    psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
               "[PSX] DiscRegion=", st->sys.cd_disc_region);
  } else {
    LOG("[PSX] BIOS only / juego no encontrado\n");
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
    if (selected_bios[0]) {
      fd = (s32)NC(G, kopen_fn, (u64)selected_bios, 0, 0, 0, 0, 0);
      if (fd > 0)
        st->sys.cd_console_region = psx_region_from_bios_name(selected_bios);
    } else if (st->sys.cd_disc_region == 1u) {
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
      /* BIOS audio warmup: feed the 48kHz sink using a fractional chunk
       * accumulator (800 samples/frame at 60Hz => 3.125 chunks of 256). */
      if (st->sys.audio_h >= 0 && st->sys.audio_out_fn && st->sys.audio_vbuf) {
        void psx_spu_render(struct psx_system *psx, s16 *out, u32 count);
        st->sys.audio_chunk_accum += 800u;
        while (st->sys.audio_chunk_accum >= 256u) {
          st->sys.audio_chunk_accum -= 256u;
          psx_spu_render(&st->sys, st->sys.audio_vbuf, 256);
          /* Log first non-zero warmup sample to confirm SPU generates audio */
          if (!st->sys.audio_phase && st->sys.audio_vbuf[0] != 0) {
            st->sys.audio_phase = 1;
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] FirstNZ frm=", warm_frames);
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] FirstNZ s0=", (u32)(u16)st->sys.audio_vbuf[0]);
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] FirstNZ V0ra=", st->sys.spu_voice_addr[0]);
          }
          s32 wret = (s32)NC(G, st->sys.audio_out_fn, (u64)st->sys.audio_h,
             (u64)st->sys.audio_vbuf, 0, 0, 0, 0);
          /* Always log first output result to confirm driver accepts buffer */
          if (warm_frames == 0) {
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] out0 ret=", (u32)wret);
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] out0 L=", (u32)(u16)st->sys.audio_vbuf[0]);
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] V0adsr=", (u32)st->sys.spu_adsr_vol[0]);
            psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                       "[SPU] MVcur=", (u32)st->sys.spu_main_vol_cur[0]);
          }
        }
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
    if (pad_read_fn && pad_h >= 0 && pad_buf) {
      u32 raw = 0, btns = 0;
      ps5_sdk_pad_read_buttons(G, pad_read_fn, pad_h, pad_buf, &raw, &btns);
      st->sys.pad_buttons_psx = (u32)psx_map_ps5_to_psx_pad(btns);
    } else {
      st->sys.pad_buttons_psx = 0xFFFFu;
    }
    psx_bus_vblank(&st->sys);
    vblank++;

    /* Re-inject CLUT: BIOS shell needs this for text visibility */
    psx_inject_bios_clut(&st->sys);

    /* ── CPU + Audio interleaved ─────────────────────────────── */
    /* Run CPU in chunks, outputting one audio buffer (256 samples) per chunk.
     * At PSX 33.8688 MHz: 33868800/48000 * 256 ≈ 180736 CPU cycles per audio buffer.
     * We use 180736 cycles/chunk so audio feeds continuously, preventing underruns. */
    if (st->sys.audio_h >= 0 && st->sys.audio_out_fn && st->sys.audio_vbuf) {
        void psx_spu_render(struct psx_system *psx, s16 *out, u32 count);
        /* 564480 total cycles / 180736 = ~3.125 chunks per frame */
        u32 cycles_left = 564480u;
        while (cycles_left > 0u) {
            u32 chunk_cycles = (cycles_left > 180736u) ? 180736u : cycles_left;
            for (u32 _i = 0; _i < chunk_cycles; _i++)
                psx_cpu_step(&st->sys);
            cycles_left -= chunk_cycles;
            /* Render and output one audio buffer after each CPU chunk */
            psx_spu_render(&st->sys, st->sys.audio_vbuf, 256);
            s32 aret = (s32)NC(G, st->sys.audio_out_fn, (u64)st->sys.audio_h,
                               (u64)st->sys.audio_vbuf, 0, 0, 0, 0);
            if (aret < 0 && (vblank % 60 == 0))
                psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                           "[PSX] AudioErr=", (u32)aret);
        }
    } else {
        /* No audio: run CPU straight through */
        for (u32 i = 0; i < 564480u; i++)
            psx_cpu_step(&st->sys);
    }
    /* ── GPU sync & Flip ─────────────────────────────────────── */
    u32 idx = st->gpu.fb_idx & 1u;
    psx_gpu_sync_to_ps5(&st->sys, (u32 *)fbs[idx]);
    NC(G, vid_flip, (u64)video_h, (u64)idx, 1, 0, 0, 0);
    st->gpu.fb_idx++;

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

    /* SPU diagnostics every 60 VBLANKs (~1 second) */
    if ((vblank % 60u) == 0) {
      struct psx_system *sy = &st->sys;
      /* Audio handle: -1 means sceAudioOutOpen failed → complete silence */
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "AHN=", (u32)sy->audio_h);
      /* Last rendered samples — L=vbuf[0], R=vbuf[1] */
      if (sy->audio_vbuf) {
        psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                   "AUDL=", (u32)(u16)sy->audio_vbuf[0]);
        psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                   "AUDR=", (u32)(u16)sy->audio_vbuf[1]);
      }
      /* SPUCNT: bit15=enable, bit14=unmute */
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "SPUCNT=", (u32)sy->spu_regs[0xD5]);
      /* Active voice bitmask (bits 0-23) */
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "VON=", sy->spu_voice_on);
      /* Master volume L/R */
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "MVOL=", ((u32)sy->spu_regs[0xC0] << 16) | sy->spu_regs[0xC1]);
      /* Voice 0: ADSR vol, vol L|R, running address, block data */
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "V0ad=", (u32)sy->spu_adsr_vol[0]);
      psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                 "V0vl=", ((u32)sy->spu_regs[0] << 16) | sy->spu_regs[1]);
      {
        u32 _ra = sy->spu_voice_addr[0] & 0x7FFFFu;
        psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                   "V0ra=", sy->spu_voice_addr[0]);
        psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                   "V0h=", (u32)sy->spu_ram[_ra]); /* ADPCM header (shift in bits 3-0) */
        /* First non-silent voice in VON: show its ra and header */
        for (u32 _vv = 1; _vv < 24; _vv++) {
          if (sy->spu_voice_on & (1u << _vv)) {
            u32 _ra2 = sy->spu_voice_addr[_vv] & 0x7FFFFu;
            u8 _h = sy->spu_ram[_ra2];
            if ((_h & 0xFu) >= 4u || *(u32*)(sy->spu_ram + _ra2) != 0u) {
              psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                         "Vxn=", _vv);
              psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                         "Vxra=", sy->spu_voice_addr[_vv]);
              psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                         "Vxh=", (u32)_h);
              psx_log_pc(G, sendto_fn, ext->log_fd, (u8 *)ext->log_addr,
                         "Vxad=", (u32)sy->spu_adsr_vol[_vv]);
              break;
            }
          }
        }
      }
    }
  }
#undef LOGPC

cleanup_fb:
  if (pad_buf && munmap_fn)
    NC(G, munmap_fn, (u64)pad_buf, 0x1000, 0, 0, 0, 0);
  if (vmem && munmap_fn)
    NC(G, munmap_fn, (u64)vmem, PSX_FB_TOTAL, 0, 0, 0, 0);
  if (phys && rel_dm)
    NC(G, rel_dm, phys, PSX_FB_TOTAL, 0, 0, 0, 0);
cleanup_vid:
  if (vid_close && video_h >= 0)
    NC(G, vid_close, (u64)video_h, 0, 0, 0, 0, 0);
#undef LOG
}
