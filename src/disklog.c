/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "../build/build_tag.h"

#include "disklog.h"
#include "iodev.h"
#include "nvme.h"
#include "string.h"
#include "types.h"
#include "utils.h"

#ifdef DISKLOG
#define DISKLOG_ENABLED true
#else
#define DISKLOG_ENABLED false
#endif

#define DISKLOG_NSID       1
#define DISKLOG_BLOCK      SZ_4K
#define DISKLOG_SIZE       ((DISKLOG_ENABLED ? 64 : 1) * DISKLOG_BLOCK)
#define DISKLOG_PART_MAGIC "NEWSAHI-DISKLOG1"
#define DISKLOG_REC_MAGIC  "M1N1DLOG"

struct disklog_header {
    char magic[8];
    u32 version;
    u32 seq;
    u64 size;
    u64 total;
    char tag[64];
} PACKED;

static u8 ring[DISKLOG_SIZE] ALIGNED(SZ_4K);
static u8 blk[DISKLOG_BLOCK] ALIGNED(SZ_4K);
static u64 total;
static u64 flushed;
static u64 rec_lba;
static u32 seq;
static int state;
static bool busy;

static u64 le64(const u8 *p)
{
    u64 v;
    memcpy(&v, p, sizeof(v));
    return v;
}

static u32 le32(const u8 *p)
{
    u32 v;
    memcpy(&v, p, sizeof(v));
    return v;
}

static const u8 disklog_type[16] = {
    0xa0, 0x4d, 0x62, 0x23, 0x0a, 0x1b, 0x0c, 0x4e,
    0x8e, 0x03, 0x89, 0xba, 0x16, 0x73, 0x6a, 0xd5,
};

static bool disklog_probe(void)
{
    if (!nvme_init())
        return false;

    if (!nvme_read(DISKLOG_NSID, 1, blk) || memcmp(blk, "EFI PART", 8)) {
        printf("disklog: no GPT on nsid %d\n", DISKLOG_NSID);
        return false;
    }

    u64 entries_lba = le64(blk + 72);
    u32 count = le32(blk + 80);
    u32 esize = le32(blk + 84);
    if (esize < 128 || esize > DISKLOG_BLOCK || DISKLOG_BLOCK % esize || count > 1024)
        return false;

    u32 per_block = DISKLOG_BLOCK / esize;
    for (u32 i = 0; i < count; i++) {
        if (!(i % per_block) && !nvme_read(DISKLOG_NSID, entries_lba + i / per_block, blk))
            return false;

        const u8 *e = blk + (i % per_block) * esize;
        if (memcmp(e, disklog_type, sizeof(disklog_type)))
            continue;

        u64 start = le64(e + 32);
        u64 end = le64(e + 40);
        if (end - start + 1 < 2 + DISKLOG_SIZE / DISKLOG_BLOCK) {
            printf("disklog: log partition is too small\n");
            return false;
        }

        if (!nvme_read(DISKLOG_NSID, start, blk) ||
            memcmp(blk, DISKLOG_PART_MAGIC, strlen(DISKLOG_PART_MAGIC))) {
            printf("disklog: log partition lacks its magic, leaving it alone\n");
            return false;
        }

        rec_lba = start + 1;
        seq = 1;
        if (nvme_read(DISKLOG_NSID, rec_lba, blk) && !memcmp(blk, DISKLOG_REC_MAGIC, 8))
            seq = ((struct disklog_header *)blk)->seq + 1;

        printf("disklog: writing record %d at LBA 0x%lx\n", seq, rec_lba);
        return true;
    }

    printf("disklog: no log partition\n");
    return false;
}

static bool disklog_write_block(u64 idx)
{
    return nvme_write(DISKLOG_NSID, rec_lba + 1 + idx, ring + idx * DISKLOG_BLOCK);
}

void disklog_flush(void)
{
    if (!DISKLOG_ENABLED || busy || state < 0)
        return;

    busy = true;

    if (!state)
        state = disklog_probe() ? 1 : -1;

    if (state > 0) {
        u64 end = total;
        u64 nblk = DISKLOG_SIZE / DISKLOG_BLOCK;
        bool ok = true;

        if (end - flushed >= DISKLOG_SIZE) {
            for (u64 i = 0; i < nblk && ok; i++)
                ok = disklog_write_block(i);
        } else if (end > flushed) {
            u64 first = (flushed % DISKLOG_SIZE) / DISKLOG_BLOCK;
            u64 last = ((end - 1) % DISKLOG_SIZE) / DISKLOG_BLOCK;
            for (u64 i = first;; i = (i + 1) % nblk) {
                ok = disklog_write_block(i);
                if (!ok || i == last)
                    break;
            }
        }

        struct disklog_header *hdr = (struct disklog_header *)blk;
        memset(blk, 0, sizeof(blk));
        memcpy(hdr->magic, DISKLOG_REC_MAGIC, sizeof(hdr->magic));
        hdr->version = 1;
        hdr->seq = seq;
        hdr->size = DISKLOG_SIZE;
        hdr->total = end;
        strncpy(hdr->tag, BUILD_TAG, sizeof(hdr->tag) - 1);

        if (ok && nvme_write(DISKLOG_NSID, rec_lba, blk) && nvme_flush(DISKLOG_NSID))
            flushed = end;
        else
            state = -1;
    }

    busy = false;
}

static bool disklog_iodev_can_write(void *opaque)
{
    UNUSED(opaque);
    return DISKLOG_ENABLED && state >= 0;
}

static ssize_t disklog_iodev_write(void *opaque, const void *buf, size_t len)
{
    UNUSED(opaque);
    const u8 *p = buf;
    size_t done = 0;

    while (done < len) {
        size_t off = (total + done) % DISKLOG_SIZE;
        size_t n = min(len - done, DISKLOG_SIZE - off);
        memcpy(ring + off, p + done, n);
        done += n;
    }
    total += len;

    return len;
}

static const struct iodev_ops iodev_disklog_ops = {
    .can_write = disklog_iodev_can_write,
    .write = disklog_iodev_write,
};

struct iodev iodev_disklog = {
    .ops = &iodev_disklog_ops,
    .usage = USAGE_CONSOLE,
    .lock = SPINLOCK_INIT,
};
