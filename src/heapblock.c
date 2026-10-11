/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "heapblock.h"
#include "assert.h"
#include "cpu_regs.h"
#include "types.h"
#include "utils.h"
#include "xnuboot.h"

/*
 * This is a non-freeing allocator, used as a backend for malloc and for uncompressing data.
 *
 * Allocating 0 bytes is allowed, and guarantees "infinite" (until the end of RAM) space is
 * available at the returned pointer as long as no other malloc/heapblock calls occur, which is
 * useful as a buffer for unknown-length uncompressed data. A subsequent call with a size will then
 * actually reserve the block.
 */

static void *heap_base;
static void *heap_limit;

void heapblock_init(void)
{
    void *top_of_kernel_data = (void *)cur_boot_args.top_of_kernel_data;

    heap_base = top_of_kernel_data;
    heapblock_alloc(0); // align base

    if (!cpu_features->apple_sysregs_unlocked) {
        u64 ro_start = mrs(SYS_IMP_APL_CTRR_C_LWR_EL1);
        u64 ro_end = ALIGN_UP(mrs(SYS_IMP_APL_CTRR_C_UPR_EL1), SZ_4K);

        if (ro_end > ro_start && ro_end > (u64)heap_base &&
            ro_start < (u64)heap_base + 256 * SZ_1M) {
            printf("Heap: skipping the SMP-RO area 0x%lx..0x%lx\n", ro_start, ro_end);
            heap_base = (void *)ro_end;
        }
    }

    printf("Heap base: %p\n", heap_base);
}

void heapblock_set_limit(void *limit)
{
    heap_limit = limit;
    printf("Heap limit: %p (%ld MiB)\n", limit, ((u8 *)limit - (u8 *)heap_base) >> 20);
}

void *heapblock_alloc(size_t size)
{
    return heapblock_alloc_aligned(size, 64);
}

void *heapblock_alloc_aligned(size_t size, size_t align)
{
    assert((align & (align - 1)) == 0);
    assert(heap_base);

    uintptr_t block = (((uintptr_t)heap_base) + align - 1) & ~(align - 1);

    if (heap_limit && block + size > (uintptr_t)heap_limit)
        panic("heapblock: out of memory (%zu bytes at 0x%lx, limit %p)\n", size, block, heap_limit);

    heap_base = (void *)(block + size);

    return (void *)block;
}
