#pragma once
#include <stdint.h>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define ICACHE  (1)
#define MULTI_THREAD (1)
#define ENABLE_RV32C (0)
#define ENABLE_TLB (1)
#define UNAGLINED_MEM_ACCESS (1)
#define COLLECT_PERF_STATUS (1)
#define AMO_SPIN_LOCK  (0)

#define NR_CPU 2
#define NR_IRQ_CONTEXT_PER_CPU 2
#define NR_IRQ (16 + 1)

#define SBI_BASE 0x01000000
#define DTB_BASE 0x02000000

#define MEM_BASE 0x80000000
#define MEM_BASE2 0x90000000
#define MEM_BASE3 0xA0000000
#define MEM_BASE4 0xB0000000
#define MEM_SIZE (1048576ULL * 256)

#define UART8250_BASE 0x10000000

#define CLINT_BASE 0x20000000
#define CLINT_IPI_OFF 0
#define CLINT_TIMER_CMP_OFF 0x4000
#define CLINT_TIMER_VAL_OFF 0xbff8
#define CLINT_TIMER_VALH_OFF (CLINT_TIMER_VAL_OFF + 4)

#define PLIC_BASE 0x30000000
#define PLIC_PRIORITY_BASE 0x0
#define PLIC_PENDING_BASE 0x1000
#define PLIC_ENABLE_BASE 0x2000
#define PLIC_ENABLE_STRIDE 0x80
#define PLIC_CONTEXT_BASE 0x200000
#define PLIC_CONTEXT_STRIDE 0x1000

#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 760
#define VRAM_BASE 0x40000000
#define VRAM_SIZE (VIDEO_WIDTH * VIDEO_HEIGHT * 4)
#define VIRTIO_BASE1 0x50000000

#define VIRTIO_BLK_BASE 0x60000000

#define NOP 0b000
#define READ 0b001
#define WRITE 0b010
#define EXEC 0b100

#define EXC_OK (-1)
#define EXC_INST_MISALIGNED 0
#define EXC_INST_ACCESS 1
#define EXC_INST_ILLEGAL 2
#define EXC_BREAKPOINT 3
#define EXC_LOAD_MISALIGNED 4
#define EXC_LOAD_ACCESS 5
#define EXC_STORE_MISALIGNED 6
#define EXC_STORE_ACCESS 7
#define EXC_SYSCALL 8
#define EXC_SUPERVISOR_SYSCALL 9
#define EXC_HYPERVISOR_SYSCALL 10
#define EXC_M_MODE_SYSCALL 11
#define EXC_INST_PAGE_FAULT 12
#define EXC_LOAD_PAGE_FAULT 13
#define EXC_STORE_PAGE_FAULT 15

typedef struct extirq_def_t
{
    int pending;
    int priority;
    void (*do_cmplt)(void);
} extirq_def_t;

extern extirq_def_t extirq_slot[NR_IRQ];

#define IRQ_NUM_VIRTIO_INTPUT (1)
#define IRQ_NUM_VIRTIO_BLK (2)
#define IRQ_NUM_UART (10)

#if defined(__x86_64__) || defined(__i386__)
#define chk_not_aligned(adr, v) 0
#else
#define chk_not_aligned(adr, v) (adr) % (v)
#endif

#define fetch_m(base_p, off, len, dst)                                \
    {                                                                 \
        uint8_t *base = (uint8_t *)base_p;                            \
        switch (len)                                                  \
        {                                                             \
        case 1:                                                       \
            ((uint8_t *)dst)[0] = *((uint8_t *)(&base[off]));         \
            break;                                                    \
        case 2:                                                       \
            if (unlikely(chk_not_aligned(off, 2)))                    \
            {                                                         \
                ((uint8_t *)dst)[0] = *((uint8_t *)(&base[off]));     \
                ((uint8_t *)dst)[1] = *((uint8_t *)(&base[off + 1])); \
            }                                                         \
            else                                                      \
            {                                                         \
                *((uint16_t *)dst) = *((uint16_t *)(&base[off]));     \
            }                                                         \
            break;                                                    \
        case 4:                                                       \
            if (unlikely(chk_not_aligned(off, 4)))                    \
            {                                                         \
                ((uint8_t *)dst)[0] = *((uint8_t *)(&base[off]));     \
                ((uint8_t *)dst)[1] = *((uint8_t *)(&base[off + 1])); \
                ((uint8_t *)dst)[2] = *((uint8_t *)(&base[off + 2])); \
                ((uint8_t *)dst)[3] = *((uint8_t *)(&base[off + 3])); \
            }                                                         \
            else                                                      \
            {                                                         \
                *((uint32_t *)dst) = *((uint32_t *)(&base[off]));     \
            }                                                         \
            break;                                                    \
        default:                                                      \
        {                                                             \
            printf("invaild fetch size %d\n", len);                   \
            return EXC_LOAD_ACCESS;                                   \
        }                                                             \
        }                                                             \
    }

#define store_m(base_p, off, len, val)                                \
    {                                                                 \
        uint8_t *base = (uint8_t *)base_p;                            \
        switch (len)                                                  \
        {                                                             \
        case 1:                                                       \
            *((uint8_t *)(&base[off])) = ((uint8_t *)val)[0];         \
            break;                                                    \
        case 2:                                                       \
            if (unlikely(chk_not_aligned(off, 2)))                    \
            {                                                         \
                *((uint8_t *)(&base[off])) = ((uint8_t *)val)[0];     \
                *((uint8_t *)(&base[off + 1])) = ((uint8_t *)val)[1]; \
            }                                                         \
            else                                                      \
            {                                                         \
                *((uint16_t *)(&base[off])) = *((uint16_t *)val);     \
            }                                                         \
            break;                                                    \
        case 4:                                                       \
            if (unlikely(chk_not_aligned(off, 4)))                    \
            {                                                         \
                *((uint8_t *)(&base[off])) = ((uint8_t *)val)[0];     \
                *((uint8_t *)(&base[off + 1])) = ((uint8_t *)val)[1]; \
                *((uint8_t *)(&base[off + 2])) = ((uint8_t *)val)[2]; \
                *((uint8_t *)(&base[off + 3])) = ((uint8_t *)val)[3]; \
            }                                                         \
            else                                                      \
            {                                                         \
                *((uint32_t *)(&base[off])) = *((uint32_t *)val);     \
            }                                                         \
            break;                                                    \
        default:                                                      \
        {                                                             \
            printf("invaild store size %d\n", len);                   \
            return EXC_STORE_ACCESS;                                  \
        }                                                             \
        }                                                             \
    }
