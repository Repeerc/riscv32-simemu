
/*  A simple minimal riscv32imac virtual machine.
    Copyright (C) <2024>  <Repeerc>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include "simemu.h"

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include <stdatomic.h>
#if (MULTI_THREAD)
#include <pthread.h>
#endif

#if __SIZEOF_POINTER__ == 4
typedef uint32_t uptr_t;
typedef void *ptr_t;
#else
typedef uint64_t uptr_t;
typedef void *ptr_t;
#endif

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
int _kbhit(void)
{
    int ch;
    ch = getchar();
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}
#define _getch getchar
#define __max(a, b) (((a) > (b)) ? (a) : (b))
#define __min(a, b) (((a) < (b)) ? (a) : (b))
#else
#include <conio.h>
#include <windows.h>
// int __cdecl __MINGW_NOTHROW usleep(useconds_t usec)
// {
//     HANDLE timer;
//     LARGE_INTEGER interval;
//     interval.QuadPart = -(10 * usec);

//     timer = CreateWaitableTimer(NULL, TRUE, NULL);
//     SetWaitableTimer(timer, &interval, 0, NULL, NULL, 0);
//     WaitForSingleObject(timer, INFINITE);
//     CloseHandle(timer);
//     return 0;
// }
#endif

// clang-format off
#define OPCODE_MASK     0b1111111
#define OPCODE_ALU_REG  0b0110011
#define OPCODE_ALU_IMM  0b0010011
#define OPCODE_MM_LOAD  0b0000011
#define OPCODE_MM_STORE 0b0100011
#define OPCODE_BRANCH   0b1100011
#define OPCODE_JAL      0b1101111
#define OPCODE_JALR     0b1100111
#define OPCODE_LUI      0b0110111
#define OPCODE_AUIPC    0b0010111
#define OPCODE_AMO      0b0101111
#define OPCODE_FENCE    0b0001111
#define OPCODE_ZICSR    0b1110011

#define CSR_MSTATUS        0x300
#define CSR_MISA           0x301
#define CSR_MEDELEG        0x302
#define CSR_MIDELEG        0x303
#define CSR_MIE            0x304
#define CSR_MTVEC          0x305
#define CSR_MSTATUSH       0x310
#define CSR_MENVCFG        0x30a
#define CSR_MENVCFGH       0x31a
#define CSR_MSCRATCH       0x340
#define CSR_MEPC           0x341
#define CSR_MCAUSE         0x342
#define CSR_MTVAL          0x343
#define CSR_MIP            0x344
#define CSR_PMPCFG0        0x3a0
#define CSR_PMPADDR0       0x3b0
#define CSR_MVENDORID      0xf11
#define CSR_MARCHID        0xf12
#define CSR_MIMPID         0xf13
#define CSR_MHARTID        0xf14

// Supervisor MODE CSR
#define CSR_SSTATUS         0x100
#define CSR_SIE             0x104
#define CSR_STVEC           0x105
#define CSR_SCOUNTEREN      0x106
#define CSR_SSCRATCH        0x140
#define CSR_SEPC            0x141
#define CSR_SCAUSE          0x142
#define CSR_STVAL           0x143
#define CSR_SIP             0x144
//SSTC
#define CSR_STIMECMP        0x14D
#define CSR_STIMECMPH       0x15D
#define CSR_SATP            0x180

//U Mode CSR

#define CSR_TIME			0xc01
#define CSR_TIMEH			0xc81

enum csr_index
{
    IDX_CSR_MSTATUS,
    IDX_CSR_MSTATUSH,
    IDX_CSR_MISA,
    IDX_CSR_MIDELEG,
    IDX_CSR_MEDELEG,
    IDX_CSR_MIE,
    IDX_CSR_MTVEC,
    IDX_CSR_MENVCFG,
    IDX_CSR_MENVCFGH,
    IDX_CSR_MSCRATCH,
    IDX_CSR_MEPC,
    IDX_CSR_MCAUSE,
    IDX_CSR_MTVAL,
    IDX_CSR_MIP,
    IDX_CSR_PMPCFG0,
    IDX_CSR_PMPADDR0,
    IDX_CSR_MVENDORID,
    IDX_CSR_MARCHID,
    IDX_CSR_MIMPID,
    IDX_CSR_MHARTID,

    IDX_CSR_SSTATUS,
    IDX_CSR_SIE,
    IDX_CSR_STVEC,
    IDX_CSR_SSCRATCH,
    IDX_CSR_SEPC,
    IDX_CSR_SCAUSE,
    IDX_CSR_STVAL,
    IDX_CSR_SIP,
    IDX_CSR_SATP,

    IDX_CSR_NUMS,
};


// Status register flags
#define SR_SIE       0x00000002 // Supervisor Interrupt Enable
#define SR_MIE       0x00000008 // Machine Interrupt Enable
#define SR_SPIE      0x00000020 // Previous Supervisor IE
#define SR_MPIE      0x00000080 // Previous Machine IE
#define SR_SPP       0x00000100 // Previously Supervisor
#define SR_MPP       0x00001800 // Previously Machine
#define SR_SUM       0x00040000 // Supervisor User Memory Access
#define SR_MPRV      0x00020000

#define SR_SIE_BIT_SFT        1
#define SR_MIE_BIT_SFT        3
#define SR_SPIE_BIT_SFT       5
#define SR_MPIE_BIT_SFT       7
#define SR_SPP_BIT_SFT        8
#define SR_MPP_BIT_SFT        11
#define SR_SUM_BIT_SFT        18

#define IRQ_S_SOFT        1
#define IRQ_VS_SOFT       2
#define IRQ_M_SOFT        3
#define IRQ_S_TIMER       5
#define IRQ_VS_TIMER      6
#define IRQ_M_TIMER       7
#define IRQ_S_EXT         9
#define IRQ_VS_EXT        10
#define IRQ_M_EXT         11
#define IRQ_S_GEXT        12

#define MIP_SSIP     (1 << IRQ_S_SOFT)
#define MIP_HSIP     (1 << IRQ_H_SOFT)
#define MIP_MSIP     (1 << IRQ_M_SOFT)
#define MIP_STIP     (1 << IRQ_S_TIMER)
#define MIP_HTIP     (1 << IRQ_H_TIMER)
#define MIP_MTIP     (1 << IRQ_M_TIMER)
#define MIP_SEIP     (1 << IRQ_S_EXT)
#define MIP_HEIP     (1 << IRQ_H_EXT)
#define MIP_MEIP     (1 << IRQ_M_EXT)

#define M_MODE  0b11
#define S_MODE  0b01
#define U_MODE  0b00

typedef struct pte_t
{
    unsigned int V:1;
    unsigned int R:1;
    unsigned int W:1;
    unsigned int X:1;
    unsigned int U:1;
    unsigned int G:1;
    unsigned int A:1;
    unsigned int D:1;
    unsigned int RSW:2;
    unsigned int PPN_0:10;
    unsigned int PPN_1:12;
}pte_t;


enum reg_abi_name
{
    ZERO,
    RA,SP,GP, 
    TP,T0,T1,T2,
    S0 = 8,FP = 8,S1,
    A0,A1,A2,A3,A4,A5,A6,A7,
    S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,
    T3,T4,T5,T6
};

char *reg_abi_str[] = 
{
    "ZERO","RA","SP","GP","TP",
    "T0","T1","T2","FP","S1",
    "A0","A1","A2","A3","A4","A5","A6","A7",
    "S2","S3","S4","S5","S6","S7","S8","S9","S10","S11",
    "T3","T4","T5","T6"
};


#define COPY_MSTATUS_TO_SSTATUS \
    cpu->CSR[IDX_CSR_SSTATUS] &= ~(SR_SIE); \
    cpu->CSR[IDX_CSR_SSTATUS] |= cpu->CSR[IDX_CSR_MSTATUS] & SR_SIE; \
    cpu->CSR[IDX_CSR_SSTATUS] &= ~(SR_SPIE); \
    cpu->CSR[IDX_CSR_SSTATUS] |= cpu->CSR[IDX_CSR_MSTATUS] & SR_SPIE; \
    cpu->CSR[IDX_CSR_SSTATUS] &= ~(SR_SPP); \
    cpu->CSR[IDX_CSR_SSTATUS] |= cpu->CSR[IDX_CSR_MSTATUS] & SR_SPP; \
    cpu->CSR[IDX_CSR_SSTATUS] &= ~(SR_SUM); \
    cpu->CSR[IDX_CSR_SSTATUS] |= cpu->CSR[IDX_CSR_MSTATUS] & SR_SUM; 

#define COPY_SSTATUS_TO_MSTATUS \
    cpu->CSR[IDX_CSR_MSTATUS] &= ~(SR_SIE); \
    cpu->CSR[IDX_CSR_MSTATUS] |= cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE; \
    cpu->CSR[IDX_CSR_MSTATUS] &= ~(SR_SPIE); \
    cpu->CSR[IDX_CSR_MSTATUS] |= cpu->CSR[IDX_CSR_SSTATUS] & SR_SPIE; \
    cpu->CSR[IDX_CSR_MSTATUS] &= ~(SR_SPP); \
    cpu->CSR[IDX_CSR_MSTATUS] |= cpu->CSR[IDX_CSR_SSTATUS] & SR_SPP; \
    cpu->CSR[IDX_CSR_MSTATUS] &= ~(SR_SUM); \
    cpu->CSR[IDX_CSR_MSTATUS] |= cpu->CSR[IDX_CSR_SSTATUS] & SR_SUM;

#define COPY_MIx_TO_SIx(x)             \
cpu->CSR[IDX_CSR_SI##x] &= ~MIP_SSIP; \
cpu->CSR[IDX_CSR_SI##x] |= (cpu->CSR[IDX_CSR_MI##x] & MIP_SSIP); \
cpu->CSR[IDX_CSR_SI##x] &= ~MIP_STIP; \
cpu->CSR[IDX_CSR_SI##x] |= (cpu->CSR[IDX_CSR_MI##x] & MIP_STIP); \
cpu->CSR[IDX_CSR_SI##x] &= ~MIP_SEIP; \
cpu->CSR[IDX_CSR_SI##x] |= (cpu->CSR[IDX_CSR_MI##x] & MIP_SEIP); 

#define COPY_SIx_TO_MIx(x) \
cpu->CSR[IDX_CSR_MI##x] &= ~MIP_SSIP; \
cpu->CSR[IDX_CSR_MI##x] |= (cpu->CSR[IDX_CSR_SI##x] & MIP_SSIP); \
cpu->CSR[IDX_CSR_MI##x] &= ~MIP_STIP; \
cpu->CSR[IDX_CSR_MI##x] |= (cpu->CSR[IDX_CSR_SI##x] & MIP_STIP); \
cpu->CSR[IDX_CSR_MI##x] &= ~MIP_SEIP; \
cpu->CSR[IDX_CSR_MI##x] |= (cpu->CSR[IDX_CSR_SI##x] & MIP_SEIP);


#include "virtio.h"

#define UART_IER            1       // Out: Interrupt Enable Register
#define UART_IER_THRI       0x02    // Enable Transmitter holding register int.
#define UART_IER_RDI        0x01    // Enable receiver data interrupt

#define UART_IIR            2       // In:  Interrupt ID Register
#define UART_IIR_NO_INT     0x01    // No interrupts pending
#define UART_IIR_THRI       0x02    // Transmitter holding register empty
#define UART_IIR_RDI        0x04    // Receiver data interrupt

#define UART_LSR            5       // In:  Line Status Register
#define UART_LSR_TEMT       0x40    // Transmitter empty
#define UART_LSR_THRE       0x20    // Transmit-hold-register empty
#define UART_LSR_DR         0x01    // Receiver data ready

uint8_t UART8250_IER = 0;   // Interrupt Enable
uint8_t UART8250_IIR = UART_IIR_NO_INT;   // Interrupt identification
uint8_t UART8250_LSR = (UART_LSR_TEMT | UART_LSR_THRE);

int uart_in_buf_wr_p = 0;
int uart_in_buf_rd_p = 0;
char uart_in_buf[64];
char uart_out_buf = 0;

// int mem_lock = 0;

// clang-format on
typedef struct inst_decode_t
{
    uint32_t imm;
    uint8_t rs1, rs2, rd, funct3, funct7;
} inst_decode_t;

typedef struct permit_bit_t
{
    uint8_t R : 1;
    uint8_t W : 1;
    uint8_t X : 1;
    uint8_t U : 1;
} permit_bit_t;

#if ENABLE_TLB

typedef struct tlb_t
{
    // pte_t *pte;
    uint32_t vpn;
    uint32_t ppn;
    uint16_t asid;
    uint8_t valid;
    permit_bit_t permit;
} tlb_t;

#define TLB_ADDR_ITEMS (1 << 7)
#define TLB_ADDR_ITEM_MASK (TLB_ADDR_ITEMS - 1)

#define TLB_ASID_ITEMS (1 << 6)
#define TLB_ASID_ITEM_MASK (TLB_ASID_ITEMS - 1)

#define TLB_ADDR_HASH(x) (((x) >> 12) & TLB_ADDR_ITEM_MASK)
#define TLB_ASID_HASH(x) ((x) & TLB_ASID_ITEM_MASK)

#endif

#define ICACHE_ITEMS (1 << 13)
#define ICACHE_ITEM_MASK (ICACHE_ITEMS - 1)
#define ICACHE_HASH(x) (((x) >> 2) & ICACHE_ITEM_MASK)

#define DCACHE_ITEMS (1 << 13)
#define DCACHE_ITEM_MASK (DCACHE_ITEMS - 1)
#define DCACHE_HASH(x) ((x) & DCACHE_ITEM_MASK)

typedef struct cache_t
{
    uint32_t vaddr;
    uint32_t val;
    uint32_t valid;
} cache_t;

typedef struct cpu_core_t
{
    uint32_t REGS[32];
    uint32_t PC;
    uint32_t CSR[32];
    uint64_t CLINT_TIMER_CMP;
    uint64_t STIMER_CMP;
    uint8_t CLINT_IPI;
    uint8_t MODE;

    uint32_t inst;
    uint32_t inst_fetch_size;
    inst_decode_t cont;
    uint32_t opcode;
    uint32_t id;

    uint64_t cycles;

    cache_t icache[ICACHE_ITEMS];
    cache_t dcache[DCACHE_ITEMS];
#if ENABLE_TLB
    uint32_t last_access_vpn;
    uint32_t last_access_ppn;
    tlb_t tlbs[TLB_ASID_ITEMS][TLB_ADDR_ITEMS];

#endif

#if COLLECT_PERF_STATUS
    uint64_t tlb_hit;
    uint64_t tlb_miss;
    uint64_t icache_hit;
    uint64_t icache_miss;
    uint64_t lock_wait_st;
    uint64_t lock_wait_amo_st;
#endif

    uint64_t debug_cycs;
    uint64_t debug_t0;

    uint16_t SATP_asid;

    int wfi;
    int running;
    int chk_irq;

    uint32_t step_init;
    uint64_t loop_time;

    uint32_t exti_context_enable[NR_IRQ_CONTEXT_PER_CPU]; // MEXTI, SEXTI, irq0~31
    uint32_t exti_context_piorid[NR_IRQ_CONTEXT_PER_CPU];
    uint32_t exti_context_pending_irq[NR_IRQ_CONTEXT_PER_CPU];

} cpu_core_t;

extirq_def_t extirq_slot[NR_IRQ];
cpu_core_t cpu_core[NR_CPU];

uint8_t *main_mem;
uint8_t sbi_fw_mem[1048576];
uint8_t fdt_fw_mem[1048576];
uint8_t *framebuffer;
volatile uint32_t load_resv_addr[NR_CPU];

volatile atomic_int LR_lock = 0;

void plic_check_interrupt();
volatile uint64_t usec_time_start = 0;

uint64_t now_microsecond_timestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000LL + tv.tv_usec;
}

uint64_t get_microsecond_timestamp()
{
    // struct timeval tv;
    // gettimeofday(&tv, NULL);
    // return tv.tv_sec * 1000000LL + tv.tv_usec;
    return now_microsecond_timestamp() - usec_time_start;
}

void debug_dump_regs(cpu_core_t *cpu)
{
    for (int i = 0; i < 32; i += 2)
    {
        printf("x%d[%s]:%08X     x%d[%s]:%08X \n",
               i, reg_abi_str[i], cpu->REGS[i],
               i + 1, reg_abi_str[i + 1], cpu->REGS[i + 1]);
    }
    printf("PC=%08x,  %lld\n", cpu->PC, cpu->cycles);
    printf("MODE=%08x\n", cpu->MODE);
    printf("Running,wfi,stall=%d,%d,%d\n", cpu->running, cpu->wfi, 0);
    printf("MIE,SIE=%d,%d\n", cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE, cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE);
    printf("MIE,MIP=%08x,%08x\n", cpu->CSR[IDX_CSR_MIE], cpu->CSR[IDX_CSR_MIP]);
    printf("SIE,SIP=%08x,%08x\n", cpu->CSR[IDX_CSR_SIE], cpu->CSR[IDX_CSR_SIP]);
}

void uart_send_char(char c)
{
    uart_in_buf[uart_in_buf_wr_p] = c;
    uart_in_buf_wr_p++;
    if (uart_in_buf_wr_p >= sizeof(uart_in_buf))
        uart_in_buf_wr_p = 0;

    // if (uart_in_buf_rd_p != uart_in_buf_wr_p)
    {
        if (!(extirq_slot[IRQ_NUM_UART].pending))
            if (UART8250_IER & UART_IER_RDI)
            {
                UART8250_IIR |= UART_IIR_RDI;
                UART8250_IIR &= ~UART_IIR_NO_INT;
                extirq_slot[IRQ_NUM_UART].pending = 1;
            }
        // UART8250_LSR |= UART_LSR_DR;
    }
}

int mem_access_dispatch(cpu_core_t *cpu, uint32_t addr, uint32_t len, void *dat, uint32_t rd)
{

    switch (addr & 0xF0000000)
    {
    case 0:
    {

        if ((addr & 0x0F000000) == SBI_BASE)
        {
            uint32_t off = addr - SBI_BASE;
            if (off <= sizeof(sbi_fw_mem))
            {
                if (rd & READ)
                    fetch_m(sbi_fw_mem, off, len, dat) else store_m(sbi_fw_mem, off, len, dat)

                        return EXC_OK;
            }
        }
        else if ((addr & 0x0F000000) == DTB_BASE)
        {
            uint32_t off = addr - DTB_BASE;
            if (off <= sizeof(fdt_fw_mem))
            {
                fetch_m(fdt_fw_mem, off, len, dat);
                return EXC_OK;
            }
        }
    }
    break;
    case (MEM_BASE4):
    case (MEM_BASE3):
    case (MEM_BASE2):
    case (MEM_BASE):
    {
        uint32_t off = addr - MEM_BASE;

        if (likely(off < MEM_SIZE))
        {
            // while (mem_lock)
            //     ;
            if (rd & (WRITE))
            {
                store_m(main_mem, off, len, dat);
            }
            else
            {
                fetch_m(main_mem, off, len, dat);
            }

            return EXC_OK;
        }
        break;
    }

    case VIRTIO_BASE1:
    {
        uint32_t off = addr - VIRTIO_BASE1;
        return virtio_input_mmio_access(off, len, rd, dat);
    }

        static atomic_bool ioblk_lk = 0;

    case VIRTIO_BLK_BASE:
    {
        uint32_t off = addr - VIRTIO_BLK_BASE;
        while (atomic_flag_test_and_set(&ioblk_lk))
            ;
        int ret = virtio_blk_mmio_access(off, len, rd, dat);

        atomic_flag_clear(&ioblk_lk);
        // plic_check_interrupt(cpu->id);
        cpu->chk_irq = 1;
        return ret;
    }

    case (VRAM_BASE):
    {
        uint32_t off = addr - VRAM_BASE;
        if (off < VRAM_SIZE)
        {
            if (rd & WRITE)
            {
                store_m(framebuffer, off, len, dat);
            }
            else
            {
                fetch_m(framebuffer, off, len, dat);
            }
            return EXC_OK;
        }
    }

    break;

    case (UART8250_BASE):
    {
        uint32_t off = addr & 0xFF;
        uint32_t val = 0;
        switch (off)
        {
        case 0:
            if (rd & READ)
            {
                // if (_kbhit())
                {
                    // val = _getch();
                    // printf("%d",cpu->id);
                    val = uart_in_buf[uart_in_buf_rd_p];
                    uart_in_buf_rd_p++;
                    if (uart_in_buf_rd_p >= sizeof(uart_in_buf))
                        uart_in_buf_rd_p = 0;
                    ((uint8_t *)dat)[0] = val;
                }
            }
            else
            {
                uart_out_buf = *((char *)dat);
                putchar(*((char *)dat));
            }
            return EXC_OK;
        case UART_IER:
            if (rd & READ)
                *((char *)dat) = UART8250_IER;
            else
                UART8250_IER = *((char *)dat);
            return EXC_OK;

        case UART_IIR:
            if (rd & READ)
                *((char *)dat) = UART8250_IIR;
            // else
            //     UART8250_IIR = *((char *)dat);
            return EXC_OK;

        case UART_LSR:
            if (rd & READ)
            {
                val = UART_LSR_TEMT | UART_LSR_THRE | (uart_in_buf_rd_p != uart_in_buf_wr_p);
                ((uint8_t *)dat)[0] = val;
            }
            return EXC_OK;
        default:
            if (rd & READ)
                *((char *)dat) = 0;
            return EXC_OK;
        }
    }
    break;
    case (PLIC_BASE):
    {
        uint32_t off = addr - PLIC_BASE;
        if (unlikely(len != 4))
            return rd ? EXC_LOAD_MISALIGNED : EXC_STORE_MISALIGNED;
        if (off >= PLIC_CONTEXT_BASE)
        {
            uint32_t context_idx = ((off - PLIC_CONTEXT_BASE) / PLIC_CONTEXT_STRIDE) % NR_IRQ_CONTEXT_PER_CPU;
            uint32_t priority_threshold = ((off - PLIC_CONTEXT_BASE) % PLIC_CONTEXT_STRIDE == 0);
            uint32_t claim_cmplt = ((off - PLIC_CONTEXT_BASE) % PLIC_CONTEXT_STRIDE == 4);
            uint32_t cpu_idx = ((off - PLIC_CONTEXT_BASE) / PLIC_CONTEXT_STRIDE) / NR_IRQ_CONTEXT_PER_CPU;

            if ((cpu_idx < NR_CPU) && (context_idx < NR_IRQ_CONTEXT_PER_CPU))
            {
                if (rd & READ)
                {
                    if (priority_threshold)
                        *((uint32_t *)dat) = cpu_core[cpu_idx].exti_context_piorid[context_idx];
                    else if (claim_cmplt)
                    {
                        *((uint32_t *)dat) = cpu_core[cpu_idx].exti_context_pending_irq[context_idx];
                        if (extirq_slot[*((uint32_t *)dat)].pending)
                        {
                            extirq_slot[*((uint32_t *)dat)].pending--;
                        }
                    }
                }
                else
                {
                    if (priority_threshold)
                        cpu_core[cpu_idx].exti_context_piorid[context_idx] = *((uint32_t *)dat);
                    if (claim_cmplt)
                    {
                        if (*((uint32_t *)dat) < NR_IRQ)
                        {
                            // if (extirq_slot[*((uint32_t *)dat)].pending)
                            // {
                            //     extirq_slot[*((uint32_t *)dat)].pending--;
                            if (extirq_slot[*((uint32_t *)dat)].do_cmplt)
                            {
                                extirq_slot[*((uint32_t *)dat)].do_cmplt();
                            }
                            //     // printf("irq:%d,rem:%d\n",*((uint32_t *)dat),extirq_slot[*((uint32_t *)dat)].pending  );
                            // }
                        }
                        cpu_core[cpu_idx].exti_context_pending_irq[context_idx] = 0;
                        if (context_idx == 0)
                            cpu_core[cpu_idx].CSR[IDX_CSR_MIP] &= ~MIP_MEIP;
                        else if (context_idx == 1)
                            cpu_core[cpu_idx].CSR[IDX_CSR_SIP] &= ~MIP_SEIP;
                    }
                }
            }
            return EXC_OK;
        }
        else if (off >= PLIC_ENABLE_BASE)
        {
            uint32_t context_idx = ((off - PLIC_ENABLE_BASE) / PLIC_ENABLE_STRIDE) % NR_IRQ_CONTEXT_PER_CPU;
            uint32_t irqbit_idx = (off - PLIC_ENABLE_BASE) % PLIC_ENABLE_STRIDE;
            uint32_t cpu_idx = ((off - PLIC_ENABLE_BASE) / PLIC_ENABLE_STRIDE) / NR_IRQ_CONTEXT_PER_CPU;
            if ((cpu_idx < NR_CPU) && (context_idx < NR_IRQ_CONTEXT_PER_CPU) && (irqbit_idx == 0))
            {
                if (rd & READ)
                    *((uint32_t *)dat) = cpu_core[cpu_idx].exti_context_enable[context_idx];

                else
                {
                    cpu_core[cpu_idx].exti_context_enable[context_idx] = *((uint32_t *)dat);
                    // printf("set plic enable:ctx=%d,cpu:%d, %d, v=%08x\n", context_idx, cpu_idx, irqbit_idx, *((uint32_t *)dat));
                }
            }
            return EXC_OK;
        }
        else if (off >= PLIC_PENDING_BASE)
        {
            uint32_t pending_bits = 0;
            if (off == PLIC_PENDING_BASE)
            {
                for (int i = 1; i < NR_IRQ; i++)
                {
                    if (extirq_slot[i].pending)
                        pending_bits |= (1 << i);
                }
                if (rd & READ)
                    *((uint32_t *)dat) = pending_bits;

                return EXC_OK;
            }
        }
        else if (off >= PLIC_PRIORITY_BASE)
        {
            uint32_t prior_idx = (off - PLIC_PRIORITY_BASE) / 4;
            if (prior_idx < NR_IRQ)
            {
                if (rd & READ)
                    *((uint32_t *)dat) = extirq_slot[prior_idx].priority;
                else
                    extirq_slot[prior_idx].priority = *((uint32_t *)dat);
                // printf("set plic PRIORITY:%d, %d\n", prior_idx, *((uint32_t *)dat));
                return EXC_OK;
            }
        }
    }
    break;
    case (CLINT_BASE):
    {
        uint32_t off = addr & 0xFFFF;
        if (unlikely(len != 4))
            return rd ? EXC_LOAD_MISALIGNED : EXC_STORE_MISALIGNED;

        if ((off == CLINT_TIMER_VAL_OFF) && rd)
        {
            *((uint32_t *)dat) = get_microsecond_timestamp();
            return EXC_OK;
        }
        else if ((off == CLINT_TIMER_VALH_OFF) && rd)
        {
            *((uint32_t *)dat) = (uint64_t)get_microsecond_timestamp() >> 32;
            return EXC_OK;
        }

        switch (off & 0xF000)
        {
        case 0x0000: // IPI
        {
            uint32_t idx = off / 4;
            if (likely(idx < NR_CPU))
            {
                if (rd & READ)
                {
                    *((uint32_t *)dat) = 0; // cpu_core[idx].CLINT_IPI;
                }
                else
                {
                    cpu_core[idx].CLINT_IPI = *((uint32_t *)dat);
                    if (cpu_core[idx].CLINT_IPI)
                    {
                        cpu_core[idx].CSR[IDX_CSR_MIP] |= MIP_MSIP;
                        cpu_core[idx].chk_irq = 1;
                    }
                    else
                        cpu_core[idx].CSR[IDX_CSR_MIP] &= ~MIP_MSIP;
                }
            }
            return EXC_OK;
        }
        break;
        case 0x4000: // MTIMECMP
        {
            uint32_t idx = (off - 0x4000) / 8;
            uint32_t HADDR = ((off - 0x4000) / 4) % 2;
            if (likely(idx < NR_CPU))
            {
                if (rd & READ)
                {
                    if (HADDR)
                        *((uint32_t *)dat) = cpu_core[idx].CLINT_TIMER_CMP >> 32;
                    else
                        *((uint32_t *)dat) = cpu_core[idx].CLINT_TIMER_CMP;
                }
                else
                {
                    if (HADDR)
                    {
                        cpu_core[idx].CLINT_TIMER_CMP &= 0x00000000FFFFFFFFULL;
                        cpu_core[idx].CLINT_TIMER_CMP |= (((uint64_t)(*((uint32_t *)dat))) << 32);
                    }
                    else
                    {
                        cpu_core[idx].CLINT_TIMER_CMP &= 0xFFFFFFFF00000000ULL;
                        cpu_core[idx].CLINT_TIMER_CMP |= *((uint32_t *)dat);
                        // printf("set tmrcmp:%lld\n", cpu_core[idx].CLINT_TIMER_CMP);
                    }
                    cpu_core[idx].CSR[IDX_CSR_MIP] &= ~MIP_MTIP;
                }
            }
            return EXC_OK;
        }
        break;
        default:
            break;
        }
        break;
    }

    default:
        break;
    }

    if (rd & READ)
        return EXC_LOAD_ACCESS;
    return EXC_STORE_ACCESS;

    // printf("MEM ACCESS ERROR at 0x%08x,%d, rd=%d, PC=0x%08x\n", addr, len, rd, cpu->PC);
    // running = 0;
}

void trap(cpu_core_t *cpu, int interrupt, int reason, uint32_t exception_pc, uint32_t exception_addr)
{

    int DELEG = 0;
    //  printf("CPU:%d, MIE:%d, mode:%d\n", cpu->id, cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE,cpu->MODE);
    if (interrupt)
        DELEG = ((1 << reason) & cpu->CSR[IDX_CSR_MIDELEG]);
    else
        DELEG = ((1 << reason) & cpu->CSR[IDX_CSR_MEDELEG]);
    // for (int i = 0; i < NR_CPU; i++)
    load_resv_addr[cpu->id] = 0xFFFFFFFF;
    if (DELEG)
    {
        if (interrupt && !(cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE))
            return;
        cpu->CSR[IDX_CSR_SSTATUS] &= ~SR_SPIE;
        cpu->CSR[IDX_CSR_SSTATUS] |= (cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE) ? SR_SPIE : 0;
        cpu->CSR[IDX_CSR_SSTATUS] &= ~SR_SIE;

        cpu->CSR[IDX_CSR_SCAUSE] = (interrupt << 31) | reason;
        cpu->CSR[IDX_CSR_SEPC] = exception_pc;
        cpu->CSR[IDX_CSR_STVAL] = exception_addr;

        cpu->CSR[IDX_CSR_SSTATUS] &= ~SR_SPP;
        cpu->CSR[IDX_CSR_SSTATUS] |= ((cpu->MODE & 1) << SR_SPP_BIT_SFT);

        cpu->MODE = S_MODE;
        cpu->PC = cpu->CSR[IDX_CSR_STVEC];

        // COPY_SSTATUS_TO_MSTATUS;
        // COPY_SIx_TO_MIx(P);
        // COPY_SIx_TO_MIx(E);

        if (cpu->PC & 0b10)
            printf("VTEC OFFSET!\n");
    }
    else
    {
        if (interrupt && (!(cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE)))
            return;
        cpu->CSR[IDX_CSR_MSTATUS] &= ~SR_MPIE;
        cpu->CSR[IDX_CSR_MSTATUS] |= (cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE) ? SR_MPIE : 0;
        cpu->CSR[IDX_CSR_MSTATUS] &= ~SR_MIE;

        cpu->CSR[IDX_CSR_MCAUSE] = (interrupt << 31) | reason;
        cpu->CSR[IDX_CSR_MEPC] = exception_pc;
        cpu->CSR[IDX_CSR_MTVAL] = exception_addr;

        cpu->CSR[IDX_CSR_MSTATUS] &= ~SR_MPP;
        cpu->CSR[IDX_CSR_MSTATUS] |= ((cpu->MODE & 0b11) << SR_MPP_BIT_SFT);

        cpu->MODE = M_MODE;
        cpu->PC = cpu->CSR[IDX_CSR_MTVEC];

        COPY_SSTATUS_TO_MSTATUS;
        COPY_SIx_TO_MIx(P);
        COPY_SIx_TO_MIx(E);

        if (cpu->PC & 0b10)
            printf("VTEC OFFSET!\n");
    }
}

void debug_dump_mmap(cpu_core_t *cpu)
{
    if (cpu->CSR[IDX_CSR_SATP])
    {
        uint32_t pg_dir_paddr = (cpu->CSR[IDX_CSR_SATP] & 0x3FFFFF) << 12;
        printf("pg_dir_paddr:%08x\n", pg_dir_paddr);
        pte_t *l1_pte = ((pte_t *)&main_mem[pg_dir_paddr - MEM_BASE]);
        uint32_t l1_ppn = 0;
        uint32_t l2_ppn = 0;

        uint32_t addr_start = 0;
        uint32_t addr_end = 0;
        uint32_t seglen = 0;

        for (int i = 0; i < 1024; i++)
        {
            if ((uptr_t)&l1_pte[i] < (uptr_t)&main_mem[MEM_SIZE])
            {
                if (l1_pte[i].V)
                {
                    l1_ppn = (l1_pte[i].PPN_1 << 22) | (l1_pte[i].PPN_0 << 12);
                    if (l1_pte[i].R & l1_pte[i].W & l1_pte[i].X)
                    {
                        addr_start = i << 22;
                        addr_end = (i << 22) + 1048576 * 4 - 1;

                        printf("vaddr:%08x~%08x <-- 4MB --> paddr:%08x ", addr_start, addr_end, l1_ppn);
                        printf(" G:%d U:%x R:%d W:%d X:%d \n", l1_pte[i].G, l1_pte[i].U, l1_pte[i].R, l1_pte[i].W, l1_pte[i].X);
                    }
                    else
                    {
                        pte_t *l2_pte = ((pte_t *)&main_mem[l1_ppn - MEM_BASE]);
                        int track = 0;

                        for (int j = 0; j < 1024; j++)
                        {

                            if ((uptr_t)&l2_pte[j] < (uptr_t)&main_mem[MEM_SIZE])
                            {
                                if (l2_pte[j].V)
                                {
                                    l2_ppn = (l2_pte[j].PPN_1 << 22) | (l2_pte[j].PPN_0 << 12);
                                    addr_start = ((i << 22) | (j << 12));
                                    addr_end = ((i << 22) | (j << 12)) + 1024 * 4 - 1;
                                    if (track == 0)
                                    {
                                        track = 1;
                                        seglen = 0;
                                        printf("paddr: %08x <-> vaddr:%08x ", l2_ppn, addr_start);
                                    }

                                    if (track)
                                    {
                                        seglen += (addr_end - addr_start + 1);
                                        if (j == 1023)
                                        {
                                            track = 0;
                                            printf("~ %08x , %dKB", addr_end, seglen / 1024);
                                            printf(" G:%d U:%x R:%d W:%d X:%d \n",
                                                   l2_pte[i].G,
                                                   l2_pte[i].U,
                                                   l2_pte[i].R,
                                                   l2_pte[i].W,
                                                   l2_pte[i].X);
                                        }
                                    }
                                }
                                else
                                {
                                    if (track)
                                    {
                                        track = 0;
                                        printf("~ %08x , %dKB", addr_end, seglen / 1024);
                                        printf(" G:%d U:%x R:%d W:%d X:%d \n",
                                               l2_pte[i].G,
                                               l2_pte[i].U,
                                               l2_pte[i].R,
                                               l2_pte[i].W,
                                               l2_pte[i].X);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

static inline int mem_addr_translate(cpu_core_t *cpu, uint32_t vaddr, uint32_t *paddr, uint32_t rd)
{

    int do_translate = 0;
    int translate_sucess = 0;
    uint32_t l2_pte_paddr = 0;
    uint32_t l1_pte_paddr = 0;
    permit_bit_t page_permit;

    if (likely((cpu->MODE < M_MODE) && (cpu->CSR[IDX_CSR_SATP] >> 31)))
        do_translate = 1;
    else if ((cpu->MODE == M_MODE) && (cpu->CSR[IDX_CSR_MSTATUS] & SR_MPRV) && (cpu->CSR[IDX_CSR_SATP] >> 31))
        do_translate = 1;

    if (!do_translate)
    {
        *paddr = vaddr;
        return EXC_OK;
    }

#if ENABLE_TLB
    // int supperpage = 0;

    if ((vaddr >> 12) == (cpu->last_access_vpn))
    {
        *paddr = (cpu->last_access_ppn << 12) | (vaddr & 0xFFF);
        return EXC_OK;
    }

    uint32_t tlb_idx = TLB_ADDR_HASH(vaddr);
    uint16_t asid_idx = TLB_ASID_HASH(cpu->SATP_asid);
    if (((cpu->tlbs[asid_idx][tlb_idx].valid) &&
         ((cpu->tlbs[asid_idx][tlb_idx].vpn) == (vaddr >> 12)) &&
         (cpu->tlbs[asid_idx][tlb_idx].asid == cpu->SATP_asid)))
    {
        page_permit = cpu->tlbs[asid_idx][tlb_idx].permit;
        // clang-format off
        if((rd & READ ) && !page_permit.R) {cpu->tlbs[asid_idx][tlb_idx].valid = 0;goto tr_fin;}
        else if((rd & EXEC ) && !page_permit.X) {cpu->tlbs[asid_idx][tlb_idx].valid = 0;goto tr_fin;}
        else if((rd & WRITE) && !page_permit.W) {cpu->tlbs[asid_idx][tlb_idx].valid = 0;goto tr_fin;}
        if((cpu->MODE == U_MODE) && !page_permit.U) {cpu->tlbs[asid_idx][tlb_idx].valid = 0; goto tr_fin;}
        // clang-format on

#if COLLECT_PERF_STATUS
        cpu->tlb_hit++;
#endif
        *paddr = (cpu->tlbs[asid_idx][tlb_idx].ppn << 12) | (vaddr & 0xFFF);
        translate_sucess = 1;
        cpu->last_access_vpn = vaddr >> 12;
        cpu->last_access_ppn = *paddr >> 12;
        // if (rd & (READ | EXEC))
        //     cpu->tlbs[asid_idx][tlb_idx].pte->A = 1;
        // if (rd & (WRITE))
        //     cpu->tlbs[asid_idx][tlb_idx].pte->D = 1;
        return EXC_OK;
    }
    else
#endif
    {
#if COLLECT_PERF_STATUS
        cpu->tlb_miss++;
#endif
        // printf("VADDR:%08x\n", vaddr);
        uint32_t pg_dir_paddr = (cpu->CSR[IDX_CSR_SATP] & 0x3FFFFF) << 12;
        uint32_t vpn_1 = (vaddr >> 22) & 0x3FF;
        uint32_t vpn_0 = (vaddr >> 12) & 0x3FF;
        uint32_t offset = vaddr & 0xFFF;
        l1_pte_paddr = pg_dir_paddr | (vpn_1 << 2);
        pte_t l1_pte;
        pte_t l2_pte;
        if ((l1_pte_paddr >= MEM_BASE) && (l1_pte_paddr < MEM_BASE + MEM_SIZE))
        {
            l1_pte = *((pte_t *)(&main_mem[l1_pte_paddr - MEM_BASE]));
            if (!l1_pte.V)
                goto tr_fin;
            if (l1_pte.R || l1_pte.W || l1_pte.X)
            {
                if (l1_pte.PPN_0 != 0)
                    goto tr_fin;

                page_permit.R = l1_pte.R;
                page_permit.W = l1_pte.W;
                page_permit.X = l1_pte.X;
                page_permit.U = l1_pte.U;

                // clang-format off
                if((rd & READ ) && !page_permit.R) goto tr_fin;
                else if((rd & EXEC ) && !page_permit.X) goto tr_fin;
                else if((rd & WRITE) && !page_permit.W) goto tr_fin;
                if((cpu->MODE == U_MODE) && !page_permit.U) goto tr_fin;
                // clang-format on

                *paddr = (l1_pte.PPN_1 << 22) | (vpn_0 << 12) | offset;
                translate_sucess = 1;
                // supperpage = 1;
                if (rd & (READ | EXEC))
                    l1_pte.A = 1;
                if (rd & (WRITE))
                    l1_pte.D = 1;
            }
            else
            {
                l2_pte_paddr = (l1_pte.PPN_1 << 22) | (l1_pte.PPN_0 << 12);
                l2_pte_paddr += vpn_0 * 4;
                if ((l2_pte_paddr >= MEM_BASE) && (l2_pte_paddr < MEM_BASE + MEM_SIZE))
                {
                    l2_pte = *((pte_t *)&main_mem[l2_pte_paddr - MEM_BASE]);
                    if (!l2_pte.V)
                        goto tr_fin;

                    page_permit.R = l2_pte.R;
                    page_permit.W = l2_pte.W;
                    page_permit.X = l2_pte.X;
                    page_permit.U = l2_pte.U;

                    // clang-format off
                        if((rd & READ ) && !page_permit.R) goto tr_fin;
                        else if((rd & EXEC ) && !page_permit.X) goto tr_fin;
                        else if((rd & WRITE) && !page_permit.W) goto tr_fin;
                        if((cpu->MODE == U_MODE) && !page_permit.U) goto tr_fin;
                    // clang-format on

                    *paddr = (l2_pte.PPN_1 << 22) | (l2_pte.PPN_0 << 12) | offset;
                    translate_sucess = 1;
                    if (rd & (READ | EXEC))
                        l2_pte.A = 1;
                    if (rd & (WRITE))
                        l2_pte.D = 1;
                }
            }
        }
    }

tr_fin:
    if (translate_sucess)
    {
#if ENABLE_TLB
        cpu->tlbs[asid_idx][tlb_idx].vpn = vaddr >> 12;
        cpu->tlbs[asid_idx][tlb_idx].ppn = *paddr >> 12;
        cpu->tlbs[asid_idx][tlb_idx].asid = cpu->SATP_asid;
        cpu->tlbs[asid_idx][tlb_idx].valid = 1;
        cpu->tlbs[asid_idx][tlb_idx].permit = page_permit;
        // if (supperpage)
        //     cpu->tlbs[asid_idx][tlb_idx].pte = ((pte_t *)(&main_mem[l1_pte_paddr - MEM_BASE]));
        // else
        //     cpu->tlbs[asid_idx][tlb_idx].pte = ((pte_t *)(&main_mem[l2_pte_paddr - MEM_BASE]));

        cpu->last_access_vpn = vaddr >> 12;
        cpu->last_access_ppn = *paddr >> 12;
#endif
        return EXC_OK;
    }

#if ENABLE_TLB
    cpu->last_access_vpn = 0;
#endif
    if (rd & READ)
        return EXC_LOAD_PAGE_FAULT;
    if (rd & EXEC)
        return EXC_INST_PAGE_FAULT;
    return EXC_STORE_PAGE_FAULT;
}

#if (MULTI_THREAD)
#define MEMLK_CHUNK_SZ (4096 / NR_CPU)
// #define MEMLK_CHUNK_SZ (4096 * 4)
// #define MEMLK_CHUNK_SZ (1048576*256)
#define MEMLK_CHUNK_NR (0x100000000ULL / MEMLK_CHUNK_SZ)
#if AMO_SPIN_LOCK
volatile atomic_int amo_spin_lock_blk[MEMLK_CHUNK_NR] = {0};
#else
volatile atomic_char g_rwlock[MEMLK_CHUNK_NR] = {0};

inline void rw_lock_read_lock(cpu_core_t *cpu, volatile atomic_char *lock)
{
    while (1)
    {
        char expected = atomic_load(lock);
        if (expected >= 0 && atomic_compare_exchange_weak(lock, &expected, expected + 1))
            return;

        // usleep(rand() % 7000);
        // usleep(1000);
#if COLLECT_PERF_STATUS
        cpu->lock_wait_st++;
#endif
    }
}

inline void rw_lock_read_unlock(volatile atomic_char *lock)
{
    atomic_fetch_sub(lock, 1);
}

inline void rw_lock_write_lock(cpu_core_t *cpu, volatile atomic_char *lock)
{
    while (1)
    {
        char expected = 0;
        if (atomic_compare_exchange_weak(lock, &expected, -1))
            return;
        // usleep(rand() % 7000);
        // usleep(1000);
#if COLLECT_PERF_STATUS
        cpu->lock_wait_amo_st++;
#endif
    }
}

inline void rw_lock_write_unlock(volatile atomic_char *lock)
{
    char expected = -1;
    atomic_compare_exchange_weak(lock, &expected, 0);
    // atomic_store(lock, 0);
}

#endif

#endif

int mem_access(cpu_core_t *cpu, uint32_t addr, uint32_t len, void *dat, uint32_t rd)
{
    uint32_t paddr = 0;
    int res;
    if (((len == 2) && (addr % 2)) || ((len == 4) && (addr % 4)))
    {
#if !(UNAGLINED_MEM_ACCESS)
        // printf("misaligned\n");
        if (rd & READ)
        {
            res = EXC_LOAD_MISALIGNED;
            trap(cpu, 0, EXC_LOAD_MISALIGNED, cpu->PC - cpu->inst_fetch_size, addr);
            return res;
        }
        if (rd & WRITE)
        {
            res = EXC_STORE_MISALIGNED;
            trap(cpu, 0, EXC_STORE_MISALIGNED, cpu->PC - cpu->inst_fetch_size, addr);
            return res;
        }
        if (rd & EXEC)
        {
            res = EXC_INST_MISALIGNED;
            trap(cpu, 0, EXC_INST_MISALIGNED, cpu->PC - cpu->inst_fetch_size, addr);
            return res;
        }
#endif

        uint32_t misalign_addr[4];
        uint8_t *u8_dat = dat;
        for (int i = 0; i < len; i++)
        {
            res = mem_addr_translate(cpu, addr, &misalign_addr[i], rd);
            if (unlikely(res != EXC_OK))
                goto translate_fail;
            addr++;
        }

        for (int i = 0; i < len; i++)
        {
            res = mem_access_dispatch(cpu, misalign_addr[i], 1, &u8_dat[i], rd);
            if (unlikely(res != EXC_OK))
            {
                addr -= (len - i);
                goto access_fail;
            }
        }
    }
    else
    {
        res = mem_addr_translate(cpu, addr, &paddr, rd);
        if (unlikely(res != EXC_OK))
            goto translate_fail;

        res = mem_access_dispatch(cpu, paddr, len, dat, rd);
        if (unlikely(res != -1))
            goto access_fail;
    }

    if (rd & WRITE)
    {
        while (atomic_flag_test_and_set_explicit(&LR_lock, __ATOMIC_ACQUIRE))
            ;
        for (int i = 0; i < NR_CPU; i++)
            if (load_resv_addr[i] == (addr >> 2))
            {
                load_resv_addr[i] = 0xFFFFFFFF;
            }
        atomic_flag_clear_explicit(&LR_lock, __ATOMIC_RELEASE);
    }

    cpu->dcache[DCACHE_HASH(addr)].val = ((uint32_t *)dat)[0];
    cpu->dcache[DCACHE_HASH(addr)].vaddr = addr;
    cpu->dcache[DCACHE_HASH(addr)].valid = 1;

    return res;

access_fail:
    printf("MEM ACCESS ERROR at v:0x%08x, p:%08x,%d, rd=%d, PC=0x%08x, res=%d\n", addr, paddr, len, rd, cpu->PC, res);
    // debug_dump_regs(cpu);
    trap(cpu, 0, res, cpu->PC - cpu->inst_fetch_size, addr);
    return res;

translate_fail:
    trap(cpu, 0, res, cpu->PC - cpu->inst_fetch_size, addr);
    return res;
}

static inline int mem_inst_fetch(cpu_core_t *cpu, uint32_t addr, uint32_t len, void *dat)
{

    uint32_t paddr = 0;
    int res = EXC_OK;
    res = mem_addr_translate(cpu, addr, &paddr, EXEC);
    if (unlikely(res != EXC_OK))
    {
        cpu->icache[ICACHE_HASH(addr)].valid = 0;
        trap(cpu, 0, res, cpu->PC, cpu->PC);
        return res;
    }

#if ICACHE
    if (cpu->icache[ICACHE_HASH(addr)].valid)
    {
        if (addr == cpu->icache[ICACHE_HASH(addr)].vaddr)
        {
            if (len == 4)
                ((uint32_t *)dat)[0] = cpu->icache[ICACHE_HASH(addr)].val;
            else
                ((uint16_t *)dat)[0] = cpu->icache[ICACHE_HASH(addr)].val;

#if COLLECT_PERF_STATUS
            cpu->icache_hit++;
#endif
            return EXC_OK;
        }
    }
#endif
#if COLLECT_PERF_STATUS
    cpu->icache_miss++;
#endif

    res = mem_access_dispatch(cpu, paddr, len, dat, READ);
    if (unlikely(res != EXC_OK))
    {
        trap(cpu, 0, EXC_INST_ACCESS, cpu->PC, cpu->PC);
        return res;
    }

#if ICACHE
    cpu->icache[ICACHE_HASH(addr)].val = ((uint32_t *)dat)[0];
    cpu->icache[ICACHE_HASH(addr)].vaddr = addr;
    cpu->icache[ICACHE_HASH(addr)].valid = 1;
#endif

    return res;
}

#define CSR_RDWR(x)                                 \
    case x:                                         \
        if (rd & READ)                              \
            *((uint32_t *)dat) = cpu->CSR[IDX_##x]; \
        else                                        \
            cpu->CSR[IDX_##x] = *((uint32_t *)dat); \
        break;

int csr_access(cpu_core_t *cpu, uint32_t addr, void *dat, uint32_t rd)
{
    // if (rd&READ)
    // if ((addr != CSR_MSTATUS) && (addr != CSR_MSCRATCH))
    //     printf("acc csr:%04x, rd:%d\n", addr, rd);

    switch (addr)
    {
    case CSR_MVENDORID:
        if (rd & READ)
            *((uint32_t *)dat) = 0;
        break;
    case CSR_MARCHID:
        if (rd & READ)
            *((uint32_t *)dat) = 0;
        break;
    case CSR_MIMPID:
        if (rd & READ)
            *((uint32_t *)dat) = 0;
        break;
    case CSR_MISA:
        if (rd & READ)
        {
            uint32_t misa = 0;
            misa |= (0b01) << 30; // XLEN=32
            misa |= 1 << ('I' - 'A');
            misa |= 1 << ('M' - 'A');
            misa |= 1 << ('A' - 'A');
#if (ENABLE_RV32C)
            misa |= 1 << ('C' - 'A');
#endif
            misa |= 1 << ('S' - 'A');
            misa |= 1 << ('U' - 'A');
            *((uint32_t *)dat) = misa;
        }
        break;
    case CSR_MHARTID:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->id;
        break;

    case CSR_TIME:
        if (rd & READ)
        {
            *((uint32_t *)dat) = get_microsecond_timestamp();
        }
        break;
    case CSR_TIMEH:
        if (rd & READ)
        {
            *((uint32_t *)dat) = get_microsecond_timestamp() >> 32;
        }
        break;

        CSR_RDWR(CSR_MTVEC)
        CSR_RDWR(CSR_MSCRATCH)
        CSR_RDWR(CSR_MEDELEG)
        CSR_RDWR(CSR_MIDELEG)
        CSR_RDWR(CSR_MEPC)
        CSR_RDWR(CSR_MCAUSE)
        // CSR_RDWR(CSR_MIP)

    case CSR_MIP:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->CSR[IDX_CSR_MIP];
        else
        {
            cpu->CSR[IDX_CSR_MIP] = *((uint32_t *)dat);
            COPY_MIx_TO_SIx(P);
        }
        break;
    case CSR_MIE:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->CSR[IDX_CSR_MIE];
        else
        {
            cpu->CSR[IDX_CSR_MIE] = *((uint32_t *)dat);
            COPY_MIx_TO_SIx(E);
        }
        break;

    case CSR_MSTATUS:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->CSR[IDX_CSR_MSTATUS];
        else
        {
            cpu->CSR[IDX_CSR_MSTATUS] = *((uint32_t *)dat);
            COPY_MSTATUS_TO_SSTATUS;
            if ((cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE) || (cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE))
            {
                // cpu->wfi = 0;
                cpu->chk_irq = 1;
            }
        }
        break;
        // CSR_RDWR(CSR_MIE)
        // CSR_RDWR(CSR_MSTATUS)

        CSR_RDWR(CSR_MSTATUSH)
        CSR_RDWR(CSR_MTVAL)

        // CSR_RDWR(CSR_SSTATUS)

    case CSR_SSTATUS:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->CSR[IDX_CSR_SSTATUS];
        else
        {
            cpu->CSR[IDX_CSR_SSTATUS] = *((uint32_t *)dat);
            // if ((cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE) || (cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE))
            {
                // debug_dump_regs(cpu);
                // printf("%08x,%08x\n",cpu->CSR[IDX_CSR_SIE],cpu->CSR[IDX_CSR_SIP] );
                // if(cpu->CSR[IDX_CSR_SIE] & cpu->CSR[IDX_CSR_SIP])
                // {
                //     printf("%08x\n",cpu->CSR[IDX_CSR_SIP] );
                // }
                // cpu->wfi = 0;
                cpu->chk_irq = 1;
            }
        }
        break;

        CSR_RDWR(CSR_STVEC)
        CSR_RDWR(CSR_SSCRATCH)
        CSR_RDWR(CSR_SEPC)
        CSR_RDWR(CSR_SCAUSE)
        CSR_RDWR(CSR_STVAL)
        CSR_RDWR(CSR_SIP)
        CSR_RDWR(CSR_SIE)

    case CSR_STIMECMP:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->STIMER_CMP;
        else
        {
            cpu->STIMER_CMP &= 0xFFFFFFFF00000000;
            cpu->STIMER_CMP |= *((uint32_t *)dat);
            cpu->CSR[IDX_CSR_SIP] &= ~MIP_STIP;
            cpu->CSR[IDX_CSR_MIP] &= ~MIP_STIP;
            // printf("set sstc\n");
        }
        break;

    case CSR_STIMECMPH:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->STIMER_CMP >> 32;
        else
        {
            cpu->STIMER_CMP &= 0x00000000FFFFFFFF;
            cpu->STIMER_CMP |= ((uint64_t)*((uint32_t *)dat)) << 32;
            cpu->CSR[IDX_CSR_SIP] &= ~MIP_STIP;
            cpu->CSR[IDX_CSR_MIP] &= ~MIP_STIP;
        }
        break;

        // CSR_RDWR(CSR_STIMECMP)
        // CSR_RDWR(CSR_STIMECMPH)
        // CSR_RDWR(CSR_SATP)

    case CSR_SATP:
        if (rd & READ)
            *((uint32_t *)dat) = cpu->CSR[IDX_CSR_SATP];
        else
        {
            cpu->CSR[IDX_CSR_SATP] = *((uint32_t *)dat);
            // memset(cpu->icache, 0, sizeof(cpu->icache));
            cpu->SATP_asid = (*((uint32_t *)dat) >> 22) & 0x1FF;
        }
        break;

    case CSR_SCOUNTEREN:
    case 0x306: // CSR_MCOUNTEREN
    case 0x320: // CSR_MCOUNTINHIBIT
    case 0x30a: // CSR_MENVCFG
    case 0x31a: // CSR_MENVCFGH
                // case 0xfb0:
        if (rd)
            *((uint32_t *)dat) = 0;
        break;

    default:
        // if ((addr & 0xF00) == 0xB00)
        // {
        //     if (rd & READ)
        //         *((uint32_t *)dat) = 0;
        //     break;
        // }
        // printf("ACCESS CSR:%x,at:%08x,%d\n", addr, cpu->PC, rd);
        // if (rd & READ)
        //     *((uint32_t *)dat) = 0;
        trap(cpu, 0, EXC_INST_ILLEGAL, cpu->PC - cpu->inst_fetch_size, addr);
        return EXC_INST_ILLEGAL;
    }

    return EXC_OK;
}

int check_interrupt(cpu_core_t *cpu)
{

    // printf ("ie:%d\n",(cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE) ||  (cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE));
    plic_check_interrupt(cpu->id);
    if (cpu->CLINT_TIMER_CMP && ((get_microsecond_timestamp() >= cpu->CLINT_TIMER_CMP)))
        cpu->CSR[IDX_CSR_MIP] |= MIP_MTIP;
    if (cpu->STIMER_CMP && (get_microsecond_timestamp() >= cpu->STIMER_CMP))
    {
        cpu->CSR[IDX_CSR_SIP] |= MIP_STIP;
        cpu->CSR[IDX_CSR_MIP] |= MIP_STIP;
    }
    if ((cpu->CSR[IDX_CSR_MSTATUS] & SR_MIE))
    {

        if (cpu->CSR[IDX_CSR_MIP] & cpu->CSR[IDX_CSR_MIE] & MIP_MEIP)
        {
            // printf("trig meip:%d\n",cpu->id);
            trap(cpu, 1, IRQ_M_EXT, cpu->PC, 0);
            return 1;
        }

        if (cpu->CSR[IDX_CSR_MIP] & cpu->CSR[IDX_CSR_MIE] & MIP_MSIP)
        {
            // printf("trig msip:%d\n",cpu->id);
            trap(cpu, 1, IRQ_M_SOFT, cpu->PC, 0);
            return 1;
        }

        if (cpu->CSR[IDX_CSR_MIE] & MIP_MTIP)
        {
            if (cpu->CSR[IDX_CSR_MIP] & MIP_MTIP)
            {
                trap(cpu, 1, IRQ_M_TIMER, cpu->PC, 0);
                return 1;
            }
        }
    }

    if ((cpu->CSR[IDX_CSR_SSTATUS] & SR_SIE))
    {
        // printf("stip:%d\n",cpu->CSR[IDX_CSR_SIP] & MIP_STIP );
        if (cpu->MODE == M_MODE)
            goto fin;

        if (cpu->CSR[IDX_CSR_SIP] & cpu->CSR[IDX_CSR_SIE] & MIP_SSIP)
        {
            // printf("trig ssip:%d\n", cpu->id);
            trap(cpu, 1, IRQ_S_SOFT, cpu->PC, 0);
            return 1;
        }

        if (cpu->CSR[IDX_CSR_SIP] & cpu->CSR[IDX_CSR_SIE] & MIP_SEIP)
        {
            // printf("trig seip:%d\n",cpu->id);
            trap(cpu, 1, IRQ_S_EXT, cpu->PC, 0);
            return 1;
        }

        if (cpu->CSR[IDX_CSR_SIE] & MIP_STIP)
        {
            if (cpu->CSR[IDX_CSR_SIP] & MIP_STIP)
            {
                // printf("trig stip:%d\n",cpu->id);
                trap(cpu, 1, IRQ_S_TIMER, cpu->PC, 0);
                return 1;
            }
        }
    }

fin:

    return 1; // cpu->CSR[IDX_CSR_SIP] | cpu->CSR[IDX_CSR_MIP] | (cpu->cycles < 1000000000);
}

void load_bin_to_ram(char *path, uint8_t *base, uint32_t off)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        printf("Failed to load:%s\n", path);
        exit(1);
    }
    size_t fsz;
    fseek(f, 0, SEEK_END);
    fsz = ftell(f);
    printf("img size:%lld\n", (uint64_t)fsz);
    fseek(f, 0, SEEK_SET);
    fread(&base[off], 1, fsz, f);
    fclose(f);
}

static inline void gen_dec(uint32_t inst, inst_decode_t *cont)
{
    cont->rd = (inst >> 7) & 0b11111;
    cont->rs1 = (inst >> 15) & 0b11111;
    cont->rs2 = (inst >> 20) & 0b11111;
    cont->funct3 = (inst >> 12) & 0b111;
    cont->funct7 = (inst >> 25) & 0b1111111;
}

void R_type_dec(uint32_t inst, inst_decode_t *cont)
{
    gen_dec(inst, cont);
}

void I_type_dec(uint32_t inst, inst_decode_t *cont)
{
    gen_dec(inst, cont);
    cont->imm = (inst >> 20) & 0xFFF;
}

void S_type_dec(uint32_t inst, inst_decode_t *cont)
{
    gen_dec(inst, cont);
    cont->imm = cont->rd | (cont->funct7 << 5);
}

void B_type_dec(uint32_t inst, inst_decode_t *cont)
{
    gen_dec(inst, cont);
    cont->imm = ((cont->rd & 1) << 11) |
                ((cont->rd) & 0b11110) |
                ((cont->funct7 & 0b0111111) << 5) |
                (((cont->funct7 & 0b1000000) >> 6) << (12));
}

void U_type_dec(uint32_t inst, inst_decode_t *cont)
{
    cont->rd = (inst >> 7) & 0b11111;
    cont->imm = inst & 0xFFFFF000;
}

void J_type_dec(uint32_t inst, inst_decode_t *cont)
{
    cont->rd = (inst >> 7) & 0b11111;
    cont->imm = (inst & 0xff000) | ((inst >> 9) & 0x800) | ((inst >> 20) & 0x7fe) | ((inst >> 11) & 0xff00000);
}

#define UND_INS                                                       \
    {                                                                 \
        printf("AT %08x, UND INS:%d\n", cpu->PC, __LINE__);           \
        printf("AT %d, INS:%08x\n", cpu->inst_fetch_size, cpu->inst); \
        /*cpu->running = 0;*/                                         \
        trap(cpu, 0, EXC_INST_ILLEGAL,                                \
             cpu->PC - cpu->inst_fetch_size,                          \
             cpu->PC - cpu->inst_fetch_size);                         \
        return;                                                       \
    }

void core_step(cpu_core_t *cpu)
{

    uint32_t steps;
    uint64_t t0 = 0;
    if (cpu->loop_time)
    {
        if (cpu->loop_time < 500)
        {
            if (cpu->step_init < 16000)
                cpu->step_init += 100;
        }
        else if (cpu->loop_time > 800)
        {
            if (cpu->step_init > 100)
                cpu->step_init -= 100;
        }
    }
    steps = cpu->step_init;
    // steps = 8000;

    if (cpu->running)
    {
        cpu->wfi = !check_interrupt(cpu);
        if (cpu->wfi)
            return;

        t0 = get_microsecond_timestamp();
        while (steps)
        {
            cpu->REGS[ZERO] = 0;
            steps--;
            if (cpu->chk_irq)
            {
                check_interrupt(cpu);
                cpu->chk_irq = 0;
            }
#if (ENABLE_RV32C)
            if (cpu->PC & 0b10)
                cpu->inst_fetch_size = 2;
#endif
            if (mem_inst_fetch(cpu, cpu->PC, cpu->inst_fetch_size, &cpu->inst) != EXC_OK)
                continue;

            cpu->cycles++;
            if ((cpu->inst & 0b11) == 0b11)
            { // RV32
#if (ENABLE_RV32C)
                if (cpu->inst_fetch_size == 2)
                {
                    uint16_t half_up_inst = 0;
                    if (mem_inst_fetch(cpu, cpu->PC + 2, cpu->inst_fetch_size, &half_up_inst) != EXC_OK)
                    {
                        printf("Fetch inst err 2, adr:%08x, sz:%d\n", cpu->PC + 2, cpu->inst_fetch_size);
                        continue;
                    }
                    cpu->inst &= 0xFFFF;
                    cpu->inst |= (half_up_inst << 16);
                }
#endif
                cpu->PC += 4;
                cpu->inst_fetch_size = 4;
                cpu->opcode = cpu->inst & OPCODE_MASK;
                switch (cpu->opcode)
                {

                case OPCODE_ALU_REG:
                {
                    R_type_dec(cpu->inst, &cpu->cont);
                    switch (cpu->cont.funct7)
                    {
                    case 0x0:
                        switch (cpu->cont.funct3)
                        {
                        case 0x0: // add,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] + cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x4: // xor,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] ^ cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x6: // or,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] | cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x7: // and,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] & cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x1: // sll,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] << cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x5: // srl,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] >> cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x2: // slt,
                            cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] < (int32_t)cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x3: // sltu,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] < cpu->REGS[cpu->cont.rs2];
                            break;
                        default:
                            UND_INS
                            break;
                        }
                        break;
                    case 0x20:
                        switch (cpu->cont.funct3)
                        {
                        case 0x0: // SUB,
                            cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] - cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x5: // SRA,
                            cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] >> (cpu->REGS[cpu->cont.rs2] & 0x1F);
                            break;
                        default:
                            UND_INS
                            break;
                        }
                        break;
                    case 0x01:
                        switch (cpu->cont.funct3)
                        {
                        case 0x0: // MUL
                            cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] * (int32_t)cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x1: // MULH s*s
                        {
                            int64_t v1 = (int64_t)cpu->REGS[cpu->cont.rs1];
                            int64_t v2 = (int64_t)cpu->REGS[cpu->cont.rs2];
                            if (v1 & 0x80000000)
                                v1 |= 0xFFFFFFFF00000000;
                            if (v2 & 0x80000000)
                                v2 |= 0xFFFFFFFF00000000;
                            int64_t res = v1 * v2;
                            res >>= 32;
                            cpu->REGS[cpu->cont.rd] = res;
                            break;
                        }
                        case 0x2: // MULHSU s*u
                        {
                            int64_t v1 = (int64_t)cpu->REGS[cpu->cont.rs1];
                            uint64_t v2 = (int64_t)cpu->REGS[cpu->cont.rs2];
                            if (v1 & 0x80000000)
                                v1 |= 0xFFFFFFFF00000000;
                            int64_t res = v1 * v2;
                            res >>= 32;
                            cpu->REGS[cpu->cont.rd] = res;
                            break;
                        }
                        case 0x3: // MULHU u*u
                            cpu->REGS[cpu->cont.rd] = ((uint64_t)cpu->REGS[cpu->cont.rs1] * (uint64_t)cpu->REGS[cpu->cont.rs2]) >> 32;
                            break;
                        case 0x4: // DIV
                            if (cpu->REGS[cpu->cont.rs2] == 0)
                                cpu->REGS[cpu->cont.rd] = 0xFFFFFFFF;
                            else if (cpu->REGS[cpu->cont.rs1] == 0x80000000 && cpu->REGS[cpu->cont.rs2] == 0xFFFFFFFF)
                                cpu->REGS[cpu->cont.rd] = 0x80000000;
                            else
                                cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] / (int32_t)cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x5: // DIV U
                            if (cpu->REGS[cpu->cont.rs2] == 0)
                                cpu->REGS[cpu->cont.rd] = 0xFFFFFFFF;
                            else
                                cpu->REGS[cpu->cont.rd] = (uint32_t)cpu->REGS[cpu->cont.rs1] / (uint32_t)cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x6: // REM
                            if (cpu->REGS[cpu->cont.rs2] == 0)
                                cpu->REGS[cpu->cont.rd] = 0xFFFFFFFF;
                            else if (cpu->REGS[cpu->cont.rs1] == 0x80000000 && cpu->REGS[cpu->cont.rs2] == 0xFFFFFFFF)
                                cpu->REGS[cpu->cont.rd] = 0;
                            else
                                cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] % (int32_t)cpu->REGS[cpu->cont.rs2];
                            break;
                        case 0x7: // REM U
                            if (cpu->REGS[cpu->cont.rs2] == 0)
                                cpu->REGS[cpu->cont.rd] = 0xFFFFFFFF;
                            else
                                cpu->REGS[cpu->cont.rd] = (uint32_t)cpu->REGS[cpu->cont.rs1] % (uint32_t)cpu->REGS[cpu->cont.rs2];
                            break;
                        default:
                            UND_INS
                            break;
                        }
                        break;
                    default:
                        UND_INS
                        break;
                    }
                    break;
                }
                //--------------------------------------------------------
                case OPCODE_ALU_IMM:
                {
                    I_type_dec(cpu->inst, &cpu->cont);
                    switch (cpu->cont.funct3)
                    {
                    case 0x0: // addi
                        cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                        cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] + cpu->cont.imm;
                        break;
                    case 0x4: // xori
                        cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                        cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] ^ cpu->cont.imm;
                        break;
                    case 0x6: // ori
                        cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                        cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] | cpu->cont.imm;
                        break;
                    case 0x7: // andi
                        cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                        cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] & cpu->cont.imm;
                        break;
                    case 0x1: // slli
                        cpu->cont.imm &= 0x1F;
                        cpu->REGS[cpu->cont.rd] = cpu->REGS[cpu->cont.rs1] << cpu->cont.imm;
                        break;
                    case 0x5:
                        cpu->cont.imm &= 0x1F;
                        if (cpu->cont.funct7 == 0x20) // srai
                            cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] >> cpu->cont.imm;
                        else // srli
                            cpu->REGS[cpu->cont.rd] = (uint32_t)cpu->REGS[cpu->cont.rs1] >> cpu->cont.imm;
                        break;
                    case 0x2: // slti
                        cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                        cpu->REGS[cpu->cont.rd] = (int32_t)cpu->REGS[cpu->cont.rs1] < (int32_t)cpu->cont.imm;
                        break;
                    case 0x3: // sltiu
                        cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                        cpu->REGS[cpu->cont.rd] = (uint32_t)cpu->REGS[cpu->cont.rs1] < (int32_t)cpu->cont.imm;
                        break;
                    default:
                        UND_INS
                        break;
                    }

                    break;
                }
                //--------------------------------------------------------
                case OPCODE_MM_LOAD:
                {
                    I_type_dec(cpu->inst, &cpu->cont);
                    cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                    uint32_t buf = 0;
                    uint32_t adr = 0;
                    int res = 0;

                    adr = cpu->REGS[cpu->cont.rs1] + cpu->cont.imm;

#if MULTI_THREAD
#if !AMO_SPIN_LOCK
                    rw_lock_read_lock(cpu, &g_rwlock[adr / MEMLK_CHUNK_SZ]);
#endif
#endif
                    switch (cpu->cont.funct3)
                    {
                    case 0x0: // lb
                        res = mem_access(cpu, adr, 1, &buf, READ);
                        if (res != EXC_OK)
                            goto load_fin;
                        cpu->REGS[cpu->cont.rd] = (buf & 0x80) ? (buf | 0xFFFFFF00) : buf;
                        break;
                    case 0x1: // lh
                        res = mem_access(cpu, adr, 2, &buf, READ);
                        if (res != EXC_OK)
                            goto load_fin;
                        cpu->REGS[cpu->cont.rd] = (buf & 0x8000) ? (buf | 0xFFFF0000) : buf;
                        break;
                    case 0x2: // lw
                        res = mem_access(cpu, adr, 4, &buf, READ);
                        if (res != EXC_OK)
                            goto load_fin;
                        cpu->REGS[cpu->cont.rd] = buf;
                        break;
                    case 0x4: // lbu
                        adr = cpu->REGS[cpu->cont.rs1] + cpu->cont.imm;
                        res = mem_access(cpu, adr, 1, &buf, READ);
                        if (res != EXC_OK)
                            goto load_fin;
                        cpu->REGS[cpu->cont.rd] = buf;
                        break;
                    case 0x5: // lhu
                        res = mem_access(cpu, adr, 2, &buf, READ);
                        if (res != EXC_OK)
                            goto load_fin;
                        cpu->REGS[cpu->cont.rd] = buf;
                        break;

                    default:
                        UND_INS
                        break;
                    }

                load_fin:
#if MULTI_THREAD
#if !AMO_SPIN_LOCK
                    rw_lock_read_unlock(&g_rwlock[adr / MEMLK_CHUNK_SZ]);
#endif
#endif

                    break;
                }
                //--------------------------------------------------------
                case OPCODE_MM_STORE:
                {
                    S_type_dec(cpu->inst, &cpu->cont);
                    cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                    uint32_t adr = 0;
                    adr = cpu->REGS[cpu->cont.rs1] + cpu->cont.imm;

#if MULTI_THREAD
#if AMO_SPIN_LOCK
                    if (atomic_flag_test_and_set_explicit(&amo_spin_lock_blk[adr / MEMLK_CHUNK_SZ], __ATOMIC_ACQUIRE))
                    {
                        cpu->lock_wait_st++;
                        while (atomic_flag_test_and_set_explicit(&amo_spin_lock_blk[adr / MEMLK_CHUNK_SZ], __ATOMIC_ACQUIRE))
                        {
                            usleep(rand() % 7000);
                        }
                    }
#else
                    rw_lock_read_lock(cpu, &g_rwlock[adr / MEMLK_CHUNK_SZ]);
#endif
                    mem_access(cpu, adr, 1 << cpu->cont.funct3, &cpu->REGS[cpu->cont.rs2], WRITE);

#else
                    mem_access(cpu, adr, 1 << cpu->cont.funct3, &cpu->REGS[cpu->cont.rs2], WRITE);
#endif

#if MULTI_THREAD
#if AMO_SPIN_LOCK
                    atomic_flag_clear_explicit(&amo_spin_lock_blk[adr / MEMLK_CHUNK_SZ], __ATOMIC_RELEASE);
#else
                    rw_lock_read_unlock(&g_rwlock[adr / MEMLK_CHUNK_SZ]);
#endif
#endif

                    break;
                }
                //--------------------------------------------------------
                case OPCODE_BRANCH:
                {
                    B_type_dec(cpu->inst, &cpu->cont);
                    cpu->cont.imm = (cpu->cont.imm & 0x1000) ? (cpu->cont.imm | 0xFFFFE000) : cpu->cont.imm;
                    switch (cpu->cont.funct3)
                    {
                    case 0x0: // beq
                        if (cpu->REGS[cpu->cont.rs1] == cpu->REGS[cpu->cont.rs2])
                            cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                        break;
                    case 0x1: // bne
                        if (cpu->REGS[cpu->cont.rs1] != cpu->REGS[cpu->cont.rs2])
                            cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                        break;
                    case 0x4: // blt
                        if ((int32_t)cpu->REGS[cpu->cont.rs1] < (int32_t)cpu->REGS[cpu->cont.rs2])
                            cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                        break;
                    case 0x5: // bge
                        if ((int32_t)cpu->REGS[cpu->cont.rs1] >= (int32_t)cpu->REGS[cpu->cont.rs2])
                            cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                        break;
                    case 0x6: // bltu
                        if (cpu->REGS[cpu->cont.rs1] < cpu->REGS[cpu->cont.rs2])
                            cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                        break;
                    case 0x7: // bgeu
                        if (cpu->REGS[cpu->cont.rs1] >= cpu->REGS[cpu->cont.rs2])
                            cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                        break;

                    default:
                        UND_INS
                        break;
                    }
                    break;
                }
                //--------------------------------------------------------
                case OPCODE_JAL:
                {
                    J_type_dec(cpu->inst, &cpu->cont);
                    cpu->cont.imm = (cpu->cont.imm & 0x00100000) ? (cpu->cont.imm | 0xFFF00000) : cpu->cont.imm;
                    cpu->REGS[cpu->cont.rd] = cpu->PC;
                    cpu->PC = cpu->PC - 4 + cpu->cont.imm;
                    break;
                }
                case OPCODE_JALR:
                {
                    uint32_t tmppc = 0;
                    I_type_dec(cpu->inst, &cpu->cont);
                    cpu->cont.imm = (cpu->cont.imm & 0x800) ? (cpu->cont.imm | 0xFFFFF000) : cpu->cont.imm;
                    tmppc = cpu->PC;
                    cpu->PC = cpu->REGS[cpu->cont.rs1] + cpu->cont.imm;
                    cpu->REGS[cpu->cont.rd] = tmppc;
                    break;
                }
                case OPCODE_LUI:
                {
                    U_type_dec(cpu->inst, &cpu->cont);
                    cpu->REGS[cpu->cont.rd] = cpu->cont.imm;
                    break;
                }
                case OPCODE_AUIPC:
                {
                    U_type_dec(cpu->inst, &cpu->cont);
                    cpu->REGS[cpu->cont.rd] = cpu->PC - 4 + cpu->cont.imm;
                    break;
                }
                case OPCODE_ZICSR:
                {
                    I_type_dec(cpu->inst, &cpu->cont);
                    uint32_t t = 0;
                    uint32_t old_val = 0;
                    int res;
                    switch (cpu->cont.funct3)
                    {
                    case 0:
                    {
                        switch (cpu->cont.imm)
                        {
                        case 1: // ebreak
                            // if (cpu->REGS[A0] == 3)
                            // {
                            //     char c = 0;
                            //     mem_access(cpu, cpu->REGS[A1], 1, &c, READ);
                            //     putchar(c);
                            // }
                            // printf("ebk\n");
                            trap(cpu, 0, EXC_BREAKPOINT, cpu->PC - 4, cpu->PC - 4);
                            break;
                        case 0b000100000101: // wfi
                            cpu->wfi = 1;
                            // cpu->CSR[IDX_CSR_SSTATUS] |= SR_SIE;
                            return;
                            // break;

                        case 0b001100000010: // mret:
                            if (unlikely(cpu->MODE != M_MODE))
                            {
                                printf("===================");
                                printf("MRET NOT IN M_MODE\n");
                                printf("===================");
                                UND_INS;
                            }
                            COPY_MSTATUS_TO_SSTATUS;
                            COPY_MIx_TO_SIx(E);
                            COPY_MIx_TO_SIx(P);

                            load_resv_addr[cpu->id] = 0xFFFFFFFF;

                            cpu->PC = cpu->CSR[IDX_CSR_MEPC];
                            cpu->CSR[IDX_CSR_MSTATUS] &= ~SR_MIE;
                            cpu->CSR[IDX_CSR_MSTATUS] |= (cpu->CSR[IDX_CSR_MSTATUS] & SR_MPIE) ? SR_MIE : 0;
                            cpu->MODE = (cpu->CSR[IDX_CSR_MSTATUS] & SR_MPP) >> SR_MPP_BIT_SFT;

                            cpu->chk_irq = 1;
                            break;

                        case 0b000100000010: // sret

                            if (unlikely(cpu->MODE != S_MODE))
                            {
                                printf("===================");
                                printf("SRET NOT IN S_MODE\n");
                                printf("===================");
                                UND_INS;
                            }
                            // COPY_SIx_TO_MIx(E);
                            // COPY_SIx_TO_MIx(P);
                            // COPY_SSTATUS_TO_MSTATUS;
                            cpu->PC = cpu->CSR[IDX_CSR_SEPC];
                            cpu->CSR[IDX_CSR_SSTATUS] &= ~SR_SIE;
                            cpu->CSR[IDX_CSR_SSTATUS] |= (cpu->CSR[IDX_CSR_SSTATUS] & SR_SPIE) ? SR_SIE : 0;
                            cpu->MODE = (cpu->CSR[IDX_CSR_SSTATUS] & SR_SPP) >> SR_SPP_BIT_SFT;

                            cpu->chk_irq = 1;
                            break;

                        case 0: // ecall
                                // printf("ecall mode %d\n", cpu->MODE);
                                // debug_dump_regs(cpu);
                            switch (cpu->MODE)
                            {
                            case U_MODE:
                                trap(cpu, 0, EXC_SYSCALL, cpu->PC - 4, cpu->PC - 4);
                                break;
                            case S_MODE:
                                // printf("ECALL IN S_MODE\n");
                                trap(cpu, 0, EXC_SUPERVISOR_SYSCALL, cpu->PC - 4, cpu->PC - 4);
                                break;
                            case M_MODE:
                                // printf("ECALL IN M_MODE\n");
                                trap(cpu, 0, EXC_M_MODE_SYSCALL, cpu->PC - 4, cpu->PC - 4);
                                break;
                            default:
                                UND_INS
                                break;
                            }
                            // debug_dump_regs(cpu);
                            break;

                        default:
                            switch (cpu->cont.funct7)
                            {
                            case 0b0001001: // SFENCE.VMA
#if ENABLE_TLB
                                uint32_t asid = cpu->REGS[cpu->cont.rs2];
                                uint32_t vaddr = cpu->REGS[cpu->cont.rs1];
                                cpu->last_access_vpn = 0;
                                if (asid & vaddr)
                                {
                                    cpu->tlbs[TLB_ASID_HASH(asid)][TLB_ADDR_HASH(vaddr)].valid = 0;
                                    cpu->icache[ICACHE_HASH(vaddr)].valid = 0;
                                    cpu->dcache[DCACHE_HASH(vaddr)].valid = 0;
                                }
                                else if (asid)
                                {
                                    // printf("sfence asid:%d\n",asid);
                                    for (int i = 0; i < TLB_ADDR_ITEMS; i++)
                                    {
                                        cpu->tlbs[TLB_ASID_HASH(asid)][i].valid = 0;
                                        cpu->icache[ICACHE_HASH(i)].valid = 0;
                                        cpu->dcache[DCACHE_HASH(i)].valid = 0;
                                    }
                                }
                                else if (vaddr)
                                {

                                    cpu->icache[ICACHE_HASH(vaddr)].valid = 0;
                                    cpu->dcache[DCACHE_HASH(vaddr)].valid = 0;
                                    for (int i = 0; i < TLB_ASID_ITEMS; i++)
                                        cpu->tlbs[i][TLB_ADDR_HASH(vaddr)].valid = 0;
                                }
                                else
                                {
                                    memset(cpu->tlbs, 0, sizeof(cpu->tlbs));
                                    memset(cpu->icache, 0, sizeof(cpu->icache));
                                    memset(cpu->dcache, 0, sizeof(cpu->dcache));
                                }
#endif
                                break;

                            default:
                                UND_INS
                                break;
                            }
                        }
                        break;
                    }
                    case 1: // csrrw
                        res = csr_access(cpu, cpu->cont.imm, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        t = cpu->REGS[cpu->cont.rs1];
                        if (old_val != t)
                            csr_access(cpu, cpu->cont.imm, &t, WRITE);
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    case 2: // csrrs
                        res = csr_access(cpu, cpu->cont.imm, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        t = old_val | cpu->REGS[cpu->cont.rs1];
                        if (old_val != t)
                            csr_access(cpu, cpu->cont.imm, &t, WRITE);
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    case 3: // csrrc
                        res = csr_access(cpu, cpu->cont.imm, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        t = old_val & ~cpu->REGS[cpu->cont.rs1];
                        if (old_val != t)
                            csr_access(cpu, cpu->cont.imm, &t, WRITE);
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;

                    case 5: // csrrwi
                        res = csr_access(cpu, cpu->cont.imm, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        t = cpu->cont.rs1;
                        if (old_val != t)
                            csr_access(cpu, cpu->cont.imm, &t, WRITE);
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    case 6: // csrrsi
                        res = csr_access(cpu, cpu->cont.imm, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        t = old_val | cpu->cont.rs1;
                        if (old_val != t)
                            csr_access(cpu, cpu->cont.imm, &t, WRITE);
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    case 7: // csrrci
                        res = csr_access(cpu, cpu->cont.imm, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        t = old_val & ~((uint32_t)cpu->cont.rs1);
                        if (old_val != t)
                            csr_access(cpu, cpu->cont.imm, &t, WRITE);
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;

                    default:
                        UND_INS
                        break;
                    }
                    break;
                }
                case OPCODE_FENCE:
                    S_type_dec(cpu->inst, &cpu->cont);
                    // if (cpu->cont.funct3 == 1)
                    {
                        memset(cpu->icache, 0, sizeof(cpu->icache));
                        memset(cpu->dcache, 0, sizeof(cpu->dcache));
                    }
                    cpu->last_access_vpn = 0;
                    break;
                case OPCODE_AMO:
                {
                    R_type_dec(cpu->inst, &cpu->cont);
                    uint32_t funct5 = cpu->cont.funct7 >> 2;
                    uint32_t tmp = 0;
                    uint32_t old_val = 0;
                    uint32_t adr = 0;
                    adr = cpu->REGS[cpu->cont.rs1];
                    int res;

#if (MULTI_THREAD)
#if AMO_SPIN_LOCK
                    if (atomic_flag_test_and_set_explicit(&amo_spin_lock_blk[adr / MEMLK_CHUNK_SZ], __ATOMIC_ACQUIRE))
                    {
                        cpu->lock_wait_amo_st++;
                        while (atomic_flag_test_and_set_explicit(&amo_spin_lock_blk[adr / MEMLK_CHUNK_SZ], __ATOMIC_ACQUIRE))
                        {
                            usleep(rand() % 7000);
                        }
                    }
#else
                    rw_lock_write_lock(cpu, &g_rwlock[adr / MEMLK_CHUNK_SZ]);

#endif
#endif

                    switch (funct5)
                    {
                    case 0x02: // lr.w
                    {
                        res = mem_access(cpu, adr, 4, &cpu->REGS[cpu->cont.rd], READ);
                        if (res != EXC_OK)
                            break;
                        while (atomic_flag_test_and_set_explicit(&LR_lock, __ATOMIC_ACQUIRE))
                            ;
                        load_resv_addr[cpu->id] = adr >> 2;
                        atomic_flag_clear_explicit(&LR_lock, __ATOMIC_RELEASE);
                        break;
                    }
                    case 0x03: // sc.w
                    {
                        if ((adr >> 2) == load_resv_addr[cpu->id])
                        // if ((adr) == load_resv_addr[cpu->id])
                        // if(1)
                        {
                            res = mem_access(cpu, adr, 4, &cpu->REGS[cpu->cont.rs2], WRITE);
                            if (res != EXC_OK)
                                break;
                            cpu->REGS[cpu->cont.rd] = 0;
                        }
                        else
                        {
                            cpu->REGS[cpu->cont.rd] = 1;
                        }
                        load_resv_addr[cpu->id] = 0xFFFFFFFF;
                        break;
                    }

                    case 0x01: // amoswap
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = cpu->REGS[cpu->cont.rs2];
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }

                    case 0x00: // amoadd
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = cpu->REGS[cpu->cont.rs2] + old_val;
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }

                    case 0x0C: // amoand
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = cpu->REGS[cpu->cont.rs2] & old_val;
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }

                    case 0x08: // amoor
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = cpu->REGS[cpu->cont.rs2] | old_val;
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }

                    case 0x04: // amoxor
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = cpu->REGS[cpu->cont.rs2] ^ old_val;
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }
                    case 0x14: // amomax
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = __max((int32_t)cpu->REGS[cpu->cont.rs2], (int32_t)old_val);
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }
                    case 0x1c: // amomaxu
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = __max(cpu->REGS[cpu->cont.rs2], old_val);
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }
                    case 0x10: // amomin
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = __min((int32_t)cpu->REGS[cpu->cont.rs2], (int32_t)old_val);
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }
                    case 0x18: // amominu
                    {
                        res = mem_access(cpu, adr, 4, &old_val, READ);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[ZERO] = 0;
                        tmp = __min(cpu->REGS[cpu->cont.rs2], old_val);
                        res = mem_access(cpu, adr, 4, &tmp, WRITE);
                        if (res != EXC_OK)
                            break;
                        cpu->REGS[cpu->cont.rd] = old_val;
                        break;
                    }

                    default:
                        cpu->running = 0;
                        printf("UNKNOWN INST OPCODE_AMO. %08x\n", cpu->PC);
                        break;
                    }

#if (MULTI_THREAD)
#if AMO_SPIN_LOCK
                    atomic_flag_clear_explicit(&amo_spin_lock_blk[adr / MEMLK_CHUNK_SZ], __ATOMIC_RELEASE);
#else
                    rw_lock_write_unlock(&g_rwlock[adr / MEMLK_CHUNK_SZ]);
#endif
#endif

                    break;
                }
                default:
                    UND_INS
                    break;
                }
            }
            else
            {

                cpu->PC += 2;
                cpu->inst_fetch_size = 2;
#if (ENABLE_RV32C)
                // Compressed Instructions
                uint32_t inst = cpu->inst & 0xFFFF;
                uint32_t funt3 = inst >> 13;
                uint32_t rd, rs1, rs2;
                switch (inst & 0b11)
                {
                case 0b00: // LOAD STORE
                    switch (funt3)
                    {
                    case 0b000: // C.ADDI4SPN rd, rs1, imm // addi rd’, sp, 4*imm
                    {
                        rd = ((inst >> 2) & 0b111) + 8;

                        uint32_t nzuimm = (inst >> 5) & 0xFF;
                        uint32_t nzuimm_3 = nzuimm & 1;
                        uint32_t nzuimm_2 = (nzuimm >> 1) & 1;
                        uint32_t nzuimm_6_9 = (nzuimm >> 2) & 0xF;
                        uint32_t nzuimm_4_5 = (nzuimm >> 6) & 0b11;
                        nzuimm = (nzuimm_2 << 2) | (nzuimm_3 << 3) | (nzuimm_4_5 << 4) | (nzuimm_6_9 << 6);
                        if (nzuimm == 0)
                        {
                            UND_INS;
                        }
                        cpu->REGS[rd] = cpu->REGS[SP] + nzuimm;
                        break;
                    }
                    case 0b010: // C.LW rd, offset(rs1) // lw rd’, (4*imm)(rs1’)
                    {
                        rd = ((inst >> 2) & 0b111) + 8;
                        rs1 = ((inst >> 7) & 0b111) + 8;

                        uint32_t uimm_3_5 = (inst >> 10) & 0b111;
                        uint32_t uimm_2 = (inst >> 6) & 1;
                        uint32_t uimm_6 = (inst >> 5) & 1;
                        uint32_t uimm = (uimm_2 << 2) | (uimm_6 << 6) | (uimm_3_5 << 3);

                        uint32_t adr = cpu->REGS[rs1] + uimm;
                        mem_access(cpu, adr, 4, &cpu->REGS[rd], READ);
                        break;
                    }
                    case 0b110: // C.SW rs2, offset(rs1)
                    {
                        rs2 = ((inst >> 2) & 0b111) + 8;
                        rs1 = ((inst >> 7) & 0b111) + 8;
                        uint32_t uimm_3_5 = (inst >> 10) & 0b111;
                        uint32_t uimm_2 = (inst >> 6) & 1;
                        uint32_t uimm_6 = (inst >> 5) & 1;
                        uint32_t uimm = (uimm_2 << 2) | (uimm_6 << 6) | (uimm_3_5 << 3);
                        uint32_t adr = cpu->REGS[rs1] + uimm;
                        mem_access(cpu, adr, 4, &cpu->REGS[rs2], WRITE);
                        break;
                    }
                    default:
                        UND_INS
                        break;
                    }
                    break;
                case 0b01: // C1
                    switch (funt3)
                    {
                    case 0b000: // C.ADDI rd, rs1, nzsimm
                    {
                        uint32_t nzsimm_5 = (inst >> 12) & 1;
                        uint32_t nzsimm = ((inst >> 2) & 0x1F) | (nzsimm_5 << 5);
                        if (nzsimm_5)
                            nzsimm |= ~((1 << 5) - 1);
                        rd = (inst >> 7) & 0x1F;
                        if (!rd)
                            if (nzsimm)
                                UND_INS;
                        cpu->REGS[rd] = cpu->REGS[rd] + nzsimm;
                        break;
                    }

                    case 0b001: // C.JAL rd, offset
                    {
                        uint32_t simm_field = (inst >> 2);
                        uint32_t simm_5 = (simm_field & 1);
                        uint32_t simm_1_3 = ((simm_field >> 1) & 0b111);
                        uint32_t simm_7 = ((simm_field >> 4) & 0b1);
                        uint32_t simm_6 = ((simm_field >> 5) & 0b1);
                        uint32_t simm_10 = ((simm_field >> 6) & 0b1);
                        uint32_t simm_8_9 = ((simm_field >> 7) & 0b11);
                        uint32_t simm_4 = ((simm_field >> 9) & 0b1);
                        uint32_t simm_11 = ((simm_field >> 10) & 0b1);
                        uint32_t simm = (simm_1_3 << 1) | (simm_4 << 4) | (simm_5 << 5) | (simm_6 << 6) |
                                        (simm_7 << 7) | (simm_8_9 << 8) | (simm_10 << 10) | (simm_11 << 11);
                        if (simm_11)
                        {
                            simm |= ~((1 << 11) - 1);
                        }
                        rd = 1;
                        cpu->REGS[rd] = cpu->PC;
                        cpu->PC = cpu->PC - 2 + simm;
                        break;
                    }

                    case 0b010: // C.LI rd, rs1, imm
                    {
                        uint32_t simm_5 = (inst >> 12) & 1;
                        uint32_t simm = ((inst >> 2) & 0x1F) | (simm_5 << 5);
                        if (simm_5)
                            simm |= ~((1 << 5) - 1);
                        rd = (inst >> 7) & 0x1F;
                        rs1 = 0;
                        cpu->REGS[rd] = simm;
                        break;
                    }

                    case 0b011: // C.ADDI16SP rd, rs1, nzsimm / C.LUI rd, imm
                    {
                        rd = (inst >> 7) & 0x1F;
                        if (rd == 2)
                        {
                            uint32_t nzsimm_5 = (inst >> 2) & 1;
                            uint32_t nzsimm_7_8 = ((inst >> 2) >> 1) & 0b11;
                            uint32_t nzsimm_6 = ((inst >> 2) >> 3) & 0b1;
                            uint32_t nzsimm_4 = ((inst >> 2) >> 4) & 0b1;
                            uint32_t nzsimm_9 = (inst >> 12) & 0b1;
                            uint32_t nzsimm = (nzsimm_4 << 4) | (nzsimm_5 << 5) | (nzsimm_6 << 6) |
                                              (nzsimm_7_8 << 7) | (nzsimm_9 << 9);
                            if (!nzsimm)
                                UND_INS;
                            if (nzsimm_9)
                                nzsimm |= ~((1 << 9) - 1);
                            rs1 = SP;
                            cpu->REGS[rd] = cpu->REGS[rs1] + nzsimm;
                        }
                        else // C.LUI rd, imm
                        {
                            uint32_t imm_12_16 = (inst >> 2) & 0x1F;
                            uint32_t imm_17 = (inst >> 12) & 0x1;
                            uint32_t imm = (imm_17 << 17) | (imm_12_16 << 12);
                            if (imm_17)
                            {
                                imm |= ~((1 << 17) - 1);
                            }
                            cpu->REGS[rd] = imm;
                        }
                        break;
                    }
                    case 0b100:
                    {
                        uint32_t op1 = (inst >> 10) & 0b111;
                        uint32_t rd_c = (inst >> 7) & 0b111;
                        if ((op1 & 0b011) == 0b010)
                        { // C.ANDI rd, rs1, imm
                            uint32_t nzsimm_5 = (inst >> 12) & 1;
                            uint32_t nzsimm = ((inst >> 2) & 0x1F) | (nzsimm_5 << 5);
                            if (!nzsimm)
                                UND_INS;
                            if (nzsimm_5)
                                nzsimm |= ~((1 << 5) - 1);
                            rd = rd_c + 8;
                            rs1 = rd_c + 8;
                            cpu->REGS[rd] = cpu->REGS[rs1] & nzsimm;
                        }
                        else
                        {
                            switch (op1)
                            {
                            case 0b000: // C.SRLI rd, rs1, imm
                            {
                                uint32_t nzuimm = ((inst >> 2) & 0x1F);
                                if (!nzuimm)
                                {
                                    UND_INS;
                                }
                                rd = rd_c + 8;
                                rs1 = rd_c + 8;
                                cpu->REGS[rd] = cpu->REGS[rs1] >> nzuimm;
                                break;
                            }
                            case 0b001: // C.SRAI rd, rs1, imm
                            {
                                uint32_t nzuimm = ((inst >> 2) & 0x1F);
                                if (!nzuimm)
                                {
                                    UND_INS;
                                }
                                rd = rd_c + 8;
                                rs1 = rd_c + 8;
                                cpu->REGS[rd] = (int32_t)cpu->REGS[rs1] >> nzuimm;
                                break;
                            }

                            case 0b011:
                            {
                                uint32_t rs2_c = (inst >> 2) & 0b111;
                                uint32_t op2 = (inst >> 5) & 0b11;
                                rd = rd_c + 8;
                                rs1 = rd_c + 8;
                                rs2 = rs2_c + 8;
                                switch (op2)
                                {
                                case 0b00: // C.SUB rd, rs1, rs2
                                    cpu->REGS[rd] = cpu->REGS[rs1] - cpu->REGS[rs2];
                                    break;
                                case 0b01: // C.XOR rd, rs1, rs2
                                    cpu->REGS[rd] = cpu->REGS[rs1] ^ cpu->REGS[rs2];
                                    break;
                                case 0b10: // C.OR rd, rs1, rs2
                                    cpu->REGS[rd] = cpu->REGS[rs1] | cpu->REGS[rs2];
                                    break;
                                case 0b11: // C.AND rd, rs1, rs2
                                    cpu->REGS[rd] = cpu->REGS[rs1] & cpu->REGS[rs2];
                                    break;
                                default:
                                    UND_INS
                                    break;
                                }
                                break;
                            }

                            default:
                                UND_INS
                                break;
                            }
                        }

                        break;
                    }

                    case 0b101: // C.J rd, offset
                    {
                        uint32_t simm_field = (inst >> 2);
                        uint32_t simm_5 = (simm_field & 1);
                        uint32_t simm_1_3 = ((simm_field >> 1) & 0b111);
                        uint32_t simm_7 = ((simm_field >> 4) & 0b1);
                        uint32_t simm_6 = ((simm_field >> 5) & 0b1);
                        uint32_t simm_10 = ((simm_field >> 6) & 0b1);
                        uint32_t simm_8_9 = ((simm_field >> 7) & 0b11);
                        uint32_t simm_4 = ((simm_field >> 9) & 0b1);
                        uint32_t simm_11 = ((simm_field >> 10) & 0b1);
                        uint32_t simm = (simm_1_3 << 1) | (simm_4 << 4) | (simm_5 << 5) | (simm_6 << 6) |
                                        (simm_7 << 7) | (simm_8_9 << 8) | (simm_10 << 10) | (simm_11 << 11);
                        if (simm_11)
                        {
                            simm |= ~((1 << 11) - 1);
                        }
                        cpu->PC = cpu->PC - 2 + simm;
                        break;
                    }

                    case 0b110: // C.BEQZ rs1, rs2, offset
                    {
                        uint32_t simm_5 = (inst >> 2) & 1;
                        uint32_t simm_1_2 = ((inst >> 3)) & 0b11;
                        uint32_t simm_6_7 = ((inst >> 5)) & 0b11;
                        uint32_t simm_3_4 = ((inst >> 10)) & 0b11;
                        uint32_t simm_8 = ((inst >> 12)) & 0b1;
                        uint32_t simm = (simm_1_2 << 1) | (simm_3_4 << 3) | (simm_5 << 5) | (simm_6_7 << 6) | (simm_8 << 8);
                        if (simm_8)
                        {
                            simm |= ~((1 << 8) - 1);
                        }
                        uint32_t rd_c = (inst >> 7) & 0b111;
                        rs1 = rd_c + 8;
                        if (cpu->REGS[rs1] == 0)
                            cpu->PC = cpu->PC - 2 + simm;
                        break;
                    }

                    case 0b111: // C.BNEZ rs1, rs2, offset
                    {
                        uint32_t simm_5 = (inst >> 2) & 1;
                        uint32_t simm_1_2 = ((inst >> 3)) & 0b11;
                        uint32_t simm_6_7 = ((inst >> 5)) & 0b11;
                        uint32_t simm_3_4 = ((inst >> 10)) & 0b11;
                        uint32_t simm_8 = ((inst >> 12)) & 0b1;
                        uint32_t simm = (simm_1_2 << 1) | (simm_3_4 << 3) | (simm_5 << 5) | (simm_6_7 << 6) | (simm_8 << 8);
                        if (simm_8)
                            simm |= ~((1 << 8) - 1);
                        uint32_t rd_c = (inst >> 7) & 0b111;
                        rs1 = rd_c + 8;
                        if (cpu->REGS[rs1] != 0)
                            cpu->PC = cpu->PC - 2 + simm;
                        break;
                    }

                    default:
                        UND_INS
                        break;
                    }
                    break;
                case 0b10: // C2

                    rd = (inst >> 7) & 0x1F;
                    switch (funt3)
                    {
                    case 0b000: // C.SLLI rd, rs1, imm
                    {
                        uint32_t nzuimm = (inst >> 2) & 0x1F;
                        cpu->REGS[rd] = cpu->REGS[rd] << nzuimm;
                        break;
                    }
                    case 0b010: // C.LWSP rd, offset(rs1)
                    {

                        uint32_t uimm_6_7 = (inst >> 2) & 0b11;
                        uint32_t uimm_2_4 = ((inst >> 2) >> 2) & 0b111;
                        uint32_t uimm_5 = (inst >> 12) & 1;
                        uint32_t uimm = (uimm_2_4 << 2) | (uimm_5 << 5) | (uimm_6_7 << 6);
                        uint32_t addr = cpu->REGS[SP] + uimm;
                        mem_access(cpu, addr, 4, &cpu->REGS[rd], READ);
                        break;
                    }
                    case 0b110: // C.SWSP rs2, offset(rs1)
                    {
                        uint32_t uimm_6_7 = (inst >> 7) & 0b11;
                        uint32_t uimm_2_5 = (inst >> 9) & 0b1111;
                        uint32_t uimm = (uimm_2_5 << 2) | (uimm_6_7 << 6);
                        uint32_t rs2 = (inst >> 2) & 0x1F;
                        uint32_t addr = cpu->REGS[SP] + uimm;
                        mem_access(cpu, addr, 4, &cpu->REGS[rs2], WRITE);
                        break;
                    }

                    case 0b100:
                    {
                        uint32_t rd_cc = (inst >> 12) & 1;
                        uint32_t rs1 = (inst >> 7) & 0x1F;
                        uint32_t rs2 = (inst >> 2) & 0x1F;
                        if (rd_cc && (rs1 > 0) && (rs1 < 32) && (rs2 == 0)) // C.JALR
                        {
                            uint32_t tmppc = 0;
                            tmppc = cpu->PC;
                            cpu->PC = cpu->REGS[rs1];
                            cpu->REGS[RA] = tmppc;
                        }
                        else if (rd_cc && (rs1 != 0) && (rs2 != 0)) // C.ADD
                        {
                            cpu->REGS[rs1] = cpu->REGS[rs1] + cpu->REGS[rs2];
                        }
                        else if ((!rd_cc) && (rs1 > 0) && (rs1 < 32) && (rs2 == 0)) // C.JR  /jalr x0, 0(rs1).
                        {
                            cpu->PC = cpu->REGS[rs1];
                        }
                        else if ((!rd_cc) && (rs1 != 0) && (rs2 != 0)) // C.MV rd, rs1, rs2
                        {
                            cpu->REGS[rs1] = cpu->REGS[rs2];
                        }
                        else if (rd_cc && (!rs1) && (!rs2)) // C.EBREAK
                        {
                            trap(cpu, 0, EXC_BREAKPOINT, cpu->PC - 2, cpu->PC - 2);
                        }
                        else
                        {
                            UND_INS
                        }
                        break;
                    }

                    default:
                        UND_INS
                        break;
                    }

                    break;

                default:
                    UND_INS
                    break;
                }
#else
                printf("RV32C is not available\n");
                UND_INS;
#endif
            } // else
        }
        t0 = get_microsecond_timestamp() - t0;
        cpu->loop_time = t0;
    }
}

void plic_check_interrupt(int j)
{
    if (cpu_core[j].exti_context_pending_irq[0])
        return;
    if (cpu_core[j].exti_context_pending_irq[1])
        return;

    for (int i = 0; i < NR_IRQ; i++)
    {
        if (extirq_slot[i].pending)
        {
            uint32_t bit = (1 << i);
            // for (int j = 0; j < NR_CPU; j++)
            {
                if (cpu_core[j].exti_context_enable[0] & bit) // M EXTI
                {
                    if (extirq_slot[i].priority >= cpu_core[j].exti_context_piorid[0])
                    {
                        cpu_core[j].exti_context_pending_irq[0] = i;
                        cpu_core[j].CSR[IDX_CSR_MIP] |= MIP_MEIP;
                    }
                }
                if (cpu_core[j].exti_context_enable[1] & bit) // S EXTI
                {
                    if (extirq_slot[i].priority >= cpu_core[j].exti_context_piorid[1])
                    {
                        cpu_core[j].exti_context_pending_irq[1] = i;
                        cpu_core[j].CSR[IDX_CSR_SIP] |= MIP_SEIP;
                        cpu_core[j].CSR[IDX_CSR_MIP] |= MIP_SEIP;
                    }
                }
            }
        }
    }
}

void cpu_core_init(cpu_core_t *cpu)
{
    memset(cpu, 0, sizeof(cpu_core_t));
    // cpu->PC = MEM_BASE + MEM_SIZE - 512 * 1024;
    // cpu->PC = MEM_BASE;
    cpu->PC = SBI_BASE;
    cpu->REGS[SP] = MEM_BASE + MEM_SIZE - 1024;
    cpu->MODE = M_MODE;
    memset(&cpu->CSR, 0, IDX_CSR_NUMS * 4);

    cpu->CSR[IDX_CSR_MSTATUS] &= ~SR_MPP;
    cpu->CSR[IDX_CSR_MSTATUS] |= (cpu->MODE << SR_MPP_BIT_SFT);
    cpu->CSR[IDX_CSR_MSTATUS] |= SR_MIE;
    cpu->CSR[IDX_CSR_MSTATUS] |= SR_MPIE;

    // cpu->CSR[IDX_CSR_MIE] |= (1 << IRQ_M_SOFT);
    // cpu->CSR[IDX_CSR_MIP] &= ~(1 << IRQ_M_SOFT);

    cpu->CSR[IDX_CSR_MTVEC] = cpu->PC;
    cpu->CSR[IDX_CSR_STVEC] = cpu->PC;

    cpu->CLINT_TIMER_CMP = 0;
    cpu->CLINT_IPI = 0;
    cpu->running = 0;
    cpu->inst_fetch_size = 4;
    cpu->step_init = 1000;
}

#if MULTI_THREAD
void *cpu_thread(void *threadid)
{
    int tid;
    tid = (uptr_t)threadid;
    // if (tid)
    //     return 0;
    //     // cpu_core[tid].running = 1;
    printf("vCPU:%d starting...\n", tid);
    // for (;;)
    while (cpu_core[tid].running)
    {
        core_step(&cpu_core[tid]);
#if MULTI_THREAD
        if (cpu_core[tid].wfi)
        {
            usleep(1000);
        }
        cpu_core[tid].wfi = 0;
#endif
    }
    return NULL;
}
#endif

#if MULTI_THREAD
pthread_t threads[NR_CPU];
#endif

void exit_emu()
{

#if MULTI_THREAD
    for (int i = 0; i < NR_CPU; i++)
        cpu_core[i].running = 0;
    for (int i = 0; i < NR_CPU; i++)
        pthread_join(threads[i], NULL);
#endif
    vdisk_uninit();
    free(main_mem);
    free(framebuffer);
    debug_dump_regs(&cpu_core[0]);
    printf("exit.\n");
    exit(0);
}

void ctrlc(int sig)
{
    char gc;
    signal(sig, SIG_IGN);
    printf("\n[Ctrl-C]:send ctrlc [R]:dump regs, [T]:dump memmap, [Q]:quit\n");
#ifdef __linux__
    _getch();
    usleep(1000000);
    gc = _getch();
#else
    gc = _getch();
#endif

    switch (gc)
    {
    case 'r':
    case 'R':
        for (int i = 0; i < NR_CPU; i++)
            debug_dump_regs(&cpu_core[i]);
        break;
    case 't':
    case 'T':
        debug_dump_mmap(&cpu_core[0]);
        break;
    case 's':
    case 'S':
#if COLLECT_PERF_STATUS
        for (int i = 0; i < NR_CPU; i++)
        {
            printf("cpu:%d, tlb hit/miss:%lld/%lld,rate: %.2f  \n", i,
                   cpu_core[i].tlb_hit,
                   cpu_core[i].tlb_miss,
                   cpu_core[i].tlb_hit * 100.0 / (cpu_core[i].tlb_hit + cpu_core[i].tlb_miss));

            printf("cpu:%d, icache hit/miss:%lld/%lld,rate: %.2f  \n", i,
                   cpu_core[i].icache_hit,
                   cpu_core[i].icache_miss,
                   cpu_core[i].icache_hit * 100.0 / (cpu_core[i].icache_hit + cpu_core[i].icache_miss));

            printf("cpu:%d, mem_st_spinlock_cnt:%lld, mem_amo_st_spinlock_cnt:%lld  ", i,
                   cpu_core[i].lock_wait_st, cpu_core[i].lock_wait_amo_st);

            uint64_t t = get_microsecond_timestamp();
            printf("  MIPS:%.3f. cycles:%lld\n", (double)(cpu_core[i].cycles - cpu_core[i].debug_cycs) / (t - cpu_core[i].debug_t0), cpu_core[i].cycles);
            cpu_core[i].debug_t0 = t;
            cpu_core[i].debug_cycs = cpu_core[i].cycles;
        }
#else

        for (int i = 0; i < NR_CPU; i++)
        {
            uint64_t t = get_microsecond_timestamp();
            printf("  MIPS:%.3f. cycles:%lld\n", (double)(cpu_core[i].cycles - cpu_core[i].debug_cycs) / (t - cpu_core[i].debug_t0), cpu_core[i].cycles);
            cpu_core[i].debug_t0 = t;
            cpu_core[i].debug_cycs = cpu_core[i].cycles;
        }
#endif

        break;
    case 'q':
    case 'Q':
        exit_emu();
    default:
        uart_send_char(gc);
        break;
    }
    printf("\n");
    signal(SIGINT, ctrlc);
}

int main(int argc, char *argv[])
{
#ifdef __linux__
    struct termios oldt, newt;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    // tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    // fcntl(STDIN_FILENO, F_SETFL, oldf);
#endif
    signal(SIGINT, ctrlc);
    main_mem = calloc(1, MEM_SIZE);
    framebuffer = calloc(1, VRAM_SIZE);
    memset((void *)load_resv_addr, 0xFF, sizeof(load_resv_addr));
    // memset((void *)amo_addr, 0xFF, sizeof(amo_addr));
    memset(uart_in_buf, 0, sizeof(uart_in_buf));

    load_bin_to_ram("\\\\wsl.localhost\\Ubuntu-22.04\\home\\nahida\\kernel\\sbi2\\opensbi\\out\\platform\\simemu\\firmware\\fw_jump.bin", sbi_fw_mem, 0);
    load_bin_to_ram("D:\\ExtendedPart\\sim_rv32_git\\riscv32-simemu\\config\\simemu-mmu.dtb", fdt_fw_mem, 0);

    // load_bin_to_ram("/home/nahida/kernel/sbi2/opensbi/out/platform/simemu/firmware/fw_jump.bin",  sbi_fw_mem, 0);
    // load_bin_to_ram("/mnt/d/ExtendedPart/sim_rv32_git/riscv32-simemu/config/simemu-mmu.dtb", fdt_fw_mem, 0);

    if (argc > 1)
        load_bin_to_ram(argv[1], main_mem, 0);
    else
        // load_bin_to_ram("D:\\ExtendedPart\\RiscV\\riscv-tests\\isa\\rv32um-p-mulh.bin", 0);
        // load_bin_to_ram("D:\\ExtendedPart\\RiscV\\pj1\\build\\rv_cm.bin", main_mem,  0);
        load_bin_to_ram("\\\\wsl.localhost\\Ubuntu-22.04\\home\\nahida\\kernel\\linux-6.12-rc7\\out\\arch\\riscv\\boot\\Image", main_mem, 0);
    // load_bin_to_ram("/home/nahida/kernel/linux-6.12-rc7/out/arch/riscv/boot/Image", main_mem, 0);

    // load_bin_to_ram("\\\\wsl.localhost\\Ubuntu-22.04\\home\\nahida\\kernel\\linux-6.11.7\\out\\arch\\riscv\\boot\\Image", 0);
    // load_bin_to_ram("\\\\wsl.localhost\\Ubuntu-22.04\\home\\nahida\\kernel\\linux-6.10.1\\out\\arch\\riscv\\boot\\Image", 0);
    // load_bin_to_ram("\\\\wsl.localhost\\Ubuntu-22.04\\home\\nahida\\kernel\\linux-6.10.1\\out\\arch\\riscv\\boot\\Image", 0);
    // load_bin_to_ram("\\\\wsl.localhost\\Ubuntu-22.04\\home\\nahida\\kernel\\opensbi-1.5.1\\out\\platform\\simemu\\firmware\\fw_jump.bin", MEM_SIZE - 512 * 1024);

    // load_bin_to_ram("/home/nahida/kernel/opensbi/out/platform/simemu/firmware/fw_jump.bin", MEM_SIZE - 512 * 1024);

    // load_bin_to_ram("../simemu.dtb", MEM_SIZE - 1*1048576);

    SDL_Event event;
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        fprintf(stderr, "Failed to initialize SDL: %s\n", SDL_GetError());
        return -1;
    }
    SDL_Window *window;
    window = SDL_CreateWindow("RISCV32 SimEmu", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, VIDEO_WIDTH, VIDEO_HEIGHT,
                              SDL_WINDOW_SHOWN);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture *texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ABGR8888,
        SDL_TEXTUREACCESS_STREAMING,
        VIDEO_WIDTH, VIDEO_HEIGHT);

    vdisk_init();

    int kext1 = 0, kext2 = 0, in_window = 0;

    for (int i = 0; i < NR_CPU; i++)
    {
        cpu_core_init(&cpu_core[i]);
        cpu_core[i].id = i;
        cpu_core[i].running = 1;
    }

    // for (int i = 1; i < NR_CPU; i++)
    //     cpu_core[i].running = 0;
    cpu_core[0].CSR[IDX_CSR_MIP] |= MIP_MTIP;
    cpu_core[1].CSR[IDX_CSR_MIP] |= MIP_MSIP;
    for (int i = 1; i < NR_CPU; i++)
        cpu_core[i].wfi = 1;

    usec_time_start = now_microsecond_timestamp();
#if MULTI_THREAD
    for (uptr_t i = 0; i < NR_CPU; i++)
    {
        pthread_create(&threads[i], NULL, cpu_thread, (ptr_t)i);
    }
#else
    uint64_t t0 = get_microsecond_timestamp();
#endif

    for (;;)
    {
#if !MULTI_THREAD
        for (int i = 0; i < NR_CPU; i++)
            core_step(&cpu_core[i]);
        // for (int i = NR_CPU - 1; i >= 0; i--)
        //     core_step(&cpu_core[i]);
        // core_step(&cpu_core[rand() % NR_CPU]);
        // core_step(&cpu_core[(rand() % 100) < 60]);
        // core_step(&cpu_core[0]);

        if (get_microsecond_timestamp() - t0 > 20000)
        {
            t0 = get_microsecond_timestamp();
#else
        usleep(20000);
        {
#endif
            if (unlikely(_kbhit()))
            {
                uart_send_char(_getch());
            }

            while (SDL_PollEvent(&event))
            {
                if ((event.type == SDL_KEYDOWN) || (event.type == SDL_KEYUP))
                {

                    if (event.key.keysym.scancode == SDL_SCANCODE_LCTRL)
                        kext1 = (event.type == SDL_KEYDOWN);
                    if (event.key.keysym.scancode == SDL_SCANCODE_GRAVE)
                        kext2 = (event.type == SDL_KEYDOWN);

                    if (kext1 && kext2)
                    {
                        SDL_SetWindowGrab(window, SDL_DISABLE);
                        SDL_SetRelativeMouseMode(SDL_DISABLE);
                        SDL_ShowCursor(SDL_ENABLE);
                        in_window = 0;
                        SDL_SetWindowTitle(window, "RISCV32 SimEmu");
                    }
                    else
                    {
                        virtio_send_key_evt(event.key.keysym.scancode, event.type == SDL_KEYDOWN);
                    }
                }

                if ((event.type == SDL_MOUSEMOTION))
                {
                    if (in_window)
                        virtio_send_mouse_rel_evt(
                            event.motion.xrel,
                            event.motion.yrel,
                            -1,
                            -1, 0);
                }

                if ((event.type == SDL_MOUSEWHEEL))
                {
                    if (in_window)
                        virtio_send_mouse_rel_evt(
                            0,
                            0,
                            -1,
                            -1, event.wheel.y);
                }

                if ((event.type == SDL_MOUSEBUTTONDOWN) || (event.type == SDL_MOUSEBUTTONUP))
                {
                    if (!in_window && (event.type == SDL_MOUSEBUTTONUP))
                    {
                        SDL_SetWindowGrab(window, SDL_ENABLE);
                        SDL_SetRelativeMouseMode(SDL_ENABLE);
                        SDL_ShowCursor(SDL_DISABLE);
                        in_window = 1;
                        SDL_SetWindowTitle(window, "RISCV32 SimEmu (Ctrl + ~ release mouse)");
                    }

                    if (in_window)
                        virtio_send_mouse_rel_evt(
                            0,
                            0,
                            (event.button.button == SDL_BUTTON_LEFT) && (event.type == SDL_MOUSEBUTTONDOWN),
                            (event.button.button == SDL_BUTTON_RIGHT) && (event.type == SDL_MOUSEBUTTONDOWN), 0);
                }

                if (event.type == SDL_QUIT)
                {
                    goto fin;
                }
            }
            SDL_UpdateTexture(texture, NULL, framebuffer, VIDEO_WIDTH * 4);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);

            if (uart_out_buf)
            {
                if (!(extirq_slot[IRQ_NUM_UART].pending))
                {
                    uart_out_buf = 0;
                    if (UART8250_IER & UART_IER_THRI)
                    {
                        UART8250_IIR |= UART_IIR_THRI;
                        UART8250_IIR &= ~UART_IIR_NO_INT;
                        extirq_slot[IRQ_NUM_UART].pending = 1;
                    }
                }
            }

            // plic_check_interrupt();
            // for (int i = 0; i < NR_CPU; i++)
            //     plic_check_interrupt(i);

#if !MULTI_THREAD
        }
#else
        }
#endif
    }
fin:
    exit_emu();
}
