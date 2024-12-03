
/*  virtio input device for the virtual machine.
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

#include <string.h>
#include <stdio.h>

#include <stdint.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#endif
typedef struct
{
#ifdef _WIN32
    HANDLE file_handle;
    HANDLE map_handle;
#else
    int file_fd;
#endif
    uint8_t *mapped_data;
    size_t size;
} MappedFile;

#include "simemu.h"
#include "virtio.h"

#define VDISK_DEFAULT_SZ_MB (32)
#define VDISK_PATH "./vdisk.img"




#define VIRTIO_BLK_F_MQ		12

static MappedFile *vdisk_mapped_file = NULL;

extern uint8_t *main_mem;
static int feature_sel = 0;
static int status = VIRTIO_CONFIG_S_NEEDS_RESET;
static virtq_t vq[NR_MAX_VQS];
static int vqs = 0;
static uint32_t int_reg = 0;

#define VIRTIO_BLK_T_IN 0
#define VIRTIO_BLK_T_OUT 1

struct virtio_blk_outhdr
{
    uint32_t type;
    uint32_t ioprio;
    uint64_t sector;
};

int virtio_blk_mmio_access(uint32_t off, uint32_t len, int rd, void *dat)
{
    uint32_t tval = 0;
    uint8_t *p_val = (uint8_t *)&tval;
    if (off < 0x100)
    {
        switch (off & ~0b11)
        {
        case VIRTIO_MMIO_MAGIC_VALUE:
            memcpy(p_val, "virt", 4);
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;
        case VIRTIO_MMIO_VERSION:
            tval = 2;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;
        case VIRTIO_MMIO_DEVICE_ID:
            tval = VIRTIO_ID_BLOCK;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;
        case VIRTIO_MMIO_VENDOR_ID:
            tval = 0x11223344;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;
        case VIRTIO_MMIO_DEVICE_FEATURES_SEL:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            feature_sel = tval;
            break;

        case VIRTIO_MMIO_DEVICE_FEATURES:
            tval = 0;
            if (feature_sel)
                tval |= (1 << (VIRTIO_F_VERSION_1 - 32));
            else
                tval |= (1 << VIRTIO_BLK_F_MQ);
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;

        case VIRTIO_MMIO_STATUS:
            if (rd & READ)
            {
                tval = status;
                fetch_m(p_val, 0, len, dat);
            }
            else
            {
                store_m(p_val, 0, len, dat);
                if (tval == 0)
                    status = 0;
                else
                    status |= tval;
            }
            break;

        case VIRTIO_MMIO_QUEUE_DESC_LOW:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            printf("set desc adr:%08x\n", tval);
            vq[vqs].desc_base = (virtq_desc_t *)&main_mem[tval - MEM_BASE];
            break;
        case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            printf("set avail adr:%08x\n", tval);
            vq[vqs].avail_base = (virtq_avail_t *)&main_mem[tval - MEM_BASE];
            break;
        case VIRTIO_MMIO_QUEUE_USED_LOW:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            printf("set used adr:%08x\n", tval);
            vq[vqs].used_base = (virtq_used_t *)&main_mem[tval - MEM_BASE];
            break;

        case VIRTIO_MMIO_QUEUE_READY:
            if (rd & READ)
            {
                tval = vq[vqs].ready; // ready;
                fetch_m(p_val, 0, len, dat)
            }
            else
            {
                store_m(p_val, 0, len, dat);
                printf("set ready:%d\n", tval);
                if (tval)
                {
                    vq[vqs].ready = tval;
                    vqs++;
                }
            }
            break;

        case VIRTIO_MMIO_QUEUE_SEL:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            printf("vmmio,sel q:%d\n", tval);
            break;
        case VIRTIO_MMIO_QUEUE_NUM:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            printf("vmmio,use sizeof q:%d\n", tval);
            break;
        case VIRTIO_MMIO_QUEUE_NUM_MAX:
            tval = NR_VIRTQ;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;

        case VIRTIO_MMIO_CONFIG:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            printf("vmmio,conf set:0x%x\n", tval);
            break;

        case VIRTIO_MMIO_INTERRUPT_STATUS:
            tval = int_reg;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;

        case VIRTIO_MMIO_INTERRUPT_ACK:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            int_reg &= ~tval;
            break;

        case VIRTIO_MMIO_QUEUE_NOTIFY:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            vq[tval].kicked = 1;

            // printf("kick vq:%d\n", tval);
            // dump_vq(vq, tval);

            {

                while (vq[tval].used_base->idx != vq[tval].avail_base->idx)
                {
                    int sel_avail_descid = vq[tval].avail_base->ring[vq[tval].used_base->idx % NR_VIRTQ];
                    int req_buf_idx = vq[tval].desc_base[sel_avail_descid].next;
                    int req_res_idx = vq[tval].desc_base[req_buf_idx].next;

                    uint32_t req_mem_addr = (vq[tval].desc_base[sel_avail_descid].addr - MEM_BASE);
                    uint32_t buf_addr = (vq[tval].desc_base[req_buf_idx].addr - MEM_BASE);
                    uint32_t res_addr = (vq[tval].desc_base[req_res_idx].addr - MEM_BASE);

                    struct virtio_blk_outhdr *req = (struct virtio_blk_outhdr *)&main_mem[req_mem_addr];
                    if ((req_mem_addr <= MEM_SIZE) && (buf_addr <= MEM_SIZE) && (res_addr <= MEM_SIZE))
                    {
                        if (req->type == VIRTIO_BLK_T_IN)
                        {
                            // printf("req %d sector:%lld, sz:%d\n",vq[tval].used_base->idx
                            //     ,req->sector, vq[tval].desc_base[req_buf_idx].len);

                            if (vdisk_mapped_file)
                            {
                                memcpy(&main_mem[buf_addr],
                                       &vdisk_mapped_file->mapped_data[req->sector * 512],
                                       vq[tval].desc_base[req_buf_idx].len);
                            }
                            else
                            {
                                memset(&main_mem[buf_addr], 0xFF, vq[tval].desc_base[req_buf_idx].len);
                            }
                            vq[tval].used_base->ring[vq[tval].used_base->idx % NR_VIRTQ].len = vq[tval].desc_base[req_buf_idx].len;
                        }
                        else if (req->type == VIRTIO_BLK_T_OUT)
                        {
                            if (vdisk_mapped_file)
                            {
                                memcpy(
                                    &vdisk_mapped_file->mapped_data[req->sector * 512],
                                    &main_mem[buf_addr],
                                    vq[tval].desc_base[req_buf_idx].len);
                            }
                        }
                        else
                        {
                            printf("virtio blk, unknown req:%d\n", req->type);
                        }

                        main_mem[res_addr] = 0;
                    }
                    else
                    {
                        printf("virtio blk, req buf addr error!\n");
                    }

                    vq[tval].used_base->ring[vq[tval].used_base->idx % NR_VIRTQ].id = sel_avail_descid;
                    vq[tval].used_base->ring[vq[tval].used_base->idx % NR_VIRTQ].len++;
                    vq[tval].used_base->idx++;
                }

                // if (!extirq_slot[IRQ_NUM_VIRTIO_BLK].pending)
                {
                    int_reg |= VIRTIO_MMIO_INT_VRING;
                    extirq_slot[IRQ_NUM_VIRTIO_BLK].pending = 1;
                }
            }

            break;

        default:
            printf("virtio access:%x,%d\n", off, rd & READ);
            break;
        }
    }
    else
    {
        off -= VIRTIO_MMIO_CONFIG;
        if (off == 0)
        { // 512-byte capacity
            if (vdisk_mapped_file)
                tval = (vdisk_mapped_file->size + 511) / 512;
            else
                tval = 0;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
        } else if (off == 0x22)
        {
            tval = 1;//NR_MAX_VQS;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
        }
        else
        {
            printf("virtio access:%x,%d\n", off, rd & READ);
        }
    }
    return EXC_OK;
}

MappedFile *mmap_file(const char *filepath, size_t size)
{
    MappedFile *mapped_file = (MappedFile *)malloc(sizeof(MappedFile));
    if (!mapped_file)
    {
        perror("Failed to allocate memory for MappedFile");
        return NULL;
    }

    mapped_file->mapped_data = NULL;
    mapped_file->size = size;

#ifdef _WIN32
    mapped_file->file_handle = CreateFile(filepath, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (mapped_file->file_handle == INVALID_HANDLE_VALUE)
    {
        perror("Failed to open file");
        free(mapped_file);
        return NULL;
    }

    mapped_file->map_handle = CreateFileMapping(mapped_file->file_handle, NULL, PAGE_READWRITE, 0, size, NULL);
    if (!mapped_file->map_handle)
    {
        perror("Failed to create file mapping");
        CloseHandle(mapped_file->file_handle);
        free(mapped_file);
        return NULL;
    }

    mapped_file->mapped_data = MapViewOfFile(mapped_file->map_handle, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (!mapped_file->mapped_data)
    {
        perror("Failed to map view of file");
        CloseHandle(mapped_file->map_handle);
        CloseHandle(mapped_file->file_handle);
        free(mapped_file);
        return NULL;
    }
#else
    mapped_file->file_fd = open(filepath, O_RDWR);
    if (mapped_file->file_fd < 0)
    {
        perror("Failed to open file");
        free(mapped_file);
        return NULL;
    }

    mapped_file->mapped_data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mapped_file->file_fd, 0);
    if (mapped_file->mapped_data == MAP_FAILED)
    {
        perror("Failed to mmap file");
        close(mapped_file->file_fd);
        free(mapped_file);
        return NULL;
    }
#endif

    return mapped_file;
}

void munmap_file(MappedFile *mapped_file)
{
    if (!mapped_file)
        return;

#ifdef _WIN32
    if (mapped_file->mapped_data)
        UnmapViewOfFile(mapped_file->mapped_data);
    if (mapped_file->map_handle)
        CloseHandle(mapped_file->map_handle);
    if (mapped_file->file_handle)
        CloseHandle(mapped_file->file_handle);
#else
    if (mapped_file->mapped_data && mapped_file->mapped_data != MAP_FAILED)
        munmap(mapped_file->mapped_data, mapped_file->size);
    if (mapped_file->file_fd >= 0)
        close(mapped_file->file_fd);
#endif

    free(mapped_file);
}

void vdisk_init()
{

    size_t fsz;
    FILE *f = fopen(VDISK_PATH, "rb");
    if (!f)
    {
        f = fopen(VDISK_PATH, "wb");
        fseek(f, VDISK_DEFAULT_SZ_MB * 1048576 - 1, SEEK_SET);
        fwrite("\0", 1, 1, f);
        fsz = VDISK_DEFAULT_SZ_MB * 1048576;
    }
    else
    {
        fseek(f, 0, SEEK_END);
        fsz = ftell(f);
    }
    fclose(f);
    vdisk_mapped_file = mmap_file(VDISK_PATH, fsz);
    if (!vdisk_mapped_file)
    {
        printf("Failed to mk/mmap vdisk file\n");
    }
}

void vdisk_uninit()
{
    if (vdisk_mapped_file)
    {
        munmap_file(vdisk_mapped_file);
    }
}
