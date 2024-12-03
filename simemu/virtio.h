#pragma once
#include <stdint.h>
#include "virtio_mmio.h"
#include "virtio_ids.h"
#include "virtio_config.h"


#define NR_VIRTQ    64
#define NR_MAX_VQS  4

typedef struct __attribute__((packed)) virtq_desc {
  uint64_t addr;
  uint32_t len;
  uint16_t flags;
  uint16_t next;
}virtq_desc_t;

typedef struct __attribute__((packed)) virtq_avail {
  uint16_t flags;
  uint16_t idx;
  uint16_t ring[NR_VIRTQ];
  uint16_t used_event;
}virtq_avail_t;

struct __attribute__((packed)) virtq_used_elem {
  uint32_t id;
  uint32_t len;
};

typedef struct __attribute__((packed)) virtq_used {
  uint16_t flags;
  uint16_t idx;
  struct virtq_used_elem ring[NR_VIRTQ];
}virtq_used_t;

typedef struct virtq_t
{
    virtq_desc_t *desc_base;
    virtq_avail_t *avail_base;
    virtq_used_t *used_base;
    int ready;
    int kicked;
}virtq_t;

void dump_vq(virtq_t *vq, int j);

int virtio_input_mmio_access(uint32_t off, uint32_t len, int rd, void *dat);
void virtio_send_key_evt(int sdl_key, int down);
void virtio_send_mouse_rel_evt(int x, int y, int lb, int rb, int whl);


int virtio_blk_mmio_access(uint32_t off, uint32_t len, int rd, void *dat);
void vdisk_init();
void vdisk_uninit();


