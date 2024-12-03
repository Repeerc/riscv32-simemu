
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
#include "simemu.h"
#include "virtio.h"
#include "input-event-codes.h"
#include "SDL_scancode.h"

#define INDEV_STR "simemu virtio indev"
uint32_t sdl_scancode_to_keycode(uint32_t sc);

extern uint8_t *main_mem;

static int feature_sel = 0;
static int status = VIRTIO_CONFIG_S_NEEDS_RESET;
static virtq_t vq[NR_MAX_VQS];
static int vqs = 0;
static uint32_t int_reg = 0;

static struct virtio_input_config conf;

enum virtio_input_config_select
{
    VIRTIO_INPUT_CFG_UNSET = 0x00,
    VIRTIO_INPUT_CFG_ID_NAME = 0x01,
    VIRTIO_INPUT_CFG_ID_SERIAL = 0x02,
    VIRTIO_INPUT_CFG_ID_DEVIDS = 0x03,
    VIRTIO_INPUT_CFG_PROP_BITS = 0x10,
    VIRTIO_INPUT_CFG_EV_BITS = 0x11,
    VIRTIO_INPUT_CFG_ABS_INFO = 0x12,
};

struct virtio_input_absinfo
{
    uint32_t min;
    uint32_t max;
    uint32_t fuzz;
    uint32_t flat;
    uint32_t res;
};

struct virtio_input_devids
{
    uint16_t bustype;
    uint16_t vendor;
    uint16_t product;
    uint16_t version;
};

struct __attribute__((packed)) virtio_input_config
{
    uint8_t select;
    uint8_t subsel;
    uint8_t size;
    uint8_t reserved[5];
    union
    {
        char string[128];
        uint8_t bitmap[128];
        struct virtio_input_absinfo abs;
        struct virtio_input_devids ids;
    } u;
};

typedef struct virtio_input_event
{
    uint16_t type;
    uint16_t code;
    uint32_t value;
} virtio_input_event_t;

int virtio_input_mmio_access(uint32_t off, uint32_t len, int rd, void *dat)
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
            tval = VIRTIO_ID_INPUT;
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
            // printf("set desc adr:%08x\n", tval);
            vq[vqs].desc_base = (virtq_desc_t *)&main_mem[tval - MEM_BASE];
            break;
        case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            // printf("set avail adr:%08x\n", tval);
            vq[vqs].avail_base = (virtq_avail_t *)&main_mem[tval - MEM_BASE];
            break;
        case VIRTIO_MMIO_QUEUE_USED_LOW:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            // printf("set used adr:%08x\n", tval);
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
                // printf("set ready:%d\n", tval);
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
            // printf("vmmio,sel q:%d\n", tval);
            break;
        case VIRTIO_MMIO_QUEUE_NUM:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            // printf("vmmio,use sizeof q:%d\n", tval);
            break;
        case VIRTIO_MMIO_QUEUE_NUM_MAX:
            tval = NR_VIRTQ;
            if (rd & READ)
                fetch_m(p_val, 0, len, dat);
            break;

        case VIRTIO_MMIO_CONFIG:
            if (rd & WRITE)
                store_m(p_val, 0, len, dat);
            // printf("vmmio,conf set:0x%x\n", tval);
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

            break;

        default:
            // printf("virtio access:%x,%d\n", off, rd & READ);
            break;
        }
    }
    else
    {
        off -= VIRTIO_MMIO_CONFIG;
        if (rd & WRITE)
        {
            // memset(&conf, 0, sizeof(conf));
            store_m(((uint8_t *)&conf), off, len, dat);
            // conf.size = 0;

            // printf(" sconf,sel:%x,subsel:%x\n", conf.select, conf.subsel);

            if ((conf.select == VIRTIO_INPUT_CFG_ID_NAME) && (conf.subsel == 0))
            {
                conf.size = sizeof(INDEV_STR);
                strcpy(conf.u.string, INDEV_STR);
            }
            else if ((conf.select == VIRTIO_INPUT_CFG_EV_BITS) && (conf.subsel == EV_KEY))
            {
                conf.size = (KEY_CNT + 7) / 8;
                memset(conf.u.bitmap, 0xff, conf.size);
                // memset(conf.u.bitmap, 0xff, 128 / 8);
            }
            else if ((conf.select == VIRTIO_INPUT_CFG_EV_BITS) && (conf.subsel == EV_REL))
            {
                conf.size = (REL_CNT + 7) / 8;
                memset(conf.u.bitmap, 0x00, conf.size);
                conf.u.bitmap[0] |= (1 << REL_X);
                conf.u.bitmap[0] |= (1 << REL_Y);
                conf.u.bitmap[1] |= (1 << (REL_WHEEL - 8));
            } 
            else
            {
                conf.size = 0;
            }
        }
        else
        {
            fetch_m(((uint8_t *)&conf), off, len, dat);
        }
    }
    return EXC_OK;
}

void dump_vq(virtq_t *vq, int j)
{
    virtq_desc_t *desc = vq[j].desc_base;
    virtq_used_t *used = vq[j].used_base;
    virtq_avail_t *avail = vq[j].avail_base;
    printf("desc:%d\n", j);
    for (int i = 0; i < NR_VIRTQ; i++)
    {
        printf("desc[%d]:addr=%08llx, len:%d, flag:%d, next:%d\n", i,
               desc[i].addr, desc[i].len, desc[i].flags, desc[i].next);
    }
    printf("avail:idx=%d, flag:%d :\n",
           avail->idx, avail->flags);
    for (int i = 0; i < NR_VIRTQ; i++)
    {
        printf("    avail[%d]: ring:%d\n", i,
               avail->ring[i]);
    }
    printf("used:idx:%d, flags:%d\n",
           used->idx, used->flags);
    for (int i = 0; i < NR_VIRTQ; i++)
    {
        printf("    used[%d]:ringid:%d, len:%d \n", i,
               used->ring[i].id, used->ring[i].len);
    }
}

void virtio_vqs_put(void *dat, uint32_t len)
{
    uint32_t idx = vq[0].used_base->idx % NR_VIRTQ;
    uint8_t *buf = &main_mem[(vq[0].desc_base[idx].addr - MEM_BASE)];

    if (vq[0].used_base->idx < vq[0].avail_base->idx)
    {
        vq[0].used_base->ring[idx].id = idx;
        vq[0].used_base->ring[idx].len = len;
        memcpy(buf, dat, len);
        vq[0].used_base->idx++;
    }
    else
    {
        // printf("vqsof\n");
    }
}

void virtio_send_key_evt(int sdl_key, int down)
{
    virtio_input_event_t evt;
    int key = sdl_scancode_to_keycode(sdl_key);

    if (!vqs || !vq[0].kicked)
        return;

    evt.type = EV_KEY;
    evt.code = key;
    evt.value = down;
    virtio_vqs_put(&evt, sizeof(evt));

    evt.type = EV_SYN;
    evt.code = SYN_REPORT;
    evt.value = 0;
    virtio_vqs_put(&evt, sizeof(evt));

    if (!extirq_slot[IRQ_NUM_VIRTIO_INTPUT].pending)
    {
        int_reg |= VIRTIO_MMIO_INT_VRING;
        extirq_slot[IRQ_NUM_VIRTIO_INTPUT].pending = 1;
    }
}

void virtio_send_mouse_rel_evt(int x, int y, int lb, int rb, int whl)
{
    virtio_input_event_t evt;
    static int last_lb, last_rb;
    if (!vqs || !vq[0].kicked)
        return;

    evt.type = EV_REL;
    evt.code = REL_X;
    evt.value = x;
    virtio_vqs_put(&evt, sizeof(evt));

    evt.type = EV_REL;
    evt.code = REL_Y;
    evt.value = y;
    virtio_vqs_put(&evt, sizeof(evt));

    evt.type = EV_REL;
    evt.code = REL_WHEEL;
    evt.value = whl;
    virtio_vqs_put(&evt, sizeof(evt));

    if ((lb != -1) && (last_lb != lb))
    {
        evt.type = EV_KEY;
        evt.code = BTN_LEFT;
        evt.value = lb;
        virtio_vqs_put(&evt, sizeof(evt));
        last_lb = lb;
    }

    if ((rb != -1) && (last_rb != rb))
    {
        evt.type = EV_KEY;
        evt.code = BTN_RIGHT;
        evt.value = rb;
        virtio_vqs_put(&evt, sizeof(evt));
        last_rb = rb;
    }

    evt.type = EV_SYN;
    evt.code = SYN_REPORT;
    evt.value = 0;
    virtio_vqs_put(&evt, sizeof(evt));

    if (!extirq_slot[IRQ_NUM_VIRTIO_INTPUT].pending)
    {
        int_reg |= VIRTIO_MMIO_INT_VRING;
        extirq_slot[IRQ_NUM_VIRTIO_INTPUT].pending = 1;
    }
}

uint32_t sdl_scancode_to_keycode(uint32_t sc)
{
    // clang-format off
    switch (sc)
    {
    case SDL_SCANCODE_A: return KEY_A;
    case SDL_SCANCODE_B: return KEY_B;
    case SDL_SCANCODE_C: return KEY_C;
    case SDL_SCANCODE_D: return KEY_D;
    case SDL_SCANCODE_E: return KEY_E;
    case SDL_SCANCODE_F: return KEY_F;
    case SDL_SCANCODE_G: return KEY_G;
    case SDL_SCANCODE_H: return KEY_H;
    case SDL_SCANCODE_I: return KEY_I;
    case SDL_SCANCODE_J: return KEY_J;
    case SDL_SCANCODE_K: return KEY_K;
    case SDL_SCANCODE_L: return KEY_L;
    case SDL_SCANCODE_M: return KEY_M;
    case SDL_SCANCODE_N: return KEY_N;
    case SDL_SCANCODE_O: return KEY_O;
    case SDL_SCANCODE_P: return KEY_P;
    case SDL_SCANCODE_Q: return KEY_Q;
    case SDL_SCANCODE_R: return KEY_R;
    case SDL_SCANCODE_S: return KEY_S;
    case SDL_SCANCODE_T: return KEY_T;
    case SDL_SCANCODE_U: return KEY_U;
    case SDL_SCANCODE_V: return KEY_V;
    case SDL_SCANCODE_W: return KEY_W;
    case SDL_SCANCODE_X: return KEY_X;
    case SDL_SCANCODE_Y: return KEY_Y;
    case SDL_SCANCODE_Z: return KEY_Z;

    case SDL_SCANCODE_1: return KEY_1;
    case SDL_SCANCODE_2: return KEY_2;
    case SDL_SCANCODE_3: return KEY_3;
    case SDL_SCANCODE_4: return KEY_4;
    case SDL_SCANCODE_5: return KEY_5;
    case SDL_SCANCODE_6: return KEY_6;
    case SDL_SCANCODE_7: return KEY_7;
    case SDL_SCANCODE_8: return KEY_8;
    case SDL_SCANCODE_9: return KEY_9;
    case SDL_SCANCODE_0: return KEY_0;

    case SDL_SCANCODE_ESCAPE: return KEY_ESC;
    case SDL_SCANCODE_BACKSPACE: return KEY_BACKSPACE;
    case SDL_SCANCODE_TAB: return KEY_TAB;
    case SDL_SCANCODE_SPACE: return KEY_SPACE;
    case SDL_SCANCODE_RETURN   : return KEY_ENTER;

    case SDL_SCANCODE_MINUS: return KEY_MINUS;
    case SDL_SCANCODE_EQUALS: return KEY_EQUAL;
    case SDL_SCANCODE_LEFTBRACKET: return KEY_LEFTBRACE;
    case SDL_SCANCODE_RIGHTBRACKET: return KEY_RIGHTBRACE;
    case SDL_SCANCODE_BACKSLASH: return KEY_BACKSLASH;

    case SDL_SCANCODE_SEMICOLON: return KEY_SEMICOLON;
    case SDL_SCANCODE_APOSTROPHE: return KEY_APOSTROPHE;
    case SDL_SCANCODE_GRAVE: return KEY_GRAVE;
    case SDL_SCANCODE_COMMA: return KEY_COMMA;
    case SDL_SCANCODE_PERIOD: return KEY_DOT;
    case SDL_SCANCODE_SLASH: return KEY_SLASH;
    case SDL_SCANCODE_CAPSLOCK: return KEY_CAPSLOCK;

    case SDL_SCANCODE_F1 : return KEY_F1 ;
    case SDL_SCANCODE_F2 : return KEY_F2 ;
    case SDL_SCANCODE_F3 : return KEY_F3 ;
    case SDL_SCANCODE_F4 : return KEY_F4 ;
    case SDL_SCANCODE_F5 : return KEY_F5 ;
    case SDL_SCANCODE_F6 : return KEY_F6 ;
    case SDL_SCANCODE_F7 : return KEY_F7 ;
    case SDL_SCANCODE_F8 : return KEY_F8 ;
    case SDL_SCANCODE_F9 : return KEY_F9 ;
    case SDL_SCANCODE_F10: return KEY_F10;
    case SDL_SCANCODE_F11: return KEY_F11;
    case SDL_SCANCODE_F12: return KEY_F12;

    case SDL_SCANCODE_PRINTSCREEN : return KEY_PRINT ;
    case SDL_SCANCODE_SCROLLLOCK  : return KEY_SCROLLLOCK ;
    case SDL_SCANCODE_PAUSE : return KEY_PAUSE ;
    case SDL_SCANCODE_INSERT: return KEY_INSERT ;
    
    case SDL_SCANCODE_HOME : return KEY_HOME;
    case SDL_SCANCODE_PAGEUP : return KEY_PAGEUP;
    case SDL_SCANCODE_DELETE : return KEY_DELETE;
    case SDL_SCANCODE_END : return KEY_END;
    case SDL_SCANCODE_PAGEDOWN: return KEY_PAGEDOWN;
    case SDL_SCANCODE_RIGHT : return KEY_RIGHT;
    case SDL_SCANCODE_LEFT : return KEY_LEFT;
    case SDL_SCANCODE_DOWN : return KEY_DOWN;
    case SDL_SCANCODE_UP  : return KEY_UP;

    case SDL_SCANCODE_NUMLOCKCLEAR  : return KEY_NUMLOCK;

    case SDL_SCANCODE_KP_DIVIDE   : return KEY_KPSLASH;
    case SDL_SCANCODE_KP_MULTIPLY  : return KEY_KPASTERISK;
    case SDL_SCANCODE_KP_MINUS    : return KEY_KPMINUS;
    case SDL_SCANCODE_KP_PLUS   : return KEY_KPPLUS;
    case SDL_SCANCODE_KP_ENTER   : return KEY_KPENTER;
    case SDL_SCANCODE_KP_1  : return KEY_KP1;
    case SDL_SCANCODE_KP_2  : return KEY_KP2;
    case SDL_SCANCODE_KP_3  : return KEY_KP3;
    case SDL_SCANCODE_KP_4  : return KEY_KP4;
    case SDL_SCANCODE_KP_5  : return KEY_KP5;
    case SDL_SCANCODE_KP_6  : return KEY_KP6;
    case SDL_SCANCODE_KP_7  : return KEY_KP7;
    case SDL_SCANCODE_KP_8  : return KEY_KP8;
    case SDL_SCANCODE_KP_9  : return KEY_KP9;
    case SDL_SCANCODE_KP_0  : return KEY_KP0;
    case SDL_SCANCODE_KP_PERIOD : return KEY_KPDOT;

    case SDL_SCANCODE_LCTRL  : return KEY_LEFTCTRL;
    case SDL_SCANCODE_LSHIFT : return KEY_LEFTSHIFT;
    case SDL_SCANCODE_LALT : return KEY_LEFTALT;
    case SDL_SCANCODE_LGUI : return KEY_LEFTMETA;
    case SDL_SCANCODE_RCTRL  : return KEY_RIGHTCTRL;
    case SDL_SCANCODE_RSHIFT : return KEY_RIGHTSHIFT;
    case SDL_SCANCODE_RALT : return KEY_RIGHTALT;
    case SDL_SCANCODE_RGUI : return KEY_RIGHTMETA;
    default:
        return 0;
    }

    // clang-format on
}
