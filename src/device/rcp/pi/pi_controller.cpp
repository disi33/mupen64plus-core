/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Mupen64plus - pi_controller.c                                         *
 *   Mupen64Plus homepage: https://mupen64plus.org/                        *
 *   Copyright (C) 2014 Bobby Smiles                                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <map>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std; 
extern std::map<uint, string> registerChanges;

extern "C" {
#include <iostream>

#include "pi_controller.h"

#define M64P_CORE_PROTOTYPES 1
#include <stdint.h>
#include <string.h>

#include <stdio.h>

#include "plugin/plugin.h"

#include "api/callbacks.h"
#include "api/m64p_types.h"
#include "device/device.h"
#include "device/dd/dd_controller.h"
#include "device/memory/memory.h"
#include "device/r4300/r4300_core.h"
#include "device/rcp/mi/mi_controller.h"
#include "device/rcp/rdp/rdp_core.h"
#include "device/rcp/ri/ri_controller.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

int validate_pi_request(struct pi_controller* pi)
{
    if (pi->regs[PI_STATUS_REG] & (PI_STATUS_DMA_BUSY | PI_STATUS_IO_BUSY)) {
        pi->regs[PI_STATUS_REG] |= PI_STATUS_ERROR;
        return 0;
    }

    return 1;
}

static void dma_pi_read(struct pi_controller* pi)
{
    if (!validate_pi_request(pi))
        return;

    uint32_t cart_addr = pi->regs[PI_CART_ADDR_REG] & ~UINT32_C(1);
    uint32_t dram_addr = pi->regs[PI_DRAM_ADDR_REG] & 0xffffff;
    uint32_t length = (pi->regs[PI_RD_LEN_REG] & UINT32_C(0x00fffffe)) + 2;
    printf("READ: dma_pi_read(struct pi_controller* pi) %#008x",cart_addr);
    const uint8_t* dram = (uint8_t*)pi->ri->rdram->dram;

    const struct pi_dma_handler* handler = NULL;
    void* opaque = NULL;

    pi->get_pi_dma_handler(pi->cart, pi->dd, cart_addr, &opaque, &handler);

    if (handler == NULL) {
        DebugMessage(M64MSG_WARNING, "Unknown PI DMA read: 0x%" PRIX32 " -> 0x%" PRIX32 " (0x%" PRIX32 ")", dram_addr, cart_addr, length);
        return;
    }

    pre_framebuffer_read(&pi->dp->fb, dram_addr);

    unsigned int cycles = handler->dma_read(opaque, dram, dram_addr, cart_addr, length);

    /* Mark DMA as busy */
    pi->regs[PI_STATUS_REG] |= PI_STATUS_DMA_BUSY;

    /* schedule end of dma interrupt event */
    cp0_update_count(pi->mi->r4300);
    add_interrupt_event(&pi->mi->r4300->cp0, PI_INT, cycles);
}

void write_rom_mapping();

void printBytes(uint8_t* mem, uint32_t cartAddr);

void backupCart();
void corruptBytes(uint8_t* mem, uint32_t cartAddr, int times);
void show_interface();
void main_state_save(int format, const char *filename);

extern int   l_CurrentFrame;
extern gfx_plugin_functions gfx;
extern bool createdCartBackup;

void corrupt_if_in_range(uint8_t* mem, uint32_t proper_cart_address) ;

void log_dma_write(uint8_t* mem, uint32_t proper_cart_address, uint32_t cart_addr, uint32_t length, uint32_t dram_addr);

static void dma_pi_write(struct pi_controller* pi)
{
    if (!validate_pi_request(pi))
        return;

    uint32_t cart_addr = pi->regs[PI_CART_ADDR_REG] & ~UINT32_C(1);
    uint32_t dram_addr = pi->regs[PI_DRAM_ADDR_REG] & 0xffffff;
    uint32_t length = (pi->regs[PI_WR_LEN_REG] & UINT32_C(0x00fffffe)) + 2;
    uint8_t* dram = (uint8_t*)pi->ri->rdram->dram;

    const struct pi_dma_handler* handler = NULL;
    void* opaque = NULL;

    pi->get_pi_dma_handler(pi->cart, pi->dd, cart_addr, &opaque, &handler);

// Start CDL
    uint32_t proper_cart_address = cart_addr-0x10000000;
    struct cart_rom* cart_rom = (struct cart_rom*)opaque;
    const uint8_t* mem = cart_rom->rom;
    if (cart_rom->rom_size > 0) {
        log_dma_write(cart_rom->rom, proper_cart_address, cart_addr, length, pi->regs[PI_DRAM_ADDR_REG]);
        corrupt_if_in_range(cart_rom->rom, proper_cart_address);
    }
// End CDL
        
    if (handler == NULL) {
        DebugMessage(M64MSG_WARNING, "Unknown PI DMA write: 0x%" PRIX32 " -> 0x%" PRIX32 " (0x%" PRIX32 ")", cart_addr, dram_addr, length);
        return;
    }

    unsigned int cycles = handler->dma_write(opaque, dram, dram_addr, cart_addr, length);

    post_framebuffer_write(&pi->dp->fb, dram_addr, length);

    /* Mark DMA as busy */
    pi->regs[PI_STATUS_REG] |= PI_STATUS_DMA_BUSY;

    /* schedule end of dma interrupt event */
    cp0_update_count(pi->mi->r4300);
    add_interrupt_event(&pi->mi->r4300->cp0, PI_INT, cycles);
}


void init_pi(struct pi_controller* pi,
             pi_dma_handler_getter get_pi_dma_handler,
             struct cart* cart,
             struct dd_controller* dd,
             struct mi_controller* mi,
             struct ri_controller* ri,
             struct rdp_core* dp)
{
    pi->get_pi_dma_handler = get_pi_dma_handler;
    pi->cart = cart;
    pi->dd = dd;
    pi->mi = mi;
    pi->ri = ri;
    pi->dp = dp;
}

void poweron_pi(struct pi_controller* pi)
{
    memset(pi->regs, 0, PI_REGS_COUNT*sizeof(uint32_t));
}
void cdl_log_pi_reg_read();
void cdl_log_pi_reg_write();
void read_pi_regs(void* opaque, uint32_t address, uint32_t* value)
{
    cdl_log_pi_reg_read();
    struct pi_controller* pi = (struct pi_controller*)opaque;
    uint32_t reg = pi_reg(address);

    *value = pi->regs[reg];
}
void cdl_clear_dma_log2();
void write_pi_regs(void* opaque, uint32_t address, uint32_t value, uint32_t mask)
{
    struct pi_controller* pi = (struct pi_controller*)opaque;
    uint32_t reg = pi_reg(address);
    cdl_log_pi_reg_write();
    switch (reg)
    {
    case PI_CART_ADDR_REG:
        if (pi->dd != NULL) {
            masked_write(&pi->regs[PI_CART_ADDR_REG], value, mask);
            dd_on_pi_cart_addr_write(pi->dd, pi->regs[PI_CART_ADDR_REG]);
            return;
        }
        break;

    case PI_RD_LEN_REG:
        masked_write(&pi->regs[PI_RD_LEN_REG], value, mask);
        dma_pi_read(pi);
        return;

    case PI_WR_LEN_REG:
        masked_write(&pi->regs[PI_WR_LEN_REG], value, mask);
        // cdl_clear_dma_log2();
        dma_pi_write(pi);
        return;

    case PI_STATUS_REG:
        if (value & mask & 2)
            clear_rcp_interrupt(pi->mi, MI_INTR_PI);
        if (value & mask & 1)
            pi->regs[PI_STATUS_REG] = 0;
        return;

    case PI_BSD_DOM1_LAT_REG:
    case PI_BSD_DOM1_PWD_REG:
    case PI_BSD_DOM1_PGS_REG:
    case PI_BSD_DOM1_RLS_REG:
    case PI_BSD_DOM2_LAT_REG:
    case PI_BSD_DOM2_PWD_REG:
    case PI_BSD_DOM2_PGS_REG:
    case PI_BSD_DOM2_RLS_REG:
        masked_write(&pi->regs[reg], value & 0xff, mask);
        return;
    }

    masked_write(&pi->regs[reg], value, mask);
}
void cdl_finish_pi_dma(uint32_t a);
void pi_end_of_dma_event(void* opaque)
{
    struct pi_controller* pi = (struct pi_controller*)opaque;
    pi->regs[PI_STATUS_REG] &= ~(PI_STATUS_DMA_BUSY | PI_STATUS_IO_BUSY);

    if (pi->dd != NULL) {
        if ((pi->regs[PI_CART_ADDR_REG] == MM_DD_C2S_BUFFER) ||
            (pi->regs[PI_CART_ADDR_REG] == MM_DD_DS_BUFFER)) {
            dd_update_bm(pi->dd);
        }
    }
    cdl_finish_pi_dma(pi->regs[PI_DRAM_ADDR_REG]);

    raise_rcp_interrupt(pi->mi, MI_INTR_PI);
}
} // end extern