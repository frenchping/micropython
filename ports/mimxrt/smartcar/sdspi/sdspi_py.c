/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Philipp Ebensberger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 *  Implement the SD-SPI class
 */


#include "py/runtime.h"
#include "py/mperrno.h"
#include "extmod/vfs.h"
#include "ticks.h"

#include "mphalport.h"
#include "machine_spi.h"
#include "fsl_sdspi.h"

#define CARD_DETECTION  0

typedef struct _mimxrt_sdspi_obj_t {
    mp_obj_base_t base;
    LPSPI_Type *spi_base;
    sdspi_host_t  host;
    sdspi_card_t  card;
} mimxrt_sdspi_obj_t;
mimxrt_sdspi_obj_t  sdspi_obj;

/*******************************************************************************
 * Functions and Definitons from fsl_sdspi_disk.c
 ******************************************************************************/
#define DSPI_BUS_BAUDRATE (25000000U)    /* Transfer baudrate - 25MHz */

status_t spi_set_frequency(uint32_t frequency)
{
    uint32_t sourceClock;
    uint32_t tcrPrescaleValue;
    LPSPI_Type *base = sdspi_obj.spi_base;

    sourceClock = CLOCK_GetClockRootFreq(kCLOCK_LpspiClkRoot); // CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    LPSPI_Enable(base, false);      // Disable the LPSPI first

    /* If returns 0, indicates failed. */
    if (LPSPI_MasterSetBaudRate(base, frequency, sourceClock, &tcrPrescaleValue))
    {
        base->TCR = (base->TCR & LPSPI_TCR_PRESCALE_MASK) | LPSPI_TCR_PRESCALE(tcrPrescaleValue);
        LPSPI_Enable(base, true);   // Enable LPSPI again
        return kStatus_Success;
    }

    return kStatus_Fail;
}

status_t spi_exchange(uint8_t *in, uint8_t *out, uint32_t size)
{
    lpspi_transfer_t masterTransfer;

    masterTransfer.txData = in;
    masterTransfer.rxData = out;
    masterTransfer.dataSize = size;
    masterTransfer.configFlags = (kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous);

    status_t status;
    for (int retry = 50; retry; retry--) {
        status = LPSPI_MasterTransferBlocking(sdspi_obj.spi_base, &masterTransfer);
        if (status != kStatus_LPSPI_Busy)
            break;
    }
    return status;
}

uint32_t timer_get_current_milliseconds(void)
{
    extern volatile uint32_t systick_ms;
    return systick_ms;
}

void spi_init(LPSPI_Type *base)
{
    uint32_t sourceClock;

    lpspi_master_config_t masterConfig;

    /*Master config*/
    LPSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate   = DSPI_BUS_BAUDRATE;
    masterConfig.cpol       = kLPSPI_ClockPolarityActiveLow;
    masterConfig.cpha       = kLPSPI_ClockPhaseSecondEdge;

    CLOCK_SetDiv(kCLOCK_LpspiDiv, 7);   // Was divide by 5. Change as divide by 8
    sourceClock = CLOCK_GetClockRootFreq(kCLOCK_LpspiClkRoot);     // PING: CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    LPSPI_MasterInit(base, &masterConfig, sourceClock);
}

void sdspi_host_init(sdspi_host_t *host)
{
    /* Saves host state and callback. */
    host->busBaudRate = DSPI_BUS_BAUDRATE;
    host->setFrequency = spi_set_frequency;
    host->exchange = spi_exchange;
    host->getCurrentMilliseconds = timer_get_current_milliseconds;
}

void sdspi_disk_initialize(mimxrt_sdspi_obj_t *self)
{
    spi_init(self->spi_base);
    sdspi_host_init(&self->host);
    self->card.host = &self->host;
    status_t status = SDSPI_Init(&self->card);
    mp_printf(MICROPY_ERROR_PRINTER, "SDSPI_Init returns: %08x\n", status);
}

/*******************************************************************************
 * Functions for card detection
 ******************************************************************************/
bool sdspi_power_on(sdspi_card_t *card)
{
    return true;
}

bool sdspi_power_off(sdspi_card_t *card)
{
    return true;
}

bool sdspi_detect(sdspi_card_t *card)
{
    return true;
}

bool sdspi_state_initialized(sdspi_card_t *card)
{
    return true;
}

/*******************************************************************************
 * Functions and Definitons for Micropython
 ******************************************************************************/
extern const mp_obj_type_t sd_spi_type;

STATIC void sdspi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    LPSPI_Type *spi_base_ptr_table[] = LPSPI_BASE_PTRS;
    for (int id = 1; id < sizeof(spi_base_ptr_table)/sizeof(LPSPI_Type *); id++)
        if (self->spi_base == spi_base_ptr_table[id]) {
            mp_printf(print, "SD_SPI: on SPI%d\n", id);
            break;
        }
    mp_printf(print, "blockCount = %d, blockSize = %d\n", self->card.blockCount, self->card.blockSize);
    mp_printf(print, "CID: manufacturerID = %d,    \t applicationID=%d\n", self->card.cid.manufacturerID, self->card.cid.applicationID);
    mp_printf(print, "     productName = %s,       \t productVersion=%d\n", self->card.cid.productName, self->card.cid.productVersion);
    mp_printf(print, "     productSerialNumber = %d, manufacturerData=%d\n", self->card.cid.productSerialNumber, self->card.cid.manufacturerData);
    mp_printf(print, "CSD: csdStructure = %d\n", self->card.csd.csdStructure);
    mp_printf(print, "     dataReadAccessTime1=%d, \t dataReadAccessTime2=%d\n",  self->card.csd.dataReadAccessTime1, self->card.csd.dataReadAccessTime2);
    mp_printf(print, "     transferSpeed = %d,     \t cardCommandClass=%d\n",     self->card.csd.transferSpeed, self->card.csd.cardCommandClass);
    mp_printf(print, "     readBlockLength = %d,   \t flags=%d\n",                self->card.csd.readBlockLength, self->card.csd.flags);
    mp_printf(print, "     deviceSize = %d,        \t deviceSizeMultiplier=%d\n", self->card.csd.deviceSize, self->card.csd.deviceSizeMultiplier);
    mp_printf(print, "     eraseSectorSize = %d,   \t writeProtectGroupSize=%d\n", self->card.csd.eraseSectorSize, self->card.csd.writeProtectGroupSize);
    mp_printf(print, "     writeSpeedFactor = %d,  \t writeBlockLength=%d\n", self->card.csd.writeSpeedFactor, self->card.csd.writeBlockLength);
    mp_printf(print, "     fileFormat = %d\n", self->card.csd.fileFormat);
}

enum { SDSPI_INIT_ARG_LPSPI };

STATIC const mp_arg_t sdspi_init_allowed_args[] = {
    [SDSPI_INIT_ARG_LPSPI] = { MP_QSTR_id, MP_ARG_REQUIRED | MP_ARG_OBJ },
};

STATIC void sdspi_init_helper(mimxrt_sdspi_obj_t *self)
{
    if (!sdspi_state_initialized(&self->card)) {}

    sdspi_disk_initialize(self);
}

STATIC mp_obj_t sdspi_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);

    // Parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(sdspi_init_allowed_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(sdspi_init_allowed_args), sdspi_init_allowed_args, args);

    // Extract arguments
    machine_spi_obj_t *spi_obj = (machine_spi_obj_t*)args[SDSPI_INIT_ARG_LPSPI].u_obj;
    if (!mp_obj_is_type(spi_obj, &machine_spi_type))
        mp_raise_TypeError(MP_ERROR_TEXT("must provide a machine.SPI object"));

    mimxrt_sdspi_obj_t *self = &sdspi_obj;
    memset(self, 0, sizeof(sdspi_obj));
    sdspi_obj.base.type = &sd_spi_type;
    sdspi_obj.spi_base  = spi_obj->spi_inst;

    // Initialize SDCard Host
    sdspi_init_helper(self);

    return MP_OBJ_FROM_PTR(self);
}

// init()
STATIC mp_obj_t sdspi_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    sdspi_init_helper(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(sdspi_init_obj, 1, sdspi_init);


// deinit()
STATIC mp_obj_t sdspi_deinit(mp_obj_t self_in)
{
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    sdspi_host_init(&self->host);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdspi_deinit_obj, sdspi_deinit);

// readblocks(block_num, buf)
STATIC mp_obj_t sdspi_readblocks(mp_obj_t self_in, mp_obj_t block_num, mp_obj_t buf)
{
    mp_buffer_info_t bufinfo;
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);

    if (kStatus_Success == SDSPI_ReadBlocks(&self->card, bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len/FSL_SDSPI_DEFAULT_BLOCK_SIZE)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    } else {
        return MP_OBJ_NEW_SMALL_INT(-MP_EIO);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sdspi_readblocks_obj, sdspi_readblocks);

// present()
STATIC mp_obj_t sdspi_present(mp_obj_t self_in) {
    // mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // return mp_obj_new_bool(sdspi_detect(self));
    mp_raise_NotImplementedError(MP_ERROR_TEXT("card detection is not implemented yet"));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdspi_present_obj, sdspi_present);

STATIC mp_obj_t sdspi_info(mp_obj_t self_in) {
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // if (sdspi_detect(self) && sdspi_state_initialized(self)) {
        uint32_t log_block_nbr = self->card.blockCount;
        uint32_t log_block_size = self->card.blockSize;

        mp_obj_t tuple[2] = {
            mp_obj_new_int_from_ull((uint64_t)log_block_nbr * (uint64_t)log_block_size),
            mp_obj_new_int_from_uint(log_block_size),
        };
        return mp_obj_new_tuple(2, tuple);
    // } else {
    //     return mp_const_none;
    // }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdspi_info_obj, sdspi_info);

// writeblocks(block_num, buf)
STATIC mp_obj_t sdspi_writeblocks(mp_obj_t self_in, mp_obj_t block_num, mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);

    if (kStatus_Success == SDSPI_WriteBlocks(&self->card, bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len/FSL_SDSPI_DEFAULT_BLOCK_SIZE)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    } else {
        return MP_OBJ_NEW_SMALL_INT(-MP_EIO);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sdspi_writeblocks_obj, sdspi_writeblocks);

// ioctl(op, arg)
STATIC mp_obj_t sdspi_ioctl(mp_obj_t self_in, mp_obj_t cmd_in, mp_obj_t arg_in) {
    mimxrt_sdspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t cmd = mp_obj_get_int(cmd_in);

    switch (cmd) {
        case MP_BLOCKDEV_IOCTL_INIT: {
            if (sdspi_detect(&self->card)) {
                if (sdspi_power_on(&self->card)) {
                    return MP_OBJ_NEW_SMALL_INT(0);
                } else {
                    sdspi_power_off(&self->card);
                    return MP_OBJ_NEW_SMALL_INT(-MP_EIO);  // Initialization failed
                }
            } else {
                return MP_OBJ_NEW_SMALL_INT(-MP_EIO);  // Initialization failed
            }
        }
        case MP_BLOCKDEV_IOCTL_DEINIT: {
            if (sdspi_power_off(&self->card)) {
                return MP_OBJ_NEW_SMALL_INT(0);
            } else {
                return MP_OBJ_NEW_SMALL_INT(-MP_EIO);  // Deinitialization failed
            }
        }
        case MP_BLOCKDEV_IOCTL_SYNC: {
            return MP_OBJ_NEW_SMALL_INT(0);
        }
        case MP_BLOCKDEV_IOCTL_BLOCK_COUNT: {
            if (sdspi_state_initialized(&self->card)) {
                return MP_OBJ_NEW_SMALL_INT(self->card.blockCount);
            } else {
                return MP_OBJ_NEW_SMALL_INT(-MP_EIO);  // Card not initialized
            }
        }
        case MP_BLOCKDEV_IOCTL_BLOCK_SIZE: {
            if (sdspi_state_initialized(&self->card)) {
                return MP_OBJ_NEW_SMALL_INT(self->card.blockSize);
            } else {
                return MP_OBJ_NEW_SMALL_INT(-MP_EIO);  // Card not initialized
            }
        }
        default: // unknown command
        {
            return mp_const_none;
        }
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sdspi_ioctl_obj, sdspi_ioctl);

STATIC const mp_rom_map_elem_t sdspi_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&sdspi_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),      MP_ROM_PTR(&sdspi_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_present),     MP_ROM_PTR(&sdspi_present_obj) },
    { MP_ROM_QSTR(MP_QSTR_info),        MP_ROM_PTR(&sdspi_info_obj) },
    // block device protocol
    { MP_ROM_QSTR(MP_QSTR_readblocks),  MP_ROM_PTR(&sdspi_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&sdspi_writeblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_ioctl),       MP_ROM_PTR(&sdspi_ioctl_obj) },
};
STATIC MP_DEFINE_CONST_DICT(sdspi_locals_dict, sdspi_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    sd_spi_type,
    MP_QSTR_SD_SPI,
    MP_TYPE_FLAG_NONE,
    make_new, sdspi_obj_make_new,
    print, sdspi_print,
    locals_dict, &sdspi_locals_dict
    );

void sdspi_init0(void) {
    return;
}
