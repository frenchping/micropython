
/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 *  To explor the machine.SPI internal structure
 *  Note: This must be the same as defined in port/mimxrt/machine_spi.c
 */

#include "fsl_lpspi.h"

#ifndef _INCLUDE_MACHINE_SPI_H
#define _INCLUDE_MACHINE_SPI_H

extern const mp_obj_type_t machine_spi_type;

typedef struct _machine_spi_obj_t {
    mp_obj_base_t base;
    uint8_t spi_id;
    uint8_t mode;
    uint8_t spi_hw_id;
    bool transfer_busy;
    LPSPI_Type *spi_inst;
    lpspi_master_config_t *master_config;
} machine_spi_obj_t;


#endif  // _INCLUDE_MACHINE_SPI_H
