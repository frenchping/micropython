/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 *  Translate dspi functions to lpspi functions
 *  This translation is used to minimize the modification to fsl_sdspi
 */

#ifndef _INCLUDE_LPSPI_DSPI_H
#define _INCLUDE_LPSPI_DSPI_H

#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "MIMXRT1021.h"

#define BOARD_SDSPI_SPI_BASE    LPSPI4

#endif  // _INCLUDE_LPSPI_DSPI_H
