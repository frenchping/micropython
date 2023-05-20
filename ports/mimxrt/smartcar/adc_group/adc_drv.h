/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _INCLUDE_DRV_ADC_H
#define _INCLUDE_DRV_ADC_H


#include "fsl_adc.h"

void adc_hw_init(int id, int period, int average);
void scan_adc_enabled(ADC_Type *base, uint8_t channel, bool enabled);
uint16_t scan_adc_convert(ADC_Type *base, uint8_t channel);


#endif /* _INCLUDE_DRV_ADC_H */

