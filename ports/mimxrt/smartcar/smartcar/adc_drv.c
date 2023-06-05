/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_adc.h"

#include "mphalport.h"

/**
 * Repeat scan feature:
 *      Conversion complete is driven by interrupt.
 *      All uses specified channels are converted one by one in sequence. Results are saved in one array.
 *      All channels are converted repeatly and old results are overwritten.
 *      Anytime user read a channel, it always returns the latest conversion value.
 * In addition, the sampling and conversion time are set as longest value, the 16 samples hw average is enabled
 **/

#define ADC_Index(base)  (((base)==ADC1)?0:1)
uint16_t    scan_enabled[2];        // This is a bitmap. Each bit represents a channel enable status.
uint16_t    scan_result[2][16];
uint8_t     scan_index[2];

void enable_channel(ADC_Type *base, int chn)
{
    adc_channel_config_t adc_channel;
    adc_channel.channelNumber = chn;
    adc_channel.enableInterruptOnConversionCompleted = true;
    ADC_SetChannelConfig(base, 0, &adc_channel);
}

void ADC1_IRQHandler(void)
{
    // uint16_t result = ADC_GetChannelConversionValue(ADC1, 0);
    scan_result[0][scan_index[0]] = ADC1->R[0];
    while (scan_enabled[0]) {      // Continue to search enabled channels
        scan_index[0]++;
        scan_index[0] &= 0xF;
        if (scan_enabled[0] & (1<<scan_index[0])) {
            ADC1->HC[0] = ADC_HC_AIEN_MASK | scan_index[0]; // enable_channel(ADC1, scan_index[0]);
            break;
        }
    }
}

void ADC2_IRQHandler(void)
{
    // uint16_t result = ADC_GetChannelConversionValue(ADC2, 0);
    scan_result[1][scan_index[1]] = ADC2->R[0];
    while (scan_enabled[1]) {       // Continue to search enabled channels
        scan_index[1]++;
        scan_index[1] &= 0xF;
        if (scan_enabled[1] & (1<<scan_index[1])) {
            ADC2->HC[0] = ADC_HC_AIEN_MASK | scan_index[1]; // enable_channel(ADC2, scan_index[1]);
            break;
        }
    }
}

void scan_adc_enabled(ADC_Type *base, uint8_t channel, bool enabled)
{
    int idx = ADC_Index(base);
    int chn = channel & 0x0F;
    uint16_t    enable_mask = 1 << chn;
    if (enabled) {
        if (channel & ~0x0F)    // This is a trick. If high nibble of channel is not 0, means enable scan mode
            scan_enabled[idx] |= enable_mask;
    }
    else
        scan_enabled[idx] &= ~enable_mask;

    if (enabled) {
        enable_channel(base, chn);
        uint32_t irqn = (idx)?ADC2_IRQn:ADC1_IRQn;
        NVIC_SetPriority(irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
        EnableIRQ(irqn);
    }
}

uint16_t scan_adc_convert(ADC_Type *base, uint8_t channel)
{
    int idx = ADC_Index(base);
    if (scan_enabled[idx]) {
        return scan_result[idx][channel];
    }

    adc_channel_config_t adc_channel;
    adc_channel.channelNumber = channel;
    adc_channel.enableInterruptOnConversionCompleted = false;

    ADC_SetChannelConfig(base, 0, &adc_channel);
    while (0U == ADC_GetChannelStatusFlags(base, 0))
        continue;
    return ADC_GetChannelConversionValue(base, 0);
}

void adc_hw_init(int id, int period, int average)
{
    if ((id <= 0) || (id > FSL_FEATURE_SOC_ADC_COUNT))
        return;     // Invalid id

    adc_config_t ADC_config_value;
    ADC_Type *ADC_base[] = ADC_BASE_PTRS;
    ADC_Type *ADC = ADC_base[id];
    id--;

    ADC_Deinit(ADC);
    mp_hal_delay_ms(10);

    // Initialize the ADC
    ADC_GetDefaultConfig(&ADC_config_value);
    ADC_config_value.enableLowPower      = true;
    ADC_config_value.enableLongSample    = true;
    ADC_config_value.samplePeriodMode    = period;  // kADC_SamplePeriodLong24Clcoks;
    ADC_Init(ADC, &ADC_config_value);

#if !(defined(FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE) && FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE)
    ADC_EnableHardwareTrigger(ADC, false);
#endif
    ADC_DoAutoCalibration(ADC);
    ADC_SetHardwareAverageConfig(ADC, average/*kADC_HardwareAverageCount16*/);
    scan_enabled[id] = 0;
    memset(&scan_result[id][0], 0, sizeof(uint16_t)*16);    // Clear the scan struct
}
