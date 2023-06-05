/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Help texts
 */

#ifndef _INCLUDE_ADC_GRP_HELP_H
#define _INCLUDE_ADC_GRP_HELP_H

typedef enum _adc_grp_help_id {
    ADC_GRP_NEW_HELP = 0,   // Constructor
    ADC_GRP_INIT_HELP,      // init()
    ADC_GRP_ADDCH_HELP,     // addch()
    ADC_GRP_CAPTURE_HELP,   // capture()
    ADC_GRP_READ_HELP,      // read()
    ADC_GRP_GET_HELP,       // get()
    ADC_GRP_LAST_HELP
} adc_grp_help_t;

#define ADC_GRP_HELP    \
    "ADC_Group class is designed to scan channels in ADC module(s) so that\n." \
    "users do not need to actively request the conversion. All the channels are\n." \
    "converted at background one by one and users may read the results in one shot.\n." \
    "Conversion is done rotately in interrupt and saved in internal buffer.\n" \
    "-------------------------------\n" \
    "Following methods(functions) are in ADC_Group:\n"\
    " Constructor:\n" \
    "   0. class sensor.ADC_Group()\n" \
    " Methods:\n" \
    "   1. ADC_Group.init()     initialize a ADC module.\n" \
    "   2. ADC_Group.addch()    add channel(s) in conversion sequence.\n" \
    "   3. ADC_Group.capture()  capture the conversion results in a buffer.\n" \
    "   4. ADC_Group.read()     read the conversion results.\n" \
    "   5. ADC_Group.get()      return the captured conversion results.\n"

#define ADC_GRP_NEW_HELP_TEXT   \
    "class sensor.ADC_Group(grp_id)\n" \
    "   Create an object represents the ADC group.\n" \
    "    -- grp_id:    the id of group. Maximum 2 groups allowed (1 or 2).\n"

#define ADC_GRP_INIT_HELP_TEXT \
    "ADC_Group.init(adc_id, *, period=ADC_Group.PMODE3, average=ADC_Group.AVG16)\n" \
    "   This is a static method which initializes a ADC module.\n" \
    "    -- adc_id:    the id of ADC instance. This is hardware id 1 or 2.\n" \
    "    -- period:    the conversion period mode.\n" \
    "                   available values are PMODE0, PMODE1, PMODE2, PMODE3.\n" \
    "    -- average:   hardware averaging counts.\n" \
    "                   available values are AVG1, AVG4, AVG8, AVG16, AVG32.\n"

#define ADC_GRP_ADDCH_HELP_TEXT \
    "ADC_Group.addch(chn, [adc_id])\n" \
    "   This method adds one channel to the conversion group. It can be called\n" \
    "   multiple times but the number channels is limited as 8 only. The value\n" \
    "   sequence of results returned by read() / get() will be same as the calling\n" \
    "   sequence of this method\n" \
    "   -- chn:     The pin name. \n" \
    "   -- adc_id:  The ADC module ID, 1 or 2. Default is same as group ID.\n"

#define ADC_GRP_CAPTURE_HELP_TEXT \
    "ADC_Group.capture()\n" \
    "   This method copy the current conversion result to an internel buffer. The\n" \
    "   buffer will be read by get() later.\n" \
    "   The purpose of this method is to allow user to snapshot the conversion and\n" \
    "   to process the result sometime later. It is good to time sensitive scenario.\n"

#define ADC_GRP_READ_HELP_TEXT \
    "ADC_Group.read()\n" \
    "   This method returns the current conversion result in an array\n"

#define ADC_GRP_GET_HELP_TEXT \
    "ADC_Group.get()\n" \
    "   This method returns the captured conversion result in an array\n"

#endif  // _INCLUDE_ADC_GRP_HELP_H
