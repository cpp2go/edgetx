/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _HAL_H_
#define _HAL_H_

#define LCD_W                           480
#define LCD_H                           320

#define ADC_MAIN                        HAL_GIMBAL
#define ADC_SAMPTIME                    1

#define ADC_CHANNEL_STICK_RV          0  // ADS1015-0
#define ADC_CHANNEL_STICK_RH          0  // ADS1015-0
#define ADC_CHANNEL_STICK_LH          0  // ADS1015-0
#define ADC_CHANNEL_STICK_LV          0  // ADS1015-0
#define ADC_GPIO_PIN_STICK_RV         ADS1X15_REG_CONFIG_MUX_SINGLE_0  // AIN0
#define ADC_GPIO_PIN_STICK_RH         ADS1X15_REG_CONFIG_MUX_SINGLE_1  // AIN1
#define ADC_GPIO_PIN_STICK_LH         ADS1X15_REG_CONFIG_MUX_SINGLE_2  // AIN2
#define ADC_GPIO_PIN_STICK_LV         ADS1X15_REG_CONFIG_MUX_SINGLE_3  // AIN3

// For ADS1015 driver, the ADC_CHANNEL_<xyz> is defined to the chip it is wired to
#define ADC_CHANNEL_POT1                1
#define ADC_CHANNEL_POT2                1
#define ADC_GPIO_PIN_POT1               ADS1X15_REG_CONFIG_MUX_SINGLE_0
#define ADC_GPIO_PIN_POT2               ADS1X15_REG_CONFIG_MUX_SINGLE_1

#define ADC_CHANNEL_BATT                1
#define ADC_CHANNEL_RTC_BAT             1
// For ADS1015 driver, the ADC_GPIO_PIN_<xyz> is defined to the mux to be used
#define ADC_GPIO_PIN_BATT               ADS1X15_REG_CONFIG_MUX_SINGLE_2
// OpenX1 RTC uses the main battery.
#define ADC_GPIO_PIN_RTC_BAT            ADS1X15_REG_CONFIG_MUX_SINGLE_3

// For ADS1015 driver, the ADC_EXT is the total number of ads1015 chip
#define ADC_EXT                         2
// For ADS1015 driver, the ADC_EXT_SAMPTIME is the idle time (ms) between reading each channel
#define ADC_EXT_SAMPTIME                100

#define ADC_EXT_CHANNELS						\
    { ADC_CHANNEL_POT1, ADC_CHANNEL_POT2, ADC_CHANNEL_BATT, ADC_CHANNEL_RTC_BAT }

#define ADC_DIRECTION {       \
    0,0,0,0, /* gimbals */    \
    0,0,     /* pots */       \
    0,	     /* vbat */       \
    0 	     /* RTC bat */       \
}

#define ADC_VREF_PREC2                80


/*
#define KEYS_GPIO_REG_MDL            3
#define KEYS_GPIO_PIN_MDL            0
#define KEYS_GPIO_REG_DOWN           3
#define KEYS_GPIO_PIN_DOWN           1
#define KEYS_GPIO_REG_UP             3
#define KEYS_GPIO_PIN_UP             3
#define SWITCHES_GPIO_REG_B          3
#define SWITCHES_GPIO_PIN_B          2
#define SWITCHES_GPIO_REG_A          3
#define SWITCHES_GPIO_PIN_A          4
#define TRIMS_GPIO_REG_LHL           3
#define TRIMS_GPIO_PIN_LHL           5
#define TRIMS_GPIO_REG_LHR           3
#define TRIMS_GPIO_PIN_LHR           6
*/
// For MCP23017 based GPIO extension:
//  The REG is not zero-indexed since REG=0 causes some issue
//    REG for the port, 1 - G0Ax, 2 - G0Bx, 3 - G1Ax, 4 - G1Bx
//    PIN for the pin num in the port
#define KEYS_GPIO_REG_ENTER         3
#define KEYS_GPIO_PIN_ENTER         0
#define KEYS_GPIO_REG_EXIT          3
#define KEYS_GPIO_PIN_EXIT          1
#define KEYS_GPIO_REG_PAGEDN        3
#define KEYS_GPIO_PIN_PAGEDN        2
#define KEYS_GPIO_REG_TELE          3
#define KEYS_GPIO_PIN_TELE          3
#define KEYS_GPIO_REG_MDL           3
#define KEYS_GPIO_PIN_MDL           4
#define KEYS_GPIO_REG_SYS           3
#define KEYS_GPIO_PIN_SYS           5

#define TRIMS_GPIO_REG_LHR           3
#define TRIMS_GPIO_PIN_LHR           6

#define TRIMS_GPIO_REG_LVU           1
#define TRIMS_GPIO_PIN_LVU           6
#define TRIMS_GPIO_REG_LVD           1
#define TRIMS_GPIO_PIN_LVD           7
#define TRIMS_GPIO_REG_RVU           1
#define TRIMS_GPIO_PIN_RVU           5
#define TRIMS_GPIO_REG_RVD           1
#define TRIMS_GPIO_PIN_RVD           4

#define KEYS_GPIO_REG_UP             2
#define KEYS_GPIO_PIN_UP             4
#define KEYS_GPIO_REG_DOWN           2
#define KEYS_GPIO_PIN_DOWN           6

#define TRIMS_GPIO_REG_RHR           2
#define TRIMS_GPIO_PIN_RHR           1
#define TRIMS_GPIO_REG_RHL           2
#define TRIMS_GPIO_PIN_RHL           0
#define SWITCHES_GPIO_REG_C_L        2
#define SWITCHES_GPIO_PIN_C_L        3
#define SWITCHES_GPIO_REG_C_H        2
#define SWITCHES_GPIO_PIN_C_H        2
#define SWITCHES_GPIO_REG_D          2
#define SWITCHES_GPIO_PIN_D          5

#define PORTRAIT_LCD false
#define LANDSCAPE_LCD_SML false
#define LANDSCAPE_LCD_STD true
#define LANDSCAPE_LCD_LRG false

#endif // _HAL_H_
