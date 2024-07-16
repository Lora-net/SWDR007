/***************************************************************************//**
 * @file
 * @brief app_gpio_config.c
 *******************************************************************************
 * # License
 * <b>Copyright 2023 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include "gpio.h"
#include "app_gpio_config.h"

struct GPIO_LookupItem gpio_lookup_table[] =
{
[SL_PIN_BUSY] =  { .GPIO_Port    = SL_BUSY_PORT,
                            .Pin          = SL_BUSY_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },


//[SL_PIN_ANTSW] =  { .GPIO_Port    = SL_ANTSW_PORT,
//                            .Pin          = SL_ANTSW_PIN,
//                            .callback     = NULL,
//                            .callbackarg  = NULL,
//                            .mode         = gpioModeDisabled,
//                            .irq          = { .falling = false, .rising = false } },


[SL_PIN_DIO] =  { .GPIO_Port    = SL_DIO_PORT,
                            .Pin          = SL_DIO_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },


[SL_PIN_NRESET] =  { .GPIO_Port    = SL_NRESET_PORT,
                            .Pin          = SL_NRESET_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },


[SL_PIN_NSS] =  { .GPIO_Port    = SL_SX_CS_PORT,
                            .Pin          = SL_SX_CS_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },

#ifdef LR11XX_E707
[SL_PIN_GNSS_LNA] =  { .GPIO_Port    = SL_GNSS_LNA_PORT,
                            .Pin          = SL_GNSS_LNA_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },

[SL_PIN_LED_RX] =  { .GPIO_Port    = SL_LED_RX_PORT,
                            .Pin          = SL_LED_RX_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },

[SL_PIN_LED_TX] =  { .GPIO_Port    = SL_LED_TX_PORT,
                            .Pin          = SL_LED_TX_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },

[SL_PIN_LED_SNIFFING] =  { .GPIO_Port    = SL_LED_SNIFFING_PORT,
                            .Pin          = SL_LED_SNIFFING_PIN,
                            .callback     = NULL,
                            .callbackarg  = NULL,
                            .mode         = gpioModeDisabled,
                            .irq          = { .falling = false, .rising = false } },
#endif

};
