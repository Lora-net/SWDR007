/***************************************************************************//**
 * @file
 * @brief app_init.c
 *******************************************************************************
 * # License
 * <b>Copyright 2023 Silicon Laboratories Inc. www.silabs.com</b>
 *
 * This code was modified by Semtech
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

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------

#include "sl_system_init.h"
#include "app_log.h"
#include "app_assert.h"
#include "app_init.h"
#include "app_process.h"
#include "app_button_press.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sid_pal_common_ifc.h"
#include "sid_api.h"
#include "sl_system_kernel.h"

#if (defined(SL_FSK_SUPPORTED) || defined(SL_CSS_SUPPORTED))
#include "app_subghz_config.h"
#endif

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// Main task stack size
#define MAIN_TASK_STACK_SIZE    (2048 / sizeof(configSTACK_DEPTH_TYPE))

// -----------------------------------------------------------------------------
//                          Public Function Prototypes
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * The function is used for some basic initialization relates to the app.
 *****************************************************************************/
void app_init(void)
{
  // Initialize the Silabs system
  sl_system_init();

  // Enable button press
  app_button_press_enable();

  app_log_info("app: app started");

  platform_parameters_t platform_parameters = {
#if defined(SL_RADIO_NATIVE)
    .platform_init_parameters.radio_cfg = (radio_efr32xgxx_device_config_t *)get_radio_cfg(),
#elif defined(SL_RADIO_EXTERNAL)
    .platform_init_parameters.radio_cfg = (radio_lr11xx_device_config_t *)lr11xx_get_radio_cfg(),
#endif
  };

  sid_error_t ret_code = sid_platform_init(&platform_parameters);
  if (ret_code != SID_ERROR_NONE) {
    app_log_error("app: sid platform init err: %d", ret_code);
  }
  app_assert(ret_code == SID_ERROR_NONE, "app: sid platform init failed");

  BaseType_t status = xTaskCreate(main_thread,
                                  "MAIN",
                                  MAIN_TASK_STACK_SIZE,
                                  NULL,
                                  1,
                                  NULL);
  app_assert(status == pdPASS, "app: main task creation failed");

  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
}
