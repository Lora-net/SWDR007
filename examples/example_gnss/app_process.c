/***************************************************************************//**
 * @file
 * @brief app_process.c
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
#include <examples/common_gnss/gnss.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "app_process.h"
#include "app_init.h"
#include "app_assert.h"
#include "app_log.h"
#include "sid_api.h"
#include "sl_sidewalk_common_config.h"
#include "sl_malloc.h"
#include "app_button_press.h"
#include "sid_clock_ifc.h"
#include "sid_time_ops.h"

#if defined(SL_BOARD_SUPPORT)
#include "sl_sidewalk_board_support.h"
#endif

#if (defined(SL_FSK_SUPPORTED) || defined(SL_CSS_SUPPORTED))
#include "app_subghz_config.h"
#endif

#if defined(SL_BLE_SUPPORTED)
#include "app_ble_config.h"
#include "sl_bt_api.h"
#endif

#if defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)
#include "sl_simple_button_instances.h"
#endif

#include "timers.h"

#ifdef ALMANAC_UPDATE
#include <examples/common_gnss/almanac_update.h>
#endif /* ALMANAC_UPDATE */

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// Maximum number Queue elements
#define MSG_QUEUE_LEN       (10U)

#define UNUSED(x) (void)(x)
// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

/*******************************************************************************
 * Issue a queue event.
 *
 * @param[in] queue The queue handle which will be used ofr the event
 * @param[in] event The event to be sent
 ******************************************************************************/
static void queue_event(QueueHandle_t queue, enum event_type event);

/*******************************************************************************
 * Function to send updated counter
 *
 * @param[in] app_context The context which is applicable for the current application
 ******************************************************************************/
static void send_counter_update(app_context_t *app_context);

/*******************************************************************************
 * Function to get time
 *
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void get_time(app_context_t *context);

/*******************************************************************************
 * Function to get current MTU
 *
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void get_mtu(app_context_t *context);

/*******************************************************************************
 * Function to execute Factory reset
 *
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void factory_reset(app_context_t *context);

/*******************************************************************************
 * Method for sending sidewalk events
 *
 * @param[in] in_isr If the event shall be handled from ISR context
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void on_sidewalk_event(bool in_isr, void *context);

/*******************************************************************************
 * Callback Method for receiving sidewalk messages
 *
 * @param[in] msg_desc Message descriptor
 * @param[in] msg The received message
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context);

/*******************************************************************************
 * Callback method for the case when a sidewalk message is sent
 *
 * @param[in] msg_desc Message descriptor
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context);

/*******************************************************************************
 * Callback function if error happened during send operation
 *
 * @param[in] error The error type
 * @param[in] msg_desc Message descriptor
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context);

/*******************************************************************************
 * Callback Function to handle status changes in the Sidewalk context
 *
 * @param[in] status  new status
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void on_sidewalk_status_changed(const struct sid_status *status, void *context);

/*******************************************************************************
 * Callback function which is called from factory reset sidewalk event
 *
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void on_sidewalk_factory_reset(void *context);

/*******************************************************************************
 * Function to switch between available links
 *
 * @param[out] app_context The context which is applicable for the current application
 * @param[out] config The configuration parameters
 *
 * @returns #true           on success
 * @returns #false          on failure
 ******************************************************************************/
static bool link_switch(app_context_t *app_context, struct sid_config *config);

/*******************************************************************************
 * Function to convert link_type configuration to sidewalk stack link_mask
 *
 * @param[in] link_type  the link_type configuration to convert
 *
 * @returns link_mask  the corresponding link_mask enumeration
 ******************************************************************************/
static uint32_t link_type_to_link_mask(uint8_t link_type);

#if defined(SL_BLE_SUPPORTED)
/*******************************************************************************
 * Function to trigger the connection request towards GW
 *
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void toggle_connection_request(app_context_t *context);
#endif

/*******************************************************************************
 * Callback funciton for scan timer
 *
 * @param[in] parameter input parameter through timer
 ******************************************************************************/
static void timer_scan_callback( void* parameter );

/*******************************************************************************
 * Function to start or stop automatically scan timer
 *
 * @param[in] sec  the second value
 ******************************************************************************/
static void auto_scan_timer_set( unsigned sec );

/*******************************************************************************
 * Function to send the scan result
 *
 * @param[in] context The context which is applicable for the current application
 ******************************************************************************/
static void send_scan_result( app_context_t* app_context );

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// Queue for sending events
static QueueHandle_t g_event_queue;
#if defined(SL_BLE_SUPPORTED)
// button send update request
static bool button_send_update_req;
#endif

static app_context_t application_context;

void *gnss_scan_done_context = &application_context;

// Startup GNSS scan periodically
static TimerHandle_t timer_peri_scan_Handle = NULL;

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
static int32_t init_and_start_link(app_context_t *context, struct sid_config *config, uint32_t link_mask)
{
  if (config->link_mask != link_mask) {
    sid_error_t ret = SID_ERROR_NONE;
    if (context->sidewalk_handle != NULL) {
      ret = sid_deinit(context->sidewalk_handle);
      if (ret != SID_ERROR_NONE) {
        app_log_error("app: sid deinit failed, link:%x, err:%d", (int)link_mask, (int)ret);
        goto error;
      }
    }

    struct sid_handle *sid_handle = NULL;
    config->link_mask = link_mask;
    // Initialise sidewalk
    ret = sid_init(config, &sid_handle);
    if (ret != SID_ERROR_NONE) {
      app_log_error("app: sid init failed, link:%x, err:%d", (int)link_mask, (int)ret);
      goto error;
    }

#if (defined(SL_SIDEWALK_COMMON_DEFAULT_LINK_CONNECTION_POLICY) && (SL_SIDEWALK_COMMON_DEFAULT_LINK_CONNECTION_POLICY == SID_LINK_CONNECTION_POLICY_MULTI_LINK_MANAGER)) \
    && defined(SL_SIDEWALK_COMMON_DEFAULT_MULTI_LINK_POLICY)
    enum sid_link_connection_policy link_conn_policy = SL_SIDEWALK_COMMON_DEFAULT_LINK_CONNECTION_POLICY;
    ret = sid_option(sid_handle, SID_OPTION_SET_LINK_CONNECTION_POLICY, &link_conn_policy, sizeof(enum sid_link_connection_policy));
    if (ret != SID_ERROR_NONE && ret != SID_ERROR_NOSUPPORT) {
      app_log_error("app: set link conn policy failed: %d", ret);
      goto error;
    } else if (ret == SID_ERROR_NOSUPPORT) {
      app_log_warning("app: link conn policy change not supported on this platform");
    } else {
      app_log_info("app: link conn policy set");
    }

    enum sid_link_multi_link_policy multi_link_policy = SL_SIDEWALK_COMMON_DEFAULT_MULTI_LINK_POLICY;
    ret = sid_option(sid_handle, SID_OPTION_SET_LINK_POLICY_MULTI_LINK_POLICY, &multi_link_policy, sizeof(enum sid_link_multi_link_policy));
    if (ret != SID_ERROR_NONE && ret != SID_ERROR_NOSUPPORT) {
      app_log_error("app: set multi-link policy failed: %d", ret);
      goto error;
    } else if (ret == SID_ERROR_NOSUPPORT) {
      app_log_warning("app: multi-link policy change not supported on this platform");
    } else {
      app_log_info("app: multi-link policy set");
    }
#endif

#ifdef ALMANAC_UPDATE
  almanac_update();
#endif /* ALMANAC_UPDATE */

    // Register sidewalk handler to the application context
    context->sidewalk_handle = sid_handle;

    // Start the sidewalk stack
    ret = sid_start(sid_handle, link_mask);
    if (ret != SID_ERROR_NONE) {
      app_log_error("app: start sid failed, link:%x, err:%d", (int)link_mask, (int)ret);
      goto error;
    }
  }
  application_context.current_link_type = link_mask;
#if defined(SL_BLE_SUPPORTED)
  button_send_update_req = false;
#endif

  return 0;

  error:
  context->sidewalk_handle = NULL;
  config->link_mask = 0;
  return -1;
}

static uint32_t link_type_to_link_mask(uint8_t link_type)
{
  switch (link_type) {
    case SL_SIDEWALK_LINK_BLE:
      return SID_LINK_TYPE_1;
      break;
    case SL_SIDEWALK_LINK_FSK:
      return SID_LINK_TYPE_2;
      break;
    case SL_SIDEWALK_LINK_CSS:
      return SID_LINK_TYPE_3;
      break;
    default:
      return SID_LINK_TYPE_ANY;
      break;
  }
}

void main_thread(void * context)
{
  //Creating application context
  (void)context;

  taskENTER_CRITICAL( );
  timer_peri_scan_Handle =
        xTimerCreate( ( const char* ) "timer_peri_scan", ( TickType_t ) pdMS_TO_TICKS( DEFAULT_AUTO_SCAN_INTERVAL ),
                      ( UBaseType_t ) pdTRUE, /* periodic mode */
                      ( void* ) 1,            /* ID */
                      ( TimerCallbackFunction_t ) timer_scan_callback );
  taskEXIT_CRITICAL( );

  // Application context creation
  application_context.event_queue     = NULL;
  application_context.main_task       = NULL;
  application_context.sidewalk_handle = NULL;
  application_context.state           = STATE_INIT;

  // Register the callback functions and the context
  struct sid_event_callbacks event_callbacks =
  {
    .context           = &application_context,
    .on_event          = on_sidewalk_event,               // Called from ISR context
    .on_msg_received   = on_sidewalk_msg_received,        // Called from sid_process()
    .on_msg_sent       = on_sidewalk_msg_sent,            // Called from sid_process()
    .on_send_error     = on_sidewalk_send_error,          // Called from sid_process()
    .on_status_changed = on_sidewalk_status_changed,      // Called from sid_process()
    .on_factory_reset  = on_sidewalk_factory_reset,       // Called from sid_process()
  };

  // Set configuration parameters
  struct sid_config config =
  {
    .link_mask = 0,
    .callbacks   = &event_callbacks,
    .link_config = NULL,
    .sub_ghz_link_config = NULL,
  };

#if defined(SL_BOARD_SUPPORT)
  sl_sidewalk_board_support_init();
#endif

#if defined(SL_BOARD_SUPPORT) && (defined(SL_TEMPERATURE_SENSOR_INTERNAL) || defined(SL_TEMPERATURE_SENSOR_EXTERNAL))
  sl_sidewalk_start_temperature_timer();
#endif

  // Queue creation for the sidewalk events
  g_event_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(enum event_type));
  app_assert(g_event_queue != NULL, "app: queue creation failed");

#if (defined(SL_FSK_SUPPORTED) || defined(SL_CSS_SUPPORTED))
  config.sub_ghz_link_config = lr11xx_app_get_sub_ghz_config( );
#endif

#if defined(SL_BLE_SUPPORTED)
  config.link_config = app_get_ble_config();
  button_send_update_req = false;
#endif

#if defined(SL_BLE_SUPPORTED)
  application_context.connection_request = false;
#endif

  // Initialize to not ready state
  application_context.state = STATE_SIDEWALK_NOT_READY;

  // Assign queue to the application context
  application_context.event_queue = g_event_queue;

  if (init_and_start_link(&application_context, &config, link_type_to_link_mask(SL_SIDEWALK_COMMON_REGISTRATION_LINK)) != 0) {
    goto error;
  }

  while (1) {
    enum event_type event = EVENT_TYPE_INVALID;

    if (xQueueReceive(application_context.event_queue, &event, portMAX_DELAY) == pdTRUE) {
      // State machine for Sidewalk events
      switch (event) {
        case EVENT_TYPE_SIDEWALK:
          sid_process(application_context.sidewalk_handle);
          break;

        case EVENT_TYPE_SEND_COUNTER_UPDATE:
          app_log_info("app: send ctr update evt");

          if (application_context.state == STATE_SIDEWALK_READY) {
            send_counter_update(&application_context);
          } else {
            app_log_warning("app: sid not ready");
          }
          break;

        case EVENT_TYPE_GET_TIME:
          app_log_info("app: get time evt");

          get_time(&application_context);
          break;

        case EVENT_TYPE_GET_MTU:
          app_log_info("app: get MTU evt");

          get_mtu(&application_context);
          break;

        case EVENT_TYPE_FACTORY_RESET:
          app_log_info("app: factory reset evt");

          factory_reset(&application_context);
          break;

        case EVENT_TYPE_LINK_SWITCH:
          app_log_info("app: link switch evt");

          if (link_switch(&application_context, &config) != true) {
            goto error;
          }
          break;

        case EVENT_TYPE_REGISTERED:
          app_log_info("app: device registered evt");

          if (SL_SIDEWALK_COMMON_DEFAULT_LINK_TYPE != SL_SIDEWALK_COMMON_REGISTRATION_LINK) {
            if (init_and_start_link(&application_context, &config, link_type_to_link_mask(SL_SIDEWALK_LINK_TO_USE)) != 0) {
              goto error;
            }
          }
          break;

#if defined(SL_BLE_SUPPORTED)
        case EVENT_TYPE_CONNECTION_REQUEST:
          app_log_info("app: conn req evt");

          toggle_connection_request(&application_context);
          break;
#endif

        case EVENT_TYPE_GNSS_SCAN_START:
          app_log_info("app: start gnss scan evt");
          start_gnss_scan( &application_context );
          break;

        case EVENT_TYPE_SCAN_RESULT_SEND:
          app_log_info("app: scan result send evt");
          send_scan_result( &application_context );
          break;

        default:
          app_log_error("app: unexpected evt: %d", (int)event);
          break;
      }
    }
  }

  error:
  // If error happens deinit sidewalk
  if (application_context.sidewalk_handle != NULL) {
    sid_stop(application_context.sidewalk_handle, config.link_mask);
    sid_deinit(application_context.sidewalk_handle);
    application_context.sidewalk_handle = NULL;
  }
  app_log_error("app: fatal error");

  sid_platform_deinit();
  vTaskDelete(NULL);
}

#if defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)
/*******************************************************************************
 * Button handler callback
 * @param[in] handle Button handler
 * @note This callback is called in the interrupt context
 ******************************************************************************/
void app_button_press_cb(uint8_t button, uint8_t duration)
{
  if (button == 0) { // PB0
#if !defined(SL_CATALOG_BTN1_PRESENT) // KG100S
    if (duration != APP_BUTTON_PRESS_DURATION_SHORT) { // long press
      app_trigger_link_switch();
    } else { // short press
      app_trigger_connect_and_send();
    }
#else // All others target others than KG100S
    (void)duration;
    app_trigger_link_switch();
#endif  // !defined(SL_CATALOG_BTN1_PRESENT)
  } else { // PB1
#if !defined(SL_CATALOG_BTN1_PRESENT) //KG100S
    app_log_error("app: KG100S btn 1 not configured");
#else
    app_trigger_connect_and_send();
#endif // !defined(SL_CATALOG_BTN1_PRESENT)
  }
}
#endif

void app_trigger_connect_and_send(void)
{
  if (application_context.current_link_type & SID_LINK_TYPE_1) { // BLE
#if defined(SL_BLE_SUPPORTED)
    if (application_context.state != STATE_SIDEWALK_READY) {
      if (!button_send_update_req) {
        button_send_update_req = true;
        app_trigger_connection_request();
      } else {
        app_log_info("app: waiting for conn");
      }
    } else {
      app_trigger_send_counter_update();
    }
#endif // defined(SL_BLE_SUPPORTED)
  } else { // FSK or CSS
    app_trigger_send_counter_update();
  }
}

#if defined(SL_BLE_SUPPORTED)
static void toggle_connection_request(app_context_t *context)
{
  if (context->state == STATE_SIDEWALK_READY) {
    app_log_info("app: sid ready, operation invalid");
  } else {
    context->connection_request = true;

    app_log_info("app: set conn req");

    sid_error_t ret = sid_ble_bcn_connection_request(context->sidewalk_handle, context->connection_request);
    if (ret != SID_ERROR_NONE) {
      app_log_error("app: conn req failed: %d", (int)ret);
    }
  }
}
#endif

void app_trigger_switching_to_default_link(void)
{
  queue_event(g_event_queue, EVENT_TYPE_REGISTERED);
}

void app_trigger_link_switch(void)
{
  queue_event(g_event_queue, EVENT_TYPE_LINK_SWITCH);
}

void app_trigger_send_counter_update(void)
{
  queue_event(g_event_queue, EVENT_TYPE_SEND_COUNTER_UPDATE);
}

void app_trigger_factory_reset(void)
{
  queue_event(g_event_queue, EVENT_TYPE_FACTORY_RESET);
}

void app_trigger_get_time(void)
{
  queue_event(g_event_queue, EVENT_TYPE_GET_TIME);
}

void app_trigger_get_mtu(void)
{
  queue_event(g_event_queue, EVENT_TYPE_GET_MTU);
}

#if defined(SL_BLE_SUPPORTED)
void app_trigger_connection_request(void)
{
  queue_event(g_event_queue, EVENT_TYPE_CONNECTION_REQUEST);
}
#endif

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
void app_queue_event_send(enum event_type event)
{
    queue_event(g_event_queue, event);
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
static void queue_event(QueueHandle_t queue,
                        enum event_type event)
{
  // Check if queue_event was called from ISR
  if ((bool)xPortIsInsideInterrupt()) {
    BaseType_t task_woken = pdFALSE;

    xQueueSendFromISR(queue, &event, &task_woken);
    portYIELD_FROM_ISR(task_woken);
  } else {
    xQueueSend(queue, &event, 0);
  }
}

static void on_sidewalk_event(bool in_isr,
                              void *context)
{
  UNUSED(in_isr);
  app_context_t *app_context = (app_context_t *)context;
  // Issue sidewalk event to the queue
  queue_event(app_context->event_queue, EVENT_TYPE_SIDEWALK);
}

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc,
                                     const struct sid_msg *msg,
                                     void *context)
{
  app_context_t* app_context = ( app_context_t* ) context;

  app_log_info("app: rcvd msg (type: %d, id: %u, size: %u)", (int)msg_desc->type, msg_desc->id, msg->size);
  app_log_info("app: %s", (char *) msg->data);

  if(((int)msg_desc->type == SID_MSG_TYPE_RESPONSE) && (app_context->scan_result.app_send_msg_id == msg_desc->id)){
      if (app_context->scan_result.total_fragments > 0) {
      /* done sending gnss fragment */
        app_context->scan_result.index += app_context->scan_result.nbytes_sent_this_fragment;
        if (++app_context->scan_result.current_fragment < app_context->scan_result.total_fragments) {
            /* send next fragment */
            app_queue_event_send( EVENT_TYPE_SCAN_RESULT_SEND );
        } else {
            if(app_context->scan_result.index != app_context->scan_result.total_bytes)
              app_log_error("app: incorrect send completion count (%d %d)",
                            app_context->scan_result.index,
                            app_context->scan_result.total_bytes);
            else
              app_log_info("app: done sending fragments (%u %u)", app_context->scan_result.index,
                           app_context->scan_result.total_bytes);
        app_context->scan_result.total_fragments = 0; // indicate done sending
        app_context->scan_result.current_fragment = 0;
      }
    }
  }
}

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc,
                                 void *context)
{
  UNUSED(context);
  app_log_info( "app: sent msg (type: %d, id: %u)", ( int ) msg_desc->type, msg_desc->id );
}

static void on_sidewalk_send_error(sid_error_t error,
                                   const struct sid_msg_desc *msg_desc,
                                   void *context)
{
  app_context_t* app_context = ( app_context_t* ) context;

  app_log_error("app: send msg failed (type: %d, id: %u, err: %d)",
                (int)msg_desc->type, msg_desc->id, (int)error);

  if(app_context->scan_result.app_send_msg_id == msg_desc->id){
    if(app_context->scan_result.total_fragments > 0) {
      app_context->scan_result.total_fragments = 0;  // indicate done sending
      app_context->scan_result.current_fragment = 0;
    }
  }
}

/*******************************************************************************
 * Sidewalk Status change handler
 ******************************************************************************/
static void on_sidewalk_status_changed(const struct sid_status *status,
                                       void *context)
{
  app_context_t *app_context = (app_context_t *)context;

  static struct sid_timespec nReady_time = { 0xffffffff, 0xffffffff };
  struct sid_timespec        curr_time;
  sid_error_t                err = SID_ERROR_NONE;

  app_log_info("app: sid status changed: %d", (int)status->state);

  switch (status->state) {
      case SID_STATE_READY:
        app_context->state = STATE_SIDEWALK_READY;

        err = sid_get_mtu( app_context->sidewalk_handle, app_context->current_link_type,
                           &app_context->scan_result.mtu );
        if( err != SID_ERROR_NONE )
          app_log_error( "app: %d = sid_get_mtu()", err );
        err = sid_clock_now( SID_CLOCK_SOURCE_UPTIME, &curr_time, NULL );
        if( err != SID_ERROR_NONE ){
          app_log_error( "app: %d = sid_clock_now()", err );
        }
        else{
          sid_time_sub( &curr_time, &nReady_time );
        }
        auto_scan_timer_set( DEFAULT_AUTO_SCAN_INTERVAL );

        break;

      case SID_STATE_NOT_READY:
        app_context->state = STATE_SIDEWALK_NOT_READY;

        auto_scan_timer_set( 0 );
        err = sid_clock_now( SID_CLOCK_SOURCE_UPTIME, &nReady_time, NULL );
        if( err != SID_ERROR_NONE )
        app_log_error( "app: %d = sid_clock_now()", err );
        app_context->scan_result.total_fragments = 0;  // abort scan-result sending

        break;

      case SID_STATE_ERROR:
        app_log_error("app: sid state err: %d", (int)sid_get_error(app_context->sidewalk_handle));
        break;

      case SID_STATE_SECURE_CHANNEL_READY:
        app_context->state = STATE_SIDEWALK_SECURE_CONNECTION;
        break;
    }

  if (status->detail.registration_status == SID_STATUS_REGISTERED) {
    app_trigger_switching_to_default_link();
  }

  app_log_info("app: REG: %u, TIME: %u, LINK: %lu",
               status->detail.registration_status,
               status->detail.time_sync_status,
               status->detail.link_status_mask);

#if defined(SL_BLE_SUPPORTED)
  if (button_send_update_req && status->state == SID_STATE_READY) {
    button_send_update_req = false;
    app_trigger_send_counter_update();
  }
#endif
}

static void on_sidewalk_factory_reset(void *context)
{
  UNUSED(context);
  app_log_info("app: factory reset notif rcvd");
  // This is the callback function of the factory reset and as the last step a reset is applied.
  NVIC_SystemReset();
}

/*******************************************************************************
 * Function that returns the next available link in the order BLE -> FSK -> CSS
 * @param[in] current_link Current link
 * @return Next available link
 * @note Returns the same link if there is no available link to switch
 ******************************************************************************/
static enum sid_link_type get_next_link(enum sid_link_type current_link)
{
  // BLE -> FSK -> CSS
  if (current_link == SID_LINK_TYPE_1) {
#if defined(SL_FSK_SUPPORTED)
    app_log_info("app: switching to FSK...");
    return SID_LINK_TYPE_2;
#elif defined(SL_CSS_SUPPORTED)
    app_log_info("app: switching to CSS...");
    return SID_LINK_TYPE_3;
#endif
    return SID_LINK_TYPE_1;
  } else if (current_link == SID_LINK_TYPE_2) {
#if defined(SL_CSS_SUPPORTED)
    app_log_info("app: switching to CSS...");
    return SID_LINK_TYPE_3;
#elif defined(SL_BLE_SUPPORTED)
    app_log_info("app: switching to BLE...");
    return SID_LINK_TYPE_1;
#endif
    return SID_LINK_TYPE_2;
  } else { // (current_link == SID_LINK_TYPE_3)
#if defined(SL_BLE_SUPPORTED)
    app_log_info("app: switching to BLE...");
    return SID_LINK_TYPE_1;
#elif defined(SL_FSK_SUPPORTED)
    app_log_info("app: switching to FSK...");
    return SID_LINK_TYPE_2;
#endif
    return SID_LINK_TYPE_3;
  }
}

static bool link_switch(app_context_t *app_context, struct sid_config *config)
{
  enum sid_link_type current_link = config->link_mask;
  enum sid_link_type next_link = get_next_link(config->link_mask);

  if (current_link != next_link) {
    if (init_and_start_link(app_context, config, next_link) != 0) {
      return false;
    }
  } else {
    app_log_warning("app: only 1 link available on this platform");
  }

  return true;
}

static void send_counter_update(app_context_t *app_context)
{
  char counter_buff[10] = { 0 };

  if (app_context->state == STATE_SIDEWALK_READY
      || app_context->state == STATE_SIDEWALK_SECURE_CONNECTION) {
    app_log_info("app: sending ctr update: %d", app_context->counter);

    // buffer for str representation of integer value
    snprintf(counter_buff, sizeof(counter_buff), "%d", app_context->counter);

    struct sid_msg msg = {
      .data = (void *)counter_buff,
      .size = sizeof(counter_buff)
    };
    struct sid_msg_desc desc = {
      .type = SID_MSG_TYPE_NOTIFY,
      .link_type = SID_LINK_TYPE_ANY,
    };

    sid_error_t ret = sid_put_msg(app_context->sidewalk_handle, &msg, &desc);
    if (ret != SID_ERROR_NONE) {
      app_log_error("app: queueing data failed: %d", (int)ret);
    } else {
      app_log_info("app: queued data msg id: %u", desc.id);
    }

    app_context->counter++;
  } else {
    app_log_error("app: sid is not ready yet");
  }
}

static void factory_reset(app_context_t *context)
{
  sid_error_t ret = sid_set_factory_reset(context->sidewalk_handle);
  if (ret != SID_ERROR_NONE) {
    app_log_error("app: factory reset notif failed");

    NVIC_SystemReset();
  } else {
    app_log_info("app: wait to proceed with factory reset");
  }
}

static void get_time(app_context_t *context)
{
  struct sid_timespec curr_time;
  sid_error_t ret = sid_get_time(context->sidewalk_handle, SID_GET_GPS_TIME, &curr_time);
  if (ret == SID_ERROR_NONE) {
    app_log_info("app: curr time: %d.%d", (int) curr_time.tv_sec, (int) curr_time.tv_nsec);
  } else {
    app_log_error("app: get time failed: %d", ret);
  }
}

static void get_mtu(app_context_t *context)
{
  size_t mtu;
  sid_error_t ret = sid_get_mtu(context->sidewalk_handle, SID_LINK_TYPE_2, &mtu);
  if (ret == SID_ERROR_NONE) {
    app_log_info("app: curr mtu: %d", mtu);
  } else {
    app_log_error("app: get MTU failed: %d", ret);
  }
}

static void timer_scan_callback( void* parameter )
{
  UNUSED( parameter );

  queue_event( g_event_queue, EVENT_TYPE_GNSS_SCAN_START );
}

static void auto_scan_timer_set( unsigned sec )
{
  uint8_t ret;

  if( sec == 0 )
  {
    if( xTimerIsTimerActive( timer_peri_scan_Handle ) != pdFALSE )
    {
      ret = xTimerStop( timer_peri_scan_Handle, 0 );
      app_log_info( "app: scan timer stopped, ret = %d", ret );
    }
  }
  else
  {
    if( xTimerIsTimerActive( timer_peri_scan_Handle ) == pdFALSE )
    {
      xTimerStart( timer_peri_scan_Handle, 0 );
    }
  }
}

static void send_scan_result( app_context_t* app_context )
{
  sid_error_t                err;
  static struct sid_msg      msg;
  static struct sid_msg_desc desc;
  static uint8_t             dummy[258];
  uint8_t                    remaining   = app_context->scan_result.total_bytes - app_context->scan_result.index;
  uint8_t                    this_length = remaining;
  uint8_t                    byte_per_fragment = app_context->scan_result.mtu - 1;

  app_log_info( "app: send_scan_result index:%d total:%d cur_frag:%d mtu=%d", app_context->scan_result.index,
                app_context->scan_result.total_bytes, app_context->scan_result.current_fragment,
                app_context->scan_result.mtu );

  if( this_length > byte_per_fragment )
      this_length = byte_per_fragment;

  app_context->scan_result.nbytes_sent_this_fragment = this_length;

  dummy[0] = app_context->scan_result.current_fragment & 7;    //bit0, bit1, bit3, current fragment
  dummy[0] |= ( app_context->scan_result.total_fragments & 7 ) << 3; //bit3 bit4 bit5, total fragment
  dummy[0] |= ( app_context->scan_result.frag_type & 3 ) << 6;  //bit6, bit7, message type,  FRAGMENT_TYPE_GNSS or FRAGMENT_TYPE_WIFI
  memcpy( dummy + 1, app_context->scan_result.buffer + app_context->scan_result.index, this_length );

  app_log_info( "app: fragment" );
  app_log_hexdump_info( dummy, this_length + 1 );
  app_log_nl( );

  msg  = ( struct sid_msg ){ .data = dummy, .size = this_length + 1 };
  desc = ( struct sid_msg_desc ){
      .type      = SID_MSG_TYPE_NOTIFY,
      .link_type = SID_LINK_TYPE_ANY,
      .link_mode = SID_LINK_MODE_CLOUD,
  };

  err = sid_put_msg( app_context->sidewalk_handle, &msg, &desc );
  switch( err )
  {
  case SID_ERROR_NONE:
  {
      app_log_info( "app: queued data message id: %d", desc.id );
      app_context->scan_result.app_send_msg_id = desc.id;
      break;
  }
  case SID_ERROR_TRY_AGAIN:
  {
      app_log_error( "app: no space in the transmit queue. Try again." );
      break;
  }
  default:
      app_log_error( "app: unknown error returned from sid_put_msg() -> %d", err );
  }
}
