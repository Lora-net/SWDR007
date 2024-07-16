/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file defines interface used by Semtech driver to perform platform specific operations
 * This code was modified by Semtech
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sid_pal_radio_ifc.h"
#include "sid_pal_delay_ifc.h"

#include "halo_lr11xx_radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define SEMTECH_BOOTUP_DELAY_US 40000
#define STATUS_FIELD_OFFSET_BITS 1
#define STATUS_OK_MASK ( LR11XX_SYSTEM_CMD_STATUS_OK << STATUS_FIELD_OFFSET_BITS )
#define DEFAULT_WAKEUP_DELAY 5
// Delay time to allow for any external PA/FEM turn ON/OFF
#define SEMTECH_STDBY_STATE_DELAY_US 10
#define SEMTECH_MAX_WAIT_ON_BUSY_CNT_US 40000
#define SEMTECH_BOOTLOADER_MAX_WAIT_ON_BUSY_CNT_US 400000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static sid_error_t lr11xx_wait_on_busy( const halo_drv_semtech_ctx_t* drv_ctx, unsigned max_wait_us );

static inline bool cmd_allowed_during_scan( const uint8_t* command );

static inline bool is_scan_start_command( const uint8_t* command );

static lr11xx_hal_status_t lr11xx_hal_rdwr( const halo_drv_semtech_ctx_t* context, const uint8_t* command,
                                            const uint16_t command_length, uint8_t* data, const uint16_t data_length,
                                            bool read );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    halo_drv_semtech_ctx_t* drv_ctx = ( halo_drv_semtech_ctx_t* ) context;
    assert( drv_ctx );
    assert( data );
    assert( data_length != 0 );
    assert( data_length <= drv_ctx->config->internal_buffer.size );

    const radio_lr11xx_device_config_t* config = drv_ctx->config;
    uint8_t*                            empty  = config->internal_buffer.p;
    memset( empty, 0, data_length );

    if( lr11xx_wait_on_busy( drv_ctx, SEMTECH_MAX_WAIT_ON_BUSY_CNT_US ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    int err = drv_ctx->bus_iface->xfer( drv_ctx->bus_iface, &config->bus_selector, empty, data, data_length );

    if( !( drv_ctx->last.stat1 & STATUS_OK_MASK ) && drv_ctx->last.command )
    {
        app_log_warning( "LR11xx: Command 0x%.4X failed; Stat1 0x%.2X", drv_ctx->last.command, drv_ctx->last.stat1 );
    }
    drv_ctx->last.command = 0;  // No command  - only read. Chip will complain about it

#ifdef LOCAL_DEBUG
    SID_HAL_LOG_INFO( "-----------------------------" );
    SID_HAL_LOG_INFO( "Direct read" );
    SID_HAL_LOG_HEXDUMP_INFO( data, data_length );
#endif

    return ( err == SID_ERROR_NONE ) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    if( NULL == context )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    const halo_drv_semtech_ctx_t* drv_ctx = ( halo_drv_semtech_ctx_t* ) context;

    if( sid_pal_gpio_set_direction( drv_ctx->config->gpios.power, SID_PAL_GPIO_DIRECTION_OUTPUT ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    if( sid_pal_gpio_output_mode( drv_ctx->config->gpios.power, SID_PAL_GPIO_OUTPUT_PUSH_PULL ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    if( sid_pal_gpio_write( drv_ctx->config->gpios.power, 0 ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    sid_pal_delay_us( SEMTECH_BOOTUP_DELAY_US );
    if( sid_pal_gpio_write( drv_ctx->config->gpios.power, 1 ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    sid_pal_delay_us( SEMTECH_BOOTUP_DELAY_US );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    halo_drv_semtech_ctx_t* drv_ctx = ( halo_drv_semtech_ctx_t* ) context;

    if( drv_ctx == NULL )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    if( drv_ctx->radio_state != SID_PAL_RADIO_SLEEP )
    {
        return LR11XX_HAL_STATUS_OK;
    }

    if( sid_pal_gpio_set_direction( drv_ctx->config->bus_selector.client_selector, SID_PAL_GPIO_DIRECTION_OUTPUT ) !=
        SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    if( sid_pal_gpio_write( drv_ctx->config->bus_selector.client_selector, 0 ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    sid_pal_delay_us( drv_ctx->config->wakeup_delay_us ? drv_ctx->config->wakeup_delay_us : DEFAULT_WAKEUP_DELAY );

    /* pull up NSS pin again to allow transactions */
    if( sid_pal_gpio_write( drv_ctx->config->bus_selector.client_selector, 1 ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    /* Wait for chip to be ready */
    if( lr11xx_wait_on_busy( drv_ctx, SEMTECH_MAX_WAIT_ON_BUSY_CNT_US ) != SID_ERROR_NONE )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    if( context == NULL || command == NULL || data == NULL || command_length == 0 || data_length == 0 )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    return lr11xx_hal_rdwr( context, command, command_length, data, data_length, true );
}

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    lr11xx_hal_status_t ret;
    /* For write data can be null and data length 0 */
    if( context == NULL || command == NULL || command_length == 0 )
    {
        app_log_error( "hal_write dropped" );
        return LR11XX_HAL_STATUS_ERROR;
    }

    ret = lr11xx_hal_rdwr( context, command, command_length, ( void* ) data, data_length, false );
    if( ret == LR11XX_HAL_STATUS_OK && is_scan_start_command( command ) )
    {
        const halo_drv_semtech_ctx_t* drv_ctx = ( halo_drv_semtech_ctx_t* ) context;
        if( drv_ctx->radio_state != SID_PAL_RADIO_SCAN )
        {
            halo_drv_semtech_ctx_t* cctx = lr11xx_get_drv_ctx( );
            if( drv_ctx->config->gpios.led_sniff != HALO_GPIO_NOT_CONNECTED )
            {
                sid_pal_gpio_write( drv_ctx->config->gpios.led_sniff, 1 );
            }
            if( command[0] == 0x04 )
            {  // if gnss start command
                if( drv_ctx->config->gpios.gnss_lna != HALO_GPIO_NOT_CONNECTED )
                {
                    sid_pal_gpio_write( drv_ctx->config->gpios.gnss_lna, 1 );
                }
            }
            cctx->radio_state = SID_PAL_RADIO_SCAN;
        }
    }
    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static sid_error_t lr11xx_wait_on_busy( const halo_drv_semtech_ctx_t* drv_ctx, unsigned max_wait_us )
{
    assert( drv_ctx );

    uint8_t     is_radio_busy = 0;
    uint32_t    cnt           = 0;
    sid_error_t err           = SID_ERROR_NONE;

    while( cnt++ < max_wait_us )
    {
        err = sid_pal_gpio_read( drv_ctx->config->gpios.radio_busy, &is_radio_busy );
        if( ( err == SID_ERROR_NONE ) && !is_radio_busy )
        {
            break;
        }
        sid_pal_delay_us( SEMTECH_STDBY_STATE_DELAY_US );
    }

    if( cnt >= max_wait_us )
    {
        return SID_ERROR_BUSY;
    }
    return SID_ERROR_NONE;
}

static inline bool is_scan_start_command( const uint8_t* command )
{
    if( command[0] == 0x04 )
    {                             // gnss
        if( command[1] == 0x09 )  // autonomous start
            return true;
        else if( command[1] == 0x0a )  // assisted start
            return true;
        else
            return false;
    }
    else if( command[0] == 0x03 )
    {                             // wifi
        if( command[1] == 0x00 )  // scan start
            return true;
        else
            return false;
    }
    else
        return false;
}

static inline bool cmd_allowed_during_scan( const uint8_t* command )
{
    if( command[0] == 0x01 )
    {
        if( command[1] == 0x00 || command[1] == 0x01 || command[1] == 0x14 )
            return true;                       // allow getStatus, getVersion, and clearIrq
        if( command[1] == 0x13 ) return true;  // allow SetDioIrqParams because its called from irq handler
    }
    else if( command[0] == 0x04 )
    {
        if( command[1] == 0x09 ) return true;  // allow gnssAutonomus
    }
    return false;
}

#ifdef BUFFER_USAGE_CHECK
static inline unsigned get_buf_max_usage( halo_drv_semtech_ctx_t* drv_ctx )
{
    unsigned n;
    for( n = drv_ctx->config->internal_buffer.size; n > 0; )
    {
        if( drv_ctx->config->internal_buffer.p[--n] != 0xaa ) return n;
    }
    return n;
}
#endif /* BUFFER_USAGE_CHECK */

static lr11xx_hal_status_t lr11xx_hal_rdwr( const halo_drv_semtech_ctx_t* context, const uint8_t* command,
                                            const uint16_t command_length, uint8_t* data, const uint16_t data_length,
                                            bool read )
{
    unsigned                max_wait_us;
    halo_drv_semtech_ctx_t* drv_ctx = ( halo_drv_semtech_ctx_t* ) context;
    assert( drv_ctx );

    if( drv_ctx->radio_state == SID_PAL_RADIO_SCAN && !cmd_allowed_during_scan( command ) )
    {
        app_log_warning( "dropping 0x%02X%02X during scan", command[0], command[1] );
        return LR11XX_HAL_STATUS_ERROR;
    }

    max_wait_us = SEMTECH_MAX_WAIT_ON_BUSY_CNT_US;
    if( command[0] == 0x80 ) /* if flash write in bootloader */
        max_wait_us = SEMTECH_BOOTLOADER_MAX_WAIT_ON_BUSY_CNT_US;
    if( lr11xx_wait_on_busy( drv_ctx, max_wait_us ) != SID_ERROR_NONE )
    {
        app_log_error( "rdwr start wait_on_busy 0x%02X%02X, waited %u", command[0], command[1], max_wait_us );
        return LR11XX_HAL_STATUS_ERROR;
    }

#ifdef LOCAL_DEBUG
    SID_HAL_LOG_INFO( "-----------------------------" );
    SID_HAL_LOG_INFO( ( read ) ? "Command (read):" : "Command (write):" );
    SID_HAL_LOG_HEXDUMP_INFO( command, command_length );
    if( !read && data_length > 0 )
    {
        SID_HAL_LOG_INFO( "Data:" );
        SID_HAL_LOG_HEXDUMP_INFO( data, data_length );
    }
#endif

    size_t   size = command_length;
    uint8_t* buff = drv_ctx->config->internal_buffer.p;
    memcpy( buff, command, command_length );
    if( !read && data_length > 0 )
    {
        size += data_length;
        memcpy( &buff[command_length], data, data_length );
    }

    int err = drv_ctx->bus_iface->xfer( drv_ctx->bus_iface, &drv_ctx->config->bus_selector, buff, buff, size );
    if( err != SID_ERROR_NONE )
    {
        app_log_error( "rdwr write xfer fail" );
        return LR11XX_HAL_STATUS_ERROR;
    }

    drv_ctx->last.stat1 = buff[0];
    drv_ctx->last.stat2 = buff[1];
    if( !( drv_ctx->last.stat1 & STATUS_OK_MASK ) && drv_ctx->last.command )
    {
        /* section 3.4.2: bit0 = interrupt status */
        app_log_warning( "during 0x%02X%02X, Command 0x%.4X failed; Stat1 0x%.2X", command[0], command[1],
                         drv_ctx->last.command, drv_ctx->last.stat1 );
        drv_ctx->last.failedCommand = drv_ctx->last.command;
    }

    drv_ctx->last.command = ( command[0] << 8 ) | command[1];

#ifdef LOCAL_DEBUG
    SID_HAL_LOG_INFO( "Read back" );
    SID_HAL_LOG_HEXDUMP_INFO( empty, size );
#endif

    if( !read )
    {
        return LR11XX_HAL_STATUS_OK;
    }

    if( lr11xx_wait_on_busy( drv_ctx, SEMTECH_MAX_WAIT_ON_BUSY_CNT_US ) != SID_ERROR_NONE )
    {
        app_log_error( "rdwr read busy" );
        return LR11XX_HAL_STATUS_ERROR;
    }

    size = data_length + 1;
    buff = drv_ctx->config->internal_buffer.p;
    memset( buff, 0, size );
    err = drv_ctx->bus_iface->xfer( drv_ctx->bus_iface, &drv_ctx->config->bus_selector, buff, buff, size );
    if( err != SID_ERROR_NONE )
    {
        app_log_error( "rdwr read xfer fail" );
        return LR11XX_HAL_STATUS_ERROR;
    }

    drv_ctx->last.stat1 = buff[0];
    if( !( drv_ctx->last.stat1 & STATUS_OK_MASK ) && drv_ctx->last.command )
    {
        app_log_warning( "Command rsp 0x%.4X failed; Stat1 0x%.2X", drv_ctx->last.command, drv_ctx->last.stat1 );
    }

#ifdef LOCAL_DEBUG
    SID_HAL_LOG_INFO( "Data" );
    SID_HAL_LOG_HEXDUMP_INFO( buff, size );
#endif

    memcpy( data, &buff[1], data_length );

    return LR11XX_HAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
