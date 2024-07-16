/*!
 * @file      gnss.c
 *
 * @brief     GNSS scan function implementation.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <examples/common_gnss/gnss.h>
#include <stdbool.h>
#include <math.h>

#include "app_log.h"
#include "halo_lr11xx_radio.h"
#include "sid_api.h"

#include "lr11xx_radio.h"

#include "app_process.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define ASSIST_LATITUDE 31.3
#define ASSIST_LONGITUDE 121.15

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_gnss_result_t gnss_result;

static lr11xx_gnss_solver_assistance_position_t assistance_position = { ASSIST_LATITUDE, ASSIST_LONGITUDE };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void start_gnss_scan( app_context_t* app_context )
{
    struct sid_timespec curr_time;
    lr11xx_status_t     status;

    if( app_context->scan_result.total_fragments > 0 )
    {
        app_log_warning( "Waiting all fragments are sent." );
        return;
    }

    void* drv_ctx = ( void* ) lr11xx_get_drv_ctx( );
    if( lr11xx_system_wakeup( drv_ctx ) != LR11XX_STATUS_OK )
    {
        app_log_error( "scan_timer: wake-up fail" );
        return;
    }

    if( radio_enable_irq( drv_ctx ) != RADIO_ERROR_NONE )
    {
        app_log_error( "Fail to enable raido irq." );
        return;
    }
    lr11xx_system_clear_irq_status( drv_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK );

    sid_error_t ret = sid_get_time( app_context->sidewalk_handle, SID_GET_GPS_TIME, &curr_time );
    if( SID_ERROR_NONE != ret )
    {
        app_log_error( "scan_timer: sid_get_time fail %d", ret );
        return;
    }

    status = lr11xx_gnss_set_assistance_position( drv_ctx, &assistance_position );
    if( status == LR11XX_STATUS_ERROR )
    {
        app_log_error( "scan_timer: set AP fail" );
        return;
    }
    status = lr11xx_gnss_scan_assisted( drv_ctx, curr_time.tv_sec, LR11XX_GNSS_OPTION_BEST_EFFORT,
                                        LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK | LR11XX_GNSS_RESULTS_DOPPLER_MASK |
                                            LR11XX_GNSS_RESULTS_BIT_CHANGE_MASK,
                                        NB_MAX_SV );
    if( status == LR11XX_STATUS_ERROR )
    {
        app_log_error( "scan_timer: assisted scan fail" );
    }
    else
    {
        app_log_info( "scan_timer: assisted scan started %lu, %f %f", curr_time.tv_sec, assistance_position.latitude,
                      assistance_position.longitude );
    }
}

void on_gnss_scan_done( void* context )
{
    uint8_t        n_sv_detected = 0;
    app_context_t* app_context   = ( app_context_t* ) context;
    void*          drv_ctx       = ( void* ) lr11xx_get_drv_ctx( );

    lr11xx_status_t status = lr11xx_gnss_get_result_size( drv_ctx, &gnss_result.length );
    if( status != LR11XX_STATUS_OK )
    {
        app_log_error( "gnss_get_result_size fail" );
        return;
    }
    app_log_info( "result size %d", gnss_result.length );
    if( gnss_result.length >= GNSS_RESULT_SIZE )
    {
        app_log_error( "result too big %d > %d", gnss_result.length, GNSS_RESULT_SIZE );
        return;
    }

    status = lr11xx_gnss_read_results( drv_ctx, gnss_result.buffer, gnss_result.length );
    if( status != LR11XX_STATUS_OK )
    {
        app_log_error( "gnss_read_results fail" );
        return;
    }

    lr11xx_gnss_get_nb_detected_satellites( drv_ctx, &n_sv_detected );

    lr11xx_gnss_detected_satellite_t sv_detected[NB_MAX_SV] = { 0 };

    app_log_info( "on_gnss_scan_done %d SV", n_sv_detected );
    lr11xx_gnss_get_detected_satellites( drv_ctx, n_sv_detected, sv_detected );
    for( uint8_t index_sv = 0; index_sv < n_sv_detected; index_sv++ )
    {
        const lr11xx_gnss_detected_satellite_t* local_sv = &sv_detected[index_sv];
        app_log_info( "  - SV %u: CNR: %i, doppler: %i", local_sv->satellite_id, local_sv->cnr, local_sv->doppler );
    }

    app_log_info( "nav:" );
    app_log_hexdump_info( gnss_result.buffer, gnss_result.length );
    app_log_nl( );

    if( n_sv_detected > 4 )
    {
        uint8_t length          = gnss_result.length - 1;
        float   total_fragments = length / ( float ) ( app_context->scan_result.mtu - 1 );  // -1 space for header
        app_context->scan_result.total_fragments = ceil( total_fragments );
        app_log_info( "mtu %d, total fragments %u", app_context->scan_result.mtu,
                      app_context->scan_result.total_fragments );
        app_context->scan_result.index       = 1;
        app_context->scan_result.total_bytes = length + 1;
        app_context->scan_result.buffer      = gnss_result.buffer;
        app_context->scan_result.frag_type   = FRAGMENT_TYPE_GNSS;
        app_queue_event_send( EVENT_TYPE_SCAN_RESULT_SEND );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
