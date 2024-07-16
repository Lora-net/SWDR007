/*!
 * @file      wifi.c
 *
 * @brief     WIFI scan implementation for LR11xx
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

#include <examples/common_wifi/wifi.h>
#include <math.h>
#include <halo_lr11xx_radio.h>
#include "app_log.h"
#include "app_process.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define UNUSED( x ) ( void ) ( x )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* often one of the access points received is a mobile-AP, so get the max +1 */
#define WIFI_MAX_RESULTS ( 6 )

/**
 * @brief Minimal number of detected access point in a scan result to consider the scan valid
 */
#define WIFI_SCAN_NB_AP_MIN ( 3 )

/**
 * @brief Size in bytes of the payload tag to indicate frame format (as defined by LR1110 WiFi positioning protocol of
 * LoRaCloud)
 */
#define WIFI_TAG_SIZE ( 1 )

/**
 * @brief Size in bytes of a WiFi Access-Point address
 */
#define WIFI_AP_ADDRESS_SIZE ( 6 )

/**
 * @brief Size in bytes to store the RSSI of a detected WiFi Access-Point
 */
#define WIFI_AP_RSSI_SIZE ( 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Structure representing a single scan result
 */
typedef struct
{
    lr11xx_wifi_mac_address_t        mac_address;  //!< MAC address of the Wi-Fi access point which has been detected
    lr11xx_wifi_channel_t            channel;      //!< Channel on which the access point has been detected
    lr11xx_wifi_signal_type_result_t type;         //!< Type of Wi-Fi which has been detected
    int8_t                           rssi;         //!< Strength of the detected signal
} wifi_scan_single_result_t;

/*!
 * @brief Structure representing a collection of scan results
 */
typedef struct
{
    uint8_t                   nbr_results;                //!< Number of results
    uint32_t                  power_consumption_uah;      //!< Power consumption to acquire this set of results
    uint32_t                  timestamp;                  //!< Timestamp at which the data set has been completed
    wifi_scan_single_result_t results[WIFI_MAX_RESULTS];  //!< Buffer containing the results
} wifi_scan_all_result_t;

/**
 * @brief Wi-Fi payload format (as defined by LR1110 WiFi positioning protocol of LoRaCloud).
 */
typedef enum wifi_mw_payload_format_e
{
    WIFI_MW_PAYLOAD_MAC      = 0x00,  //!< Only the MAC addresses of the detected Access Points are sent
    WIFI_MW_PAYLOAD_MAC_RSSI = 0x01,  //!< Both MAC address and RSSI of detected Access Points are sent
} wifi_mw_payload_format_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

wifi_configuration_scan_t wifi_configuration = {
    .signal_type         = LR11XX_WIFI_TYPE_SCAN_B,
    .base.channel_mask   = 0x0421,
    .scan_mode           = LR11XX_WIFI_SCAN_MODE_BEACON,
    .base.max_result     = WIFI_MAX_RESULTS,
    .nb_scan_per_channel = 10,
    .timeout_per_scan    = 90,
    .abort_on_timeout    = true,
};

/*!
 * @brief The format of the Wi-Fi scan results to be used.
 */
static wifi_mw_payload_format_t payload_format = WIFI_MW_PAYLOAD_MAC;

/*!
 * @brief The buffer containing results to be sent over the air
 */
static uint8_t wifi_result_buffer[WIFI_TAG_SIZE + ( ( WIFI_AP_RSSI_SIZE + WIFI_AP_ADDRESS_SIZE ) * WIFI_MAX_RESULTS )];

static lr11xx_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_RESULTS];
static wifi_scan_all_result_t              wifi_results;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*******************************************************************************
 * Get the wifi scan result
 *
 * @param[in] drv_ctx The context which is applicable for the current application
 * @param[out] wifi_results Get the wifi scan result
 * @returns -1: Fail 0:OK
 ******************************************************************************/
static int smtc_wifi_get_results( void* drv_ctx, wifi_scan_all_result_t* wifi_results );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void start_wifi_scan( app_context_t* app_context )
{
    UNUSED( app_context );

    if( app_context->scan_result.total_fragments > 0 )
    {
        app_log_warning( "Waiting all fragments are sent." );
        return;
    }

    void* drv_ctx = ( void* ) lr11xx_get_drv_ctx( );

    if( lr11xx_system_wakeup( drv_ctx ) != LR11XX_STATUS_OK )
    {
        app_log_error( "scan_wifi wake-up fail" );
        return;
    }

    if( radio_enable_irq( drv_ctx ) != RADIO_ERROR_NONE )
    {
        app_log_error( "Fail to enable raido irq." );
        return;
    }
    lr11xx_system_clear_irq_status( drv_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK );

    lr11xx_status_t status =
        lr11xx_wifi_scan( drv_ctx, LR11XX_WIFI_TYPE_SCAN_B, /* const lr1110_wifi_signal_type_scan_t signal_type */
                          0x0421,                           /* const lr1110_wifi_channel_mask_t channels */
                          LR11XX_WIFI_SCAN_MODE_BEACON,     /* const lr1110_wifi_mode_t scan_mode */
                          6,                                /* const uint8_t max_results */
                          10,                               /* const uint8_t nb_scan_per_channel */
                          90,                               /* const uint16_t timeout_in_ms */
                          true                              /* const bool abort_on_timeout */
        );

    if( status != LR11XX_STATUS_OK )
    {
        app_log_error( "lr11xx_wifi_scan fail" );
        return;
    }
}

void on_wifi_scan_done( void* context )
{
    uint8_t        wifi_buffer_size = 0;
    app_context_t* app_context      = ( app_context_t* ) context;
    void*          drv_ctx          = ( void* ) lr11xx_get_drv_ctx( );
    memset( &wifi_results, 0, sizeof( wifi_results ) );
    int ret = smtc_wifi_get_results( drv_ctx, &wifi_results );
    if( ret < 0 )
    {
        app_log_error( "smtc_wifi_get_results() fail" );
        return;
    }

    /* Check if there are results to be sent */
    if( wifi_results.nbr_results < WIFI_SCAN_NB_AP_MIN )
    {
        app_log_warning( "only %u results", wifi_results.nbr_results );
        return;
    }

    /* Add the payload format tag */
    wifi_result_buffer[wifi_buffer_size] = payload_format;
    wifi_buffer_size += WIFI_TAG_SIZE;

    /* Concatenate all results in send buffer */
    for( uint8_t i = 0; i < wifi_results.nbr_results; i++ )
    {
        /* Copy Access Point RSSI address in result buffer (if requested) */
        if( payload_format == WIFI_MW_PAYLOAD_MAC_RSSI )
        {
            wifi_result_buffer[wifi_buffer_size] = wifi_results.results[i].rssi;
            wifi_buffer_size += WIFI_AP_RSSI_SIZE;
        }
        /* Copy Access Point MAC address in result buffer */
        memcpy( &wifi_result_buffer[wifi_buffer_size], wifi_results.results[i].mac_address, WIFI_AP_ADDRESS_SIZE );
        wifi_buffer_size += WIFI_AP_ADDRESS_SIZE;
    }

    app_log_info( "wifi_result_buffer:" );
    app_log_hexdump_info( wifi_result_buffer, wifi_buffer_size );
    app_log_nl( );

    float total_fragments = wifi_buffer_size / ( float ) ( app_context->scan_result.mtu - 1 );  // -1 space for header
    app_context->scan_result.total_fragments = ceil( total_fragments );
    app_log_info( "mtu %d, total fragments %u", app_context->scan_result.mtu,
                  app_context->scan_result.total_fragments );
    app_context->scan_result.index       = 0;
    app_context->scan_result.total_bytes = wifi_buffer_size;
    app_context->scan_result.buffer      = wifi_result_buffer;
    app_context->scan_result.frag_type   = FRAGMENT_TYPE_WIFI;

    app_queue_event_send( EVENT_TYPE_SCAN_RESULT_SEND );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static int smtc_wifi_get_results( void* drv_ctx, wifi_scan_all_result_t* wifi_results )
{
    lr11xx_status_t status;
    uint8_t         nb_results;
    uint8_t         max_nb_results;
    uint8_t         result_index = 0;

    status = lr11xx_wifi_get_nb_results( drv_ctx, &nb_results );
    if( status != LR11XX_STATUS_OK )
    {
        app_log_error( "lr11xx_wifi_get_nb_results() fail" );
        return -1;
    }
    app_log_info( "nb_results %u", nb_results );

    /* check if the array is big enough to hold all results */
    max_nb_results = sizeof( wifi_results_mac_addr ) / sizeof( wifi_results_mac_addr[0] );
    if( nb_results > max_nb_results )
    {
        app_log_error( "Wi-Fi scan result size exceeds %u (%u)", max_nb_results, nb_results );
        return -1;
    }

    memset( wifi_results_mac_addr, 0, sizeof wifi_results_mac_addr );
    status = lr11xx_wifi_read_basic_complete_results( drv_ctx, 0, nb_results, wifi_results_mac_addr );
    if( status != LR11XX_STATUS_OK )
    {
        app_log_error( "lr11xx_wifi_read_basic_complete_results() fail" );
        return -1;
    }

    /* add scan to results */
    for( uint8_t index = 0; index < nb_results; index++ )
    {
        const lr11xx_wifi_basic_complete_result_t* local_basic_result = &wifi_results_mac_addr[index];
        lr11xx_wifi_channel_t                      channel;
        bool                                       rssi_validity;
        lr11xx_wifi_mac_origin_t                   mac_origin_estimation;

        lr11xx_wifi_parse_channel_info( local_basic_result->channel_info_byte, &channel, &rssi_validity,
                                        &mac_origin_estimation );

        if( mac_origin_estimation != LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP )
        {
            wifi_results->results[result_index].channel = channel;

            wifi_results->results[result_index].type =
                lr11xx_wifi_extract_signal_type_from_data_rate_info( local_basic_result->data_rate_info_byte );

            memcpy( wifi_results->results[result_index].mac_address, local_basic_result->mac_address,
                    LR11XX_WIFI_MAC_ADDRESS_LENGTH );

            wifi_results->results[result_index].rssi = local_basic_result->rssi;
            {
                const uint8_t* mac = local_basic_result->mac_address;
                app_log_info( "%u) ch%u %02x %02x %02x %02x %02x %02x %ddBm", index, channel, mac[0], mac[1], mac[2],
                              mac[3], mac[4], mac[5], local_basic_result->rssi );
            }
            wifi_results->nbr_results++;
            result_index++;
        }
        else
            app_log_info( "%u) mobile-AP", index );
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
