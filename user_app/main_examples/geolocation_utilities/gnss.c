/*!
 * \file      gnss.c
 *
 * \brief     functions for GNSS example
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_hal_gpio.h"
#include "modem_pinout.h"

#include "device_management_defs.h"
#include "modem_utilities.h"
#include "lr1110_types.h"
#include "lr1110_gnss.h"
#include "lr1110_system.h"
#include "gnss.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint16_t gnss_last_rslt_size;
static uint8_t  gnss_last_rslt_buffer[254];  // the maximum length is 254 for each streaming
static uint16_t gnss_temp_rslt_size;
static uint8_t  gnss_temp_rslt_buffer[254];  // temp buffer for gnss result

extern const lr1110_gnss_almanac_full_update_bytestream_t full_almanac;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static bool start_assisted_gnss_scan( void );
static bool start_singleframe_assisted_gnss_scan( lr1110_gnss_date_t date );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*!
 * \brief Init for gnss scan
 *
 */
bool gnss_init( const lr1110_gnss_solver_assistance_position_t assistance_position )
{
    /*
    lr1110_status_t status = lr1110_gnss_set_constellations_to_use( NULL, LR1110_GNSS_BEIDOU_MASK );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to set constellations\n" );
        return false;
    }
    */

    if( lr1110_gnss_set_assistance_position( NULL, &assistance_position ) != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set assistance position\n" );
        return false;
    }

    return true;
}

/**
 * @brief Start a GNSS scan and stream data if scan is a success
 *
 * Remarks: modem_suspend_radio_access must be called before and modem_resume_radio_access after
 */
bool scan_and_stream_gnss( uint8_t stack_id, uint8_t port )
{
    // Enable LNA for GNSS with LR1110 Evaluation board with passive antenna
    hal_gpio_set_value( RADIO_LNA_CTRL, 1 );

    // Start a GNSS assisted scan
    bool success = start_assisted_gnss_scan( );

    // Disable LNA
    hal_gpio_set_value( RADIO_LNA_CTRL, 0 );

    // Set LFclk to default value
    lr1110_system_cfg_lfclk( NULL, LR1110_SYSTEM_LFCLK_RC, true );

    if( success == true )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "Start to stream GNSS result\n" );

        gnss_last_rslt_buffer[0] = TAG_NAV_MF;
        if( smtc_modem_stream_add_data( stack_id, port, gnss_last_rslt_buffer,
                                        ( uint8_t )gnss_last_rslt_size ) != RC_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Failed to stream gnss data\n" );
        }
    }

    return success;
}

/**
 * @brief Start a GNSS assisted scan and display result on UART
 *
 * Remarks: modem_suspend_radio_access must be called before and modem_resume_radio_access after
 *
 */
static bool start_assisted_gnss_scan( void )
{
    bool                  status;
    lr1110_gnss_date_t    date;
    int                   n_frames = 4;  // one multi-frame request includes 4 NAVs

    if( smtc_modem_get_time( &date ) != RC_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to get time\n" );
        return false;
    }

    if( date == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "No time available\n" );  // time is required for assisted scan
        return false;
    }

    status = lr1110_gnss_set_constellations_to_use( NULL, LR1110_GNSS_GPS_MASK | LR1110_GNSS_BEIDOU_MASK );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to set constellations\n" );
        return false;
    }

    status = lr1110_gnss_set_constellations_to_use( NULL, LR1110_GNSS_GPS_MASK | LR1110_GNSS_BEIDOU_MASK );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to set constellations\n" );
        return false;
    }

    gnss_last_rslt_size = 2;  // the 1st byte is for Tag type (TAG_NAV_MF); 2nd for the total payload length
    gnss_last_rslt_buffer[1] = (uint8_t) gnss_last_rslt_size;
    for( int n = 0; n < n_frames; n++ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "DBG: %d, n=%d\n", __LINE__, n+1 );

        status = start_singleframe_assisted_gnss_scan( date );
        if( status != true )
        {
            if( n > 0 )
                return true;
            else
                return false;
        }
        SMTC_MODEM_HAL_TRACE_WARNING( "DBG: %d\n", __LINE__ );

        // copy payload and update gnss_last_rslt_size
        // the 1st "1" is the byte for payload length; the "-1" because we'll skip the first byte of the payload
        if( gnss_last_rslt_size + 1 + ( gnss_temp_rslt_size - 1 ) > 254 )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "GNSS payload too long to fit in gnss_last_rslt_buffer. Return now\n" );
            return true;
        }

        // we skip the first byte of the payload because it's useless for geolocation resolving and confusing
        gnss_last_rslt_buffer[gnss_last_rslt_size++] = gnss_temp_rslt_size - 1;
        memcpy1(gnss_last_rslt_buffer + gnss_last_rslt_size, gnss_temp_rslt_buffer + 1, gnss_temp_rslt_size - 1);
        gnss_last_rslt_size += gnss_temp_rslt_size - 1;
        gnss_last_rslt_buffer[1] = (uint8_t) gnss_last_rslt_size;
        SMTC_MODEM_HAL_TRACE_INFO( "n = %d, gnss_last_rslt_size = %d\n", n, gnss_last_rslt_size );
    }

    return true;
}

static bool start_singleframe_assisted_gnss_scan( lr1110_gnss_date_t date )
{
    SMTC_MODEM_HAL_TRACE_INFO( "Start GNSS assisted scan\n" );

    lr1110_gnss_detected_satellite_t gnss_last_sat_detect[24];
    lr1110_status_t                  status;
    uint8_t                          nb_satellite_detected;

    // Start a GNSS assisted scan
    status = lr1110_gnss_scan_assisted(
        NULL, date, LR1110_GNSS_OPTION_DEFAULT,
        LR1110_GNSS_IRQ_PSEUDO_RANGE_MASK, 10 );

    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to start an assisted GNSS scan\n" );
        return false;
    }

    status = lr1110_gnss_get_nb_detected_satellites( NULL, &nb_satellite_detected );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to get nb detected satellite\n" );
        return false;
    }
    else if( nb_satellite_detected == 0 )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "No satellite found\n" );
        return false;
    }
    else if( nb_satellite_detected > sizeof( gnss_last_sat_detect ) )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "Satellite buffer is too small to accept %d. Will only get %d\n",
                                   nb_satellite_detected, sizeof( gnss_last_sat_detect ) );
        nb_satellite_detected = sizeof( gnss_last_sat_detect );
    }

    SMTC_MODEM_HAL_TRACE_INFO( "Find %d satellites\n", nb_satellite_detected );

    status = lr1110_gnss_get_detected_satellites( NULL, nb_satellite_detected, gnss_last_sat_detect );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to get detected satellites\n" );
        return false;
    }

    for( uint8_t i = 0; i < nb_satellite_detected; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "ID: %d - CNR: %d\n", gnss_last_sat_detect[i].satellite_id,
                                     gnss_last_sat_detect[i].cnr );
    }

    status = lr1110_gnss_get_result_size( NULL, &gnss_temp_rslt_size );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to get result size\n" );
        return false;
    }

    // we make sure gnss_temp_rslt_size can be hold in one byte here
    if( gnss_temp_rslt_size > sizeof( gnss_temp_rslt_buffer ) - 1 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "GNSS result buffer is too small to accept %d bytes\n", gnss_temp_rslt_size );
        return false;
    }

    status = lr1110_gnss_read_results( NULL, gnss_temp_rslt_buffer, gnss_temp_rslt_size );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to get result\n" );
        return false;
    }
    SMTC_MODEM_HAL_TRACE_ARRAY( "GNSS result", gnss_temp_rslt_buffer, gnss_temp_rslt_size );

    return true;
}

/**
 * @brief Trigger an almanac update with local almanac
 *
 * Remarks: modem_suspend_radio_access must be called before and modem_resume_radio_access after
 */
void update_local_almanac( void )
{
    uint32_t almanac_crc;

    if( lr1110_gnss_get_almanac_crc( NULL, &almanac_crc ) != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to read crc almanac\n" );
    }
    else
    {
        uint32_t local_almanac_crc =
            ( full_almanac[6] << 24 ) + ( full_almanac[5] << 16 ) + ( full_almanac[4] << 8 ) + ( full_almanac[3] );

        if( almanac_crc != local_almanac_crc )
        {
            SMTC_MODEM_HAL_TRACE_INFO( "Local almanac doesn't match LR1110 almanac -> start update\n" );

            // Load almanac in flash - do it only once
            uint16_t almanac_idx = 0;
            while( almanac_idx < sizeof( full_almanac ) )
            {
                if( lr1110_gnss_one_satellite_almanac_update( NULL, full_almanac + almanac_idx ) != LR1110_STATUS_OK )
                {
                    SMTC_MODEM_HAL_TRACE_ERROR( "Fail to update almanac\n" );
                }
                almanac_idx += 20;
            }

            // check almanac crc again to decide if the update is successful or not
            if( lr1110_gnss_get_almanac_crc( NULL, &almanac_crc ) != LR1110_STATUS_OK )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "Fail to read crc almanac back\n" );
            }
            else
            {
                if( almanac_crc != local_almanac_crc )
                {
                    SMTC_MODEM_HAL_TRACE_ERROR( "Local almanac doesn't match LR1110 almanac -> update fails\n" );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_INFO( "Almanac update succeeds\n" );
                }
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_INFO( "Local almanac match LR1110 almanac -> no update\n" );
        }
    }
}
