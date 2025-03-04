/*!
 * \file      modem_context.c
 *
 * \brief     share functions + context of the soft modem .
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

#include "modem_context.h"
#include "smtc_modem_hal.h"
#include "smtc_real.h"
#include "device_management_defs.h"
#include "lorawan_api.h"
#include "modem_utilities.h"  // for crc
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"
#include "alc_sync.h"
#include "lr1mac_utilities.h"
#include "fragmented_data_block.h"
#include "modem_supervisor.h"

#if defined( _GNSS_SNIFF_ENABLE )
#if defined( LR1110_MODEM )
#include "gnss_ctrl_api.h"
#elif defined( LR1110_TRANSCEIVER )
#include "almanac_update.h"
#include "lr1110_gnss.h"
#else
#error "GNSS functionality can't be used !"
#endif
#endif  //_GNSS_SNIFF_ENABLE

#if defined( LR1110_MODEM )
#include "pool_mem.h"
#include "smtc_hal_mcu.h"
#endif  // LR1110_MODEM

#if defined( USE_LR1110_SE )
#include "lr1110_system.h"
#endif  // USE_LR1110_SE

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define MODEM_APPKEY_CRC_STATUS_VALID ( 0 )
#define MODEM_APPKEY_CRC_STATUS_INVALID ( 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
typedef struct modem_context_nvm_idx_s
{
    uint16_t dm_port;
    uint16_t dm_upload_sctr;
    uint8_t  appkey_crc_status;
    uint32_t appkey_crc;
    uint32_t rfu[3];
    uint32_t crc;  // !! crc MUST be the last field of the structure !!
} modem_context_nvm_t;

#if !defined( LR1110_MODEM )
static int16_t                modem_appkey_status  = MODEM_APPKEY_CRC_STATUS_INVALID;
static int32_t                modem_appkey_crc     = 0;
static uint8_t                modem_status         = 0;
static uint8_t                modem_dm_interval    = DEFAULT_DM_REPORTING_INTERVAL;
static uint8_t                modem_dm_port        = DEFAULT_DM_PORT;
static uint8_t                modem_frag_port      = DEFAULT_FRAG_PORT;
static uint8_t                modem_appstatus[8]   = { 0 };
static smtc_modem_class_t     modem_dm_class       = SMTC_MODEM_CLASS_A;
static e_modem_suspend_t      is_modem_suspend     = MODEM_NOT_SUSPEND;
static uint32_t               modem_start_time     = 0;
static uint8_t                modem_dm_upload_sctr = 0;
static e_modem_upload_state_t modem_upload_state   = MODEM_UPLOAD_NOT_INIT;
static s_modem_stream_t       modem_stream_state   = {  //
    .port       = DEFAULT_DM_PORT,              //
    .state      = MODEM_STREAM_NOT_INIT,        //
    .encryption = false
};
static uint32_t dm_info_bitfield_periodic                = DEFAULT_DM_REPORTING_FIELDS;  // context for periodic GetInfo
static uint32_t dm_info_bitfield_now                     = 0;                            // User GetInfo
static uint8_t  tag_number                               = 0;
static uint8_t  tag_number_now                           = 0;
static uint8_t  number_of_muted_day                      = 0;
static s_dm_retrieve_pending_dl_t dm_pending_dl          = { .up_count = 0, .up_delay = 0 };
static uint32_t                   user_alarm             = 0x7FFFFFFF;
static uint8_t                    asynchronous_msgnumber = 0;
static uint8_t                    modem_event_count[MODEM_NUMBER_OF_EVENTS];
static uint8_t                    modem_event_status[MODEM_NUMBER_OF_EVENTS];
static uint8_t                    asynch_msg[MODEM_NUMBER_OF_EVENTS];
static s_modem_dwn_t              modem_dwn_pkt;
static bool                       is_modem_reset_requested    = false;
static bool                       is_modem_charge_loaded      = false;
static uint32_t                   modem_charge_offset         = 0;
static bool                       start_time_was_set          = false;
static uint16_t                   user_define_charge_counter  = 0;
static charge_counter_value_t     charge_counter_to_send      = CHARGE_COUNTER_MODEM;
static rf_output_t                modem_rf_output             = MODEM_RFO_LP_LF;
static uint8_t                    duty_cycle_disabled_by_host = false;
static uint32_t                   crc_fw;
static smtc_modem_adr_profile_t   modem_adr_profile;
static uint32_t                   modem_upload_avgdelay;
static uint16_t                   nb_adr_mobile_timeout;
static bool                       is_modem_in_test_mode = false;
static int8_t                     rx_pathloss_db        = 0;
static int8_t                     tx_power_offset_db    = -2;
static radio_planner_t*           modem_rp              = NULL;
static modem_power_config_t       power_config_lut[POWER_CONFIG_LUT_SIZE];
#else
struct
{
    int16_t                    modem_appkey_status;
    int32_t                    modem_appkey_crc;
    uint8_t                    modem_status;
    uint8_t                    modem_dm_interval;
    uint8_t                    modem_dm_port;
    uint8_t                    modem_frag_port;
    uint8_t                    modem_appstatus[8];
    smtc_modem_class_t         modem_dm_class;
    e_modem_suspend_t          is_modem_suspend;
    uint32_t                   modem_start_time;
    uint8_t                    modem_dm_upload_sctr;
    e_modem_upload_state_t     modem_upload_state;
    s_modem_stream_t           modem_stream_state;
    uint32_t                   dm_info_bitfield_periodic;  // context for periodic GetInfo
    uint32_t                   dm_info_bitfield_now;       // User GetInfo
    uint8_t                    tag_number;
    uint8_t                    tag_number_now;
    uint8_t                    number_of_muted_day;
    s_dm_retrieve_pending_dl_t dm_pending_dl;
    uint32_t                   user_alarm;
    uint8_t                    asynchronous_msgnumber;
    uint8_t                    modem_event_count[MODEM_NUMBER_OF_EVENTS];
    uint8_t                    modem_event_status[MODEM_NUMBER_OF_EVENTS];
    uint8_t                    asynch_msg[MODEM_NUMBER_OF_EVENTS];
    s_modem_dwn_t              modem_dwn_pkt;
    bool                       is_modem_reset_requested;
    bool                       is_modem_charge_loaded;
    uint32_t                   modem_charge_offset;
    uint8_t                    start_time_was_set;
    uint16_t                   user_define_charge_counter;
    charge_counter_value_t     charge_counter_to_send;
    rf_output_t                modem_rf_output;
    uint8_t                    duty_cycle_disabled_by_host;
    uint32_t                   crc_fw;
    smtc_modem_adr_profile_t   modem_adr_profile;
    uint32_t                   modem_upload_avgdelay;
    uint16_t                   nb_adr_mobile_timeout;
    bool                       is_modem_in_test_mode;
    int8_t                     rx_pathloss_db;
    int8_t                     tx_power_offset_db;
    radio_planner_t*           modem_rp;
    modem_power_config_t       power_config_lut[POWER_CONFIG_LUT_SIZE];
} modem_ctx_context;

// clang-format off
#define  modem_appkey_status                modem_ctx_context.modem_appkey_status
#define  modem_appkey_crc                   modem_ctx_context.modem_appkey_crc
#define  modem_status                       modem_ctx_context.modem_status
#define  modem_dm_interval                  modem_ctx_context.modem_dm_interval
#define  modem_dm_port                      modem_ctx_context.modem_dm_port
#define  modem_frag_port                    modem_ctx_context.modem_frag_port
#define  modem_appstatus                    modem_ctx_context.modem_appstatus
#define  modem_dm_class                     modem_ctx_context.modem_dm_class
#define  is_modem_suspend                   modem_ctx_context.is_modem_suspend
#define  modem_start_time                   modem_ctx_context.modem_start_time
#define  modem_dm_upload_sctr               modem_ctx_context.modem_dm_upload_sctr
#define  modem_upload_state                 modem_ctx_context.modem_upload_state
#define  modem_stream_state                 modem_ctx_context.modem_stream_state
#define  dm_info_bitfield_periodic          modem_ctx_context.dm_info_bitfield_periodic
#define  dm_info_bitfield_now               modem_ctx_context.dm_info_bitfield_now
#define  tag_number                         modem_ctx_context.tag_number
#define  tag_number_now                     modem_ctx_context.tag_number_now
#define  number_of_muted_day                modem_ctx_context.number_of_muted_day
#define  dm_pending_dl                      modem_ctx_context.dm_pending_dl
#define  user_alarm                         modem_ctx_context.user_alarm
#define  asynchronous_msgnumber             modem_ctx_context.asynchronous_msgnumber
#define  modem_event_count                  modem_ctx_context.modem_event_count
#define  modem_event_status                 modem_ctx_context.modem_event_status
#define  asynch_msg                         modem_ctx_context.asynch_msg
#define  modem_dwn_pkt                      modem_ctx_context.modem_dwn_pkt
#define  is_modem_reset_requested           modem_ctx_context.is_modem_reset_requested
#define  is_modem_charge_loaded             modem_ctx_context.is_modem_charge_loaded
#define  modem_charge_offset                modem_ctx_context.modem_charge_offset
#define  start_time_was_set                 modem_ctx_context.start_time_was_set
#define  user_define_charge_counter         modem_ctx_context.user_define_charge_counter
#define  charge_counter_to_send             modem_ctx_context.charge_counter_to_send
#define  modem_rf_output                    modem_ctx_context.modem_rf_output
#define  duty_cycle_disabled_by_host        modem_ctx_context.duty_cycle_disabled_by_host
#define  crc_fw                             modem_ctx_context.crc_fw
#define  modem_adr_profile                  modem_ctx_context.modem_adr_profile
#define  modem_upload_avgdelay              modem_ctx_context.modem_upload_avgdelay
#define  nb_adr_mobile_timeout              modem_ctx_context.nb_adr_mobile_timeout
#define  is_modem_in_test_mode              modem_ctx_context.is_modem_in_test_mode
#define  rx_pathloss_db                     modem_ctx_context.rx_pathloss_db
#define  tx_power_offset_db                 modem_ctx_context.tx_power_offset_db
#define  modem_rp                           modem_ctx_context.modem_rp
#define  power_config_lut                   modem_ctx_context.power_config_lut
// clang-format on

#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * \brief   convert requested DM bytes fields to bitfield
 *
 * \param [in]  requested_info_list     Array of bytes with requested DM code in each bytes
 * \param [in]  len                     Number of byte that composed requested_info_list
 * \param [in]  bitfields *             Returned bitfield
 * \return     void
 */
static void convert_requested_dm_info_bytes_to_bitfield( const uint8_t* requested_info_list, uint8_t len,
                                                         uint32_t* bitfields )
{
    // Reset bitfield
    *bitfields = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        if( requested_info_list[i] != e_inf_crashlog )
        {
            *bitfields |= ( 1 << requested_info_list[i] );
        }
    }
    return;
}

/*!
 * \brief   Check if the biggest requested DM status field can be inserted
 *          in the payload in regard of the max payload size requested
 *
 * \param [in]  info_requested              Requested bitfield
 * \param [in]  max_size                    Max size of the payload
 * \param [out] e_dm_cmd_length_valid       Return valid or not
 */
static e_dm_cmd_length_valid check_dm_status_max_size( uint32_t info_requested, uint8_t max_size )
{
    for( uint8_t i = 0; i < e_inf_max; i++ )
    {
        if( ( info_requested & ( 1 << i ) ) )
        {
            if( max_size < dm_info_field_sz[i] )
            {
                SMTC_MODEM_HAL_TRACE_ERROR(
                    "max_size must be greater than the smallest requested "
                    "information\n" );
                return DM_CMD_LENGTH_NOT_VALID;
            }
        }
    }
    return DM_CMD_LENGTH_VALID;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_context_init( )
{
    modem_status                  = 0;
    modem_dm_interval             = DEFAULT_DM_REPORTING_INTERVAL;
    modem_dm_port                 = DEFAULT_DM_PORT;
    modem_frag_port               = DEFAULT_FRAG_PORT;
    modem_dm_class                = SMTC_MODEM_CLASS_A;
    is_modem_suspend              = MODEM_NOT_SUSPEND;
    modem_start_time              = 0;
    modem_dm_upload_sctr          = 0;
    modem_upload_state            = MODEM_UPLOAD_NOT_INIT;
    modem_stream_state.port       = DEFAULT_DM_PORT;
    modem_stream_state.state      = MODEM_STREAM_NOT_INIT;
    modem_stream_state.encryption = false;
    dm_info_bitfield_periodic     = DEFAULT_DM_REPORTING_FIELDS;  // context for periodic GetInfo
    dm_info_bitfield_now          = 0;                            // User GetInfo
    tag_number                    = 0;
    tag_number_now                = 0;
    number_of_muted_day           = 0;
    dm_pending_dl.up_count        = 0;
    dm_pending_dl.up_delay        = 0;
    user_alarm                    = 0;
    asynchronous_msgnumber        = 0;
    is_modem_reset_requested      = false;
    is_modem_charge_loaded        = false;
    modem_charge_offset           = 0;
    start_time_was_set            = false;
    user_define_charge_counter    = 0;
    charge_counter_to_send        = CHARGE_COUNTER_MODEM;
    modem_rf_output               = MODEM_RFO_LP_LF;
    duty_cycle_disabled_by_host   = false;
    crc_fw                        = compute_crc_fw( );
    modem_adr_profile             = SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED;
    modem_upload_avgdelay         = 0;
    nb_adr_mobile_timeout         = DEFAULT_ADR_MOBILE_MODE_TIMEOUT;
    is_modem_in_test_mode         = false;
    rx_pathloss_db                = 0;
    tx_power_offset_db            = -2;
    modem_rp                      = NULL;
    modem_appkey_status           = MODEM_APPKEY_CRC_STATUS_INVALID;
    modem_appkey_crc              = 0;
    memset( modem_appstatus, 0, 8 );
    memset( modem_event_count, 0, MODEM_NUMBER_OF_EVENTS );
    memset( modem_event_status, 0, MODEM_NUMBER_OF_EVENTS );
    memset( asynch_msg, 0, MODEM_NUMBER_OF_EVENTS );
    memset( &modem_dwn_pkt, 0, sizeof( s_modem_dwn_t ) );
    // init power config tab to 0x80 as it corresponds to an expected power of 128dbm, value that is never reached
    memset( power_config_lut, 0x80, POWER_CONFIG_LUT_SIZE * sizeof( modem_power_config_t ) );
}

void modem_event_init( void )
{
    for( int i = 0; i < MODEM_NUMBER_OF_EVENTS; i++ )
    {
        set_modem_event_count_and_status( i, 0, 0 );
    }
}

uint8_t get_modem_event_count( uint8_t event_type )
{
    if( event_type >= MODEM_NUMBER_OF_EVENTS )
    {
        smtc_modem_hal_mcu_panic( );
    }

    return ( modem_event_count[event_type] );
}

uint8_t get_modem_event_status( uint8_t event_type )
{
    if( event_type >= MODEM_NUMBER_OF_EVENTS )
    {
        smtc_modem_hal_mcu_panic( );
    }
    return ( modem_event_status[event_type] );
}

void set_modem_event_count_and_status( uint8_t event_type, uint8_t value, uint8_t status )
{
    if( event_type < MODEM_NUMBER_OF_EVENTS )
    {
        modem_event_count[event_type]  = value;
        modem_event_status[event_type] = status;
    }
}

void increment_modem_event_count_and_status( uint8_t event_type, uint8_t status )
{
    if( event_type < MODEM_NUMBER_OF_EVENTS )
    {
        if( modem_event_count[event_type] < 255 )
        {
            modem_event_count[event_type]++;
        }
        // Set last status even if the number of event max is reached
        modem_event_status[event_type] = status;
    }
}

void decrement_asynchronous_msgnumber( void )
{
    if( asynchronous_msgnumber > 0 )
    {
        asynchronous_msgnumber--;
    }
    else
    {
        asynchronous_msgnumber = 0;
    }
}

uint8_t get_asynchronous_msgnumber( void )
{
    return ( asynchronous_msgnumber );
}

void increment_asynchronous_msgnumber( uint8_t event_type, uint8_t status )
{
    // Next condition should never append because only one asynch msg by type of message
    if( asynchronous_msgnumber > MODEM_NUMBER_OF_EVENTS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( " Modem reach the max number of asynch message" );
        return;
    }
    uint8_t tmp;
    tmp = get_modem_event_count( event_type );
    if( tmp == 0 )
    {
        asynchronous_msgnumber++;
        asynch_msg[asynchronous_msgnumber] = event_type;
    }

    increment_modem_event_count_and_status( event_type, status );

    //     SMTC_MODEM_HAL_TRACE_INFO( "increment event %d\n", event_type );
    //     switch( event_type )
    //     {
    //     case EVENTJOINED:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_JOINED\n" );
    //         break;
    //     case EVENT_ALARM:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_ALARM\n" );
    //         break;
    //     case EVENT_DOWNDATA:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_DOWNDATA\n" );
    //         break;
    //     case EVENT_TXDONE:
    //         SMTC_MODEM_HAL_TRACE_INFO( "event count tx done = %d\n", get_modem_event_count( EVENT_TXDONE ) );
    //         break;
    //     case EVENT_FILEDONE:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_FILEDONE\n" );
    //         break;
    //     case EVENT_STREAMDONE:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_STREAMDONE\n" );
    //         break;
    //     case EVENT_SETCONF:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_SETCONF\n" );
    //         break;
    //     case EVENT_MUTE:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_MUTE\n" );
    //         break;
    //     case EVENT_SWITCH_MOBILE_TO_STATIC:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_SWITCH_MOBILE_TO_STATIC\n" );
    //         break;
    //     case EVENT_NEW_LINK_ADR:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENT_NEW_LINK_ADR\n" );
    //         break;
    // #if defined( _GNSS_SNIFF_ENABLE )
    //     case EVENTGNSS:
    //         SMTC_MODEM_HAL_TRACE_INFO( "increment event EVENTGNSS\n" );
    //         break;
    // #endif  // _GNSS_SNIFF_ENABLE
    //     default:

    //         break;
    //     }
}

uint8_t get_last_msg_event( void )
{
    return asynch_msg[asynchronous_msgnumber];
}

uint32_t get_modem_uptime_s( void )
{
    return ( smtc_modem_hal_get_time_in_s( ) - modem_start_time );
}

void set_modem_start_time_s( uint32_t time )
{
    if( !start_time_was_set )
    {
        start_time_was_set = true;
        modem_start_time   = time;
    }
}

e_set_error_t set_modem_dm_interval( uint8_t interval )
{
    if( modem_dm_interval != interval )
    {
        modem_dm_interval = interval;
    }

    return ( SET_OK );
}
uint8_t get_modem_dm_interval( void )
{
    return ( modem_dm_interval );
}
uint32_t get_modem_dm_interval_second( void )
{
    uint8_t  dm_interval = get_modem_dm_interval( );
    uint32_t temp        = 0;
    switch( ( dm_interval >> 6 ) & 0x03 )
    {
    case DM_INTERVAL_UNIT_SEC:
        temp = ( dm_interval & 0x3F );
        break;
    case DM_INTERVAL_UNIT_DAY:
        temp = ( dm_interval & 0x3F ) * 3600 * 24;
        break;
    case DM_INTERVAL_UNIT_HOUR:
        temp = ( dm_interval & 0x3F ) * 3600;
        break;
    case DM_INTERVAL_UNIT_MIN:
        temp = ( dm_interval & 0x3F ) * 60;
        break;
    default:  // never reach
        smtc_modem_hal_mcu_panic( );
        break;
    }
    return temp;
}

e_set_error_t set_modem_class( smtc_modem_class_t LoRaWAN_class )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "modem class %d\n", LoRaWAN_class );
    if( LoRaWAN_class == SMTC_MODEM_CLASS_A )
    {
        lorawan_api_class_c_enabled( false );
    }
    else if( LoRaWAN_class == SMTC_MODEM_CLASS_C )
    {
        lorawan_api_class_c_enabled( true );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "modem class invalid" );
        return ( SET_ERROR );
    }
    modem_dm_class = LoRaWAN_class;
    return ( SET_OK );
}

smtc_modem_class_t get_modem_class( void )
{
    return ( modem_dm_class );
}

e_set_error_t set_modem_dm_port( uint8_t port )
{
    if( ( port == 0 ) || ( port >= 224 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "modem port invalid\n" );
        return ( SET_ERROR );
    }
    else
    {
        if( modem_dm_port != port )
        {
            modem_dm_port = port;
            modem_store_context( );
        }
        return ( SET_OK );
    }
}

uint8_t get_modem_dm_port( void )
{
    return ( modem_dm_port );
}

e_set_error_t set_modem_frag_port( uint8_t port )
{
    SMTC_MODEM_HAL_TRACE_ERROR( "set_modem_frag_port not implemented\n" );
    return ( SET_ERROR );
}

uint8_t get_modem_frag_port( void )
{
    return ( modem_frag_port );
}

smtc_modem_adr_profile_t get_modem_adr_profile( void )
{
    return modem_adr_profile;
}

uint8_t get_modem_region( void )
{
    return lorawan_api_get_region( );
}

e_set_error_t set_modem_region( uint8_t region )
{
    if( lorawan_api_set_region( ( smtc_real_region_types_t ) region ) != OKLORAWAN )
    {
        return SET_ERROR;
    }
    return SET_OK;
}

eModemJoinState_t get_join_state( void )
{
    eModemJoinState_t joinstate;
    if( get_modem_status_joining( ) == true )
    {
        joinstate = MODEM_JOIN_ONGOING;
    }
    else if( lorawan_api_isjoined( ) == NOT_JOINED )
    {
        joinstate = MODEM_NOT_JOINED;
    }
    else
    {
        joinstate = MODEM_JOINED;
    }
    return ( joinstate );
}

void set_modem_appstatus( const uint8_t* app_status )
{
    memcpy( modem_appstatus, app_status, dm_info_field_sz[e_inf_appstatus] );
}

void get_modem_appstatus( uint8_t* app_status )
{
    memcpy( app_status, modem_appstatus, dm_info_field_sz[e_inf_appstatus] );
}

void modem_supervisor_add_task_join( void )
{
    smodem_task task_join;
    task_join.id       = JOIN_TASK;
    task_join.priority = TASK_HIGH_PRIORITY;

    uint32_t current_time_s = smtc_modem_hal_get_time_in_s( );

    task_join.time_to_execute_s = smtc_modem_hal_get_random_nb_in_range( 0, 5 );

#if defined( TEST_BYPASS_JOIN_DUTY_CYCLE )
    task_join.time_to_execute_s += current_time_s;
#else
    if( lorawan_api_modem_certification_is_enabled( ) == false )
    {
        // current time is already taken in count in lr1mac time computation
        task_join.time_to_execute_s += lorawan_api_next_join_time_second_get( );
    }
#endif

    SMTC_MODEM_HAL_TRACE_PRINTF( " Start a New join in %d seconds \n", task_join.time_to_execute_s - current_time_s );
    set_modem_status_joining( true );
    modem_supervisor_add_task( &task_join );
}

void modem_supervisor_add_task_dm_status( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = DM_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_dm_status_now( void )
{
    smodem_task task_dm;
    task_dm.id                = DM_TASK_NOW;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) +
                                smtc_modem_hal_get_random_nb_in_range( DM_STATUS_NOW_MIN_TIME, DM_STATUS_NOW_MAX_TIME );
    modem_supervisor_add_task( &task_dm );
}
void modem_supervisor_add_task_crash_log( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = CRASH_LOG_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute +
                                smtc_modem_hal_get_random_nb_in_range( DM_STATUS_NOW_MIN_TIME, DM_STATUS_NOW_MAX_TIME );
    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_add_task_alc_sync_time_req( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = ALC_SYNC_TIME_REQ_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;

    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_remove_task_alc_sync( void )
{
    if( modem_supervisor_get_task_priority( ALC_SYNC_TIME_REQ_TASK ) != TASK_FINISH )
    {
        modem_supervisor_remove_task( ALC_SYNC_TIME_REQ_TASK );
    }
    if( modem_supervisor_get_task_priority( ALC_SYNC_ANS_TASK ) != TASK_FINISH )
    {
        modem_supervisor_remove_task( ALC_SYNC_ANS_TASK );
    }
}

bool modem_supervisor_is_alc_sync_running( void )
{
    bool ret = false;
    if( modem_supervisor_get_task_priority( ALC_SYNC_TIME_REQ_TASK ) != TASK_FINISH )
    {
        ret = true;
    }
    return ( ret );
}

void modem_supervisor_add_task_alc_sync_ans( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = ALC_SYNC_ANS_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_alm_dbg_ans( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = DM_ALM_DBG_ANS;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_modem_mute( void )
{
    smodem_task task_dm;
    task_dm.id                = MUTE_TASK;
    task_dm.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + 86400;  // Every 24h
    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_add_task_retrieve_dl( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = RETRIEVE_DL_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.sizeIn            = 0;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_frag( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = FRAG_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_stream( void )
{
    // Modem supervisor copy everything,
    // so this is safe even when it is going to be invalidated.
    smodem_task stream_task;

    stream_task.id                = STREAM_TASK;
    stream_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 1, 3 );
    stream_task.priority          = TASK_HIGH_PRIORITY;
    stream_task.fPort             = modem_get_stream_port( );
    // stream_task.dataIn        not used in task
    // stream_task.sizeIn        not used in task
    // stream_task.PacketType    not used in task
    modem_supervisor_add_task( &stream_task );
    // stream task is now on going: set modem status accordingly
    set_modem_status_streaming( true );
}

/*!
 * \brief   return the modem status
 * \remark
 * \param [IN] void
 * \param [OUT] modem status bit 6 : streaming in progress
 *                           bit 5 : file upload in progress
 *                           bit 4 : radio suspend
 *                           bit 3 : modem join
 *                           bit 2 : modem mute
 *                           bit 1 : reset after panic
 *                           bit 0 : reset after brownout
 * */
uint8_t get_modem_status( void )
{
    // If the stack is no more join, the modem status was not aware of the disconnection
    if( get_join_state( ) != MODEM_JOINED )
    {
        set_modem_status_modem_joined( false );
    }
    return ( modem_status );
}

#if defined( _GNSS_SNIFF_ENABLE )

void get_modem_gnss_status( uint8_t* gnss_status )
{
#if defined( LR1110_MODEM )
    Gnss_context_status( gnss_status );
#elif defined( LR1110_TRANSCEIVER )
    uint8_t buffer_response[ALM_UPDATE_UPLINK_PAYLOAD_LENGTH];
    // call radio_suspend to secure the direct radio access
    // modem_context_suspend_radio_access( );
    almanac_update_create_uplink_payload( NULL, buffer_response );
    // Discard first byte as it is already handle by the dm uplink process in modem_context
    // modem_context_resume_radio_access( );
    memcpy( gnss_status, &buffer_response[1], ALM_UPDATE_UPLINK_PAYLOAD_LENGTH - 1 );
#else
#error "GNSS functionality can't be used !"
#endif
}
#endif  // _GNSS_SNIFF_ENABLE

void set_modem_status_reset_after_brownout( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_brownout ) )
                                     : ( modem_status & ~( 1 << modem_status_brownout ) );
}

void set_modem_status_reset_after_crash( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_crash ) )
                                     : ( modem_status & ~( 1 << modem_status_crash ) );
}

void set_modem_status_modem_mute( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_mute ) )
                                     : ( modem_status & ~( 1 << modem_status_mute ) );
}

void set_modem_status_modem_joined( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_joined ) )
                                     : ( modem_status & ~( 1 << modem_status_joined ) );
}

void set_modem_status_radio_suspend( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_suspend ) )
                                     : ( modem_status & ~( 1 << modem_status_suspend ) );
}

void set_modem_status_file_upload( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_upload ) )
                                     : ( modem_status & ~( 1 << modem_status_upload ) );
}

void set_modem_status_joining( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_joining ) )
                                     : ( modem_status & ~( 1 << modem_status_joining ) );
}

void set_modem_status_streaming( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_streaming ) )
                                     : ( modem_status & ~( 1 << modem_status_streaming ) );
}

bool get_modem_status_reset_after_crash( void )
{
    return ( ( modem_status >> modem_status_crash ) & 0x01 );
}

bool get_modem_status_file_upload( void )
{
    return ( ( modem_status >> modem_status_upload ) & 0x01 );
}

bool get_modem_status_joining( void )
{
    return ( ( modem_status >> modem_status_joining ) & 0x01 );
}

bool get_modem_status_streaming( void )
{
    return ( ( modem_status >> modem_status_streaming ) & 0x01 );
}

void reset_modem_charge( void )
{
    radio_planner_t* rp = modem_context_get_modem_rp( );
    rp_stats_init( &rp->stats );
}

uint32_t get_modem_charge_ma_s( void )
{
    radio_planner_t* rp = modem_context_get_modem_rp( );
    uint32_t         total_consumption =
        rp->stats.tx_total_consumption_ma + rp->stats.rx_total_consumption_ma + rp->stats.none_total_consumption_ma;
    return ( ( total_consumption / 1000 ) + modem_charge_offset );
}

uint32_t get_modem_charge_ma_h( void )
{
    return get_modem_charge_ma_s( ) / 3600;
}

uint16_t get_modem_user_define_charge_ma_h( void )
{
    return user_define_charge_counter;
}

void set_modem_user_define_charge_ma_h( const uint16_t value )
{
    user_define_charge_counter = value;
}

void choose_modem_charge_counter( void )
{
    charge_counter_to_send = CHARGE_COUNTER_MODEM;
}

void choose_user_define_charge_counter( void )
{
    charge_counter_to_send = CHARGE_COUNTER_USER_DEFINE;
}

charge_counter_value_t get_charge_counter_to_send( void )
{
    return charge_counter_to_send;
}

uint8_t get_modem_voltage( void )
{
    return smtc_modem_hal_get_voltage( );
}
int8_t get_modem_temp( void )
{
    return smtc_modem_hal_get_temperature( );
}

e_dm_cmd_length_valid dm_check_dminfo_size( e_dm_info_t cmd, uint8_t length )
{
    if( cmd >= e_inf_max )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }

    if( length != dm_info_field_sz[cmd] )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }

    return DM_CMD_LENGTH_VALID;
}

e_dm_error_t dm_set_conf( e_dm_info_t tag, uint8_t* data, uint8_t length )
{
    e_dm_error_t ret = DM_OK;

    if( dm_check_dminfo_size( tag, length ) != DM_CMD_LENGTH_VALID )
    {
        tag = e_inf_max;
        ret = DM_ERROR;
    }
    else
    {
        switch( tag )
        {
        case e_inf_adrmode: {
            // update modem context adr
            modem_adr_profile = ( smtc_modem_adr_profile_t ) data[0];

            status_lorawan_t status = lorawan_api_dr_strategy_set( ( dr_strategy_t ) data[0] );
            if( status == ERRORLORAWAN )
            {
                ret = DM_ERROR;
            }
            break;
        }
        case e_inf_joineui: {
            uint8_t p_tmp[8];
            memcpy1_r( p_tmp, data, 8 );
            lorawan_api_appeui_key_set( &p_tmp[0] );
            break;
        }
        case e_inf_interval:
            set_modem_dm_interval( data[0] );
            modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
            break;
        case e_inf_region:
            // TODO: see how to handle stack id here
            smtc_modem_leave_network( 0 );
            set_modem_region( data[0] );
            break;
        default:
            tag = e_inf_max;
            ret = DM_ERROR;
            break;
        }
    }
    if( ret == DM_OK )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_SETCONF, tag );
    }

    return ret;
}

e_modem_mute_t get_modem_muted( void )
{
    e_modem_mute_t mute;

    if( number_of_muted_day == MODEM_INFINITE_MUTE )
    {
        mute = MODEM_INFINITE_MUTE;
    }
    else if( number_of_muted_day > 0 )
    {
        mute = MODEM_TEMPORARY_MUTE;
    }
    else
    {
        mute = MODEM_NOT_MUTE;
    }
    return mute;
}

uint8_t dm_get_number_of_days_mute( void )
{
    return number_of_muted_day;
}

void dm_set_number_of_days_mute( uint8_t days )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "MUTE for %d days\n", days );
    if( number_of_muted_day != days )
    {
        number_of_muted_day = days;

        if( number_of_muted_day > 0 )
        {
            set_modem_status_modem_mute( true );
        }
        else
        {
            set_modem_status_modem_mute( false );
        }
    }
}

uint8_t get_dm_info_tag_list( uint8_t* dm, e_dm_info_rate_t flag )
{
    uint8_t* p = dm;
    uint32_t info_req;

    if( flag == DM_INFO_NOW )
    {
        info_req = dm_info_bitfield_now;
    }
    else
    {
        info_req = dm_info_bitfield_periodic;
    }

    for( uint8_t i = 0; i < e_inf_max; i++ )
    {
        if( ( info_req & ( 1 << i ) ) )
        {
            *p++ = i;  // If bit is set, added id Code in payload
        }
    }
    return p - dm;
}

e_set_error_t set_dm_info( const uint8_t* requested_info_list, uint8_t len, e_dm_info_rate_t flag )
{
    e_set_error_t ret      = SET_OK;
    uint32_t      info_req = 0;

    for( uint8_t i = 0; i < len; i++ )
    {
        // Ignore DM status with variable length and forbiden fields
        if( ( requested_info_list[i] == e_inf_upload ) || ( requested_info_list[i] == e_inf_stream ) ||
            ( requested_info_list[i] == e_inf_alcsync ) || ( requested_info_list[i] == e_inf_dbgrsp ) ||
            ( requested_info_list[i] == e_inf_gnssloc ) || ( requested_info_list[i] == e_inf_wifiloc ) ||
            ( requested_info_list[i] == e_inf_rfu_0 ) || ( requested_info_list[i] == e_inf_rfu_1 ) ||
            ( requested_info_list[i] >= e_inf_max ) )
        {
            ret = SET_ERROR;
            SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM info code (0x%02x)\n", requested_info_list[i] );
        }
    }
    if( ret == SET_OK )
    {
        for( uint8_t i = 0; i < len; i++ )
        {
            if( requested_info_list[i] == e_inf_crashlog )
            {
                if( flag == DM_INFO_NOW )
                {
                    modem_supervisor_add_task_crash_log( 0 );
                }
                else
                {
                    ret = SET_ERROR;
                }
            }
        }
    }

    if( ret == SET_OK )
    {
        convert_requested_dm_info_bytes_to_bitfield( requested_info_list, len, &info_req );
        if( flag == DM_INFO_NOW )
        {
            dm_info_bitfield_now = info_req;
            tag_number_now       = 0;  // Reset tag_number used by dm_status_payload to
                                       // start a report from beginning
        }
        else
        {
            if( dm_info_bitfield_periodic != info_req )
            {
                dm_info_bitfield_periodic = info_req;
                tag_number                = 0;  // Reset tag_number used by dm_status_payload to start
                                                // a report from beginning
            }
        }
    }

    return ret;
}

bool dm_status_payload( uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len, uint8_t max_size,
                        e_dm_info_rate_t flag )
{
    uint8_t* p_tmp = dm_uplink_message;
    uint8_t* p     = dm_uplink_message;
    uint32_t info_requested;
    uint8_t* tag     = NULL;
    bool     pending = false;

    // Used DM code given in parameter
    if( flag == DM_INFO_NOW )
    {
        info_requested = dm_info_bitfield_now;
        tag            = &tag_number_now;
        // Used DM code in context
    }
    else
    {
        info_requested = dm_info_bitfield_periodic;
        tag            = &tag_number;
    }

    if( check_dm_status_max_size( info_requested, max_size ) != DM_CMD_LENGTH_VALID )
    {
        *dm_uplink_message_len = 0;
        SMTC_MODEM_HAL_TRACE_ERROR( "check_dm_status_max_size\n" );
        return false;
    }

    if( *tag >= e_inf_max )
    {
        *tag = 0;
    }
    // SMTC_MODEM_HAL_TRACE_PRINTF("info_requested = %d \n",info_requested);
    while( ( *tag ) < e_inf_max )
    {
        // SMTC_MODEM_HAL_TRACE_WARNING("tag %d - %d\n",*tag, (info_requested >> *tag) & 0x01
        // );
        if( ( info_requested & ( 1 << *tag ) ) )
        {
            *p_tmp++ = *tag;  // Add id Code in payload then the value(s)
            switch( *tag )
            {
            case e_inf_status:
                *p_tmp = get_modem_status( );
                break;
            case e_inf_charge: {
                uint32_t charge;
                if( get_charge_counter_to_send( ) == CHARGE_COUNTER_MODEM )
                {
                    charge = get_modem_charge_ma_h( );
                }
                else
                {
                    charge = get_modem_user_define_charge_ma_h( );
                }

                *p_tmp         = charge & 0xFF;
                *( p_tmp + 1 ) = ( charge >> 8 ) & 0xFF;
                break;
            }
            case e_inf_voltage:
                *p_tmp = get_modem_voltage( );
                break;
            case e_inf_temp:
                *p_tmp = get_modem_temp( );
                break;
            case e_inf_signal: {
                int16_t rssi = lorawan_api_last_rssi_get( );
                if( rssi >= -128 && rssi <= 63 )
                {
                    *p_tmp = ( int8_t )( rssi + 64 );  // strength of last downlink (RSSI [dBm]+64)
                }
                else if( rssi > 63 )
                {
                    *p_tmp = 127;
                }
                else if( rssi < -128 )
                {
                    *p_tmp = -128;
                }
                *( p_tmp + 1 ) = lorawan_api_last_snr_get( ) << 2;  // strength of last downlink (SNR [0.25 dB])
                break;
            }
            case e_inf_uptime: {
                uint32_t time  = get_modem_uptime_s( ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case e_inf_rxtime: {
                s_modem_dwn_t dwnframe = { 0 };
                get_modem_downlink_frame( &dwnframe );
                uint32_t time  = ( smtc_modem_hal_get_time_in_s( ) - ( dwnframe.timestamp / 1000 ) ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case e_inf_firmware: {
#if defined( LR1110_MODEM )
                // return the crc value of fuota dedicated for test have to be
                // re implement when fuota availble
                *( p_tmp + 0 ) = crc_fw & 0xFF;
                *( p_tmp + 1 ) = ( crc_fw >> 8 ) & 0xFF;
                *( p_tmp + 2 ) = ( crc_fw >> 16 ) & 0xFF;
                *( p_tmp + 3 ) = ( crc_fw >> 24 ) & 0xFF;
                *( p_tmp + 4 ) = frag_get_session_counter( ) & 0xFF;
                *( p_tmp + 5 ) = ( frag_get_session_counter( ) >> 8 ) & 0xFF;
                *( p_tmp + 6 ) = frag_get_nb_frag_received( ) & 0xFF;
                *( p_tmp + 7 ) = ( frag_get_nb_frag_received( ) >> 8 ) & 0xFF;
#else
                memset( p_tmp, 0, 8 );  // TODO remove this
#endif  // LR1110_MODEM
            }
            break;
            case e_inf_adrmode:
                *p_tmp = get_modem_adr_profile( );
                break;
            case e_inf_joineui: {
                uint8_t p_tmp_app_eui[8];
                lorawan_api_appeui_key_get( p_tmp_app_eui );
                memcpy1_r( p_tmp, p_tmp_app_eui, 8 );
                break;
            }
            case e_inf_interval:
                *p_tmp = get_modem_dm_interval( );
                break;
            case e_inf_region:
                *p_tmp = get_modem_region( );
                break;
            case e_inf_rfu_0:
                // Nothing to do
                break;
            case e_inf_crashlog:

                break;
            case e_inf_rstcount:
                *p_tmp         = lorawan_api_nb_reset_get( ) & 0xFF;
                *( p_tmp + 1 ) = lorawan_api_nb_reset_get( ) >> 8;
                break;
            case e_inf_deveui: {
                uint8_t p_tmp_dev_eui[8];
                lorawan_api_deveui_get( p_tmp_dev_eui );
                memcpy1_r( p_tmp, p_tmp_dev_eui, 8 );
                break;
            }
            case e_inf_rfu_1:
                // Nothing to do
                break;
            case e_inf_session:
                *p_tmp         = lorawan_api_devnonce_get( ) & 0xFF;
                *( p_tmp + 1 ) = lorawan_api_devnonce_get( ) >> 8;
                break;
            case e_inf_chipeui: {
                uint8_t p_tmp_chip_eui[8] = { 0 };
#if defined( LR1110_MODEM )
                hal_mcu_read_chip_eui( p_tmp_chip_eui );
#endif
#if defined( USE_LR1110_SE )
                lr1110_system_read_uid( NULL, ( uint8_t* ) &p_tmp_chip_eui );
#endif  // LR1110_MODEM
                memcpy1_r( p_tmp, p_tmp_chip_eui, 8 );
                break;
            }
            case e_inf_streampar:
                *p_tmp         = modem_get_stream_port( );
                *( p_tmp + 1 ) = modem_get_stream_encryption( );
                break;
            case e_inf_appstatus:
                get_modem_appstatus( p_tmp );
                break;
            case e_inf_almstatus:
#if defined( _GNSS_SNIFF_ENABLE )
                get_modem_gnss_status( p_tmp );
#endif  // _GNSS_SNIFF_ENABLE
                break;
            default:
                SMTC_MODEM_HAL_TRACE_ERROR( "Construct DM payload report, unknown code 0x%02x\n", *tag );
                break;
            }

            p_tmp += dm_info_field_sz[*tag];
            // Check if last message can be enqueued
            if( ( p_tmp - dm_uplink_message ) <= max_size )
            {
                p = p_tmp;
            }
            else
            {
                // last message can't be enqueued
                pending = true;
                break;  // break for loop
            }
        }
        ( *tag )++;
    }

    *dm_uplink_message_len = p - dm_uplink_message;
    return pending;
}

void dm_frag_uplink_payload( uint8_t max_payload_length, uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len )
{
    frag_set_max_length_up_payload( max_payload_length );

    frag_construct_uplink_payload( );

    frag_get_tx_buffer( &dm_uplink_message[0], dm_uplink_message_len );
}

#if defined( _GNSS_SNIFF_ENABLE )
void dm_alm_dbg_uplink_payload( uint8_t max_payload_length, uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len )
{
#if defined( LR1110_MODEM )
    uint8_t* gnss_payload      = ( uint8_t* ) &POOL_MEM.GNSS_MEM.Buf_data[0];
    uint16_t gnss_payload_size = GnssGetSize( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "GnssGetSize %d\n", gnss_payload_size );
    SMTC_MODEM_HAL_TRACE_PRINTF( "max_payload_length %d\n", max_payload_length );
    SMTC_MODEM_HAL_TRACE_ARRAY( "dm_alm_dbg_uplink_payload", gnss_payload, gnss_payload_size );
    if( ( gnss_payload[0] == GNSS_DM_MSG ) && ( gnss_payload_size <= max_payload_length ) && ( gnss_payload_size > 0 ) )
    {
        *dm_uplink_message_len = gnss_payload_size - 1;
        memcpy( dm_uplink_message, &gnss_payload[1], *dm_uplink_message_len );
    }
    else
    {
        *dm_uplink_message_len = 0;
    }
#elif defined( LR1110_TRANSCEIVER )
    // Not supported on Basic Modem with Lr1110 tranceiver
#else
#error "GNSS functionality can't be used !"
#endif
}
#endif  // _GNSS_SNIFF_ENABLE

e_set_error_t set_modem_suspend( bool suspend )
{
    set_modem_status_radio_suspend( suspend );
    is_modem_suspend = ( ( suspend == true ) ? MODEM_SUSPEND : MODEM_NOT_SUSPEND );
    return SET_OK;
}

e_modem_suspend_t get_modem_suspend( void )
{
    return ( ( is_modem_suspend == MODEM_SUSPEND ) ? true : false );
}

e_set_error_t set_modem_adr_profile( smtc_modem_adr_profile_t adr_profile, const uint8_t* adr_custom_data,
                                     uint8_t adr_custom_length )
{
    /* error case : 1) user_dr invalid
                    2) user_dr = custom but length not equal to 16
                    3) user_dr not custom but length not equal to 0*/
    if( ( adr_profile > SMTC_MODEM_ADR_PROFILE_CUSTOM ) ||
        ( ( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM ) && ( adr_custom_length != 16 ) ) ||
        ( ( adr_profile < SMTC_MODEM_ADR_PROFILE_CUSTOM ) && ( adr_custom_length != 0 ) ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "user_dr = %d not compatible with adr data length = %d \n ", adr_profile,
                                    adr_custom_length );
        return SET_ERROR;
    }

    // save profile in context:
    modem_adr_profile = adr_profile;

    status_lorawan_t status = ERRORLORAWAN;

    switch( adr_profile )
    {
    case SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( STATIC_ADR_MODE );
        break;
    case SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( MOBILE_LONGRANGE_DR_DISTRIBUTION );
        break;
    case SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( MOBILE_LOWPER_DR_DISTRIBUTION );
        break;
    case SMTC_MODEM_ADR_PROFILE_CUSTOM: {
        uint16_t MaskDrTmp       = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( );
        uint32_t adrDistribution = 0;
        uint8_t  cpt_tmp         = 0;
        for( uint8_t i = 0; i < 16; i++ )
        {
            if( adr_custom_data[i] > 15 )  // DR are defined from 0 to 15 by definition in LoRaWAN spec
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ADR with DataRate out of range\n" );
                return SET_ERROR;
            }
            if( ( ( MaskDrTmp >> adr_custom_data[i] ) & 0x01 ) == 1 )
            {
                cpt_tmp++;

                if( adr_custom_data[i] == 0x00 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0xF0000000 ) >> 28 ) != 0xF ) ? ( 1 << 28 ) : 0;
                }
                else if( adr_custom_data[i] == 0x01 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x0F000000 ) >> 24 ) != 0xF ) ? ( 1 << 24 ) : 0;
                }
                else if( adr_custom_data[i] == 0x02 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x00F00000 ) >> 20 ) != 0xF ) ? ( 1 << 20 ) : 0;
                }
                else if( adr_custom_data[i] == 0x03 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x000F0000 ) >> 16 ) != 0xF ) ? ( 1 << 16 ) : 0;
                }
                else if( adr_custom_data[i] == 0x04 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x0000F000 ) >> 12 ) != 0xF ) ? ( 1 << 12 ) : 0;
                }
                else if( adr_custom_data[i] == 0x05 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x00000F00 ) >> 8 ) != 0xF ) ? ( 1 << 8 ) : 0;
                }
                else if( adr_custom_data[i] == 0x06 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x000000F0 ) >> 4 ) != 0xF ) ? ( 1 << 4 ) : 0;
                }
                else if( adr_custom_data[i] == 0x07 )
                {
                    adrDistribution += ( ( ( adrDistribution & 0x0000000F ) ) != 0xF ) ? 1 : 0;
                }
            }
        }
        if( cpt_tmp == 0 )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "ADR with a bad DataRate value\n" );
            return SET_ERROR;
        }
        lorawan_api_dr_custom_set( adrDistribution );
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( USER_DR_DISTRIBUTION );
        break;
    }
    default: {
        SMTC_MODEM_HAL_TRACE_ERROR( "Unknown adr profile %d\n ", adr_profile );
        return SET_ERROR;
    }
    break;
    }

    if( status == ERRORLORAWAN )
    {
        return SET_ERROR;
    }
    else
    {
        return SET_OK;
    }
}

void set_modem_downlink_frame( uint8_t* data, uint8_t data_length, lr1mac_down_metadata_t* metadata )
{
    memcpy( modem_dwn_pkt.data, data, data_length );
    modem_dwn_pkt.length    = data_length;
    modem_dwn_pkt.timestamp = metadata->timestamp;
    modem_dwn_pkt.snr       = metadata->rx_snr << 2;
    modem_dwn_pkt.rssi      = metadata->rx_rssi + 64;
    modem_dwn_pkt.port      = metadata->rx_fport;
    SMTC_MODEM_HAL_TRACE_ARRAY( "Downlink frame ", modem_dwn_pkt.data, modem_dwn_pkt.length );
    SMTC_MODEM_HAL_TRACE_PRINTF( "ModemDwnPort = %d , ", modem_dwn_pkt.port );
    SMTC_MODEM_HAL_TRACE_PRINTF( "ModemDwnSNR = %d , ModemDwnRssi = %d \n ", modem_dwn_pkt.snr, modem_dwn_pkt.rssi );
}
void get_modem_downlink_frame( s_modem_dwn_t* modem_dwn_in )
{
    modem_dwn_in->timestamp = modem_dwn_pkt.timestamp;
    modem_dwn_in->snr       = modem_dwn_pkt.snr;
    modem_dwn_in->rssi      = modem_dwn_pkt.rssi;
    modem_dwn_in->port      = modem_dwn_pkt.port;
    modem_dwn_in->length    = modem_dwn_pkt.length;
    memcpy( modem_dwn_in->data, modem_dwn_pkt.data, modem_dwn_pkt.length );
}

void set_dm_retrieve_pending_dl( uint8_t up_count, uint8_t up_delay )
{
    dm_pending_dl.up_count = up_count;
    dm_pending_dl.up_delay = ( up_delay < 20 ) ? 20 : up_delay;
}

void get_dm_retrieve_pending_dl( s_dm_retrieve_pending_dl_t* pending_dl )
{
    pending_dl->up_count = dm_pending_dl.up_count;
    pending_dl->up_delay = dm_pending_dl.up_delay;
}

void decrement_dm_retrieve_pending_dl( void )
{
    if( dm_pending_dl.up_count > 0 )
    {
        dm_pending_dl.up_count--;
    }
}

void modem_store_context( void )
{
    modem_context_nvm_t ctx = {
        .dm_port = modem_dm_port,
        //.dm_upload_sctr    = modem_dm_upload_sctr,
        .appkey_crc_status = modem_appkey_status,
        .appkey_crc        = modem_appkey_crc,
        .rfu               = { 0 },
    };

    ctx.crc = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );
    smtc_modem_hal_context_store( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );
    modem_load_context( );
}

/*!
 * \brief    load modem context in non volatile memory
 * \remark
 * \retval void
 */
void modem_load_context( void )
{
    modem_context_nvm_t ctx;

    smtc_modem_hal_context_restore( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );

    if( crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 ) == ctx.crc )
    {
        modem_dm_port = ctx.dm_port;
        // modem_dm_upload_sctr = ctx.dm_upload_sctr;
        modem_appkey_status = ctx.appkey_crc_status;
        modem_appkey_crc    = ctx.appkey_crc;

        SMTC_MODEM_HAL_TRACE_PRINTF( "Modem Load Config :\n Port = %d \n Upload_sctr = %d\n", modem_dm_port,
                                     modem_dm_upload_sctr );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Restore Modem context fail => Factory Reset Modem\n" );
        modem_context_factory_reset( );
    }
}

void modem_context_factory_reset( void )
{
    modem_context_nvm_t ctx = {
        .dm_port = DEFAULT_DM_PORT,
        // .dm_upload_sctr    = 0,
        .appkey_crc_status = MODEM_APPKEY_CRC_STATUS_INVALID,
        .appkey_crc        = 0,
    };

    ctx.crc = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );
    smtc_modem_hal_context_store( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );

    is_modem_reset_requested = true;
    SMTC_MODEM_HAL_TRACE_INFO( "modem_context_factory_reset done\n" );
}

void modem_set_dm_upload_sctr( uint8_t session_counter )
{
    modem_dm_upload_sctr = session_counter & 0xf;
    // modem_store_context( ); // upload counter no more store in the flash
}

uint8_t modem_get_dm_upload_sctr( void )
{
    return ( modem_dm_upload_sctr & 0xf );
}

e_modem_upload_state_t modem_get_upload_state( void )
{
    return ( modem_upload_state );
}

void modem_set_upload_state( e_modem_upload_state_t upload_state )
{
    modem_upload_state = upload_state;
}

e_modem_stream_state_t modem_get_stream_state( void )
{
    return ( modem_stream_state.state );
}

uint8_t modem_get_stream_port( void )
{
    return ( modem_stream_state.port );
}

bool modem_get_stream_encryption( void )
{
    return ( modem_stream_state.encryption );
}

void modem_set_stream_state( e_modem_stream_state_t stream_state )
{
    modem_stream_state.state = stream_state;
}

void modem_set_stream_port( uint8_t port )
{
    modem_stream_state.port = port;
}

void modem_set_stream_encryption( bool enc )
{
    modem_stream_state.encryption = enc;
}

void modem_set_dm_info_bitfield_periodic( uint32_t value )
{
    if( dm_info_bitfield_periodic != value )
    {
        dm_info_bitfield_periodic = value;
    }
}
uint32_t modem_get_dm_info_bitfield_periodic( void )
{
    return ( dm_info_bitfield_periodic );
}
uint32_t modem_get_user_alarm( void )
{
    return ( user_alarm );
}

void modem_set_user_alarm( uint32_t alarm )
{
    user_alarm = alarm;
}

bool get_modem_reset_requested( void )
{
    return is_modem_reset_requested;
}
void set_modem_reset_requested( bool reset_req )
{
    is_modem_reset_requested = reset_req;
}

rf_output_t modem_get_rfo_pa( void )
{
    return modem_rf_output;
}

uint8_t modem_set_rfo_pa( rf_output_t rf_output )
{
    if( rf_output >= MODEM_RFO_MAX )
    {
        return SET_ERROR;
    }
    modem_rf_output = rf_output;
    return SET_OK;
}

void modem_set_duty_cycle_disabled_by_host( uint8_t disabled_by_host )
{
    duty_cycle_disabled_by_host = disabled_by_host;
}

uint8_t modem_get_duty_cycle_disabled_by_host( void )
{
    return duty_cycle_disabled_by_host;
}

void modem_set_upload_avgdelay( uint32_t avgdelay_in_s )
{
    modem_upload_avgdelay = avgdelay_in_s;
}

uint32_t modem_get_upload_avgdelay( void )
{
    return modem_upload_avgdelay;
}

void modem_set_adr_mobile_timeout_config( uint16_t nb_tx )
{
    nb_adr_mobile_timeout = nb_tx;
}

uint16_t modem_get_adr_mobile_timeout_config( void )
{
    return nb_adr_mobile_timeout;
}

uint16_t modem_get_current_adr_mobile_count( void )
{
    return lorawan_api_no_rx_packet_count_in_mobile_mode_get( );
}

void modem_reset_current_adr_mobile_count( void )
{
    lorawan_api_no_rx_packet_count_in_mobile_mode_set( 0 );
}

bool modem_available_new_link_adr_request( void )
{
    return lorawan_api_available_link_adr_get( );
}
void modem_set_test_mode_status( bool enable )
{
    is_modem_in_test_mode = enable;
}

bool modem_get_test_mode_status( void )
{
    return is_modem_in_test_mode;
}

void modem_context_set_rx_pathloss_db( int8_t rx_pathloss )
{
    rx_pathloss_db = rx_pathloss;
}

int8_t modem_context_get_rx_pathloss_db( void )
{
    return rx_pathloss_db;
}

void modem_context_set_tx_power_offset_db( int8_t tx_power_offset )
{
    tx_power_offset_db = tx_power_offset;
}

int8_t modem_context_get_tx_power_offset_db( void )
{
    return tx_power_offset_db;
}

radio_planner_t* modem_context_get_modem_rp( void )
{
    return modem_rp;
}

void modem_context_set_modem_rp( radio_planner_t* rp )
{
    modem_rp = rp;
}

void modem_context_empty_callback( void* ctx )
{
    // SMTC_MODEM_HAL_TRACE_ERROR( " empty call back \n" );
}

bool modem_context_suspend_radio_access( void )
{
    rp_radio_params_t fake_radio_params = { 0 };

    rp_task_t rp_task = {
        .hook_id               = 0,
        .launch_task_callbacks = modem_context_empty_callback,
        .duration_time_ms      = 20000,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .type                  = RP_TASK_TYPE_NONE,
        .start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 4,
    };

    rp_hook_status_t status = rp_task_enqueue( modem_rp, &rp_task, NULL, 0, &fake_radio_params );

    return ( status == RP_HOOK_STATUS_OK ) ? true : false;
}

bool modem_context_resume_radio_access( void )
{
    bool status = true;

    if( rp_task_abort( modem_rp, 0 ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to abort hook\n" );
        status = false;
    }
    smtc_modem_hal_stop_radio_tcxo( );

    if( ral_init( &modem_rp->radio->ral ) != RAL_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to init ral\n" );
        status = false;
    }

    if( ral_set_sleep( &modem_rp->radio->ral, true ) != RAL_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to set sleep\n" );
        status = false;
    }

    return status;
}

void modem_context_set_power_config_lut( uint8_t config[30] )
{
    // save raw buff into power lut
    for( uint8_t i = 0; i < POWER_CONFIG_LUT_SIZE; i++ )
    {
        power_config_lut[i].expected_power   = config[5 * i];
        power_config_lut[i].configured_power = config[( 5 * i ) + 1];
        power_config_lut[i].pa_param1        = config[( 5 * i ) + 2];
        power_config_lut[i].pa_param2        = config[( 5 * i ) + 3];
        power_config_lut[i].pa_ramp_time     = config[( 5 * i ) + 4];
    }
}

modem_power_config_t* modem_context_get_power_config_lut( void )
{
    return power_config_lut;
}

void modem_context_set_appkey( const uint8_t app_key[16] )
{
    uint32_t new_crc = crc( app_key, 16 );

    if( ( modem_appkey_status == MODEM_APPKEY_CRC_STATUS_INVALID ) || ( modem_appkey_crc != new_crc ) )
    {
        modem_appkey_crc    = new_crc;
        modem_appkey_status = MODEM_APPKEY_CRC_STATUS_VALID;
        lorawan_api_app_key_set( app_key );
        // Store appkey crc and status
        modem_store_context( );
    }
}

void modem_context_appkey_is_derived( void )
{
    modem_appkey_status = MODEM_APPKEY_CRC_STATUS_INVALID;
    modem_store_context( );
}

bool modem_context_get_network_type( void )
{
    return lorawan_api_get_network_type( );
}

void modem_context_set_network_type( bool network_type )
{
    lorawan_api_set_network_type( network_type );
}
/* --- EOF ------------------------------------------------------------------ */
