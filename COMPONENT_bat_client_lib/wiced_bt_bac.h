/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * Battery Status Client Profile library.
 * This file is for backward compatiblity
 *
 */

#ifndef __WICED_BT_BAC_H_
#define __WICED_BT_BAC_H_

#ifndef ENABLE_BAC_LIB_320

#include "wiced_bt_battery_client.h"

#define wiced_bt_bac_event_t                    wiced_bt_battery_client_event_t
#define wiced_bt_bac_event_data_t               wiced_bt_battery_client_event_data_t

#define battery_level_rsp                       data
#define battery_level_notification              data

#define wiced_bt_bac_init                       wiced_bt_battery_client_init
#define wiced_bt_bac_read_rsp                   wiced_bt_battery_client_read_rsp
#define wiced_bt_bac_process_notification       wiced_bt_battery_client_process_notification
#define wiced_bt_bac_client_connection_up       wiced_bt_battery_client_connection_up
#define wiced_bt_bac_client_connection_down     wiced_bt_battery_client_connection_down
#define wiced_bt_bac_discovery_result           wiced_bt_battery_client_discovery_result
#define wiced_bt_bac_client_discovery_complete  wiced_bt_battery_client_discovery_complete
#define wiced_bt_bac_discover                   wiced_bt_battery_client_discover

#define WICED_BT_BAC_EVENT_BATTERY_LEVEL_NOTIFICATION   WICED_BT_BAC_EVENT_NOTIFICATION
#define WICED_BT_BAC_EVENT_BATTERY_LEVEL_RSP            WICED_BT_BAC_EVENT_RSP

wiced_bt_gatt_status_t wiced_bt_bac_enable_notification( uint16_t conn_id );
wiced_bt_gatt_status_t wiced_bt_bac_disable_notification( uint16_t conn_id );
wiced_bt_gatt_status_t wiced_bt_bac_read_battery_level(uint16_t conn_id);

#endif // ENABLE_BAC_LIB_320
#endif // __WICED_BT_BAC_H_
