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
 * This file is applicable for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0
 *
 */
#ifndef __BATTERY_CLIENT__
#define __BATTERY_CLIENT__

#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_battery_client.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#if BTSTACK_VER < 0x03000001
#include "bac_lib_v1.h"
#else
#include "bac_lib_v3.h"
#endif

#ifdef WICED_BT_TRACE_ENABLE
#define     BAC_LIB_TRACE                          WICED_BT_TRACE
#else
#define     BAC_LIB_TRACE(...)
#endif

/******************************************************
 *                  Constants
 ******************************************************/
/* service discovery states */
enum
{
    BAC_CLIENT_STATE_IDLE                       = 0x00,
    BAC_CLIENT_STATE_CONNECTED                  = 0x01,
    BAC_CLIENT_STATE_DISCOVER_CHARACTERISTIC    = 0x02,
    BAC_CLIENT_STATE_DISCOVER_BATTERY_LEVEL_CCD = 0x03,
};

typedef enum {
   BAS_BATTERY_LEVEL_IDX,
#ifdef BAS_1_1
   BAS_BATTERY_LEVEL_STATUS_IDX,
   BAS_ESTIMATED_SERVICE_DATE_IDX,
   BAS_BATTERY_CRITICAL_STATUS_IDX,
   BAS_BATTERY_ENERGY_STATUS_IDX,
   BAS_BATTERY_TIME_STATUS_IDX,
   BAS_BATTERY_HEALTH_STATUS_IDX,
   BAS_BATTERY_HEALTH_INFO_IDX,
   BAS_BATTERY_INFO_IDX,
   BAS_MANUFACTURE_NAME_IDX,
   BAS_MANUFACTURE_NUMBER_IDX,
   BAS_SERIAL_NUMBER_IDX,
   BAS_PRESENTATION_FORMAT_IDX,
#endif
   MAX_SUPPORTED_CHAR,
} bas_service_idx_e;

#define RETRY_ENABLE_DISABLE_TIME_IN_MS  400
#define RETRY_READ_TIME_IN_MS            600

enum
{
    BAC_RETRY_IDLE,
    BAC_RETRY_ENABLE,
    BAC_RETRY_DISABLE,
    BAC_RETRY_READ,
};

/******************************************************
 *                  Structures
 ******************************************************/
#define MAX_FIELD_SIZE 16
typedef struct
{
    uint8_t dynamic;                    /* boolean data to indicate if the data format length is dynamic */
    uint8_t len;                        /* when dynamic, this is min length, the field_len will contain each field length. when static, this is total format length. Use 0 for string (undefined length) */
    uint8_t field_len[MAX_FIELD_SIZE];  /* each field length when flag is set */
} char_format_t;

typedef struct
{
    uint16_t uuid;              /**< characteristic UUID type */
    uint8_t  mandatory_properties;/* mondatory property */
    char_format_t fmt;          /* field format */
    uint16_t handle;            /**< characteristic declaration handle */
    uint16_t val_handle;        /**< characteristic value attribute handle */
    uint16_t cccd_handle;       /* descriptor handle */
    wiced_bt_gatt_char_properties_t properties; /**< characteristic properties (see @link wiced_bt_gatt_char_properties_e wiced_bt_gatt_char_properties_t @endlink) */
    uint8_t  retry_action;
} bac_char_t;

typedef struct
{
    wiced_bt_battery_client_callback_t *p_callback;    /* Application BAC Callback */

    uint16_t conn_id;                       /* connection identifier */
    uint16_t bac_s_handle;                  /* Battery Service discovery start handle */
    uint16_t bac_e_handle;                  /* Battery Service discovery end handle */
    uint8_t  bac_current_state;             /* to avoid other requests during execution of previous request*/
    uint8_t  pending_action;
    wiced_timer_t retry_timer;              /* retry timer */

    /* during discovery below gets populated and gets used later on application request in connection state */
#ifdef WICED_BT_TRACE_ENABLE
    char *   char_name[MAX_SUPPORTED_CHAR];         /* characteristic name */
#endif
    bac_char_t characteristics[MAX_SUPPORTED_CHAR];
} wiced_bt_battery_client_cb_t;

/******************************************************
 *                  Extern
 ******************************************************/
extern wiced_bt_battery_client_cb_t wiced_bt_battery_client_cb;

/******************************************************
 *                  Function prototyping
 ******************************************************/
wiced_bt_gatt_status_t bac_gatt_send_read_by_handle(int i);

#endif // __BATTERY_CLIENT__
