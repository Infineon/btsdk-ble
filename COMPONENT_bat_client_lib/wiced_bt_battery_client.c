/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "bac_lib.h"
#include "wiced_memory.h"

// To cache handle of battery server characterstic descriptor
uint16_t wicked_bt_battery_client_server_characterstic_handle;

/******************************************************
 *                Variables Definitions
 ******************************************************/
#ifdef WICED_SDK_MAJOR_VER		// wiced_timer_t defined differently in BTSDK vs BTSTACK
wiced_bt_battery_client_cb_t wiced_bt_battery_client_cb = {0,0,0,0,0,0,{},
#else
wiced_bt_battery_client_cb_t wiced_bt_battery_client_cb = {0,0,0,0,0,0,{0},
#endif
#ifdef WICED_BT_TRACE_ENABLE
    {"Level",
 #ifdef BAS_1_1
     "Level Status",
     "Estimated Service Date",
     "Critical Status",
     "Energy Status",
     "Time Status",
     "Health Status",
     "Health Information",
     "Information",
     "Manufacture Name",
     "Manufacture Number",
     "Serial Number",
     "Format Presentation",
 #endif
    },
#endif
	0,
    //uint16_t uuid                                         mandatory_properties                                    format
    //-------------------------------------------           ----------------------------------                      ---
    {{UUID_CHARACTERISTIC_BATTERY_LEVEL,                    LEGATTDB_CHAR_PROP_READ,                                {WICED_FALSE,1}},
#ifdef BAS_1_1
     {UUID_CHARACTERISTIC_BATTERY_LEVEL_STATUS,             LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,    {WICED_TRUE,3,{2,1,1}}},
     {UUID_CHARACTERISTIC_BATTERY_ESTIMATED_SERVICE_DATE,   LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,    {WICED_FALSE,3}},
     {UUID_CHARACTERISTIC_BATTERY_CRITICAL_STATUS,          LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE,  {WICED_FALSE,1}},
     {UUID_CHARACTERISTIC_BATTERY_ENERGY_STATUS,            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,    {WICED_TRUE,1,{2,2,2,2,2,2}}},
     {UUID_CHARACTERISTIC_BATTERY_TIME_STATUS,              LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,    {WICED_TRUE,4,{3,3}}},
     {UUID_CHARACTERISTIC_BATTERY_HEALTH_STATUS,            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,    {WICED_TRUE,1,{1,2,1,2}}},
     {UUID_CHARACTERISTIC_BATTERY_HEALTH_INFO,              LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE,  {WICED_TRUE,1,{2,2}}},
     {UUID_CHARACTERISTIC_BATTERY_INFO,                     LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE,  {WICED_TRUE,3,{3,3,2,2,2,1,2,1}}},
     {UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NAME,         LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE,  {WICED_FALSE,0}},
     {UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NUMBER,       LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE,  {WICED_FALSE,0}},
     {UUID_CHARACTERISTIC_BATTERY_SERIAL_NUMBER,            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE,  {WICED_FALSE,0}},
     {UUID_CHARACTERISTIC_BATTERY_FORMAT_PRESENTATION,      LEGATTDB_CHAR_PROP_READ,                                {WICED_FALSE,1}},
#endif
    }};

/******************************************************
 *               Functions
 ******************************************************/
void battery_client_retry_timeout(TIMER_PARAM_TYPE count)
{
    wiced_bt_gatt_status_t status;
    int i;
    uint8_t next_action;

    for (i=0; i<MAX_SUPPORTED_CHAR; i++)
    {
        switch (wiced_bt_battery_client_cb.characteristics[i].retry_action)
        {
        case BAC_RETRY_ENABLE:
        case BAC_RETRY_DISABLE:
            if (wiced_bt_battery_client_cb.characteristics[i].properties & (LEGATTDB_CHAR_PROP_INDICATE|LEGATTDB_CHAR_PROP_NOTIFY))
            {
                uint16_t type = GATT_CLIENT_CONFIG_NONE;

                if (wiced_bt_battery_client_cb.characteristics[i].retry_action == BAC_RETRY_ENABLE)
                {
                    type = wiced_bt_battery_client_cb.characteristics[i].properties & LEGATTDB_CHAR_PROP_INDICATE ? GATT_CLIENT_CONFIG_INDICATION : GATT_CLIENT_CONFIG_NOTIFICATION;
                }
                BAC_LIB_TRACE("%s Battery %s ... ",
                          type == GATT_CLIENT_CONFIG_INDICATION ? "Eanbling indication" :
                          type == GATT_CLIENT_CONFIG_NOTIFICATION ? "Enabling notification" :
                          "Disabling",
                          wiced_bt_battery_client_cb.char_name[i],
                          wiced_bt_battery_client_cb.characteristics[i].cccd_handle);
                status = bac_set_gatt_client_config_descriptor( wiced_bt_battery_client_cb.conn_id, wiced_bt_battery_client_cb.characteristics[i].cccd_handle, type );
                if (status == WICED_BT_GATT_SUCCESS)
                {
                    BAC_LIB_TRACE("success\n");
                    wiced_bt_battery_client_cb.characteristics[i].retry_action = BAC_RETRY_IDLE;                }
                else
                {
                    BAC_LIB_TRACE("failed (%d)\n", status);
                }
                wiced_start_timer(&wiced_bt_battery_client_cb.retry_timer, RETRY_ENABLE_DISABLE_TIME_IN_MS);
                return;
            }
            wiced_bt_battery_client_cb.characteristics[i].retry_action = BAC_RETRY_IDLE;
            break;

        case BAC_RETRY_READ:
            // check if this uuid is supported for this device
            if (wiced_bt_battery_client_cb.characteristics[i].val_handle)
            {
                BAC_LIB_TRACE("Send Read Battery %s value ...", wiced_bt_battery_client_cb.char_name[i]);
                status = bac_gatt_send_read_by_handle(i);
                if (status == WICED_BT_GATT_SUCCESS)
                {
                    BAC_LIB_TRACE("success\n");
                    wiced_bt_battery_client_cb.characteristics[i].retry_action = BAC_RETRY_IDLE;                }
                else
                {
                    BAC_LIB_TRACE("failed (%d)\n", status);
                }
                wiced_start_timer(&wiced_bt_battery_client_cb.retry_timer, RETRY_READ_TIME_IN_MS);
                return;
            }
            wiced_bt_battery_client_cb.characteristics[i].retry_action = BAC_RETRY_IDLE;
            break;

        default:
            break;
        }
    }

    next_action = wiced_bt_battery_client_cb.pending_action;
    wiced_bt_battery_client_cb.pending_action = BAC_RETRY_IDLE;

    switch (next_action)
    {
    case BAC_RETRY_ENABLE:
        wiced_bt_battery_client_enable(wiced_bt_battery_client_cb.conn_id);
        break;

    case BAC_RETRY_DISABLE:
        wiced_bt_battery_client_disable(wiced_bt_battery_client_cb.conn_id);
        break;

    case BAC_RETRY_READ:
        wiced_bt_battery_client_read(wiced_bt_battery_client_cb.conn_id);
        break;
    }
}

#ifdef WICED_BT_TRACE_ENABLE
char * wiced_bt_battery_client_uuid_to_str(uint16_t uuid)
{
    int i;
    for (i=0; i<MAX_SUPPORTED_CHAR; i++)
    {
        if( uuid == wiced_bt_battery_client_cb.characteristics[i].uuid )
        {
            return wiced_bt_battery_client_cb.char_name[i];
        }
    }
    return "";
}
#endif

wiced_result_t wiced_bt_battery_client_init(wiced_bt_battery_client_callback_t *p_callback)
{
    wiced_bt_battery_client_cb.p_callback = p_callback;

    wiced_init_timer (&wiced_bt_battery_client_cb.retry_timer, &battery_client_retry_timeout, 0, WICED_MILLI_SECONDS_TIMER );
    return WICED_SUCCESS;
}

void wiced_bt_battery_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_battery_client_cb.conn_id = p_conn_status->conn_id;
    wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;
}

void wiced_bt_battery_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_battery_client_cb.conn_id = 0;
    wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_IDLE;
}

wiced_bt_gatt_status_t wiced_bt_battery_client_discover( uint16_t conn_id, uint16_t start_handle, uint16_t end_handle)
{
    if ((start_handle == 0) || (end_handle == 0))
        return WICED_BT_GATT_INVALID_HANDLE;

    wiced_bt_battery_client_cb.bac_s_handle = start_handle;
    wiced_bt_battery_client_cb.bac_e_handle = end_handle;
    wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_DISCOVER_CHARACTERISTIC;

    return wiced_bt_util_send_gatt_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, start_handle, end_handle);
}

/*
 * While application performs GATT discovery it shall pass discovery results for
 * for the handles that belong to BAC service, to this function.
 * Process discovery results from the stack.
 */
void wiced_bt_battery_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    int i; /* characteristics index */
    wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
    wiced_bt_gatt_char_descr_info_t *p_info = &p_data->discovery_data.char_descr_info;

//    BAC_LIB_TRACE("discovery_type:%d\n", p_data->discovery_type);

    if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS) &&
        (p_char->char_uuid.len == LEN_UUID_16))
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        for (i=0; i<MAX_SUPPORTED_CHAR; i++)
        {
//            BAC_LIB_TRACE("%d uuid:%04x %04x\n", i, p_char->char_uuid.uu.uuid16, wiced_bt_battery_client_cb.characteristics[i].uuid);
            if (p_char->char_uuid.uu.uuid16 == wiced_bt_battery_client_cb.characteristics[i].uuid)
            {
                // check for mandatory properties
                if ((p_char->characteristic_properties & wiced_bt_battery_client_cb.characteristics[i].mandatory_properties) ==
                    wiced_bt_battery_client_cb.characteristics[i].mandatory_properties)
                {
                    wiced_bt_battery_client_cb.characteristics[i].properties = p_char->characteristic_properties;
                    wiced_bt_battery_client_cb.characteristics[i].val_handle = p_char->val_handle;
                    wiced_bt_battery_client_cb.characteristics[i].handle = p_char->handle;
                    wiced_bt_battery_client_cb.characteristics[i].cccd_handle = 0;
                    BAC_LIB_TRACE("Found handle:%04x-%04x uuid:%04x Battery %s\n", p_char->handle, p_char->val_handle, p_char->char_uuid.uu.uuid16, wiced_bt_battery_client_cb.char_name[i]);
                }
                else
                {
                    BAC_LIB_TRACE("Error! Found handle:%04x-%04x uuid:%04x Battery %s,\n\tthe properties (x%x) does not meet the mandatory requirement (x%x)\n",
                        p_char->handle, p_char->val_handle, p_char->char_uuid.uu.uuid16,
                        wiced_bt_battery_client_cb.char_name[i],
                        p_char->characteristic_properties,
                        wiced_bt_battery_client_cb.characteristics[i].mandatory_properties);
                }
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_info->type.len == LEN_UUID_16))
    {
        if (p_info->type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
        {
            wiced_bt_battery_client_cb.characteristics[wiced_bt_battery_client_cb.cur_char_idx].cccd_handle = p_info->handle;
            //                    BAC_LIB_TRACE("Found CCCD handle:%04x for Battery %s\n", wiced_bt_battery_client_cb.characteristics[wiced_bt_battery_client_cb.cur_char_idx].cccd_handle, wiced_bt_battery_client_cb.char_name[wiced_bt_battery_client_cb.cur_char_idx]);
        }
        else if (p_info->type.uu.uuid16 == UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION)
        {
            wicked_bt_battery_client_server_characterstic_handle = p_info->handle;
            BAC_LIB_TRACE("Found Server Characteristic Descriptor, handle:%04x for Char %s\n",
                wicked_bt_battery_client_server_characterstic_handle, wiced_bt_battery_client_cb.char_name[wiced_bt_battery_client_cb.cur_char_idx]);
        }
    }
}

void wiced_bt_battery_client_discovery_complete(wiced_bt_gatt_discovery_complete_t* p_data)
{
#if BTSTACK_VER < 0x03000001
    wiced_bt_gatt_discovery_type_t disc_type = p_data->disc_type;
#else
    wiced_bt_gatt_discovery_type_t disc_type = p_data->discovery_type;
#endif
    wiced_bt_battery_client_event_data_t data_event;
    int i, j;

    BAC_LIB_TRACE("wiced_bt_battery_client_discovery_complete state:%d, %d\n", wiced_bt_battery_client_cb.bac_current_state, bac_gatt_discovery_complete_type(p_data));

    if (disc_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with BAC characteristics, start reading descriptor handles
        // Since Battery Level is mandatory, make sure the characteristics are present
        if ((wiced_bt_battery_client_cb.characteristics[BAS_BATTERY_LEVEL_IDX].handle == 0) ||
            (wiced_bt_battery_client_cb.characteristics[BAS_BATTERY_LEVEL_IDX].val_handle == 0))
        {
            // Error! Missing mandatory Battery Level characteristic
            BAC_LIB_TRACE("Error! Cannot find handles for mandatory Battery %s characteristics\n", wiced_bt_battery_client_cb.char_name[0]);
            wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;

            /* Tell the application that the GATT Discovery failed */
            data_event.discovery.conn_id = p_data->conn_id;
            data_event.discovery.status = WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;
            wiced_bt_battery_client_cb.p_callback(WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE, &data_event);
            return;
        }

        wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_DISCOVER_DESCRIPTORS;

        // Find the first characteristic which may have descriptors
        for (i = 0; i < MAX_SUPPORTED_CHAR; i++)
        {
            if (wiced_bt_battery_client_cb.characteristics[i].handle == 0)
            {
                // characteristic of this type is not present
                continue;
            }
            // found characteristic. Need to find next characteristic. If the characteristic were saved in the
            // order they were received, we could have done i + 1. but unfortunately they are saved everywhere in the characteristic array.
            // try to find the right one.
            uint16_t next_handle = wiced_bt_battery_client_cb.bac_e_handle;

#ifdef BAS_1_1
            for (j = 0; j < MAX_SUPPORTED_CHAR; j++)
            {
                if ((i == j) || (wiced_bt_battery_client_cb.characteristics[j].handle == 0))
                {
                    continue;
                }
                if (wiced_bt_battery_client_cb.characteristics[j].handle > wiced_bt_battery_client_cb.characteristics[i].val_handle)
                {
                    // this char is after current. But is it the next one;
                    if (wiced_bt_battery_client_cb.characteristics[j].handle < next_handle)
                    {
                        next_handle = wiced_bt_battery_client_cb.characteristics[j].handle - 1;
                    }
                }
            }
#endif
            // descriptors for the current characteristic may be located between val_handle + 1 and the next_handle which can be the
            // handle of the next characteristic or the last handle of the service.
            if (wiced_bt_battery_client_cb.characteristics[i].val_handle + 1 <= next_handle)
            {
                // this characteristic has descriptors
                wiced_bt_battery_client_cb.cur_char_idx = i;

                wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                    wiced_bt_battery_client_cb.characteristics[i].val_handle + 1, next_handle);
                break;
            }
        }
        if (i == MAX_SUPPORTED_CHAR)
        {
            BAC_LIB_TRACE("No characteristics with descriptors\n");
            wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;

            /* Tell the application that the GATT Discovery failed */
            data_event.discovery.conn_id = p_data->conn_id;
#ifndef ENABLE_BAC_LIB_320
            data_event.discovery.notification_supported = FALSE; // For BWC
#endif
            data_event.discovery.status = WICED_BT_GATT_SUCCESS;
            wiced_bt_battery_client_cb.p_callback(WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE, &data_event);
        }
    }
    else if (disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        // Completed descriptors discovery for characteristic with index wiced_bt_battery_client_cb.cur_char_idx.
        // If there are other characteristics, we may need to do more discovery procedures.
        for (i = wiced_bt_battery_client_cb.cur_char_idx + 1; i < MAX_SUPPORTED_CHAR; i++)
        {
            if (wiced_bt_battery_client_cb.characteristics[i].handle == 0)
            {
                // characteristic of this type is not present
                continue;
            }
            // found characteristic. Need to find the next characteristic. If the characteristic were saved in the
            // order they were received, we could have done i + 1. but unfortunately they are saved everywhere in the characteristic array.
            // try to find the right one.
            uint16_t next_handle = wiced_bt_battery_client_cb.bac_e_handle;
            for (j = 0; j < MAX_SUPPORTED_CHAR; j++)
            {
                if (wiced_bt_battery_client_cb.characteristics[j].handle > wiced_bt_battery_client_cb.characteristics[i].val_handle)
                {
                    // this char is after current. But is it the next one;
                    if (wiced_bt_battery_client_cb.characteristics[j].handle < next_handle)
                    {
                        next_handle = wiced_bt_battery_client_cb.characteristics[j].handle;
                    }
                }
            }
            // descriptors for the current characteristic may be located between val_handle + 1 and the next_handle which can be the
            // handle of the next characteristic or the last handle of the service.
            if (wiced_bt_battery_client_cb.characteristics[i].val_handle + 1 < next_handle)
            {
                // this characteristic has descriptors
                wiced_bt_battery_client_cb.cur_char_idx = i;

                wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                    wiced_bt_battery_client_cb.characteristics[i].val_handle + 1, next_handle);
                break;
            }
        }
        if (i == MAX_SUPPORTED_CHAR)
        {
            /* Tell the application that the GATT Discovery was successful */
            wiced_bt_battery_client_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;
            data_event.discovery.conn_id = p_data->conn_id;
            data_event.discovery.status = WICED_BT_GATT_SUCCESS;
#ifndef ENABLE_BAC_LIB_320
            data_event.discovery.notification_supported = wiced_bt_battery_client_cb.characteristics[BAS_BATTERY_LEVEL_IDX].properties & LEGATTDB_CHAR_PROP_NOTIFY ? TRUE : FALSE; // For BWC
#endif
            wiced_bt_battery_client_cb.p_callback(WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE, &data_event);
        }
    }
}

static wiced_bt_gatt_status_t battery_client_start_action(char * msg, uint8_t action)
{
    int i;

    if (wiced_bt_battery_client_cb.bac_current_state != BAC_CLIENT_STATE_CONNECTED)
    {
        BAC_LIB_TRACE("Illegal State: %d, cannot %s\n",wiced_bt_battery_client_cb.bac_current_state, msg);
        return WICED_BT_GATT_ERROR;
    }

    if (wiced_is_timer_in_use(&wiced_bt_battery_client_cb.retry_timer))
    {
        if (wiced_bt_battery_client_cb.pending_action == BAC_CLIENT_STATE_IDLE)
        {
            wiced_bt_battery_client_cb.pending_action = action;
            return WICED_BT_GATT_SUCCESS;
        }
        else
        {
            BAC_LIB_TRACE("Failed: Device busy, cannot %s\n", msg);
            return WICED_BT_GATT_ERROR;
        }
    }

    for (i=0; i<MAX_SUPPORTED_CHAR; i++)
    {
        wiced_bt_battery_client_cb.characteristics[i].retry_action = action;
    }

    battery_client_retry_timeout(0);
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t wiced_bt_battery_client_read(uint16_t conn_id)
{
    return battery_client_start_action("read BAS values", BAC_RETRY_READ);
}

void wiced_bt_battery_client_enable(uint16_t conn_id)
{
    battery_client_start_action("enable cccd", BAC_RETRY_ENABLE);
}

void wiced_bt_battery_client_disable(uint16_t conn_id)
{
    battery_client_start_action("disable cccd", BAC_RETRY_DISABLE);
}

static wiced_bool_t battery_client_valid_data(wiced_bt_gatt_operation_complete_t *p_data, int i)
{
    uint8_t expected_len;

    // check if it is a valid index
    if (i >= MAX_SUPPORTED_CHAR)
    {
        return WICED_FALSE;
    }

    if (!p_data)
    {
        WICED_BT_TRACE("Invalid wiced_bt_gatt_operation_complete_t pointer\n");
        return WICED_FALSE;
    }

    if (!p_data->response_data.att_value.p_data)
    {
        WICED_BT_TRACE("Error! Invalid response data pointer\n");
        return FALSE;
    }

    expected_len = wiced_bt_battery_client_cb.characteristics[i].fmt.len;

    // check if the expected data length is dynamic
    if (wiced_bt_battery_client_cb.characteristics[i].fmt.dynamic)
    {
        uint8_t * p_flag = &p_data->response_data.att_value.p_data[0];
        uint8_t * p_field = wiced_bt_battery_client_cb.characteristics[i].fmt.field_len;
        uint8_t bit_mask = 0;
        uint8_t flag;

        // check for each field length, the field length is terminated by 0
        while (*p_field)
        {
            // if mask is invalid, load data
            if (!bit_mask)
            {
                flag = *p_flag++;     // received data flag
                bit_mask = 0x1;
            }

            // if received data flag bit is set, the field length needs to be added
            if (flag & bit_mask)
            {
                expected_len += *p_field;    // get the length and move pointer to the next field
            }

            bit_mask <<= 1;                 // check for next field
            p_field++;

        }
    }
    else
    {
        // if this is string data (len=0), the length is unknown.
        if (expected_len == 0)
        {
            return WICED_TRUE;
        }
    }

    if (p_data->response_data.att_value.len != expected_len)
    {
        WICED_BT_TRACE("Error! Battery %s expected length=%d, actual len=%d\n",
                    wiced_bt_battery_client_cb.char_name[i],
                    expected_len,
                    p_data->response_data.att_value.len);
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

/*
 * With given event, call user callback functon .
 * Application passes it here if handle belongs to our service.
 */
static void battery_client_callback(wiced_bt_battery_client_event_t event, wiced_bt_gatt_operation_complete_t *p_data)
{
    int i;
    uint16_t handle = p_data->response_data.att_value.handle;
    wiced_bt_battery_client_event_data_t data_event;

    for (i=0; i<MAX_SUPPORTED_CHAR; i++)
    {
        // Find the handle
        if (handle == wiced_bt_battery_client_cb.characteristics[i].val_handle)
        {
            if (battery_client_valid_data(p_data, i))
            {
//                BAC_LIB_TRACE("Data for Battery %s: handle: %04x\n", wiced_bt_battery_client_cb.char_name[i], handle);

                data_event.data.conn_id = p_data->conn_id;
                data_event.data.status = p_data->status;
                data_event.data.uuid = wiced_bt_battery_client_cb.characteristics[i].uuid;
                data_event.data.handle = handle;
                data_event.data.len = p_data->response_data.att_value.len;
                data_event.data.offset = p_data->response_data.att_value.offset;
                data_event.data.p_data = p_data->response_data.att_value.p_data;
#ifndef ENABLE_BAC_LIB_320
                data_event.data.battery_level = *p_data->response_data.att_value.p_data; // For BWC
#endif
                wiced_bt_battery_client_cb.p_callback(event, &data_event);
            }
            return;
        }
    }

    BAC_LIB_TRACE("Error, handle %04x not found\n", handle);
}

void wiced_bt_battery_client_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    battery_client_callback(WICED_BT_BAC_EVENT_RSP, p_data);
}

void wiced_bt_battery_client_process_notification(wiced_bt_gatt_operation_complete_t *p_data)
{
    battery_client_callback(WICED_BT_BAC_EVENT_NOTIFICATION, p_data);
}

void wiced_bt_battery_client_process_indication(wiced_bt_gatt_operation_complete_t *p_data)
{
    battery_client_callback(WICED_BT_BAC_EVENT_INDICATION, p_data);
    WICED_BT_TRACE( "Send indication confirm id:%d handle:%04x\n", p_data->conn_id,  p_data->response_data.handle);
    wiced_bt_gatt_client_send_indication_confirm( p_data->conn_id, p_data->response_data.handle );
}

#ifndef ENABLE_BAC_LIB_320
//////////////////////////////////////////////
// backward compatible functions
//////////////////////////////////////////////
wiced_bt_gatt_status_t wiced_bt_bac_enable_notification( uint16_t conn_id )
{
    wiced_bt_battery_client_enable( conn_id );
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t wiced_bt_bac_disable_notification(uint16_t conn_id)
{
    wiced_bt_battery_client_disable( conn_id );
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t wiced_bt_bac_read_battery_level(uint16_t conn_id)
{
    BAC_LIB_TRACE("Send Read Battery %s value ...", wiced_bt_battery_client_cb.char_name[BAS_BATTERY_LEVEL_IDX]);
    wiced_bt_gatt_status_t status = bac_gatt_send_read_by_handle(BAS_BATTERY_LEVEL_IDX);
    if (status == WICED_BT_GATT_SUCCESS)
    {
        BAC_LIB_TRACE("success\n");
    }
    else
    {
        BAC_LIB_TRACE("failed (%d)\n", status);
    }
    return status;
}

#endif
