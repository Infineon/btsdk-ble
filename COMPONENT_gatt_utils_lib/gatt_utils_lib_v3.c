/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/**
 * file gattt_utils_lib_v3.c
 *
 * Set of utility functions used by various applications
 */
#ifndef BTSTACK_VER
#ifndef COMPONENT_btstack_v1
#include "wiced_bt_version.h"
#endif
#endif

#if BTSTACK_VER >= 0x03000001

#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "gatt_utils_lib.h"

uint8_t gatt_util_lib_read_buffer[GATT_READ_HANDLE_BUFFER_SIZE];

/**
 * Set value of Client Configuration Descriptor
 */
wiced_bt_gatt_status_t wiced_bt_util_set_gatt_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    wiced_bt_gatt_write_hdr_t write_hdr = { 0 };

    // Allocating a buffer to send the write request
    uint8_t* val = wiced_bt_get_buffer(sizeof(uint16_t));
    uint8_t* p;

    if (val)
    {
        p = val;
        UINT16_TO_STREAM(p, value);
        write_hdr.auth_req = GATT_AUTH_REQ_NONE;
        write_hdr.handle = handle; /* hard coded server ccd */
        write_hdr.offset = 0;
        write_hdr.len = 2;

        // Register with the server to receive notification
        status = wiced_bt_gatt_client_send_write(conn_id, GATT_REQ_WRITE, &write_hdr, val, NULL);
    }
    return status;
}

/**
 * Format and send GATT discover request
 */
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param;

    memset(&param, 0, sizeof(param));
    if (uuid != 0)
    {
        param.uuid.len = LEN_UUID_16;
        param.uuid.uu.uuid16 = uuid;
    }
    param.s_handle = s_handle;
    param.e_handle = e_handle;

    return wiced_bt_gatt_client_send_discover(conn_id, type, &param);
}

/**
 * This utility function copies an UUID
 */
int wiced_bt_util_uuid_cpy(wiced_bt_uuid_t *p_dst, wiced_bt_uuid_t *p_src)
{
    if (p_src->len == LEN_UUID_16)
    {
        p_dst->uu.uuid16 =  p_src->uu.uuid16;
    }
    else if (p_src->len == LEN_UUID_32)
    {
        p_dst->uu.uuid32 =  p_src->uu.uuid32;
    }
    else if (p_src->len == LEN_UUID_128)
    {
        memcpy(p_dst->uu.uuid128, p_src->uu.uuid128, LEN_UUID_128);
    }
    else
    {
        return -1;
    }
    p_dst->len = p_src->len;
    return 0;
}

/**
 * This utility function Compares two UUIDs.
 * This function can only compare UUIDs of same length
 */
int wiced_bt_util_uuid_cmp(wiced_bt_uuid_t *p_uuid1, wiced_bt_uuid_t *p_uuid2)
{
    if (p_uuid1 == p_uuid2)
        return 0;

    /* Different UUID length */
    if (p_uuid1->len != p_uuid2->len)
        return -1;

    if (p_uuid1->len == LEN_UUID_16)
    {
        return (p_uuid1->uu.uuid16 != p_uuid2->uu.uuid16);
    }
    if (p_uuid1->len == LEN_UUID_32)
    {
        return (p_uuid1->uu.uuid32 != p_uuid2->uu.uuid32);
    }
    if (p_uuid1->len == LEN_UUID_128)
    {
        return memcmp(p_uuid1->uu.uuid128, p_uuid2->uu.uuid128, LEN_UUID_128);
    }
    return -1;
}

/**
 * Get attribute data from the look up table
 */
const gatt_db_lookup_table_t * wiced_bt_util_get_attribute(gatt_db_lookup_table_t * p_attribute, uint16_t handle)
{
    uint16_t limit = app_gatt_db_ext_attr_tbl_size;

    while(limit--)
    {
        if(p_attribute->handle == handle)
        {
            return p_attribute;
        }

        p_attribute++;
    }
    WICED_BT_TRACE("Requested attribute 0x%04x not found!!!", handle);
    return NULL;
}

#endif // #if BTSTACK_VER >= 0x03000001
