/**
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

/**
 * file gattt_utils_lib.c
 *
 * Set of gatt utility functions used by various applications
 */

#if !defined BTSTACK_VER && !defined COMPONENT_btstack_v1
#include "wiced_bt_version.h"
#endif

#if BTSTACK_VER < 0x03000001

#include "wiced.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "string.h"

/**
 * Set value of Client Configuration Descriptor
 */
wiced_bt_gatt_status_t wiced_bt_util_set_gatt_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value)
{
    wiced_bt_gatt_status_t status;
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 1];
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )buf;
    uint16_t               u16 = value;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = handle;
    p_write->offset   = 0;
    p_write->len      = 2;
    p_write->auth_req = GATT_AUTH_REQ_NONE;
    p_write->value[0] = u16 & 0xff;
    p_write->value[1] = (u16 >> 8) & 0xff;

    // Register with the server to receive notification
    status = wiced_bt_gatt_send_write (conn_id, GATT_WRITE, p_write);
    return status;
}

/**
 * Format and send GATT discover request
 *
 * @param[in]  conn_id     : connection identifier.
 * @param[in]  type        : GATT discovery type.
 * @param[in]  uuid        : UUID of the attribute to search for. When searching for any discovery type other than GATT_DISCOVER_SERVICES_BY_UUID,
 *                           the value 0 indicating it is searching for any uuid for that discvoery type with given handle range.
 *                           When discoverying by type GATT_DISCOVER_SERVICES_BY_UUID, uuid=0000 is a valid uuid16 value to discover.
 * @param[in]  s_handle    : Start handle.
 * @param[in]  e_handle    : Start handle.
 *
 */
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param;
    wiced_bt_gatt_status_t          status;

    memset(&param, 0, sizeof(param));

    // When we are discoverying for GATT_DISCOVER_SERVICES_BY_UUID or when discovery for any other
    // type with a particular uuid, we need to copy the uuid value and specify the uuid type.
    //
    if ( (type==GATT_DISCOVER_SERVICES_BY_UUID) || (uuid != 0) )
    {
        param.uuid.len = LEN_UUID_16;
        param.uuid.uu.uuid16 = uuid;
    }

    param.s_handle = s_handle;
    param.e_handle = e_handle;

    status = wiced_bt_gatt_send_discover(conn_id, type, &param);
    return status;
}

/**
 * Format and send GATT discover request
 */
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_discover_by_uuid128(uint16_t conn_id, uint8_t uuid[LEN_UUID_128], uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param;
    wiced_bt_gatt_status_t          status;

    param.uuid.len = LEN_UUID_128;
    memcpy(&param.uuid.uu.uuid128, uuid, LEN_UUID_128);
    param.s_handle = s_handle;
    param.e_handle = e_handle;

    status = wiced_bt_gatt_send_discover(conn_id, GATT_DISCOVER_SERVICES_BY_UUID, &param);
    return status;
}

/**
 * Format and send GATT Read by Handle request
 */
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_read_by_handle(uint16_t conn_id, uint16_t handle)
{
    wiced_bt_gatt_read_param_t param;
    wiced_bt_gatt_status_t     status;

    memset(&param, 0, sizeof(param));
    param.by_handle.handle = handle;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_HANDLE, &param);
    return status;
}

/**
 * Format and send Read by Type GATT request
 */
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_read_by_type(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, uint16_t uuid)
{
    wiced_bt_gatt_read_param_t param;
    wiced_bt_gatt_status_t     status;

    memset(&param, 0, sizeof(param));
    param.char_type.s_handle        = s_handle;
    param.char_type.e_handle        = e_handle;
    param.char_type.uuid.len        = 2;
    param.char_type.uuid.uu.uuid16  = uuid;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_TYPE, &param);
    return status;
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
        return - 1;
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

#endif // #if BTSTACK_VER < 0x03000001
