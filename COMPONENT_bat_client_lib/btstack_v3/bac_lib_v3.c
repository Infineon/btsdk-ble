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
 * This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572
 *
 */

#ifndef BTSTACK_VER
#ifndef COMPONENT_btstack_v1
#include "wiced_bt_version.h"
#endif
#endif

#if BTSTACK_VER >= 0x03000001

#include "bac_lib.h"
#include "wiced_memory.h"

uint8_t bac_read_buffer[512];
wiced_bt_gatt_status_t bac_gatt_send_read_by_handle(int i)
{
    return wiced_bt_gatt_client_send_read_handle( wiced_bt_battery_client_cb.conn_id, wiced_bt_battery_client_cb.characteristics[i].val_handle, 0, bac_read_buffer, sizeof(bac_read_buffer), GATT_AUTH_REQ_NONE);
}

/*
 * bac_set_gatt_client_config_descriptor
 *
 * wiced_bt_util_set_gatt_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value) is not working properly, we replace with new function
 */
wiced_bt_gatt_status_t bac_set_gatt_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    wiced_bt_gatt_write_hdr_t write_hdr = { 0 };

    // Allocating a buffer to send the write request
    uint8_t* val = wiced_bt_get_buffer(sizeof(uint16_t));

    if (val)
    {
        WICED_MEMCPY(val, &value, sizeof(uint16_t));
        write_hdr.auth_req = GATT_AUTH_REQ_NONE;
        write_hdr.handle = handle; /* hard coded server ccd */
        write_hdr.offset = 0;
        write_hdr.len = 2;

        // Register with the server to receive notification
        status = wiced_bt_gatt_client_send_write(conn_id, GATT_REQ_WRITE, &write_hdr, val, NULL);
    }
    return status;
}

#endif // #if BTSTACK_VER >= 0x03000001
