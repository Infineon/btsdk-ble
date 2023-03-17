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

/** @file
 *
 * Battery Status Client Profile library.
 * This file is applicable for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0
 *
 */

#ifndef BTSTACK_VER
#ifndef COMPONENT_btstack_v1
#include "wiced_bt_version.h"
#endif
#endif

#if BTSTACK_VER < 0x03000001

#include "bac_lib.h"

/*
 * bac_gatt_send_read -- send a read-by-handle request to server
 *
 *  parameter i -- index to array of characteristics
 */
wiced_bt_gatt_status_t bac_gatt_send_read_by_handle(int i)
{
    wiced_bt_gatt_read_param_t read_req;

    memset( &read_req, 0, sizeof( wiced_bt_gatt_read_param_t ) );
    read_req.by_handle.auth_req = GATT_AUTH_REQ_NONE;
    read_req.by_handle.handle = wiced_bt_battery_client_cb.characteristics[i].val_handle;
    return wiced_bt_gatt_send_read(wiced_bt_battery_client_cb.conn_id, GATT_READ_BY_HANDLE, &read_req);
}

#endif // #if BTSTACK_VER < 0x03000001
