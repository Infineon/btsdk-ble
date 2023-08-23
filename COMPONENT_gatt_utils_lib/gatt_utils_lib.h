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
 * @file gattt_utils_lib.h
 *
 * Set of gatt utility functions used by various applications.
 * This header file is used by btstack v3 or newer. For older stack, please
 * use the header file wiced_bt_gatt_util.h located in each device's CSP.
 *
 */
#ifndef BTSTACK_VER
#ifndef COMPONENT_btstack_v1
#include "wiced_bt_version.h"
#endif
#endif

#if BTSTACK_VER >= 0x03000001

#ifndef GATT_UTILS_LIB_H
#define GATT_UTILS_LIB_H

#include "wiced_bt_gatt.h"
#include "string.h"

#if __has_include ("cycfg_gatt_db.h")
#include "cycfg_gatt_db.h"
#else
/* External Lookup Table Entry */
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
}
gatt_db_lookup_table_t;
#endif

#define GATT_READ_HANDLE_BUFFER_SIZE 512

/* External definitions */
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t gatt_util_lib_read_buffer[GATT_READ_HANDLE_BUFFER_SIZE];

#define wiced_bt_util_send_gatt_read_by_handle(conn_id, handle) \
        wiced_bt_gatt_client_send_read_handle(conn_id, handle, 0, gatt_util_lib_read_buffer, GATT_READ_HANDLE_BUFFER_SIZE, GATT_AUTH_REQ_NONE)

/**
 * @brief   Set value of Client Configuration Descriptor
 *
 * @param[in]   conn_id   : GATT connection ID
 * @param[in]   handle    : Handle of the descriptor to modify
 * @param[in]   value     : Value to set
 *
 * @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 */
wiced_bt_gatt_status_t wiced_bt_util_set_gatt_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value);

/**
 * @brief   Format and send GATT discover request
 *
 * @param[in]  conn_id     : connection identifier.
 * @param[in]  type        : GATT discovery type.
 * @param[in]  uuid        : UUID of the attribute to search for.
 * @param[in]  s_handle    : Start handle.
 * @param[in]  e_handle    : Start handle.
 *
 * @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 */
wiced_bt_gatt_status_t wiced_bt_util_send_gatt_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid, uint16_t s_handle, uint16_t e_handle);

/**
 * @brief   This utility function copies an UUID
 *
 * @param[out] p_dst       : Destination UUID
 * @param[in]  p_src       : Source UUID
 *
 * @retval -1 Error
 * @retval 0 OK
 */
int wiced_bt_util_uuid_cpy(wiced_bt_uuid_t *p_dst, wiced_bt_uuid_t *p_src);

/**
 * @brief   This utility function Compares two UUIDs.
 *          This function can only compare UUIDs of same length
 *
 * @param[out] p_uuid1       : UUID1
 * @param[in]  p_uuid2       : UUID2
 *
 * @retval -1 Error
 * @retval 0 OK
 */
int wiced_bt_util_uuid_cmp(wiced_bt_uuid_t *p_uuid1, wiced_bt_uuid_t *p_uuid2);

/**
 * @brief   Get attribute data from the look up table
 *
 * @param[in]  gatt_db_lookup_table_t * p_attribute : Attribute look up table
 * @param[in]  uint16_t handle                      : The handle of the attribute.
 *
 * @return A pointer points to the attribute data. It returns NULL if the handle is invalid.
 */
const gatt_db_lookup_table_t * wiced_bt_util_get_attribute(gatt_db_lookup_table_t * p_attribute, uint16_t handle);

#endif // GATT_UTILS_LIB_H

#endif // #if BTSTACK_VER >= 0x03000001
