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

#ifndef _BAC_LIB_V1_
#define _BAC_LIB_V1_

/******************************************************
 *  Macros for btstack version 3 compatibilty
 ******************************************************/
#define wiced_bt_gatt_client_send_indication_confirm(id, handle) wiced_bt_gatt_send_indication_confirm(id, handle)
#define bac_gatt_discovery_complete_type(p) p->disc_type
#define WICED_BT_GATT_ATTRIBUTE_NOT_FOUND WICED_BT_GATT_NOT_FOUND
#define bac_set_gatt_client_config_descriptor(id, handle, val) wiced_bt_util_set_gatt_client_config_descriptor(id, handle, val)

#endif // _BAC_LIB_V1_
