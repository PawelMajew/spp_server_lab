///////////////////////////////////////////////////////////////////////
// File: gatt.h
//
// Subject: Standardy i Systemy Komunikacyjne, AGH, EiT
//
// Author: Paweł Majewski
//
///////////////////////////////////////////////////////////////////////
#ifndef GATT_H
#define GATT_H

///////////////////////////////////////////////////////////////////////////////////
// HEDER FILE
///////////////////////////////////////////////////////////////////////////////////

#include "ble_spp_server.h"

/**
 * @brief Handles GATT server events for the Generic Attribute Profile (GATT) server.
 *
 * This function processes GATT server events, such as registration events,
 * and dispatches the events to the appropriate callback functions.
 *
 * @param[in] event     GATT server callback event.
 * @param[in] gatts_if  GATT interface.
 * @param[in] param     Pointer to the GATT server callback parameters.
 */
//ZAD_1

//TODO_1

///////////////////////////////////

/**
 * @brief Initializes the SPP (Serial Port Profile) task.
 *
 * This function initializes the necessary elements for handling SPP,
 * including the UART interface initialization and the creation of a command queue.
 * Additionally, it creates a task to handle SPP commands.
 */
void spp_task_init(void);

#endif /* GATT_H */