/**
 * @file    crc8.h
 * @author  Simon Lövgren
 * @brief   Header for crc8 module
 */

#ifndef CRC8_H
#define CRC8_H

/**
 * *********************************************************************************************************************
 * Includes
 * *********************************************************************************************************************
 */
#include <inttypes.h>

/**
 * *********************************************************************************************************************
 * Public functions
 * *********************************************************************************************************************
 */

/**
 *  @brief      Calculates CRC8 of given data.
 *  @param[in]  pBuffer
 *              Pointer to buffer with data to calculate CRC8 of.
 *  @param[in]  length
 *              Length of data to calculate CRC8 of.
 *  @returns    CRC8 checksum
 */
uint8_t crc8( uint8_t *pBuffer, uint32_t length );

#endif // CRC8_H