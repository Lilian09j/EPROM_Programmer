/*
 * com_usb.h
 *
 *  Created on: Jun 21, 2025
 *      Author: lilia
 */

#ifndef INC_COM_USB_H_
#define INC_COM_USB_H_

#include "main.h"

#define APP_ADDR 0x01
#define STM_ADDR 0xF4
#define LE_INDEX 2//3rd byte : payload length
#define FC_WRITE 0xD3
#define FC_ID_MEM 0xD5//get memory identification (manufacturer byte ID and device code)
#define FC_READ 0xD4//function code
#define FC_ERROR 0xA5//in case of wring check sum
#define FC_INDEX 5//4rd byte
#define START_ADDR_INEX 6//2 bytes of address (%04X)
#define NBR_BYTES_INDEX 8//over 3 bytes (%06X)
#define CONTENT_INDEX 11
#define END_CHAR 0x16
#define INDEX_ID_MEM 6

void read_and_send_memory_bytes_through_usb(uint16_t start_addr, uint32_t nbr_bytes);
void send_acknoledgment_to_GUI(uint8_t FC, uint16_t start_addr, uint32_t nbr_bytes);
bool verify_check_sum(uint8_t* rx_buf, uint32_t nbr_bytes_to_write);
void printHex(const uint8_t *hex_tab, size_t length);
void send_memory_id_to_GUI(uint8_t FC, uint8_t manufacturerID, uint8_t deviceCode);

#endif /* INC_COM_USB_H_ */
