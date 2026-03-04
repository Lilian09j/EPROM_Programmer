/*
 * eprom_cmds.h
 *
 *  Created on: Aug 5, 2025
 *      Author: lilia
 */

#ifndef INC_EPROM_CMDS_H_
#define INC_EPROM_CMDS_H_

#include <shift_registers.h>

////deviceCode :
//D27C256 0x8D
//D2764A 0x08
//D27256 0x04
//D27C512 0x0D
//M27C512 0x3D
////ManufacturerId :
//STMicroelectronics 0x20
//Intel 0x89
typedef enum {D27C256=0x898D, D2764A=0x8908, D27256=0x8904, D27C512=0x890D, M27C512=0x203D, Unknown, AT28C64}EPROM_ID;
typedef enum {EPROM=0, E2PROM=1}EPROM_type;

typedef struct {
	EPROM_ID eprom_id;
	uint32_t nbr_of_bytes;//e.g. D27C512 has 2^16 addressable address bytes = 65536
	EPROM_type familly;
}eprom_type_parameters;

extern eprom_type_parameters plugged_eprom;

uint16_t read_ID_chip_EPROM(void);
uint8_t read_byte_EPROM(uint16_t address);
void writeEPROM(uint16_t address, uint8_t data);
void EPROM_reading_config(void);
void identify_chip(void);

#endif /* INC_EPROM_CMDS_H_ */
