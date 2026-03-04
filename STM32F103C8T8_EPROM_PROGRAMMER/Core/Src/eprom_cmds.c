/*
 * eprom_cmds.c
 *
 *  Created on: Aug 5, 2025
 *      Author: lilia
 */

#include <eprom_cmds.h>

eprom_type_parameters plugged_eprom;

void EPROM_reading_config(void){
	HAL_GPIO_WritePin(MAX734__SHDN_GPIO_Port, MAX734__SHDN_Pin, RESET);

	//supply /OE with 0v to read from EPROM memory
	HAL_GPIO_WritePin(CMD0__OE_GPIO_Port, CMD0__OE_Pin, SET);
	HAL_GPIO_WritePin(CMD1__OE_GPIO_Port, CMD1__OE_Pin, RESET);

	HAL_GPIO_WritePin(CMD_A9_12V_GPIO_Port, CMD_A9_12V_Pin, RESET);//set A9 to TTL level by selecting the channel S1 of the DG419DY
	HAL_GPIO_WritePin(CMD_6V_VDD_EPROM_GPIO_Port, CMD_6V_VDD_EPROM_Pin, SET);//EPROM 5v supply for reading
	HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, RESET);//enable (chip select active low)
}

//for UV-EPROM 27 family only as EEPROM 28 family just has 32 bytes available initially at 0xFF
uint16_t read_ID_chip_EPROM(void){
	static uint8_t manufacturerID = 0, deviceCode = 0;
	static uint16_t address = 0;

	HAL_GPIO_WritePin(CMD_A9_12V_GPIO_Port, CMD_A9_12V_Pin, SET);//set A9 to 12v by selecting the channel S2 of the DG419DY

	address = 0b1100000000000000;//A0 low, A14 & A15 high (A9 wont take this TTL value as it was set to 12v)
	manufacturerID = read_byte_EPROM(address);

	address = 0b1100000000000001;//A0 high, A14 & A15 high
	deviceCode = read_byte_EPROM(address);

	printf("manufacturerID : %02x, deviceCode : %02x", manufacturerID, deviceCode);

	return (manufacturerID<<8)&deviceCode;
}

/*
 * Read a byte from the EEPROM at the specified address.
 */
uint8_t read_byte_EPROM(uint16_t address){
	uint8_t data = 0;

	set_EPROM_address(address);//set the 16 bits of address bus from A0 to A15 of the EPROM (if 27C512)
//	EPROM_reading_config();
//	HAL_Delay(1);
	data = read_eprom_8_bits_data_bus();

	return data;//return the 8 bits of data bus read
}

/*
 * Write a byte to the EEPROM at the specified address
 */
void writeEPROM(uint16_t address, uint8_t data){

	set_EPROM_address(address);
	set_EPROM_data(data);
	HAL_Delay(1);

	if(plugged_eprom.familly == EPROM){
		HAL_GPIO_WritePin(CMD_6V_VDD_EPROM_GPIO_Port, CMD_6V_VDD_EPROM_Pin, RESET);//EPROM 6v supply for writing, command NPN
		HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, SET);//unselected (chip select active low)
		HAL_Delay(5);

		//supply /OE with 12.5v to write in the EPROM memory
		HAL_GPIO_WritePin(CMD0__OE_GPIO_Port, CMD0__OE_Pin, RESET);
		HAL_GPIO_WritePin(CMD1__OE_GPIO_Port, CMD1__OE_Pin, SET);

		HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, RESET);//enable (chip select active low)
		HAL_Delay(1);//here 1ms (if D27C512) but 1us is enough
		HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, SET);//disable (chip select active low)
		HAL_Delay(5);

		HAL_GPIO_WritePin(CMD_6V_VDD_EPROM_GPIO_Port, CMD_6V_VDD_EPROM_Pin, SET);//EPROM 5v supply for reading
		HAL_Delay(5);
	}
	else{
		address &= (uint16_t)0xBFFF;//0b1011111111111111 to put A14 (/WE) at LOW to write
		set_EPROM_address(address);

		//supply /OE with 5v to stop displaying the output
		HAL_GPIO_WritePin(CMD0__OE_GPIO_Port, CMD0__OE_Pin, RESET);
		HAL_GPIO_WritePin(CMD1__OE_GPIO_Port, CMD1__OE_Pin, RESET);

		HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, SET);//disable (chip select active low)

		HAL_Delay(5);//TODO can be suppressed ?

		low_pulse_CE_1us();//using timer3 1MHz, delay (if 28C64) between 0.1 and 1us
		//HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, RESET);//enable (chip select active low)
		//HAL_GPIO_WritePin(EPROM__CE_GPIO_Port, EPROM__CE_Pin, SET);//disable (chip select active low)

		address |= (uint16_t)0x4000;//0b0100000000000000 to put A14 (/WE) at HIGH to go in reading mode
		set_EPROM_address(address);
	}
}

void identify_chip(void){

	plugged_eprom.eprom_id = read_ID_chip_EPROM();

	if(plugged_eprom.eprom_id != Unknown){
		return;
	}
//	if(plugged_eprom->eprom_id == D28C64)//TODO add later
//		plugged_eprom->familly = E2PROM;
//	else
//		plugged_eprom->familly = EPROM;

	plugged_eprom.familly = EPROM;

	if(plugged_eprom.eprom_id == D27C256 || plugged_eprom.eprom_id == D27256)
		plugged_eprom.nbr_of_bytes = 32768;//2^15 addresses
	else if(plugged_eprom.eprom_id == D2764A)
		plugged_eprom.nbr_of_bytes = 8192;//2^13 addresses
	else//(plugged_eprom->eprom_id == M27C512 || plugged_eprom->eprom_id == D27C512)
		plugged_eprom.nbr_of_bytes = 65536;//2^16 addresses
}
