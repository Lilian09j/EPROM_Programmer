/*
 * shift_registers.c
 *
 *  Created on: Jui 10, 2025
 *      Author: lilia
 */
#include <shift_registers.h>

//int address1 = 0b1100011011110000;

uint8_t registers_write_addr[NUMBER_OF_HC595_FOR_ADDR];//in little-endian The Least Significant Byte (LSB) is stored at the lowest memory address
uint8_t registers_write_data[NUMBER_OF_HC595_FOR_DATA];

void HC165_clock_rising_eadge(void){
	HAL_GPIO_WritePin(HC165_CLK_DATA_GPIO_Port, HC165_CLK_DATA_Pin, RESET);//clock low
	HAL_Delay(1);//1ms
	HAL_GPIO_WritePin(HC165_CLK_DATA_GPIO_Port, HC165_CLK_DATA_Pin, SET);//rising edge
	HAL_Delay(1);//1ms
	HAL_GPIO_WritePin(HC165_CLK_DATA_GPIO_Port, HC165_CLK_DATA_Pin, RESET);//clock low
	HAL_Delay(1);//1ms
}

void HC165_load_parallel_inputs(void){
	HAL_GPIO_WritePin(HC165_CLK_DATA_GPIO_Port, HC165_CLK_DATA_Pin, RESET);//clock low

	//load all the 8 inputs inside the toggles
	HAL_GPIO_WritePin(HC165__PL_DATA_GPIO_Port, HC165__PL_DATA_Pin, RESET);//asynchronous parallel load input (active LOW)
	HAL_Delay(1);//1ms

	// Clock to load values in latches (synchronous)
	HC165_clock_rising_eadge();

	HAL_GPIO_WritePin(HC165__PL_DATA_GPIO_Port, HC165__PL_DATA_Pin, SET);
	HAL_Delay(1);//1ms
}

/**
 * @brief 	Read the 8 bits serial output from PISO register present on the S999A
 * @param 	none
 * @retval 	serial_output : Byte with MSB first
 */
uint8_t HC165_read_8_inputs(void){

	uint8_t serial_output = 0;

	HC165_load_parallel_inputs();

	//shift the serial output for each of the 8 parallel inputs
	for(int i=0; i<(REGISTER_PINS_NUMBER_HC165_FOR_DATA); i++){
		//MSB first D7 D6 ... D0
		serial_output |= HAL_GPIO_ReadPin(HC165_DS_DATA_GPIO_Port, HC165_DS_DATA_Pin) << (REGISTER_PINS_NUMBER_HC165_FOR_DATA-(i+1));
		// Shift out the next bit
		HC165_clock_rising_eadge();
	}
	//  printf("serial_output: %x %b\n",serial_output,serial_output);

	return serial_output;//0b(D7 D6 D5 D4 D3 D2 D1 D0)
}

//reset the array for all register pins (set all register pins to LOW)
void clearRegister(uint8_t* register_bytes, uint8_t nbr_of_bytes){
  for(int i=0; i<nbr_of_bytes; i++){
	  register_bytes[i] = 0;
  }
}


// set value recorded in array "registers" and display on the end
//write value on shift register
void write_HC595_Registers_ADDR(void){//for EPROM address bus
  static bool val = false;//initialized only once at the first call of the function

  HAL_GPIO_WritePin(HC595_STCP_ADDR_Port, HC595_STCP_ADDR_Pin, RESET);//LOW

  for(int i = NUMBER_OF_HC595_FOR_ADDR - 1; i>=0; i--){ //loop to apply all values for each pin 74hc595 (send MSB first)
	  for(int j = PIN_IN_OUT_NUMBER_PER_REGISTERS - 1; j>=0; j--){
		  HAL_GPIO_WritePin(HC595_SHCP_ADDR_Port, HC595_SHCP_ADDR_Pin, RESET);//need to be low for change column soon
		  val = (registers_write_addr[i]>>j)&0x01;// catch value inside array registers
		  HAL_GPIO_WritePin(HC595_DS_ADDR_Port, HC595_DS_ADDR_Pin, val);//apply the value to a pin of 74hc595
		  HAL_GPIO_WritePin(HC595_SHCP_ADDR_Port, HC595_SHCP_ADDR_Pin, SET);// rising edge for clock and then next column
	  }
  }
  HAL_GPIO_WritePin(HC595_STCP_ADDR_Port, HC595_STCP_ADDR_Pin, SET);// apply (load) values to all 74hc595 pins
}

// set value recorded in array "registers" and display on the end
//write value on shift register
void write_HC595_Registers_DATA(void){//for EPROM data bus
  static bool val = false;//initialized only once at the first call of the function

  HAL_GPIO_WritePin(HC595_STCP_DATA_Port, HC595_STCP_DATA_Pin, RESET);//LOW

  for(int i = NUMBER_OF_HC595_FOR_DATA - 1; i>=0; i--){ //loop to apply all values for each pin 74hc595 (send MSB first)
	  for(int j = PIN_IN_OUT_NUMBER_PER_REGISTERS - 1; j>=0; j--){
		  HAL_GPIO_WritePin(HC595_SHCP_DATA_Port, HC595_SHCP_DATA_Pin, RESET);//need to be low for change column soon
		  val = (registers_write_data[i]>>j)&0x01;// catch value inside array registers
		  HAL_GPIO_WritePin(HC595_DS_DATA_Port, HC595_DS_DATA_Pin, val);//apply the value to a pin of 74hc595
		  HAL_GPIO_WritePin(HC595_SHCP_DATA_Port, HC595_SHCP_DATA_Pin, SET);// rising edge for clock and then next column
	  }
  }
  HAL_GPIO_WritePin(HC595_STCP_DATA_Port, HC595_STCP_DATA_Pin, SET);// apply (load) values to all 74hc595 pins
}

////NOT USED
////set an individual pin HIGH or LOW
//void setRegisterPin(uint8_t* register_bytes, uint8_t nbr_of_bytes, uint8_t index, bool value){
//  static uint8_t temp_reg = 0;
//
//  for(int i=0; i<nbr_of_bytes; i++){
//	  temp_reg = 0;
//	  for(int j=0; j<PIN_IN_OUT_NUMBER_PER_REGISTERS; j++){
//		  if(i*PIN_IN_OUT_NUMBER_PER_REGISTERS + j == index)
//			  temp_reg |= value;
//		  else
//			  temp_reg |= (register_bytes[i]>>j)&0x01;
//	  }
//	  register_bytes[i] = temp_reg;
//  }
//}

/*
 * Output the address bits and outputEnable signal using shift registers
 * set the 16 bits of address bus from A0 to A16 of the EPROM
 */
void set_EPROM_address(uint16_t address){
//  for(int i=0; i<REGISTER_PINS_NUMBER_HC595_FOR_ADDR; i++){
//	  setRegisterPin(registers_write_addr, NUMBER_OF_HC595_FOR_ADDR, i, address & (0b1<<i));
//  }

	//WARNING : THE BIT ORDER OF THE SHIFT REGISTER OUTPUTS FOR ADRRESS HAS BEEN REARRANGED TO SIMPLIFY THE ROOTING OF THE BOARD
	uint16_t address_modif = 0;
	address_modif |= ((address>>0)&0x01)<<A0_new_position;
	address_modif |= ((address>>1)&0x01)<<A1_new_position;
	address_modif |= ((address>>2)&0x01)<<A2_new_position;
	address_modif |= ((address>>3)&0x01)<<A3_new_position;
	address_modif |= ((address>>4)&0x01)<<A4_new_position;
	address_modif |= ((address>>5)&0x01)<<A5_new_position;
	address_modif |= ((address>>6)&0x01)<<A6_new_position;
	address_modif |= ((address>>7)&0x01)<<A7_new_position;
	address_modif |= ((address>>8)&0x01)<<A8_new_position;
	address_modif |= ((address>>9)&0x01)<<A9_new_position;
	address_modif |= ((address>>10)&0x01)<<A10_new_position;
	address_modif |= ((address>>11)&0x01)<<A11_new_position;
	address_modif |= ((address>>12)&0x01)<<A12_new_position;
	address_modif |= ((address>>13)&0x01)<<A13_new_position;
	address_modif |= ((address>>14)&0x01)<<A14_new_position;
	address_modif |= ((address>>15)&0x01)<<A15_new_position;

	for(int i=0; i<NUMBER_OF_HC595_FOR_ADDR; i++){
		registers_write_addr[i] = (address_modif>>(i*PIN_IN_OUT_NUMBER_PER_REGISTERS))&0xFF;
	}
	write_HC595_Registers_ADDR();// write value on shift register
}

/*
 * Output the 8 data bits and outputEnable signal using shift registers
 * set the 8 bits of data bus from D0 to D7 of the EPROM
 */
void set_EPROM_data(uint8_t data_byte){
//  for(int i=0; i<REGISTER_PINS_NUMBER_HC595_FOR_DATA; i++){
//	  setRegisterPin(registers_write_addr, NUMBER_OF_HC595_FOR_ADDR, i, address & (0b1<<i));
//  }
	for(int i=0; i<NUMBER_OF_HC595_FOR_DATA; i++){
		registers_write_data[i] = (data_byte>>i)&0xFF;
	}
	write_HC595_Registers_DATA();// write value on shift register
}

/**
 * @brief 	Read the value of the E-E2PROM data bus
 * 			Warning : The 8 bits of the value are O7 (MSB) to D0 pin and O0 (LSB) to D7 pin of the 75HC165 serial output
 * @param 	none
 * @retval 	byte_HC165 : unsigned integer value of the 8 bits data bus
 */
uint8_t read_eprom_8_bits_data_bus(void){
	uint8_t byte_HC165 = 0, data_bus = 0;

	byte_HC165 = HC165_read_8_inputs();//0b(O0 O1 O2 O3 O4 O5 O6 O7) 0b(D7 D6 D5 D4 D3 D2 D1 D0)

	//invert order
	for(int i=0; i<REGISTER_PINS_NUMBER_HC165_FOR_DATA; i++){
		data_bus |= ((byte_HC165>>(REGISTER_PINS_NUMBER_HC165_FOR_DATA-1-i))&0x01)<<i;
	}
//	printf("data_bus: %X\n", data_bus);

	return data_bus;
}
