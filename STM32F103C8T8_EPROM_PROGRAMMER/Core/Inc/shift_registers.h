/*
 * shift_registers.h
 *
 *  Created on: Jui 10, 2025
 *      Author: lilia
 */

#ifndef INC_SHIFT_REGISTERS_H_
#define INC_SHIFT_REGISTERS_H_

#include "main.h"

#define PIN_IN_OUT_NUMBER_PER_REGISTERS 8

//shift registers (serial input - parallel outputs) for ADDRESS bus WRITING
#define NUMBER_OF_HC595_FOR_ADDR 2//number of shift registers used
#define REGISTER_PINS_NUMBER_HC595_FOR_ADDR NUMBER_OF_HC595_FOR_ADDR*PIN_IN_OUT_NUMBER_PER_REGISTERS//number of total register pins

//shift registers (serial input - parallel outputs) for DATA bus WRITING
#define NUMBER_OF_HC595_FOR_DATA 1//number of shift registers used
#define REGISTER_PINS_NUMBER_HC595_FOR_DATA NUMBER_OF_HC595_FOR_DATA*PIN_IN_OUT_NUMBER_PER_REGISTERS//number of total register pins

//shift registers (parallel input - serial outputs) for DATA bus READING
#define NUMBER_OF_HC165_FOR_DATA 1//number of shift registers used
#define REGISTER_PINS_NUMBER_HC165_FOR_DATA NUMBER_OF_HC165_FOR_DATA*PIN_IN_OUT_NUMBER_PER_REGISTERS//number of total register pins

//WARNING : THE BIT ORDER OF THE SHIFT REGISTER OUTPUTS FOR ADRRESS HAS BEEN REARRANGED TO SIMPLIFY THE ROOTING OF THE BOARD
#define A0_new_position 15//A0 is connected to the output Q7 of the second serial register
#define A1_new_position 1
#define A2_new_position 2
#define A3_new_position 3
#define A4_new_position 4
#define A5_new_position 5
#define A6_new_position 6
#define A7_new_position 7
#define A8_new_position 11
#define A9_new_position 9
#define A10_new_position 14
#define A11_new_position 13
#define A12_new_position 8
#define A13_new_position 10
#define A14_new_position 0
#define A15_new_position 12

void HC165_clock_rising_eadge(void);
void HC165_load_parallel_inputs(void);
uint8_t HC165_read_8_inputs(void);
void write_HC595_Registers_ADDR(void);
void write_HC595_Registers_DATA(void);
void set_EPROM_address(uint16_t address);
void set_EPROM_data(uint8_t data_byte);
uint8_t read_eprom_8_bits_data_bus(void);

#endif /* INC_SHIFT_REGISTERS_H_ */
