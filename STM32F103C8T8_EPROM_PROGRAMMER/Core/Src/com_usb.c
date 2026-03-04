/*
 * com_usb.c
 *
 *  Created on: Jun 21, 2025
 *      Author: lilia
 */
#include <com_usb.h>
#include "usbd_cdc_if.h"
#include "eprom_cmds.h"

//void read_and_send_memory_bytes_through_usb(uint8_t* memory, uint32_t mem_size, uint16_t start_addr, uint32_t nbr_bytes);
void read_and_send_memory_bytes_through_usb(uint16_t start_addr, uint32_t nbr_bytes){
	static uint8_t csc = 0, byte_read = 0;
	uint32_t csc_full = 0;
	//uint16_t end_addr = start_addr+nbr_bytes;

	if((plugged_eprom.nbr_of_bytes)-start_addr < nbr_bytes){
		printf("reading asked out of bounds");
		return;
	}

	uint32_t payloadLength = 2+3+nbr_bytes;//2 bytes start address + 3 bytes nbr of bytes + nbr of bytes

	uint32_t size_answer = NBR_BYTES_INDEX+3+nbr_bytes+2;//CSC size + end char size + '\0' size
	char answer_frame[size_answer];
	memset(answer_frame, '\0', sizeof(answer_frame));

	answer_frame[0] = STM_ADDR;
	answer_frame[1] = APP_ADDR;
	answer_frame[LE_INDEX] = (payloadLength>>16)&0xFF;
	answer_frame[LE_INDEX+1] = (payloadLength>>8)&0xFF;
	answer_frame[LE_INDEX+2] = payloadLength&0xFF;
	answer_frame[FC_INDEX] = FC_READ;//FC of the GUI request
	answer_frame[START_ADDR_INEX] = start_addr>>8;
	answer_frame[START_ADDR_INEX+1] = start_addr&0xFF;
	answer_frame[NBR_BYTES_INDEX] = (nbr_bytes>>16)&0xFF;
	answer_frame[NBR_BYTES_INDEX+1] = (nbr_bytes>>8)&0xFF;
	answer_frame[NBR_BYTES_INDEX+2] = nbr_bytes&0xFF;

	//TODO : ONLY IF 28C64 BIT 14 HIGH TO BE IN READING MODE
//	start_addr += 0x4000;//0b1000000 00000000;
	EPROM_reading_config();
	for(int i=0; i<nbr_bytes; i++){
//		answer_frame[CONTENT_INDEX+i] += memory[start_addr+i];
		byte_read = read_byte_EPROM(start_addr+i);
		answer_frame[CONTENT_INDEX+i] += byte_read;
	}

	for(int i=0; i<size_answer; i++){
		csc_full += answer_frame[i];
	}

	csc = csc_full&0xFF;

	answer_frame[sizeof(answer_frame)-2] = csc;
	answer_frame[sizeof(answer_frame)-1] = END_CHAR;

	CDC_Transmit_FS((uint8_t*)answer_frame, (uint16_t)sizeof(answer_frame));
//	printf("TX - ASCII : %s\n",answer_frame);
	printf("TX - HEX : ");
	printHex((uint8_t*)answer_frame, sizeof(answer_frame));

	return;
}

void send_acknoledgment_to_GUI(uint8_t FC, uint16_t start_addr, uint32_t nbr_bytes){
	uint8_t csc = 0;
	uint32_t csc_full = 0;

	uint32_t payloadLength = 2+3;//2 bytes start address + 3 bytes nbr of bytes + nbr of bytes=0

	uint32_t size_answer = NBR_BYTES_INDEX+2+3;//2 bytes master slave + 1 byte FC + 3 bytes payload + 2 bytes start address + 3 bytes nbr of bytes + 0 bytes of data + CSC + END
	char answer_frame[size_answer];
	memset(answer_frame, '\0', sizeof(answer_frame));

	answer_frame[0] = STM_ADDR;
	answer_frame[1] = APP_ADDR;
	answer_frame[LE_INDEX] = (payloadLength>>16)&0xFF;
	answer_frame[LE_INDEX+1] = (payloadLength>>8)&0xFF;
	answer_frame[LE_INDEX+2] = payloadLength&0xFF;
	answer_frame[FC_INDEX] = FC;//FC of the GUI request
	answer_frame[START_ADDR_INEX] = start_addr>>8;
	answer_frame[START_ADDR_INEX+1] = start_addr&0xFF;
	answer_frame[NBR_BYTES_INDEX] = (nbr_bytes>>16)&0xFF;
	answer_frame[NBR_BYTES_INDEX+1] = (nbr_bytes>>8)&0xFF;
	answer_frame[NBR_BYTES_INDEX+2] = nbr_bytes&0xFF;

	for(int i=0; i<size_answer; i++){
		csc_full += answer_frame[i];
	}

	csc = csc_full&0xFF;

	answer_frame[sizeof(answer_frame)-2] = csc;
	answer_frame[sizeof(answer_frame)-1] = END_CHAR;

	CDC_Transmit_FS((uint8_t*)answer_frame, (uint16_t)sizeof(answer_frame));
	printf("TX - ASCII : %s\n",answer_frame);

//	printf("sent (hexa):");
//	for(int i=0; i<sizeof(answer_frame); i++){
//		printf("%02X ",answer_frame[i]);
//	}
//	printf("\n");

	printf("TX - HEX : ");
	printHex((uint8_t*)answer_frame, sizeof(answer_frame));

	return;
}

void send_memory_id_to_GUI(uint8_t FC, uint8_t manufacturerID, uint8_t deviceCode){
	uint8_t csc = 0;
	uint32_t csc_full = 0;

	uint32_t payloadLength = 2;//2 bytes (manufacturer ID and deviceCode)

	uint32_t size_answer = 10;//2 bytes master slave + 1 byte FC + 3 bytes payload + 2 bytes ID + CSC + END
	char answer_frame[size_answer];
	memset(answer_frame, '\0', sizeof(answer_frame));

	answer_frame[0] = STM_ADDR;
	answer_frame[1] = APP_ADDR;
	answer_frame[LE_INDEX] = (payloadLength>>16)&0xFF;
	answer_frame[LE_INDEX+1] = (payloadLength>>8)&0xFF;
	answer_frame[LE_INDEX+2] = payloadLength&0xFF;
	answer_frame[FC_INDEX] = FC;//FC of the GUI request
	answer_frame[INDEX_ID_MEM] = manufacturerID;
	answer_frame[INDEX_ID_MEM+1] = deviceCode;

	for(int i=0; i<size_answer; i++){
		csc_full += answer_frame[i];
	}

	//printf("csc_full : %ld\n", csc_full);
	csc = csc_full&0xFF;

	answer_frame[sizeof(answer_frame)-2] = csc;
	answer_frame[sizeof(answer_frame)-1] = END_CHAR;

	CDC_Transmit_FS((uint8_t*)answer_frame, (uint16_t)sizeof(answer_frame));
	printf("TX - ASCII : %s\n",answer_frame);

	printf("TX - HEX : ");
	printHex((uint8_t*)answer_frame, sizeof(answer_frame));

	return;
}

bool verify_check_sum(uint8_t* rx_buf, uint32_t nbr_bytes_to_write){
	  uint32_t csc_full = 0;
	  uint8_t csc_computed = 0;
	  uint8_t csc_read = 0;

	  if(rx_buf[FC_INDEX]==FC_READ){
		  for(int i=0; i<=NBR_BYTES_INDEX+2; i++){
			  csc_full += rx_buf[i];//computed check sum
		  }
		  csc_read = rx_buf[NBR_BYTES_INDEX+2+1];//check sum read
	  }
	  else if(rx_buf[FC_INDEX]==FC_WRITE){
		  for(int i=0; i<=NBR_BYTES_INDEX+2+nbr_bytes_to_write; i++){
			  csc_full += rx_buf[i];//computed check sum
//			  printf("i : %d; rx_buf[i] : %d; CSC full : %02x\n",i,rx_buf[i],(uint8_t)csc_full&0xFF);
		  }
		  csc_read = rx_buf[NBR_BYTES_INDEX+2+nbr_bytes_to_write+1];//check sum read
	  }
	  else{//MEM_ID
		  for(int i=0; i<INDEX_ID_MEM; i++){
			  csc_full += rx_buf[i];//computed check sum
		  }
		  //in that case the CSC index in the request correspond to INDEX_ID_MEM for the answer
		  csc_read = rx_buf[INDEX_ID_MEM];//check sum read
	  }
	  //printf("csc_read : %ld, csc_full : %ld\n", csc_read, csc_full);
	  csc_computed = csc_full&0xFF;
	  if(csc_computed != csc_read){
		  printf("erreur CSC\n");
		  return false;
	  }
	  else
		  return true;
}

void printHex(const uint8_t *hex_tab, size_t length){
    for (size_t i = 0; i < length; ++i) {
        printf("%02X ", hex_tab[i]);
    }
    printf("\n");
}


