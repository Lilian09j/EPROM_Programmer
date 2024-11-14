#define Pin_DS 2 //SHIFT_DATA
#define Pin_STCP 4 //SHIFT_LATCH
#define Pin_SHCP 3 //SHIFT_CLOCK
#define numberOf74hc595 2//nbr de registres a decalage utilises
#define numOfRegisterPins numberOf74hc595*8//number of total register pins

#define CMD_OE_BAR_0V A4 //output enable active low from UV-EPROM INTEL 27512
#define CMD_OE_BAR_12V_5V A3 //12v si high et CMD_OE_BAR_OV low, 0 ou 5v sinon
#define CE_bar A5 //Chip select active low of EPROM, A0 corresponds to digital pin 14
#define CMD_ALIM_EPROM_5V_6V A2//5v for reading if high ether 6v to write if low
#define EPROM_D0 6
#define EPROM_D7 13

#define MEMORY_SIZE 512

bool CE_enable = false, CE_disable = true;

//global variable
bool registers[numOfRegisterPins];//create an array with boolean, size of pin of 74HC595

void clearRegisters();
void writeRegisters();
void setRegisterPin(int index, int value);
void setAddress(int address);
byte readEPROM(int address);
void writeEPROM(int address, byte data);
void printContents();

void setup() {
  Serial.begin(9600);
  
  pinMode(Pin_DS, OUTPUT);
  pinMode(Pin_STCP, OUTPUT);
  pinMode(Pin_SHCP, OUTPUT);
  
  clearRegisters();//reset the array for all register pins
  writeRegisters();// write value on shift register
  
  for(int pin = EPROM_D7; pin >= EPROM_D0; pin -= 1){
    pinMode(pin, INPUT);
  }
  /*
  int address_to_write[] = {0x00F7, 0x00F8, 0x00F9};
  byte data_to_write[] = {0xAA, 0xAB, 0xAC};
  writeEPROM(0x00F3, 0x3A);
  
  for (int i=0; i<3; i++){
    writeEPROM(address_to_write[i], data_to_write[i]);
  }
  */
  printContents();
}

void loop(){

}  

//set all register pins to LOW
void clearRegisters(){
  for(int i = numOfRegisterPins - 1; i >=  0; i--){
     registers[i] = LOW;
  }
} 

// set value recorded in array "registers" and display on the end
void writeRegisters(){
  int val = 0;
  digitalWrite(Pin_STCP, LOW); // Until LOW modification wont be apply
  for(int i = numOfRegisterPins - 1; i >=  0; i--){ // loop for aplly all value for each pin 74hc595
    digitalWrite(Pin_SHCP, LOW);//need to be low for change column soon
    val = registers[i];// catch value insinde array registers
    digitalWrite(Pin_DS, val);//apply the value to a pin of 74hc595
    digitalWrite(Pin_SHCP, HIGH);// rising edge for clock and then next column
  }
  digitalWrite(Pin_STCP, HIGH);// apply value to all pin of 74hc595

}

//set an individual pin HIGH or LOW
void setRegisterPin(int index, int value){
  registers[index] = value;
}

/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void setAddress(int address){
  for(int i=0; i<numOfRegisterPins; i++){
    setRegisterPin(i, address & (0b1<<i));
  }
  writeRegisters();// write value on shift register
}

/*
 * Read a byte from the EEPROM at the specified address.
 */
byte readEPROM(int address) {
  digitalWrite(CMD_ALIM_EPROM_5V_6V, HIGH);//alim 5v
  setAddress(address);//set the 16 bits of address bus from A0 to A16 of the EPROM
  digitalWrite(CMD_OE_BAR_0V, HIGH);//put to low state the pin to enable Outputs
  digitalWrite(CE_bar, CE_enable);
  for (int pin = EPROM_D0; pin <= EPROM_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  byte data = 0;
  for(int pin = EPROM_D7; pin >= EPROM_D0; pin -= 1){
    data = (data << 1) + digitalRead(pin);
  }
  return data;//return the 8 bits of data bus read
}

/*
 * Write a byte to the EEPROM at the specified address.
 */
void writeEPROM(int address, byte data) {
  digitalWrite(CMD_ALIM_EPROM_5V_6V, LOW);//alim 6v, commande NPN
  digitalWrite(CE_bar, CE_disable);
  delay(20);
  setAddress(address);
  for (int pin = EPROM_D0; pin <= EPROM_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }

  for (int pin = EPROM_D0; pin <= EPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }

  //envoyer 12.5v dans OE pour ecriture
  digitalWrite(CMD_OE_BAR_0V, LOW);
  digitalWrite(CMD_OE_BAR_12V_5V, HIGH);

  digitalWrite(CE_bar, CE_enable);
  delayMicroseconds(1);
  digitalWrite(CE_bar, CE_disable);
  delay(10);

  digitalWrite(CMD_ALIM_EPROM_5V_6V, HIGH);//alim 5v
  delay(10);
}

/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents() {
  Serial.println("\r\n-----------------------------------\r\n");

  for (int base = 0; base <= MEMORY_SIZE-1; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}

