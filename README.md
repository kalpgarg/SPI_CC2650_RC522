# SPI_CC2650_RC522
Contains library file for SPI communication between CC2650 or other similar ICs and RFID reader RC522
Add above files into your project's folder. Make sure you have already included header files. Then first call spi_init() and then PCD_Init() into your already running main Task or create one if it's doesn't present.
Then to read UID of tag you need following code:

if ( PICC_IsNewCardPresent() && PICC_ReadCardSerial()) {
              dump_byte_array(uid.uidByte,uid.size);
        }
        
void dump_byte_array(uint8_t*buffer, uint8_t bufferSize) {
        uint8_t i;
      for (i= 0; i < bufferSize; i++) {
        printf("buffer= %u \n",buffer[i]);
   }
}


