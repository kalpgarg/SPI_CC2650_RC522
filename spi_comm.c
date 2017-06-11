/*
 * spi_comm.c
 *
 *  Created on: 01-Jun-2017
 *      Author: kalp garg
 */
/*
 * created on 01-06-17
 */

#include <ti/drivers/SPI.h>
#include "spi_comm.h"
#include <stdio.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <Board.h>

SPI_Handle g_handle;
static PIN_State ledPinState;
static PIN_Handle ledPinHandle;
PIN_Config LedPinTable[] =
{
    Board_LED0    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    Board_LED1    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    PIN_TERMINATE                                                                      /* Terminate list */
};


void spi_init(){
    SPI_Handle handle;
    SPI_Params params;

    SPI_Params_init(&params);
    params.bitRate=SPI_MASTER_BAUD_RATE;
    params.mode=SPI_MASTER;
    params.dataSize=SPI_DATA_SIZE;
    params.frameFormat=SPI_POL0_PHA0;

    handle = SPI_open(Board_SPI0, &params);

    if (!handle) {
       //puts("err\n");
    }

    //puts("spiInit \n");
    g_handle=handle;
}

void spi_transfer(uint8_t data){
   // //puts("wr st. \n");
    SPI_Transaction spiTransaction;
    SPI_Handle handle=g_handle;

    spiTransaction.count=sizeof(data);
    spiTransaction.txBuf=&data;
    spiTransaction.rxBuf=NULL;


    do{
        SPI_transfer(handle,&spiTransaction);
     }
        while(spiTransaction.status != SPI_TRANSFER_COMPLETED);
   // //printf("%u \n", data);

    switch(spiTransaction.status){

    case SPI_TRANSFER_COMPLETED:
       // //puts("stc \n");
    break;

    case SPI_TRANSFER_FAILED || SPI_TRANSFER_CANCELED:
        //puts("err\n");
    break;

    default: //puts(" def \n"); break;
    }
    ////puts("wr comp.\n");
}

uint8_t r_spi_transfer(uint8_t data){
   // //puts("rd st.\n");
    uint8_t rxdata;
    SPI_Transaction spiTransaction;
    SPI_Handle handle=g_handle;
    spiTransaction.count=1;
    spiTransaction.txBuf=&data;
    spiTransaction.rxBuf=&rxdata;

    do{
    SPI_transfer(handle,&spiTransaction);

    }
    while(spiTransaction.status != SPI_TRANSFER_COMPLETED);
    switch(spiTransaction.status){

    case SPI_TRANSFER_COMPLETED:
    break;

   case SPI_TRANSFER_FAILED || SPI_TRANSFER_CANCELED:
       //puts("err");
   break;
    }

  // //printf("%u \n",rxdata);
  // //puts("r comp. \n");
    return rxdata;
}

/**
 * Writes a uint8_t to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(uint8_t reg,   ///< The register to write to. One of the PCD_Register enums.
                                    uint8_t value          ///< The value to write.
                                ) {

    PIN_setOutputValue(ledPinHandle, Board_LED0,0);
    spi_transfer(reg & 0x7E);                       // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    spi_transfer(value);
    PIN_setOutputValue(ledPinHandle, Board_LED0,1);
    //puts("w\n");
} // End PCD_WriteRegister()

/**
 * Writes a number of uint8_ts to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister_m (uint8_t reg,   ///< The register to write to. One of the PCD_Register enums.
                                    uint8_t count,         ///< The number of uint8_ts to write to the register
                                    uint8_t *values        ///< The values to write. uint8_t array.
                                ) {
    PIN_setOutputValue(ledPinHandle, Board_LED0,0);
    spi_transfer(reg & 0x7E);                       // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index;
    for ( index =0; index < count; index++) {
        spi_transfer(values[index]);
    }
    PIN_setOutputValue(ledPinHandle, Board_LED0,1);
    //puts("w m");
} // End PCD_WriteRegister()



uint8_t PCD_ReadRegister( uint8_t reg )   ///< The register to read from. One of the PCD_Register enums.
{
    uint8_t value;
    PIN_setOutputValue(ledPinHandle, Board_LED0,0);
    spi_transfer(reg | 0x80);                    // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    value = r_spi_transfer(0x00);                        // Read the value back. Send 0 to stop reading.
    //puts("r \n");
    PIN_setOutputValue(ledPinHandle, Board_LED0,1);
    return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of uint8_ts from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegister_m( uint8_t reg,   ///< The register to read from. One of the PCD_Register enums.
                                uint8_t count,         ///< The number of uint8_ts to read
                                uint8_t *values,       ///< uint8_t array to store the values in.
                                uint8_t rxAlign        ///< Only bit positions rxAlign..7 in values[0] are updated.
                                ) {
    if (count == 0) {
        return;
    }
    uint8_t address = 0x80 | reg;              // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index = 0;                         // Index in values array.
    count--;                                // One read is performed outside of the loop
    PIN_setOutputValue(ledPinHandle, Board_LED0,0);
    spi_transfer(address);                   // Tell MFRC522 which address we want to read
    if (rxAlign) {      // Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        uint8_t value = r_spi_transfer(address);
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;
    }
    while (index < count) {
        values[index] = r_spi_transfer(address);   // Read value and tell that we want to read the same address again.
        index++;
    }
    values[index] = r_spi_transfer(0x00);         // Read the final uint8_t. Send 0 to stop reading.
    PIN_setOutputValue(ledPinHandle, Board_LED0,1);
    //puts("r m");
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(   PCD_Register reg,   ///< The register to update. One of the PCD_Register enums.
                                        uint8_t mask           ///< The bits to set.
                                    ) {
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp | mask);         // set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask( PCD_Register reg,   ///< The register to update. One of the PCD_Register enums.
                                        uint8_t mask           ///< The bits to clear.
                                      ) {
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask));      // clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PCD_CalculateCRC(  uint8_t *data,     ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                                uint8_t length,    ///< In: The number of uint8_ts to transfer.
                                                uint8_t *result    ///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
                     ) {
    //puts("pcd_calculateCRC");
    PCD_WriteRegister(CommandReg, PCD_Idle);        // Stop any active command.
    PCD_WriteRegister(DivIrqReg, 0x04);             // Clear the CRCIRq interrupt request bit
    PCD_WriteRegister(FIFOLevelReg, 0x80);          // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegister_m(FIFODataReg, length, data);   // Write data to the FIFO
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);     // Start the calculation

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
    // TODO check/modify for other architectures than Arduino Uno 16bit

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
    uint16_t i;
    for ( i= 5000; i > 0; i--) {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n = PCD_ReadRegister(DivIrqReg);
        if (n & 0x04) {                                 // CRCIRq bit set - calculation done
            PCD_WriteRegister(CommandReg, PCD_Idle);    // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = PCD_ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;
        }
    }
    // 89ms passed and nothing happend. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init() {
    ledPinHandle = PIN_open(&ledPinState, LedPinTable);

    PIN_setOutputValue(ledPinHandle, Board_LED1,0);
    __delay_cycles(10000);
    PIN_setOutputValue(ledPinHandle, Board_LED1,1);


    PCD_Reset();

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x32);
    uint8_t n=PCD_ReadRegister(ModWidthReg);
    //printf("\n n= %u",n);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);          // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9);     // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25us.
    PCD_WriteRegister(TReloadRegH, 0x03);       // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);

    PCD_WriteRegister(TxASKReg, 0x40);      // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(ModeReg, 0x3D);       // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn();                        // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset() {
    PCD_WriteRegister(CommandReg, PCD_SoftReset);   // Issue the SoftReset command.
    // The datasheet does not mention how long the SoftRest command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74us. Let us be generous: 50ms.
   // __delay_cycles(500000);
    // Wait for the PowerDown bit in CommandReg to be cleared
    while (PCD_ReadRegister(CommandReg) & (1<<4)) {
     //puts("in Wh \n"); // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
    }
    //puts("pcdR \n");
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn() {
    //puts("a on");
    uint8_t value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03) {
        //puts("c true");
        PCD_WriteRegister(TxControlReg, value | 0x03);
    }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff() {
    PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t PCD_GetAntennaGain() {
    return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void PCD_SetAntennaGain(uint8_t mask) {
    if (PCD_GetAntennaGain() != mask) {                     // only bother if there is a change
        PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));      // clear needed to allow 000 pattern
        PCD_SetRegisterBitMask(RFCfgReg, mask); // only set RxGain[2:0] bits
    }
} // End PCD_SetAntennaGain()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PCD_TransceiveData(    uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                                    uint8_t sendLen,       ///< Number of uint8_ts to transfer to the FIFO.
                                                    uint8_t *backData,     ///< NULL or pointer to buffer if data should be read back after executing the command.
                                                    uint8_t *backLen,      ///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
                                                    uint8_t *validBits,    ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
                                                    uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                    bool checkCRC       ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
                                 ) {
    //puts("pcd_transceivedata");
    uint8_t waitIRq = 0x30;        // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()


/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PCD_CommunicateWithPICC(   uint8_t command,       ///< The command to execute. One of the PCD_Command enums.
                                                        uint8_t waitIRq,       ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                                        uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                                        uint8_t sendLen,       ///< Number of uint8_ts to transfer to the FIFO.
                                                        uint8_t *backData,     ///< NULL or pointer to buffer if data should be read back after executing the command.
                                                        uint8_t *backLen,      ///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
                                                        uint8_t *validBits,    ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
                                                        uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                        bool checkCRC       ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
                                     ) {
    //puts("pcd_communicatewithpicc");
    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    PCD_WriteRegister(CommandReg, PCD_Idle);            // Stop any active command.
    PCD_WriteRegister(ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
    PCD_WriteRegister(FIFOLevelReg, 0x80);              // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegister_m(FIFODataReg, sendLen, sendData);  // Write sendData to the FIFO
    PCD_WriteRegister(BitFramingReg, bitFraming);       // Bit adjustments
    PCD_WriteRegister(CommandReg, command);             // Execute the command
    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86us.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    uint16_t i;
    for (i = 2000; i > 0; i--) {
        uint8_t n = PCD_ReadRegister(ComIrqReg);   // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) {   // One of the interrupts that signal success has been set.
           // printf("n= %u \n",n);
            break;
        }
        if (n & 0x01) {                     // Timer interrupt - nothing received in 25ms
            //puts("s t");
            return STATUS_TIMEOUT;
        }
        //printf("%u \n",i);
    }
    /*do{
        PCD_ReadRegister(ComIrqReg);
    }while(!())*/
    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0) {
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        //printf("%u \n",errorRegValue);
        //puts("st e");
        return STATUS_ERROR;
    }

    uint8_t _validBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        uint8_t n = PCD_ReadRegister(FIFOLevelReg);    // Number of uint8_ts in the FIFO
        //printf("%u \n n(2)",n);
        if (n > *backLen) {
            //puts(" s n r");
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                           // Number of uint8_ts returned
        //puts("bef r_m");
        PCD_ReadRegister_m(FIFODataReg, n, backData, rxAlign);    // Get received data from FIFO
        _validBits = PCD_ReadRegister(ControlReg) & 0x07;       // RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
        if (validBits) {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08) {     // CollErr
        //puts("st coll");
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4) {
            //puts("st mif nack");
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
        if (*backLen < 2 || _validBits != 0) {
            //puts("st crc_wr");
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t controlBuffer[2];
        enum StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != STATUS_OK) {
            //puts("not st ok");
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
            //puts("st_crc_wr");
            return STATUS_CRC_WRONG;
        }
    }
    //puts("st ok");
    return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_RequestA( uint8_t *bufferATQA,   ///< The buffer to store the ATQA (Answer to request) in
                                            uint8_t *bufferSize    ///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
                                        ) {
    //puts("picc_request A");
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_WakeupA(  uint8_t *bufferATQA,   ///< The buffer to store the ATQA (Answer to request) in
                                            uint8_t *bufferSize    ///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
                                        ) {
    return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_REQA_or_WUPA( uint8_t command,       ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
                                                uint8_t *bufferATQA,   ///< The buffer to store the ATQA (Answer to request) in
                                                uint8_t *bufferSize    ///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
                                            ) {
    //puts("picc_reqa_wupa");
    uint8_t validBits;
    enum StatusCode status;

    if (bufferATQA == NULL || *bufferSize < 2) { // The ATQA response is 2 uint8_ts long.
        //puts("status no room \n");
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                                  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,false);
    if (status != STATUS_OK) {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0) {       // ATQA must be exactly 16 bits.
        //printf("%u bufferSize",*bufferSize);
        //printf("%u validBits",validBits);
        //puts("status error 1\n");
        return STATUS_ERROR;
    }
    return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 *      - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 *      - The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 uint8_ts.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 *      UID size    Number of UID uint8_ts     Cascade levels      Example of PICC
 *      ========    ===================     ==============      ===============
 *      single               4                      1               MIFARE Classic
 *      double               7                      2               MIFARE Ultralight
 *      triple              10                      3               Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_Select(   Uid *uid,           ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
                                            uint8_t validBits      ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
                                         ) {
    bool uidComplete;
    bool selectDone;
    bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    enum StatusCode result;
    uint8_t count;
    uint8_t index;
    uint8_t uidIndex;                  // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits;       // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];                 // The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
    uint8_t bufferUsed;                // The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
    uint8_t rxAlign;                   // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;                // Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    //      uint8_t 0: SEL                 Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //      uint8_t 1: NVB                 Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits.
    //      uint8_t 2: UID-data or CT      See explanation below. CT means Cascade Tag.
    //      uint8_t 3: UID-data
    //      uint8_t 4: UID-data
    //      uint8_t 5: UID-data
    //      uint8_t 6: BCC                 Block Check Character - XOR of uint8_ts 2-5
    //      uint8_t 7: CRC_A
    //      uint8_t 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //      UID size    Cascade level   uint8_t2   uint8_t3   uint8_t4   uint8_t5
    //      ========    =============   =====   =====   =====   =====
    //       4 uint8_ts        1           uid0    uid1    uid2    uid3
    //       7 uint8_ts        1           CT      uid0    uid1    uid2
    //                      2           uid3    uid4    uid5    uid6
    //      10 uint8_ts        1           CT      uid0    uid1    uid2
    //                      2           CT      uid3    uid4    uid5
    //                      3           uid6    uid7    uid8    uid9

    // Sanity checks
    if (validBits > 80) {
        //puts("status invalid \n");
        return STATUS_INVALID;
    }

    // Prepare MFRC522
    PCD_ClearRegisterBitMask(CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete) {
        // Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
        switch (cascadeLevel) {
            case 1:
                buffer[0] = PICC_CMD_SEL_CL1;
                uidIndex = 0;
                useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 uint8_ts
                break;

            case 2:
                buffer[0] = PICC_CMD_SEL_CL2;
                uidIndex = 3;
                useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 uint8_ts
                break;

            case 3:
                buffer[0] = PICC_CMD_SEL_CL3;
                uidIndex = 6;
                useCascadeTag = false;                      // Never used in CL3.
                break;

            default:
                //puts("status internal error");
                return STATUS_INTERNAL_ERROR;
                break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0) {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag) {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t uint8_tsToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
        if (uint8_tsToCopy) {
            uint8_t maxuint8_ts = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (uint8_tsToCopy > maxuint8_ts) {
                uint8_tsToCopy = maxuint8_ts;
            }
            for (count = 0; count < uint8_tsToCopy; count++) {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag) {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone) {
            // Find out how many bits and uint8_ts to send and receive.
            if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK) {
                    //puts("result=");
                    //printf("%u \n",result);
                    return result;
                }
                txLastBits      = 0; // 0 => All 8 bits are valid.
                bufferUsed      = 9;
                // Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer  = &buffer[6];
                responseLength  = 3;
            }
            else { // This is an ANTICOLLISION.
                //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                txLastBits      = currentLevelKnownBits % 8;
                count           = currentLevelKnownBits / 8;    // Number of whole uint8_ts in the UID part.
                index           = 2 + count;                    // Number of whole uint8_ts: SEL + NVB + UIDs
                buffer[1]       = (index << 4) + txLastBits;    // NVB - Number of Valid Bits
                bufferUsed      = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer  = &buffer[index];
                responseLength  = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                           // Having a separate variable is overkill. But it makes the next line easier to read.
            PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,false);
            if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20) { // CollPosNotValid
                    //puts("status collision");
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0) {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
                    //puts("status internal error");
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count           = (currentLevelKnownBits - 1) % 8; // The bit to modify
                index           = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
                buffer[index]   |= (1 << count);
            }
            else if (result != STATUS_OK) {
                //puts("result=");
                //printf("%u \n",result);
                return result;
            }
            else { // STATUS_OK
                if (currentLevelKnownBits >= 32) { // This was a SELECT.
                    selectDone = true; // No more anticollision
                    // We continue below outside the while.
                }
                else { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID uint8_ts from buffer[] to uid->uidByte[]
        index           = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        uint8_tsToCopy     = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < uint8_tsToCopy; count++) {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
            //puts("status error 2");
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK) {
            //puts("result=");
            //printf("%u \n",result);
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
            //puts("status crc wrng");
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;
    //puts("status ok");
    return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_HaltA() {
    enum StatusCode result;
    uint8_t buffer[4];

    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }

    // Send the command.
    // The standard says:
    //      If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //      HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0,NULL,0,false);
    if (result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }
    return result;
} // End PICC_HaltA()

void PCD_StopCrypto1() {
    // Clear MFCrypto1On bit
    PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return bool
 */
bool PICC_IsNewCardPresent() {
    //puts("is new card \n");
    uint8_t bufferATQA[2];
    uint8_t bufferSize = sizeof(bufferATQA);

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return bool
 */
bool PICC_ReadCardSerial() {
    //puts("picc_read card serial \n");
    enum StatusCode result = PICC_Select(&uid,0);
    return (result == STATUS_OK);
} // End


/**
 * Dumps debug info about the connected PCD to Serial.
 * Shows all known firmware versions
 */
void PCD_DumpVersionToSerial() {
    // Get the MFRC522 firmware version
    uint8_t v = PCD_ReadRegister(VersionReg);
    //Serial.print(F("Firmware Version: 0x"));
    //Serial.print(v, HEX);
    // Lookup which version
 /*   switch(v) {
        case 0x88: Serial.println(F(" = (clone)"));  break;
        case 0x90: Serial.println(F(" = v0.0"));     break;
        case 0x91: Serial.println(F(" = v1.0"));     break;
        case 0x92: Serial.println(F(" = v2.0"));     break;
        default:   Serial.println(F(" = (unknown)"));
    }*/
    // When 0x00 or 0xFF is returned, communication probably failed
    if ((v == 0x00) || (v == 0xFF))
      puts("conn fail/n");
} // End PCD_DumpVersionToSerial()


