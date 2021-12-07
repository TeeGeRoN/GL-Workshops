/* SST25VF - drive the Microchip SST25VF Serial Flash using SPI
*/

#include "SST25VF.h"

/* Initializes the SST25VF.
* param _hspi - SPI_Handler
* */
void sst25vfInitSPI(SPI_HandleTypeDef *_hspi){
    hspi = *_hspi;
    desel();
}

/* Initializes the UART for SST25VF.
* param _huart - UART_Handler
* */
void sst25vfInitUART(UART_HandleTypeDef *_huart){
    huart = *_huart;
}
/* Select the FLASH: Chip Select low */
void sel() {
    HAL_GPIO_WritePin(GPORT, CS_PIN, 0);
}

/* Deselect the FLASH: Chip Select high */
void desel() {
    HAL_GPIO_WritePin(GPORT, CS_PIN, 1);
}
 /* write enable*/
void wren() { //дозволити запис
    sel();
    sendBt(WREN); //0x06
    desel();
    HAL_Delay(10);
}
/* Enable write status register */
void ewsr() {
	sel();
	sendBt(EWSR);
	desel();
}
/*Enable SO to output RY/BY# status during AAI programming */
void ebsy() {
	sel();
	sendBt(EBSY);
	desel();
}
/* write disable*/
void wrdi() { //заборонити запис
    sel();
    sendBt(WRDI); //0x04
    desel();
}
/* Disable SO as RY/BY# status during AAI programming */
void dbsy() {
    sel();
    sendBt(DBSY);
    desel();
}
/* Erase Full Memory Array */
void chipErase() {
    wren();
    sel();
    sendBt(CHIP_ERASE); //0x60
    desel();
    HAL_Delay(50);
}
/* set write status register to 0 (enable to write) */
void unlock() {
    sel();
	sendBt(WRITE_STATUS);
	sendBt(0);
    desel();
    HAL_Delay(10);
}
/* Read status from register */
char readStatus() {
    sel();
    sendBt(READ_STATUS);
        char result = sendBt(DUMMY_BYTE);
    desel();
    return result;
}

uint8_t writeReg(uint8_t addr, uint8_t val)
{
	uint8_t result;
	sel();
	sendBt(addr);
	result = sendBt(val);
	desel();
	return result;
}

/* Convert an address to the desired format.
* param command - the text to be printed.
* param address for converting.
* */
void prepareCommand(char command, uint32_t address) {

    sel();
    sendBt(command);
                /* Send WriteAddr high nibble address byte to write to */
    sendBt((address & 0xFF0000) >> 16);
                /* Send WriteAddr medium nibble address byte to write to */
    sendBt((address & 0xFF00) >> 8);
                /* Send WriteAddr low nibble address byte to write to */
    sendBt(address & 0xFF);
}

/* Write the part of text to the flash memory.
* param buff - the text to be printed
* param address for initial recording
* param len - length of the buff.
* */
uint8_t writePage(uint8_t *buff, uint32_t address, uint16_t len)
{
	uint8_t *buffer_start = buff;

	ewsr();					// Enable Write Status Register
	unlock();				// Disable write protection
	ebsy();					// Enable SO to output RY/BY# status during AAI programming
	wren();					// Send Write Enable

	prepareCommand(AAIWRITE, address);
	sendBt(*buff++);
	sendBt(*buff++);
	while(buff < buffer_start + len)
	  {
		desel();
		sel();
		while (sendBt(0xFF) != 0xFF){};
		desel();
		sel();
		sendBt(AAIWRITE);
		sendBt(*buff++);
		sendBt(*buff++);
	  }
	desel();
	sel();
	while (sendBt(0xFF) != 0xFF){};
	desel();
	wrdi();
	dbsy();
	waitForEndWritting();

	return writeReg(READ_STATUS, 0x00);	// Read Status Register
}
/* Wait for the end of writing */
void waitForEndWritting(void)
{
	uint8_t status = 0;
	sel();
	sendBt(READ_STATUS);
	do
	{
		status = sendBt(DUMMY_BYTE);
	} while ((status & 1) == 1); 	// Wait while write is in progress
	desel();
}
/* Byte sending to the param:address */
uint8_t sendBt(uint32_t address) {
	uint32_t tx = address;
	uint32_t res = 0;
	while (HAL_SPI_GetState(&hspi) == HAL_SPI_STATE_RESET){};
    HAL_SPI_TransmitReceive(&hspi, &tx, &res, 1, 50);
    while (HAL_SPI_GetState(&hspi) == HAL_SPI_STATE_RESET){};
    HAL_Delay(10);
    return res;
}
/* Byte reading from param:address */
char readByte(uint32_t address) {
    prepareCommand(READ, address); //0x03
    char result = sendBt(0);
    desel();
    return result;
}
/* Function read all memory and passet it into UART */
void readAllMemToUart() {
	int step = PAGE_SIZE; //default 4096
    char TxArray[PAGE_SIZE]={0};
    char RxArray[PAGE_SIZE]={0};
    //MAXSIZEMEM
    for (uint32_t addr = 0; addr < MAXSIZEMEM; addr += step)
    {
        prepareCommand(READ, addr);
        // Exchange data
        HAL_SPI_TransmitReceive(&hspi, TxArray, RxArray, step, 50);
        HAL_UART_Transmit(&huart, RxArray, step, 100);
        HAL_UART_Transmit(&huart, "\r\n", 2, 100);
        desel();
    }
}
/* Function search and read lines of time capsule from memory*/
void searchMemToUart()
{
	// Search line length. I thought it usually wouldn't exceed 150 characters,
	// but if that happens, just increase the value.
	int searchCount = 150;
	char buffer[PAGE_SIZE] = {0};
	for (uint32_t addr = 0; addr < MAXSIZEMEM; addr += PAGE_SIZE)
	{
		readFlash(buffer, addr, searchCount);
		HAL_Delay(50);
		for (int j = 0; j < searchCount; j++){
			if (buffer[j] == '\n' || buffer[j] == '\0' || buffer[j] == 255)
		  		break;
		  	HAL_UART_Transmit(&huart, &buffer[j], 1, 100);
		}
		if (buffer[0] == 255)
			return;
		HAL_UART_Transmit(&huart, "\r\n", 2, 100);
	}
}
/* Reading flash into param:buff, from param:addr
 * with size param:len*/
void readFlash(uint8_t *buff, uint32_t addr, uint16_t len)
{
	prepareCommand(READ, addr);
	while(len--)
		*buff++ = sendBt(DUMMY_BYTE);
	desel();
}
/* Erase the sector on param:addr*/
void sectorErase(uint32_t addr)
{
  wren();
  prepareCommand(SECER, addr);
  desel();
  waitForEndWritting();
}
/* Bulk Erase */
void bulkErase(void)
{
  wren();
  sel();
  sendBt(BULER);
  desel();
  waitForEndWritting();
}
