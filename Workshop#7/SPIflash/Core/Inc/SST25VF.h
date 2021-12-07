
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef SST25VF_H
#define SST25VF_H



// command codes for SST25VF
#define WRITE_STATUS    0x01 // called WRSR in datasheet
#define WRITE           0x02 // Byte-Program
#define READ            0x03
#define WRDI            0x04
#define READ_STATUS     0x05 // called RDSR 
#define WREN            0x06
#define DUMMY_BYTE		0xA5	// Dummy Byte
//#define HSREAD          0x0B // High-Speed-Read
#define SECER 			0xD8 	// Sector Erase
#define BULER         	0xC7	// Bulk Erase
#define EWSR            0x50 // Enable-Write-Status-Register
#define EBSY			0x70
#define DBSY			0x80	// Disable SO as RY/BY# status during AAI programming
#define CHIP_ERASE      0x60
#define AAIWRITE        0xAD // word based write
#define GPORT           GPIOD
#define CS_PIN          GPIO_PIN_7
#define MAXSIZEMEM      (uint32_t)0x1fffff
#define PAGE_SIZE		4096

SPI_HandleTypeDef hspi;
UART_HandleTypeDef huart;

    char readByte(uint32_t address);

    void readCont(uint32_t address, uint8_t * buffer, uint32_t count);

    void writeByte(uint32_t address, char byte);


    void writeCont(uint32_t address, char * buffer, int count);

    void readAllMemToUart();

    void chipErase();
    char readStatus();


void sst25vfInitSPI(SPI_HandleTypeDef *_hspi);
void sst25vfInitUART(UART_HandleTypeDef *_huart);

void searchMemToUart();
uint8_t sendBt(uint32_t address);
void writeStatus(char status);
void prepareCommand(char command, uint32_t address);
void sel();
void desel();
void wren();
void unlock();
void wrdi();
uint8_t Memory_Driver_Write(uint8_t *buf, uint32_t address, uint16_t length);
uint8_t writeReg(uint8_t reg, uint8_t val);
void waitForEndWritting(void);
void ewsr();
void ebsy();
void wrdi();
void dbsy();

#endif
