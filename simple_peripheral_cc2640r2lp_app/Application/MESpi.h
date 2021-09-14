/*
 * Copyright (c) 2019 MedEngine GmbH
 * All rights reserved.
 * Author: Alex Mellers
 * SPI Wrapper functions
 *
 */

#include "stdbool.h"
#include "stdint.h"

#define SPI_WREN 0x06 //Write enable
#define SPI_RDSR 0x05 //Read status register
#define SPI_READ 0x03 //Read data
#define SPI_RDID 0x9F //Read identification
#define SPI_PP 0x02 //Page program
#define SPI_SE 0x20 //Erase 4kB sector
#define SPI_BE 0x52 //Erase 64kB block
#define SPI_CE 0x60 //Erase whole memory
#define SPI_REMS 0x90 //Read manufacturer and device ID

#define MAX_PAGE_ADDR 0x3FFF

#undef INCLUDE_MEMORY_TEST
#undef LED_FOR_MEMORY_TEST

#define DATA_RW_LENGTH 256
#define ADDRESS_LENGTH 3

typedef struct {
    uint8_t first_page_addr[ADDRESS_LENGTH];
    uint8_t last_page_addr[ADDRESS_LENGTH];
    uint8_t memory_address[ADDRESS_LENGTH];
    uint8_t data_to_save[DATA_RW_LENGTH];
    uint8_t data_to_read[DATA_RW_LENGTH];
    uint8_t status_register[1];
    uint8_t first_page_read;
} MemoryManager;

extern MemoryManager memManager;

#ifdef INCLUDE_MEMORY_TEST
typedef enum  {
    READ_ONLY = 0,
    READ_WRITE = 1,
    MEM_FAIL = 2
} memoryTestCodes;
#endif

extern void driveCSLow();

extern void driveCSHigh();

extern bool initSPI();

extern bool closeSPI();

extern bool readPageSPI(uint8_t * rxBuf);

extern bool passMemoryAddress(uint8_t * addr);

extern bool writeSPI(uint8_t * buf, size_t len);

extern bool readSPI(uint8_t * buf, size_t len);

extern bool readWriteSPI(uint8_t * bufRx, uint8_t * bufTx, size_t lenRx, size_t lenTx);

extern bool sendSPICommand(uint8_t cmd);

extern bool readManDevID();

extern bool readFromMemory(uint8_t * memAddr);

extern void writeEnable();

extern bool storeAndVerify();

extern bool didErasingEnd();

extern bool blockErase(uint8_t * blockAddr);

extern bool sectorErase(uint8_t * sectorAddr);

extern bool chipErase();

extern void incrementAddress();

extern void incrementReadAddress(uint8_t * customAddress);

extern uint8_t getStorageStatus();

extern bool getMemoryPointers();

extern bool verifyWritingProcess(uint8_t * memAddr);

#ifdef INCLUDE_MEMORY_TEST
extern memoryTestCodes memoryTest();
#endif

extern void updateAddress(uint8_t * sectorAddr);

extern uint16_t compareAddresses();

