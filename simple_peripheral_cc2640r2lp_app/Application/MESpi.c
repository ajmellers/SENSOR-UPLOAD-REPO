/*
 * Copyright (c) 2019 MedEngine GmbH
 * All rights reserved.
 * Author: Alex Mellers (christian.mellor@medengine.co)
 *
 * SPI Wrapper functions to prevent D
 *
 *
 */

#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/SPI.h>

#include <ti/sysbios/BIOS.h>
#include <driverlib/ssi.h>

#include "Board.h"
#include "MESpi.h"
#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )

static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;
static PIN_Handle hSpiPin = NULL;

MemoryManager memManager;

// Table of pins to be "parked" in when no device is selected. Leaving the pin
// floating causes increased power consumption in WinBond W25XC10.
static PIN_Config BoardSpiInitTable[] = {
    CC2640R2_LAUNCHXL_SPI0_MOSI | PIN_PULLDOWN,
    CC2640R2_LAUNCHXL_SPI0_MISO | PIN_PULLDOWN,
    CC2640R2_LAUNCHXL_SPI0_CLK | PIN_PULLDOWN,
    PIN_TERMINATE
};

static uint8_t nUsers = 0;
static PIN_Handle handleCSPin = NULL;

/*********************************************************************
 * @fn          driveCSHigh
 *
 * @brief       Set SPI CS line state to low.
 *
 * @param       None.
 *
 * @return      None.
 */
void driveCSLow(){
    PIN_setOutputValue(handleCSPin,CC2640R2_LAUNCHXL_SPI_FLASH_CS,0);
    Task_sleep(10);
}

/*********************************************************************
 * @fn          driveCSHigh
 *
 * @brief       Set SPI CS line state to high.
 *
 * @param       None.
 *
 * @return      None.
 */
void driveCSHigh(){
    Task_sleep(10);
    PIN_setOutputValue(handleCSPin,CC2640R2_LAUNCHXL_SPI_FLASH_CS,1);
    Task_sleep(10);
}

/*********************************************************************
 * @fn          initSPI
 *
 * @brief       Initialize SPI communication.
 *
 * @param       None.
 *
 * @return      True on successful init process.
 */
bool initSPI() {
    if (hSpiPin != NULL) {
        // Remove IO configuration of SPI lines
        PIN_close(hSpiPin);

        hSpiPin = NULL;
    }
    SPI_init();

    if (spiHandle == NULL) {
        SPI_Params_init( & spiParams);
        spiParams.bitRate = 1000000;
        spiParams.dataSize = 8;
        spiParams.mode = SPI_MASTER;
        spiParams.transferMode = SPI_MODE_BLOCKING;
        spiParams.frameFormat = SPI_POL0_PHA0;
        /* Attempt to open SPI. */
        spiHandle = SPI_open(Board_SPI0, & spiParams);

        if (spiHandle == NULL) {
            Task_exit();
        }
    }
    delay_ms(100);

    nUsers++;
    return true;
}

/*********************************************************************
 * @fn          closeSPI
 *
 * @brief       End SPI communication.
 *
 * @param       None.
 *
 * @return      True on successful close process.
 */
bool closeSPI() {
    nUsers--;

    if (nUsers > 0) {
        // Don't close the driver if still in use
        return false;
    }

    if (spiHandle != NULL) {
        // Close the RTOS driver
        SPI_close(spiHandle);
        spiHandle = NULL;
    }

    if (hSpiPin == NULL) {
        // Configure SPI lines as IO and set them according to BoardSpiInitTable
        //    hSpiPin = PIN_open(&pinState, BoardSpiInitTable);
    }

    nUsers = 0;

    return true;
}


/*********************************************************************
 * @fn          readManDevID
 *
 * @brief       Read memory chip's manufacturer and device ID. memManager's
 *              data_to_read buffer will contain the function result, which
 *              should be 0xC2, 0x15 for MX25L3206E.
 *
 * @param       None.
 *
 * @return      True on successful read process.
 */
bool readManDevID() {
    uint8_t txBuf[ADDRESS_LENGTH+1] = {SPI_REMS,0x00,0x00,0x00};

    if (nUsers > 0) {
        driveCSLow();
        if (writeSPI(txBuf, ADDRESS_LENGTH+1)) {
            if (readSPI(memManager.data_to_read, 2)) {
                driveCSHigh();
                return true;
            }
            return false;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          readFromMemory
 *
 * @brief       Read 256B of data from the memory chip data_to_read
 *              buffer will contain the read data. When passing page
 *              address, flash storage accepts data in big endian.
 *              CC2640 holds data in little endian.
 *
 * @param       memAddr pointer to 3 element memory page address array
 *
 * @return      True on successful read process.
 */
bool readFromMemory(uint8_t * memAddr) {
    if (nUsers > 0) {
        driveCSLow();
           if (sendSPICommand(SPI_RDSR)) {
               if (readSPI(memManager.status_register, 1)) {
                   driveCSHigh();
                   while (memManager.status_register[0] == 0x01) {
                       driveCSLow();
                       if (sendSPICommand(SPI_RDSR)) {
                           if (readSPI(memManager.status_register, 1)) {
                               driveCSHigh();
                               }
                           }
                       }
                       driveCSLow();
                          if (sendSPICommand(SPI_READ)) {
                              if (passMemoryAddress(memAddr)) {
                                  if (readPageSPI(memManager.data_to_read)) {
                                      driveCSHigh();
                                      memManager.first_page_read = 1;
                                      return true;
                                  }
                                  return false;
                              }
                              return false;
                          }
                       return false;
                   }
                   return false;
           }
           return false;
    }
    return false;

}

/*********************************************************************
 * @fn          getMemoryPointers
 *
 * @brief       Scan memory to get first and last address of saved pages.
 *              memManager's first_page_addr and last_page_addr will
 *              store function's output. memory_address will store next
 *              free address to store new page.
 *
 * @param       None.
 *
 * @return      True on successful address scanning.
 */
bool getMemoryPointers() {
    uint8_t * ffBuff;
    ffBuff = malloc(DATA_RW_LENGTH * sizeof(uint8_t));
    int i;
    for (i = 0; i < DATA_RW_LENGTH; i++) {
        ffBuff[i] = 0xFF;
    }
    for (i = 0; i < ADDRESS_LENGTH; i++) {
        memManager.memory_address[i] = 0x00;
    }
    uint8_t maxPageAddr[ADDRESS_LENGTH] = {0x00, 0xFF, 0x3F};
    if (readFromMemory(memManager.memory_address)){
    }
    while ((memcmp(memManager.data_to_read, ffBuff, DATA_RW_LENGTH) == 0) && (memcmp(memManager.memory_address, maxPageAddr, ADDRESS_LENGTH) != 0)) {
        incrementAddress();
        if (readFromMemory(memManager.memory_address)){}
    }
    uint16_t tmpAddress = 0x0000;
    tmpAddress += memManager.memory_address[2] << 8;
    tmpAddress += memManager.memory_address[1];
    if (tmpAddress == 0x0000) {
        memManager.first_page_addr[1] = (uint8_t) tmpAddress;
        memManager.first_page_addr[2] = (uint8_t)(tmpAddress >> 8);
    }
    else if ((tmpAddress > 0x0000) && (tmpAddress < MAX_PAGE_ADDR)) {
        memManager.first_page_addr[1] = (uint8_t) tmpAddress;
        memManager.first_page_addr[2] = (uint8_t)(tmpAddress >> 8);
    } else {
        for (i = 0; i < ADDRESS_LENGTH; i++) {
            memManager.memory_address[i] = 0x00;
            memManager.first_page_addr[i] = 0x00;
            memManager.last_page_addr[i] = 0x00;
        }
    }
    memcpy(memManager.memory_address, memManager.first_page_addr, ADDRESS_LENGTH);
    if (readFromMemory(memManager.memory_address)){

    }
    while ((memcmp(memManager.data_to_read, ffBuff, DATA_RW_LENGTH) != 0) && (memcmp(memManager.memory_address, maxPageAddr, ADDRESS_LENGTH) != 0)) {
        incrementAddress();
        if (readFromMemory(memManager.memory_address)){
        }
    }
    tmpAddress = 0x0000;
    tmpAddress += memManager.memory_address[2] << 8;
    tmpAddress += memManager.memory_address[1];
    if (tmpAddress == 0x0000) {
        memManager.last_page_addr[1] = (uint8_t) tmpAddress;
        memManager.last_page_addr[2] = (uint8_t)(tmpAddress >> 8);
    }
    else if ((tmpAddress > 0x0000) && (tmpAddress < MAX_PAGE_ADDR)) {
        tmpAddress -= 0x0001;
        memManager.last_page_addr[1] = (uint8_t) tmpAddress;
        memManager.last_page_addr[2] = (uint8_t)(tmpAddress >> 8);
    } else {
        tmpAddress = MAX_PAGE_ADDR;
        memManager.last_page_addr[1] = (uint8_t) tmpAddress;
        memManager.last_page_addr[2] = (uint8_t)(tmpAddress >> 8);
    }
    free(ffBuff);
    return true;
}

/*********************************************************************
 * @fn          writeEnable
 *
 * @brief       Set WEL bit in status register
 *
 * @param       None.
 *
 * @return      None.
 */
void writeEnable() {
    driveCSLow();
    if (sendSPICommand(SPI_WREN)) {
        driveCSHigh();
        driveCSLow();
        if (sendSPICommand(SPI_RDSR)) {
            if (readSPI(memManager.status_register, 1)) {
                driveCSHigh();
            }
       }

    }
}

/*********************************************************************
 * @fn          storeAndVerify
 *
 * @brief       Save 256B of data to the memory chip. Verify writing process.
 *              memManager's data_to_save buffer should contain 256B of data
 *              to be saved. data_to_read buffer will contain verify output.
 *
 * @param       None.
 *
 * @return      True on successful save and verify process.
 */
bool storeAndVerify() {
    memManager.status_register[0] = 0x00;
    if (nUsers > 0) {
        driveCSLow();
        while (memManager.status_register[0] == 0x00) {
            writeEnable();
        }
        driveCSHigh();
        driveCSLow();
        if (sendSPICommand(SPI_PP)) {
            if (passMemoryAddress(memManager.memory_address)) {
                if (writeSPI(memManager.data_to_save, DATA_RW_LENGTH)) {
                    driveCSHigh();
                    driveCSLow();
                    if (sendSPICommand(SPI_RDSR)) {
                        if (readSPI(memManager.status_register, 1)) {
                            driveCSHigh();
                            while (memManager.status_register[0] == 0x01) {
                                driveCSLow();
                                if (sendSPICommand(SPI_RDSR)) {
                                    if (readSPI(memManager.status_register, 1)) {
                                        driveCSHigh();
                                    }
                                }
                            }
//                            if (verifyWritingProcess(memManager.memory_address)) {
                                memcpy(memManager.last_page_addr, memManager.memory_address, ADDRESS_LENGTH);
                                incrementAddress();
                                return true;
//                            }
                        }
                        return false;
                    }
                    return false;
                }
                return false;
            }
            return false;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          verifyWritingProcess
 *
 * @brief       Check if it's possible to perform read/write actions
 *              on the memory.
 *
 * @param       memAddr pointer to 3 element memory page address array
 *
 * @return      True if data has been saved correctly.
 */
bool verifyWritingProcess(uint8_t * memAddr) {
    if (readFromMemory(memAddr)) {
        if (memcmp(memManager.data_to_read, memManager.data_to_save, 23) == 0) {
            return true;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          didErasingEnd
 *
 * @brief       Check if erasing process has been completed.
 *
 * @param       None.
 *
 * @return      True on successful erase operation.
 */
bool didErasingEnd() {
    if (sendSPICommand(SPI_RDSR)) {
        if (readSPI(memManager.status_register, 1)) {
            driveCSHigh();
            while (memManager.status_register[0] == 0x01 || memManager.status_register[0] == 0x03) {
                driveCSLow();
                if (sendSPICommand(SPI_RDSR)) {
                    if (readSPI(memManager.status_register, 1)) {
                        driveCSHigh();
                    }
                }
            }
            return true;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          blockErase
 *
 * @brief       Delete data from the memory 64kB block. memManager's
 *              memory_address buffer should contain any block address.
 *
 * @param       blockAddr pointer to 3 element block address array
 *
 * @return      True on successful erase operation.
 */
bool blockErase(uint8_t * blockAddr) {
    memManager.status_register[0] = 0x00;
    if (nUsers > 0) {
        driveCSLow();
        while (memManager.status_register[0] == 0x00) {
            writeEnable();
        }
        driveCSHigh();
        driveCSLow();
        if (sendSPICommand(SPI_BE)) {
            if (passMemoryAddress(blockAddr)) {
                driveCSHigh();
                driveCSLow();
                if (didErasingEnd()) {
                    return true;
                }
                return false;
            }
            return false;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          sectorErase
 *
 * @brief       Delete data from the memory 4kB sector. Flash storage
 *              accepts data in big endian. CC2640 holds data in little
 *              endian. Updates first/last_page_addr.
 *
 * @param       sectorAddr pointer to 3 element sector address array
 *
 * @return      True on successful erase operation.
 */
bool sectorErase(uint8_t * sectorAddr) {
    memManager.status_register[0] = 0x00;
    if(memManager.first_page_read == 1) {
        if (nUsers > 0) {
            driveCSLow();
            while (memManager.status_register[0] == 0x00) {
                writeEnable();
            }
            driveCSHigh();
            driveCSLow();
            if (sendSPICommand(SPI_SE)) {
                if (passMemoryAddress(sectorAddr)) {
                    driveCSHigh();
                    driveCSLow();
                    if (didErasingEnd()) {
                        memManager.first_page_read = 0;
                        updateAddress(sectorAddr);
                        return true;
                    }
                    return false;
                }
                return false;
            }
            return false;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          chipErase
 *
 * @brief       Delete data from the whole memory chip. Flash storage
 *              accepts data in big endian. CC2640 holds data in little
 *              endian.
 *
 * @param       None.
 *
 * @return      True on successful erase operation.
 */
bool chipErase() {
    memManager.status_register[0] = 0x00;
    if (nUsers > 0) {
        driveCSLow();
        while (memManager.status_register[0] == 0x00) {
            writeEnable();
        }
        driveCSHigh();
        driveCSLow();
        if (sendSPICommand(SPI_CE)) {
            driveCSHigh();
            driveCSLow();
            if (didErasingEnd()) {
            }
            return false;
        }
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn          readWriteSPI
 *
 * @brief       Send SPI command to perform actions on the memory chip.
 *
 * @param       bufRx - pointer to SPI rx buffer
 * @param       bufTx - pointer to SPI tx buffer
 * @param       lenRx - rx buffer size
 * @param       lenTx - tx buffer size
 *
 * @return      True on successful SPI transfer.
 */
bool readWriteSPI(uint8_t * bufRx, uint8_t * bufTx, size_t lenRx, size_t lenTx) {
    SPI_Transaction masterTransaction;

    masterTransaction.count = lenRx+lenTx;
    masterTransaction.txBuf = bufTx;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = bufRx;

    return SPI_transfer(spiHandle, & masterTransaction) == TRUE;
}

/*********************************************************************
 * @fn          sendSPICommand
 *
 * @brief       Send SPI command to perform actions on the memory chip.
 *
 * @param       cmd - SPI command
 *
 * @return      True on successful SPI transfer.
 */
bool sendSPICommand(uint8_t cmd) {
    uint8_t spiCommandBuff[1] = {cmd};

    return readWriteSPI(NULL, spiCommandBuff, 0, 1);
}

/*********************************************************************
 * @fn          readPageSPI
 *
 * @brief       Read 256B of data from SPI.
 *
 * @param       buf - pointer to SPI rx buffer
 *
 * @return      True on successful SPI transfer.
 */
bool readPageSPI(uint8_t * buf) {
    return readWriteSPI(buf, NULL, DATA_RW_LENGTH, 0);
}

/*********************************************************************
 * @fn          passMemoryAddress
 *
 * @brief       Write memory address to SPI.
 *
 * @param       addr - pointer to 3B memory address buffer
 *
 * @return      True on successful SPI transfer.
 */
bool passMemoryAddress(uint8_t * addr) {
    return readWriteSPI(NULL, addr, 0, ADDRESS_LENGTH);
}

/*********************************************************************
 * @fn          writeSPI
 *
 * @brief       Write data to SPI.
 *
 * @param       buf - pointer to SPI tx buffer
 * @param       len - length of data to be sent
 *
 * @return      True on successful SPI transfer.
 */
bool writeSPI(uint8_t * buf, size_t len) {
    return readWriteSPI(NULL, buf, NULL, DATA_RW_LENGTH);
}

/*********************************************************************
 * @fn          readSPI
 *
 * @brief       Read data from SPI.
 *
 * @param       buf - pointer to SPI rx buffer
 * @param       len - length of data to be read
 *
 * @return      True on successful SPI transfer.
 */
bool readSPI(uint8_t * buf, size_t len) {
    return readWriteSPI(buf, NULL, len, 0);
}

/*********************************************************************
 * @fn          incrementAddress
 *
 * @brief       Increment memory address for next 256B page. memManager's
 *              memory_address buffer contains current address.
 *
 * @param       None.
 *
 * @return      None.
 */
void incrementAddress() {
    uint16_t tmpAddress = 0x0000;
    tmpAddress += memManager.memory_address[2] << 8;
    tmpAddress += memManager.memory_address[1];
    if (tmpAddress < MAX_PAGE_ADDR) {
        tmpAddress += 0x0001;
        memManager.memory_address[1] = (uint8_t) tmpAddress;
        memManager.memory_address[2] = (uint8_t)(tmpAddress >> 8);
    } else {
        memManager.memory_address[1] = (uint8_t) 0x00;
        memManager.memory_address[2] = (uint8_t) 0x00;
    }
}

/*********************************************************************
 * @fn          incrementReadAddress
 *
 * @brief       Increment memory address for next 256B page for given address.
 *
 * @param       None.
 *
 * @return      None.
 */
void incrementReadAddress(uint8_t * customAddress) {
    uint16_t tmpAddress = 0x0000;
    tmpAddress += customAddress[2] << 8;
    tmpAddress += customAddress[1];
    if (tmpAddress < MAX_PAGE_ADDR) {
        tmpAddress += 0x0001;
        customAddress[1] = (uint8_t) tmpAddress;
        customAddress[2] = (uint8_t)(tmpAddress >> 8);
    }
}

/*********************************************************************
 * @fn          getStorageStatus
 *
 * @brief       Calculate saved memory storage, where 0xFFFF function
 *              output indicates full memory.
 *
 * @param       None.
 *
 * @return      16bit storage status.
 */
uint8_t getStorageStatus() {
//    uint16_t tmpAddr = 0x0000;
//    tmpAddr += (memManager.last_page_addr[2] << 8);
//    tmpAddr += memManager.last_page_addr[1];
//    if (tmpAddr > 0x0000 && tmpAddr < MAX_PAGE_ADDR) {
//        tmpAddr += 0x0001;
//    }
//    tmpAddr -= memManager.first_page_addr[1];
//    tmpAddr -= (memManager.first_page_addr[2] << 8);
//    return ((tmpAddr * 0xFFFF)/MAX_PAGE_ADDR );
    uint16_t tmpAddr = 0x0000;
    uint8_t memStatus = 0x00;
    if(memManager.last_page_addr[2]<memManager.first_page_addr[2]) {
        tmpAddr += MAX_PAGE_ADDR;
    }
    tmpAddr += (memManager.last_page_addr[2] << 8) + memManager.last_page_addr[1];
    tmpAddr -= (memManager.first_page_addr[1] + (memManager.first_page_addr[2] << 8));
    memStatus = (tmpAddr/(MAX_PAGE_ADDR/0xFF))&0x00FF; //gives occupancy in range 0-FF
    return memStatus;
}

/*********************************************************************
 * @fn          memoryTest
 *
 * @brief       Check if it's possible to perform read/write actions
 *              on the memory. Red led - read only, white led - read and
 *              write ok.
 *
 * @param       None.
 *
 * @return      Memory test code (fail, read only, read and write).
 */
#ifdef INCLUDE_MEMORY_TEST
memoryTestCodes memoryTest() {
    uint8_t *memAddr;
    memAddr = calloc(ADDRESS_LENGTH, sizeof(uint8_t));
#ifdef LED_FOR_MEMORY_TEST
    PIN_setOutputValue(pinState, CC2640R2_LAUNCHXL_PIN_1LED, 0);
    PIN_setOutputValue(pinState, CC2640R2_LAUNCHXL_PIN_2LED, 0);
#endif
    sectorErase(memAddr);
    if (storeAndVerify()) {
        free(memAddr);
        #ifdef LED_FOR_MEMORY_TEST
        PIN_setOutputValue(pinState, CC2640R2_LAUNCHXL_PIN_2LED, 1);
        #endif
        return READ_WRITE;

    }
    sectorErase(memAddr);
    if (readManDevID()) {
        if ((memManager.data_to_read[0] == 0xC2) && (memManager.data_to_read[1] == 0x15)) {
            free(memAddr);
            #ifdef LED_FOR_MEMORY_TEST
            PIN_setOutputValue(pinState, CC2640R2_LAUNCHXL_PIN_1LED, 1);
            #endif
            return READ_ONLY;

        }
    }
    free(memAddr);
    return MEM_FAIL;
}
#endif

/*********************************************************************
 * @fn          updateAddress
 *
 * @brief       Update first_page_addr after erasing sector. Requires
 *              erasing sectors in specific order.
 *
 * @param       sectorAddr - pointer to 3B memory address buffer
 *
 * @return      None.
 */
void updateAddress(uint8_t * sectorAddr) {
    if (sectorAddr[1] != 0xF0) {
        sectorAddr[1] += 0x10;
    } else {
        sectorAddr[1] = 0x00;
        if(sectorAddr[2] == 0x3F) {
            sectorAddr[2] = 0x00;
        } else {
            sectorAddr[2] +=0x01;
        }
    }
    memManager.first_page_addr[1] = sectorAddr[1];
    memManager.first_page_addr[2] = sectorAddr[2];
}

uint16_t compareAddresses() {
    uint16_t difference = 0;
    if(memManager.last_page_addr[0] == memManager.first_page_addr[0] && memManager.last_page_addr[2] == memManager.first_page_addr[2]) {
        if(memManager.last_page_addr[1] > memManager.first_page_addr[1]) {
            difference = (uint16_t)(memManager.last_page_addr[1] - memManager.first_page_addr[1]);
            return difference;
        } else
            return 17;
    } else {
        return 17;
    }
}
