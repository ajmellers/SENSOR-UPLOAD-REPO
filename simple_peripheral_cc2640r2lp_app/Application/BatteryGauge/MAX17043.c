/*
 * Copyright (c) 201 MedEngine GmbH
 * All rights reserved.
 * Author: Christian Mellor (christian.mellor@medengine.co)
 *
 * LSMDS1 Driver
 *
 *
 *
 */



#include "MAX17043.h"
#include "MEi2c.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )

bool startGaugeTransaction(uint8_t device)
{
    // Select slave
    return MESelectSlave(device);
}

void endGaugeTransaction()
{
    MEDeselectSlave();
}



void MAX17043_reset (){
    uint8_t data;
    data = 0x00;
    startGaugeTransaction(MAX17043_ADDRESS);
    MEWriteReg(MAX17043_COMMAND, &data, 1);
    data = 0x54;
    MEWriteReg(MAX17043_COMMAND, &data, 1);

    endGaugeTransaction();

}

void MAX17043_quickStart (){
    uint8_t data;
    data= 0x40;
    startGaugeTransaction(MAX17043_ADDRESS);
    MEWriteReg(MAX17043_MODE, &data, 1);
    data= 0x00;

    MEWriteReg(MAX17043_MODE, &data, 1);

    endGaugeTransaction();

}

void MAX17043Run (){
    MAX17043_reset ();
    delay_ms(200);
    MAX17043_quickStart ();
}

void MAX17043Version(uint8_t *destination){
    startGaugeTransaction(MAX17043_ADDRESS);
    uint8_t buffer[1] = { 0 };
    MEReadReg(MAX17043_VERSION, &buffer[0], 1);
    memcpy(destination, &buffer[0], 6);
    endGaugeTransaction();

}

void MAX17043StateOfCharge(uint8_t *destination){
   startGaugeTransaction(MAX17043_ADDRESS);
   uint8_t buffer[2] = { 0 };
   MEReadReg(0x02, &buffer[0], 2); // State of charge apparently high byte is percentage
   memcpy(destination, &buffer[0], 2);     //    SOC_HIGH_REG = 0x04, SOC_LOW_REG      = 0x05
   endGaugeTransaction();
}
