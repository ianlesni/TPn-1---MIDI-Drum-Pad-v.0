/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#define WAIT_TIME_MS 500 

// Create a DigitalOutput object to toggle an LED whenever data is received.
static DigitalOut led(LED1);
  
// Create a UnbufferedSerial object with a default baud rate.
static UnbufferedSerial serial_port(USBTX, USBRX);

char comando = 0x90;
uint8_t nota = 0x60;
uint8_t velocity = 0x64;
uint8_t velocity_off = 0x00;

int main(void)
{
    DigitalIn tecla(D2);
    tecla.mode(PullDown);
    // Set desired properties (9600-8-N-1).
    serial_port.baud(9600);
    serial_port.format(8,SerialBase::None,1);
    
    
    while (true)
    {

        if(tecla == 1)
        {
             serial_port.write(&comando, 1);
             serial_port.write(&nota, 1);
             serial_port.write(&velocity, 1);

            led = !led;
            thread_sleep_for(WAIT_TIME_MS);
             
             serial_port.write(&comando, 1);
             serial_port.write(&nota, 1);
             serial_port.write(&velocity_off, 1);

        }
    }

}
