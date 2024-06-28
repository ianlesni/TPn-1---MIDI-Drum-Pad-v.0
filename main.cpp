/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
//=====[Libraries]=============================================================
#include "mbed.h"

//=====[Defines]===============================================================
#define WAIT_TIME_MS 500 
#define PIEZO_TRESHOLD_mV 30 
#define PIEZO_TRESHOLD_ADC_READ 37.23
#define NUMBER_OF_PIEZO_SAMPLES 100 //A definir porque no tengo criterio

#define MAX_VEL 127
#define MIN_VEL 1
#define PIEZO_MAX_PEAK_VOLT 1.86 //Máximo valor registrado( golpe muy fuerte) para este piezo

//=====[Declaration and initialization of public global objects]===============
AnalogIn piezo(A0);

static DigitalOut ledPad(LED1);// Create a DigitalOutput object to toggle an LED whenever data is received.
  
static UnbufferedSerial serialPort(USBTX, USBRX);// Create a UnbufferedSerial object with a default baud rate.

//=====[Declaration and initialization of public global variables]=============
char command = 0x90;
uint8_t note = 0x60;
uint8_t velocity = 0x64;
uint8_t velocityOff = 0x00;

float piezoMax = 0.0;

//=====[Declarations (prototypes) of public functions]=========================
void outputsInit();
float piezoSearchMax();
float piezoVoltToVel (float piezoMaxValue);

//=====[Main function, the program entry point after power on or reset]========
int main(void)
{
    DigitalIn button(D2);
    button.mode(PullDown);
    // Set desired properties (9600-8-N-1).
    serialPort.baud(9600);
    serialPort.format(8,SerialBase::None,1);
    
    outputsInit();
    
    while (true)
    {

        if( piezo.read_voltage() > PIEZO_TRESHOLD_mV)
        {
           piezoMax = piezoSearchMax();           
        }

        if(button == 1)
        {
             serialPort.write(&command, 1);
             serialPort.write(&note, 1);
             serialPort.write(&velocity, 1);

            ledPad = !ledPad;
            thread_sleep_for(WAIT_TIME_MS);
             
             serialPort.write(&command, 1);
             serialPort.write(&note, 1);
             serialPort.write(&velocityOff, 1);

        }
    }

}

//=====[Implementations of public functions]===================================
void outputsInit()
{
    ledPad = 0;
}

float piezoSearchMax()
{
    float piezoMaxValue = 0.0;
    float piezoSample = 0.0;
    int i = 0;
    for(i = 0; i < NUMBER_OF_PIEZO_SAMPLES; i++)
    {
        piezoSample = piezo.read_voltage();
         if(piezoSample > piezoMaxValue)
        {
            piezoMaxValue = piezoSample;
        }
    }
    
    return piezoMaxValue;

}

float piezoVoltToVel (float piezoMaxValue)
{
    float vel = 0.0;
    float slope = 0.0;
    float m = 0.0;

    slope = (MAX_VEL-MIN_VEL)/(PIEZO_MAX_PEAK_VOLT-PIEZO_TRESHOLD_mV);
    m = MIN_VEL - PIEZO_TRESHOLD_mV * slope;
    vel = piezoMaxValue*slope + m;
    return vel; 
}