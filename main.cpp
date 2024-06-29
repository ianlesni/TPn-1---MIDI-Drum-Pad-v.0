/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
//=====[Libraries]=============================================================
#include "mbed.h"
#include <cstdint>

//=====[Defines]===============================================================


#define NUMBER_OF_PIEZO_SAMPLES 400 
#define SAMPLE_FREQ_Hz 40000
#define SAMPLE_TIME_INTERVAL_uS 25


#define MAX_VEL 127
#define MIN_VEL 50 //Con velocitys más bajas apenas se escucha
#define PIEZO_MAX_PEAK_VOLT_mV 2000 //Máximo valor registrado( golpe muy fuerte) para este piezo
#define PIEZO_THRESHOLD_mV 90 

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
float piezoRead = 0.0;
uint8_t piezoTestInt = 0;

float slope = 0.0;
float intercept = 0.0;

//=====[Declarations (prototypes) of public functions]=========================
void outputsInit();
void calculateSlopeIntercept(void);
float piezoSearchMax();
uint8_t piezoVoltToVel (float piezoMaxValue);


//=====[Main function, the program entry point after power on or reset]========
int main(void)
{
    DigitalIn B1_USER(BUTTON1);
    // Set desired properties (9600-8-N-1).
    serialPort.baud(9600);
    serialPort.format(8,SerialBase::None,1);

    outputsInit();
    calculateSlopeIntercept();

    serialPort.write("MIDI DrumPad",12);
    while (true)
    {
        piezoRead = piezo.read();
        piezoRead = piezoRead*3.3*1000;

        if(piezoRead  > PIEZO_THRESHOLD_mV)
        {
            ledPad = 1;
            piezoMax = piezoSearchMax();
            velocity = piezoVoltToVel(piezoMax);  
            ledPad = 0;    
            //Apago el golpe anterior
            serialPort.write(&command, 1);
            serialPort.write(&note, 1);
            serialPort.write(&velocityOff, 1);
            //Mando el golpe actual
            serialPort.write(&command, 1);
            serialPort.write(&note, 1);
            serialPort.write(&velocity, 1);     
        }

    }

}

//=====[Implementations of public functions]===================================
void outputsInit()
{
    ledPad = 0;
}
void calculateSlopeIntercept()
{
    uint16_t deltaVel = MAX_VEL - MIN_VEL; // Delta de velocidad
    uint16_t deltaVolt = PIEZO_MAX_PEAK_VOLT_mV - PIEZO_THRESHOLD_mV; // Delta de voltaje

    slope = (float)deltaVel / deltaVolt; //Calculo de pendiente
    intercept = MIN_VEL - PIEZO_THRESHOLD_mV * slope; //Calculo ordenada al origen
}

float piezoSearchMax()
{
    float piezoMaxValue = 0.0;
    float piezoSample = 0.0;
    int i = 0;

    for(i = 0; i < NUMBER_OF_PIEZO_SAMPLES; i++) //Muestreo el golpe detectado
    {
        piezoSample = piezo.read();
        piezoSample = piezoSample*3.3*1000; //Convierto la lectura a mV

         if(piezoSample > piezoMaxValue) //Busco el máximo valor del golpe
        {
            piezoMaxValue = piezoSample;
        }
        wait_us(SAMPLE_TIME_INTERVAL_uS); 
    }
    
    return piezoMaxValue;

}

uint8_t piezoVoltToVel (float piezoMaxValue)
{
    uint8_t vel = 0;
    float velFloat = 0.0;

    velFloat = piezoMaxValue*slope + intercept; //Calculo el parametro velocity
    
    vel = (uint8_t)roundf(velFloat);
    if (vel > MAX_VEL) vel = MAX_VEL;
    if (vel < MIN_VEL) vel = MIN_VEL;

    return vel; 
}