/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
//=====[Libraries]=============================================================
#include "mbed.h"
#include <cstdint>

//=====[Defines]===============================================================
#define DEBOUNCE_DELAY_MS 30 //Tiempo tipico de debounce

#define NUMBER_OF_PIEZO_SAMPLES 400 
#define SAMPLE_FREQ_Hz 40000
#define SAMPLE_TIME_INTERVAL_uS 25 

#define MAX_VEL 127
#define MIN_VEL 25//Con velocitys más bajas apenas se escucha
#define DELTA_VEL (MAX_VEL - MIN_VEL)
#define PIEZO_MAX_PEAK_VOLT_mV 2000 //Máximo valor registrado( golpe muy fuerte) para este piezo
#define PIEZO_THRESHOLD_mV 90 //Nivel por encima del piso de ruido
#define DELTA_VOLT (PIEZO_MAX_PEAK_VOLT_mV - PIEZO_THRESHOLD_mV)

//=====[Declaration and initialization of public global objects]===============
AnalogIn piezo(A0);

static DigitalOut ledPad(LED1);// Create a DigitalOutput object to toggle an LED whenever data is received.
  
static UnbufferedSerial serialPort(USBTX, USBRX);// Create a UnbufferedSerial object with a default baud rate.

//=====[Declaration and initialization of public global variables]=============
uint8_t piezoMaxVelocity = 0x64;

typedef enum{ NOTE_ON = 0x90,NOTE_OFF = 0x80}MIDI_MSGS;

float piezoMax = 0.0; //Piezo max value
float piezoRead = 0.0;
uint8_t piezoTestInt = 0;

float slope = 0.0;
float intercept = 0.0;

typedef enum{
    KICK = 36,
    SNARE = 38,
    SIDE_STICK = 37,
    HI_HAT_CLOSED = 42,
    HI_HAT_HALF_OPEN = 44,
    HI_HAT_OPEN = 46,
    HH_Pedal_CHICK = 65,
    TOM_HI = 48,
    TOM_MID = 45,
    TOM_LOW = 41,
    RIDE = 51,
    BELL = 53,
    CRASH_L = 49,
    CRASH_R = 57,
    CRASH_R_CHOKED = 58,
    CHINA = 52,
    SPLASH = 55
}INSTRUMENT_NOTES;
uint8_t noteIndex = 0;
uint8_t instrumentNote[] = {KICK,SNARE,SIDE_STICK,HI_HAT_CLOSED,HI_HAT_HALF_OPEN,
                            HI_HAT_OPEN,HH_Pedal_CHICK,TOM_HI,TOM_MID,TOM_LOW,RIDE,
                            BELL,CRASH_L,CRASH_R,CRASH_R_CHOKED,CHINA,SPLASH};

typedef enum{LED_ON = 1, LED_OFF = 0}LED_STATE;
typedef enum{BUTTON_PRESSED = 1, BUTTON_RELEASED = 0}BUTTON_STATE;

typedef struct{
    DigitalIn* alias;
    uint8_t currentState;
    uint8_t lastState;
}button_t;

uint8_t upButtonCurrentState = 0;
uint8_t upButtonLastState = 0;
uint8_t downButtonCurrentState = 0;
uint8_t downButtonLastState = 0;

//=====[Declarations (prototypes) of public functions]=========================
void outputsInit(void);
void calculateSlopeIntercept(void);
float piezoSearchMax(void);
uint8_t piezoConvertVoltToVel (float piezoMaxValue);
void MIDISendNoteOn(uint8_t note,uint8_t velocity);
void MIDISendNoteOff(uint8_t note);
void piezoUpdate(void);
uint8_t buttonUpdate(button_t* button);

//=====[Main function, the program entry point after power on or reset]========
int main(void)
{
    DigitalIn upButton(BUTTON1);
    DigitalIn downButton(D1);

    button_t upButtonStruct {&upButton,BUTTON_RELEASED,BUTTON_RELEASED};
    button_t downButtonStruct {&downButton,BUTTON_RELEASED,BUTTON_RELEASED};
    // Set desired properties (9600-8-N-1).
    serialPort.baud(9600);
    serialPort.format(8,SerialBase::None,1);

    outputsInit();
    calculateSlopeIntercept();
    uint8_t numOfInstrumentNotes = sizeof(instrumentNote) / sizeof(instrumentNote[0]);

    while (true)
    {

        piezoUpdate();
        
        if(buttonUpdate(&upButtonStruct) == BUTTON_PRESSED)
        {
            noteIndex++;
            if (noteIndex >= numOfInstrumentNotes) noteIndex = 0; 
        }

        if(buttonUpdate(&downButtonStruct) == BUTTON_PRESSED)
        {
            noteIndex--;
            if (noteIndex < 0) noteIndex = numOfInstrumentNotes - 1;  
        }

    }

}

//=====[Implementations of public functions]===================================
uint8_t buttonUpdate(button_t* button)
{
    button->currentState = button->alias->read();
    if (button->currentState != button->lastState)
    {
        wait_us(DEBOUNCE_DELAY_MS * 1000); 
        if (button->currentState == button->alias->read()) //Si efectivamente se presionó
        {
                button->lastState = button->currentState;
                return  button->currentState;//El boton efectivamente cambió de estado y devuelve el estado          
        }      
    }
    return 3;//Hubo un bounce
}

void outputsInit()
{
    ledPad = 0;
}
void calculateSlopeIntercept()
{
    slope = (float)DELTA_VEL / DELTA_VOLT; //Calculo de pendiente
    intercept = MIN_VEL - PIEZO_THRESHOLD_mV * slope; //Calculo ordenada al origen
}
void piezoUpdate()
{
    piezoRead = piezo.read();
    piezoRead = piezoRead*3.3*1000; //Convierto la lectura en mV
    if(piezoRead  > PIEZO_THRESHOLD_mV) //Compara contra el threshold 
    {
        ledPad = 1;
        piezoMax = piezoSearchMax(); //Busco el valor máximo de tensión del golpe
        piezoMaxVelocity = piezoConvertVoltToVel(piezoMax);  //Transformo a velocidad el valor máximo
        ledPad = 0;            
        MIDISendNoteOff(instrumentNote[noteIndex]);//Apago el golpe anterior
        MIDISendNoteOn(instrumentNote[noteIndex],piezoMaxVelocity);//Mando el golpe actual
    }
           
}
float piezoSearchMax()
{
    float piezoMaxValue = 0.0;
    float piezoSample = 0.0;

    for(int i = 0; i < NUMBER_OF_PIEZO_SAMPLES; i++) //Muestreo el golpe detectado
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

uint8_t piezoConvertVoltToVel (float piezoMaxValue)
{
    uint8_t vel = 0;
    float velFloat = 0.0;

    velFloat = piezoMaxValue* slope + intercept; //Calculo el parametro velocity
    
    vel = (uint8_t)roundf(velFloat);
    if (vel > MAX_VEL) vel = MAX_VEL;
    if (vel < MIN_VEL) vel = MIN_VEL;

    return vel; 
}
void MIDISendNoteOn(uint8_t note,uint8_t velocity)
{
    uint8_t command = NOTE_ON;
    serialPort.write(&command, 1);
    serialPort.write(&note, 1);
    serialPort.write(&velocity, 1); 
}

void MIDISendNoteOff(uint8_t note)
{
    uint8_t command = NOTE_ON;
    uint8_t velocityOff = 0x00;
    serialPort.write(&command, 1);
    serialPort.write(&note, 1);
    serialPort.write(&velocityOff, 1);
}