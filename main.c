#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"

// Praepozessor-Makros
#define SAMPLERATE 44000
#define WINDOW_SIZE 440

//Funktionen-Deklarationen
void adcIntHandler(void);
void setup(void);
// hier nach Bedarf noch weitere Funktionsdeklarationen einfuegen
void fourierTransform(/*volatile long double** pSpectrum*/);     // Funktion der Fourier-Transform, der auch die Power-Spektrum bestimmt
unsigned int maxFreq(/*volatile long double* pSpectrum*/);       // Funktion fuer die Bestimmung der maximalen Frequenz
void printFreq(unsigned int maxIndex);              // Funktion, die die LEDs steuert mit Rücksicht auf dem maxIndex
volatile float pSpectrum[WINDOW_SIZE];

// globale Variablen, alle volatile, da ein Interrupt Routine diese bearbeiten koennen
volatile const float DoublePi = 6.283185308;
volatile int32_t bufferSample[WINDOW_SIZE];              // zirkularer Buffer, enthaelt die letzte 440 Messwerten
volatile unsigned int interruptIndex = 0;        // zaehlt wie viel das Interruptroutine abgelaufen ist

// hier nach Bedarf noch weitere globale Variablen einfuegen

void main(void){ // nicht veraendern!! Bitte Code in adcIntHandler einfuegen
    setup();
    while(1){}
}

void setup(void){//konfiguriert den Mikrocontroller

    // konfiguriere SystemClock
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    uint32_t period = SysCtlClockGet()/SAMPLERATE;

    // aktiviere Peripherie
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // aktiviere Gleitkommazahlen-Modul
    FPUEnable();
    FPUStackingEnable();
    FPULazyStackingEnable();
    FPUFlushToZeroModeSet(FPU_FLUSH_TO_ZERO_EN);

    // konfiguriere GPIO
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    // konfiguriere Timer
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);
    TimerControlTrigger(TIMER0_BASE,TIMER_A,true);
    TimerEnable(TIMER0_BASE,TIMER_A);

    // konfiguriere ADC
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_RATE_FULL,1);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE,3);
    ADCIntRegister(ADC0_BASE,3,adcIntHandler);
    ADCIntEnable(ADC0_BASE,3);

}

void adcIntHandler(void){
    // lokale Variablen deklarieren
    volatile bool isTrafoCycle = false;             // bool Variable, der uns sagt ob es ein Trafo durchgefuert werden soll/wurde oder nicht
    int i = 0;                                      // Laufvariable(n) deklarieren und auf Null setzen
    uint32_t adcInputValue;                         // Variable der die inputValue speichert
    uint64_t quadInputValue;                        // Varaible der die Quadratur der inputValue speichert
    unsigned int maxIndex = 0;                      // Index der maximalen Amplitude

    // zaehlen wie viel mal diese Routine abgelaufen ist, seit der letzten Transformation und isTrafoCycle entsprechend einstellen
    interruptIndex++;
    isTrafoCycle = (interruptIndex == WINDOW_SIZE);

    // Eingangssignal lesen, dann als Quadratur speichern
    ADCSequenceDataGet(ADC0_BASE,3,&adcInputValue);
    quadInputValue = adcInputValue * adcInputValue;

    // neuen Eingangswert in ringBuffer speichern (vom Ende nach dem Anfang, damit es keine Daten verloren werden)
    for(i = WINDOW_SIZE-2; i >= 0; i--){
                bufferSample[i+1] = bufferSample[i];
    }
    bufferSample[0] = quadInputValue;

    // wenn das Programm sich in TrafoCycle befindet, die Werte der bufferSample Fourier-Transformieren
    if(isTrafoCycle == true){

        // call fourier transform
        fourierTransform(/*&pSpectrum*/);

        // Maximum des pSpectrums und sein Index bestimmen
        maxIndex = maxFreq(/*pSpectrum*/);
        // LEDs entsprechend maxIndex steuern
        printFreq(maxIndex);

        // interrupt Zaehler loeschen
        interruptIndex = 0;
    }

    // am Ende von adcIntHandler, Interrupt-Flag loeschen
    ADCIntClear(ADC0_BASE,3);
}

// fourierTransform Funktion; es nimmt einen Zeiger zum pSpectrum über. Alle andere benötigte Variablen sind entweder global oder lokal definiert.
void fourierTransform(/*volatile long double **pSpectrum*/){

    unsigned int i = 0, k = 0, n = 0;
    float Xre[WINDOW_SIZE];                                        // Realteil des Spektrums deklarieren
    float Xim[WINDOW_SIZE];                                        // Imaginaerteil des Spektrums deklarieren

    // Das Real- bzw Imaginaerteil des Spektrums separat berechnen, damit die Nutzung von externe complex Bibliotheken komplett vermeidet wird
    // Implementation ist hier brute-force
    for(i=0;i<WINDOW_SIZE;i++){
        Xre[i]=0;
        Xim[i]=0;

    }
    for(; k < WINDOW_SIZE; k++){

        for(n = 0; n < WINDOW_SIZE; n++){

            Xre[k] += (float) bufferSample[n] * (float) cosf(DoublePi * k * n / WINDOW_SIZE);   // k-te Realteil berechnen
            Xim[k] -= (float) bufferSample[n] * (float) sinf(DoublePi * k * n / WINDOW_SIZE);   // k-te Imaginaerteil berechnen

        }
    }
    // powerSpectrum nach Pythagoras berechnen, sqrt wird nicht gebraucht, da die Frequenz untersucht wird, nicht die Amplitude
    for(i=0; i < WINDOW_SIZE; i++){
        pSpectrum[i] = Xre[i] * Xre[i] + Xim[i] * Xim[i];
    }
}

// Frequenz (Index) der maximalen Amplitude bestimmen
unsigned int maxFreq(/*volatile long double* pSpectrum*/){
    unsigned int i = 1;             // lokale Laufvariable deklarieren und zu 1 stellen, wegen cucc
    float currentMax = 0;           // temporaere Maximum, kann von Null starten, da die Werte von pSpectrum immer groesser als Null sind
    unsigned int maxIndex = 0;      // speichert den maximalen Index

    // nach Maximalwert suchen (ganzes Array wird durchgesucht)
    for(; i < WINDOW_SIZE; i++){
        if(pSpectrum[i] > currentMax){ maxIndex = i; currentMax = pSpectrum[i]; }
    }
    //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, maxIndex);
    // Index (Frequenz) der maximalen Index zurueckgeben
    return maxIndex;
}

void printFreq(unsigned int maxIndex){
    // LEDs nach maxIndex steuern
    if(maxIndex > 412) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x80); }
    else if(maxIndex > 357) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x40); }
    else if(maxIndex > 302) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x20); }
    else if(maxIndex > 247) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x10); }
    else if(maxIndex > 192) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x08); }
    else if(maxIndex > 137) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x04); }
    else if(maxIndex > 82) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x02); }
    else if(maxIndex >= 0) { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x01); }
    else { GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x00); }
}
