#include <stdint.h>
#include <stdbool.h>
#include "ws_fir.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
// hier noch Ihren Filterheader einbinden

// Praeprozessor-Makros
#define SAMPLERATE 44000
#define FILTERORDER 50
// Funktionen-Deklarationen
void adcIntHandler(void);
void setup(void);
// hier nach Bedarf noch weitere Funktionsdeklarationen einfuegen

// global variables
int32_t bufferSample[FILTERORDER];
int32_t sampleIndex = 0;
// hier nach Bedarf noch weitere globale Variablen einfuegen

void main(void) // nicht veraendern!! Bitte Code in adcIntHandler einfuegen
{
    setup();
    while(1){}
}

void setup(void){// konfiguriert den Mikrocontroller

    // konfiguriere System-Clock
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
   // Bitte Code hier einfuegen
   // ...
    uint32_t adcInputValue;
    unsigned int n = 49;     //convolution indexing
    unsigned int k = 0;     //----
      int i = 0;      //index
      int64_t quadInput = 0;   //larger variable to be able to contain squared numbers
      int64_t Ausgang=0;
      ADCSequenceDataGet(ADC0_BASE,3,&adcInputValue);
      // Bitte Code hier einfuegen
      // ...

      quadInput = adcInputValue * adcInputValue;      //quadrature of amplitude to determine signal energy

      sampleIndex++;
      for(i = FILTERORDER-2; i >= 0; i--){
            bufferSample[i+1] = bufferSample[i];
        }
        bufferSample[0] = quadInput;
       if(sampleIndex==50){
           sampleIndex= 0;

           //use filter via convolution

           for(k = 0; k < FILTERORDER; k++){
               Ausgang += bufferSample[k] * numer[n - k];
           }
           if(Ausgang > 5000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                              GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                              0x01); }
              if(Ausgang > 20000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0x03); }
              if(Ausgang > 35000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0x07); }
              if(Ausgang > 50000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0x0F); }
              if(Ausgang > 65000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0x1F); }
              if(Ausgang > 80000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0x3F); }
              if(Ausgang > 95000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0x7F); }
              if(Ausgang > 110000) {GPIOPinWrite(GPIO_PORTB_BASE,
                                                 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
                                                 0xFF); }

       }
   // am Ende von adcIntHandler, Interrupt-Flag loeschen
   ADCIntClear(ADC0_BASE,3);
}
