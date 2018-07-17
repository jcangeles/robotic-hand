#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "finalProject_hand.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c123gh6pm.h"
//*****************************************************************************
/*
Finger-Mapping
ID					ADC						Flex#			V_max(straight)	V_min(bent)
1.	thumb		pd0		ain7		44				2.239					.710
2.	index		pd1		ain6		46				1.950					.600
3.	middle	pd2		ain5		32				2.150					.616		
4.	ring		pd3		ain4		19				2.269					1.166				
5.	pinky		pe1		ain2		7					2.167					.721		

UART Bluetooth Notes:
Transmitter		PD7		Blue
Receiver			PD6		Red
Notes: For computer connection use COM5 (Outgoing)
*/
uint32_t ui32ADC0Value[8];
volatile uint32_t ui32TempThumb;
volatile uint32_t ui32TempIndex;
volatile uint32_t ui32TempMiddle;
volatile uint32_t ui32TempRing;
volatile uint32_t ui32TempPinky;

uint32_t ADCMin_thumb  = 825	, ADCMax_thumb  = 2200;
uint32_t ADCMin_index  = 303	, ADCMax_index  = 1700;
uint32_t ADCMin_middle = 883	, ADCMax_middle = 2101;
uint32_t ADCMin_ring   = 850	, ADCMax_ring   = 2495;
uint32_t ADCMin_pinky  = 850	, ADCMax_pinky  = 2331;

uint8_t normalize(uint32_t ui32ADCcurr, uint32_t ADCmax, uint32_t ADCmin, uint8_t ID){
		uint8_t fingerNormalized;
		if(ui32ADCcurr < ADCmin) ui32ADCcurr = ADCmin;
		if(ui32ADCcurr > ADCmax) ui32ADCcurr = ADCmax;
		fingerNormalized = 31*(ui32ADCcurr - ADCmin)/(ADCmax - ADCmin);
		fingerNormalized |= ID;
		return fingerNormalized;
}

void UART_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);		    // Enable Peripheral Clocks 
    // First open the lock and select the bits we want to modify in the GPIO commit register.
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    // Now modify the configuration of the pins that we unlocked.
    GPIOPinConfigure(GPIO_PD7_U2TX);
	  GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	
		UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void ADC0_init(void){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);				// activate the clock of ADC0
		SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.
	
		ADCHardwareOversampleConfigure(ADC0_BASE, 64);
	
		ADCSequenceDisable(ADC0_BASE, 0); //disable ADC0 before the configuration is complete
		ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS0, processor-trigger, priority 0
		ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH7);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH6);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH5);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH4);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH2|ADC_CTL_IE|ADC_CTL_END); //completion of this step will set RIS, last sample of the sequence
	
		IntPrioritySet(INT_ADC0SS0, 0x00);  	 			// configure ADC0 SS0 interrupt priority as 0
		IntEnable(INT_ADC0SS0);    									// enable interrupt ?? in NVIC (ADC0 SS0)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS0);     // arm interrupt of ADC0 SS0
	
		ADCSequenceEnable(ADC0_BASE, 0); //enable ADC0
}

void ADC0_Handler(){
		ADCIntClear(ADC0_BASE, 0);
		ADCProcessorTrigger(ADC0_BASE, 0);
		ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);
		ui32TempThumb 	= ui32ADC0Value[0];
		ui32TempIndex 	= ui32ADC0Value[1];
		ui32TempMiddle 	= ui32ADC0Value[2];
		ui32TempRing 		= ui32ADC0Value[3];
		ui32TempPinky 	= ui32ADC0Value[4];
}

void Timer0A_Init(unsigned long period){    
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);			  	// Enable Peripheral Clocks
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 			// configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);      	// reload value
	IntPrioritySet(INT_TIMER0A, 0x00);  	 								// configure Timer0A interrupt priority as 0
  IntEnable(INT_TIMER0A);    														// enable interrupt 19 in NVIC (Timer0A)
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt
  TimerEnable(TIMER0_BASE, TIMER_A);      							// enable timer0A
}

void Timer0A_Handler(void){
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);			// acknowledge flag for Timer0A timeout
		
    UARTCharPut(UART2_BASE, normalize(ui32TempThumb,	ADCMax_thumb,	ADCMin_thumb, 0x20));
		UARTCharPut(UART2_BASE, normalize(ui32TempIndex,	ADCMax_index,	ADCMin_index, 0x40));
		UARTCharPut(UART2_BASE, normalize(ui32TempMiddle,	ADCMax_middle,ADCMin_middle,0x60));
		UARTCharPut(UART2_BASE, normalize(ui32TempRing,		ADCMax_ring,	ADCMin_ring,	0x80));
		UARTCharPut(UART2_BASE, normalize(ui32TempPinky,	ADCMax_pinky,	ADCMin_pinky, 0xA0));
}

int main(void){
		unsigned long period = 4000000;	
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); 	// configure the system clock to be 40MHz
		ADC0_init();
		UART_init();
		Timer0A_Init(period);
		IntMasterEnable();
		IntEnable(INT_UART2); //enable the UART interrupt
		UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
		ADCProcessorTrigger(ADC0_BASE, 0);

	  while(1){}
}
