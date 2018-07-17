#include <stdint.h>
#include <stdbool.h>
#include "finalProject_base.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"

#define PWM_FREQUENCY 55
#define PWM_MAX 130
#define	PWM_MIN 35
//*****************************************************************************
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32Load;	
uint8_t PWM_temp;
uint8_t PWM_thumb;
uint8_t PWM_index;
uint8_t PWM_middle;
uint8_t PWM_ring;
uint8_t PWM_pinky;

void PWM_init(void){

		SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	
    GPIOPinConfigure(GPIO_PB6_M0PWM0);		//THUMB
		GPIOPinConfigure(GPIO_PB4_M0PWM2);		//INDEX
		GPIOPinConfigure(GPIO_PE4_M0PWM4);		//MIDDLE
    GPIOPinConfigure(GPIO_PC4_M0PWM6);		//RING
		GPIOPinConfigure(GPIO_PA6_M1PWM2);		//PINKY

		GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_4); // | GPIO_PIN_4
		GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
		GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
		GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	
		ui32PWMClock = SysCtlClockGet() / 64; 
		ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	
		PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);
		PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
		
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT |PWM_OUT_2_BIT| PWM_OUT_4_BIT | PWM_OUT_6_BIT, true);
		PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
		
		PWMGenEnable(PWM0_BASE, PWM_GEN_0);
		PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
		PWMGenEnable(PWM0_BASE, PWM_GEN_3); 
		PWMGenEnable(PWM1_BASE, PWM_GEN_1); 
}

void UART_init(void){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

		GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	
		UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void UART_Handler(void){
		UARTIntClear(UART2_BASE, UART_INT_RX);
		if (UARTCharsAvail(UART2_BASE)){
			PWM_temp = UARTCharGet(UART2_BASE);
			if(PWM_temp >= 32 && PWM_temp < 64){
					PWM_thumb = PWM_temp;
					PWM_thumb &= ~0x20;
				  PWM_thumb = ((PWM_MAX-PWM_MIN)*(31-PWM_thumb)/31 + PWM_MIN);
				  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWM_thumb* ui32Load / 1000);
			} else if(PWM_temp >= 64 && PWM_temp < 96){
					PWM_index = PWM_temp;
					PWM_index &= ~0x40;
					PWM_index = ((PWM_MAX-PWM_MIN)*(31-PWM_index)/31 + PWM_MIN);
					PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWM_index* ui32Load / 1000);
			} else if(PWM_temp >= 96 && PWM_temp < 128){
					PWM_middle = PWM_temp;
					PWM_middle &= ~0x60;
					PWM_middle = ((PWM_MAX-PWM_MIN)*(31-PWM_middle)/31 + PWM_MIN);
				  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWM_middle* ui32Load / 1000);
			} else if(PWM_temp >= 128 && PWM_temp < 160){
					PWM_ring = PWM_temp;
					PWM_ring &= ~0x80;
					PWM_ring = ((PWM_MAX-PWM_MIN)*(31-PWM_ring)/31 + PWM_MIN);
				  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM_ring* ui32Load / 1000);
			} else if(PWM_temp >= 160 && PWM_temp < 192){
					PWM_pinky = PWM_temp;
					PWM_pinky &= ~0xA0;
					PWM_pinky = ((PWM_MAX-PWM_MIN)*(31-PWM_pinky)/31 + PWM_MIN);
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWM_pinky* ui32Load / 1000);
			}
		}
}


int main(void){
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
		//testing
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		UART_init();
		PWM_init();
	
		IntMasterEnable();
		IntEnable(INT_UART2);
		UARTIntEnable(UART2_BASE, UART_INT_RX);
	
		while(1){}
}
