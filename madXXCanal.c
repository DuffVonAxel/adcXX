/*
 * File:   madXXCanal.c
 * Author: Fred_Dell
 *
 * Created on 30 de Abril de 2021, 13:12
 */
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _ADC_

#include <xc.h>
#include "biblioteca_v13.h"
//#include "biblioteca_v11b.h"

void main(void) 
{
	unsigned int a,b,c,d;
	unsigned char teste[5];
	adcIniciarCanal(4);
	lcdIniciar(&PORTD,lcd20x4);
	
	lcdTexto("AN0:          mV",1,1);
	lcdTexto("AN1:          mV",2,1);
	lcdTexto("AN2:          mV",3,1);
	lcdTexto("AN3:          mV",4,1);
	
	while(1)
	{
		a=adcCanal(0);
		b=adcCanal(1);
		c=adcCanal(2);
		d=adcCanal(3);
		
		valorASCII(a,teste,4);
		lcdTexto(teste,1,5);
		valorASCII(adcVolts(0),teste,4);
		lcdTexto(teste,1,11);
		
		valorASCII(b,teste,4);
		lcdTexto(teste,2,5);
		valorASCII(adcVolts(1),teste,4);
		lcdTexto(teste,2,11);
		
		valorASCII(c,teste,4);
		lcdTexto(teste,3,5);
		valorASCII(adcVolts(2),teste,4);
		lcdTexto(teste,3,11);
		
		valorASCII(d,teste,4);
		lcdTexto(teste,4,5);
		valorASCII(adcVolts(3),teste,4);
		lcdTexto(teste,4,11);
		
		__delay_ms(50);
	}
}
