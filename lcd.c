/***************************************************************************/
/*************** Proyecto: CONTADOR DE PECES CON LASER   *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz        **********************************/
/*************** Fecha: Diciembre 2017  ************************************/
/*************** Descripción del proyecto adjunto con el	****************/
/*************** informe final                          	****************/
/***************************************************************************/
/*
CONEXIONES LCD:
D4 PB4
D5 PB5
D6 PB6
D7 PB7

VSS VO GROUND
VDD +5.0 V

 */
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <math.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

void ConfiguracionPuertos(void )
{
	SYSCTL_RCGC2_R|=0x1F;					// Prendemos Puertos A-B-C-D-E-F
	while((SYSCTL_PRGPIO_R&0x1F)!=0x1F);	// Esperamos a la activacion del clock

	GPIO_PORTB_AMSEL_R &= ~0xFF;			// Desactivamos función analógica
	GPIO_PORTB_PCTL_R = 0x00000000; 		// Configuramos como GPIO;
	GPIO_PORTB_DIR_R |= 0xFF; 				// Configuramos como Salida
	GPIO_PORTB_AFSEL_R &= ~0xFF;			// Desacivamos funciones alternativas
	GPIO_PORTB_DR8R_R |= 0xFF;				// Prendemos 8mA Driver
	GPIO_PORTB_DEN_R |= 0xFF; 				// Habilitamos función digital

	/*
	GPIO_PORTE_AMSEL_R &= ~0x07;			// Desactivar función analógica
	GPIO_PORTE_PCTL_R &= ~0x00000FFF;		// Configuramos como GPIO
	GPIO_PORTE_DIR_R &=~0x07;				// Declaramos como entrada
	GPIO_PORTE_AFSEL_R &= ~0x07;			// Desactivar funciones alternativas
	GPIO_PORTE_DEN_R |= 0x07;				// Activar funciónes digitales
	 */

}

void retardo_ms (uint32_t milisegundos){
	/* Esta función genera un retardo exacto contado en milisegundos
	 * solo basta ingresar y generará un retardo con error de +/- 1/16millones
	 */
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;           //Deshabilitamos Systick
	NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R & ~0xFFFFFF) + (16000*milisegundos - 1);  //Programamos valor de recarga
	NVIC_ST_CURRENT_R = (NVIC_ST_CURRENT_R & ~0xFFFFFF) + 1;       //Borramos la cuenta
	NVIC_ST_CTRL_R |= (NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC);     //Habilitamos Systick
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT)==0);
}

void enviarXcodigo(int code)
{
	GPIO_PORTB_DATA_R&=~0xFF;
	GPIO_PORTB_DATA_R|=(code&0xF0)+(1<<2);	//higher 4bit
	retardo_ms(10);
	GPIO_PORTB_DATA_R&=~0xFF;
	GPIO_PORTB_DATA_R|=((code&0x0F)<<4)+(1<<2);		//	lower 4bit
	retardo_ms(10);
}

void enviarxComando(int command)
{
	GPIO_PORTB_DATA_R&=~0xFF;
	GPIO_PORTB_DATA_R|=((command&0x0F)<<4)+(1<<2);		//	lower 4bit
	retardo_ms(10);
}
void InicializacionLCD(void)
{
	enviarxComando(LCD_CLEARDISPLAY);
	GPIO_PORTB_DATA_R=1+4;
	retardo_ms(10);
	enviarxComando(0x30);
	enviarxComando(0x30);
	enviarxComando(0x30);
	enviarxComando(0x20);
	enviarxComando(LCD_FUNCTIONSET);
	
}

int main()
{
	ConfiguracionPuertos();
	retardo_ms(1000);
	int j=4;
	int i;
	InicializacionLCD();

	while (2){

		/*for (i=0;i<16;i++)
		{
			enviarXcodigo(j);
		}
		 */
		//retardo_ms(10);
		enviarXcodigo(j);
		//retardo_ms(10);
		//enviarXcodigo(3);
		if (j<0xFFFF) j++;
		else j=0;
	}


}
