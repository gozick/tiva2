/* PROBLEMA 2
 * Titulo: Lab3
 * Autor: Pablo Diaz
 * Ciclo: 2017-2
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"


void configPuertos(void){ //Configuramos puerto F sw1 sw2 LEDS
	unsigned long temp;
	SYSCTL_RCGCGPIO_R |= 0xFF; // Se habilita la señal de reloj del Puerto F
	temp = SYSCTL_RCGC2_R; // Espera a que se estabilice el reloj
	GPIO_PORTF_LOCK_R = 0x4C4F434B; //Quitamos proteccion
	// Habilitar el bit0 (PF0 - pin protegido)
	GPIO_PORTF_CR_R |= 0X1; // Se habilita la modificación de varios registros del PF0
	GPIO_PORTF_LOCK_R = 0; // Se activa la proteccion
	GPIO_PORTF_AMSEL_R &= ~(0X1F); // Se deshabilitan las fun. Analog. de los bits 0 al 4
	GPIO_PORTF_PCTL_R &= ~(0X000FFFFF); // Se asegura el uso GPIO
	GPIO_PORTF_DIR_R &= ~(0X11); //Se configura las entradas
	GPIO_PORTF_DIR_R |= 0X0E;//Se configura las salidas
	GPIO_PORTF_AFSEL_R &= ~(0X1F); // Se configuran los bits 0 al 4 como GPIO
	GPIO_PORTF_PUR_R |= 0X11; // Se activan la resist. pull-up de las entradas bits 0 y 4
	GPIO_PORTF_DR8R_R |= 0X0E; //Se activa salida de 8mA
	GPIO_PORTF_DEN_R |= 0X1F; // Se habilitan las salidas digitales de los bits 0 al 4

	GPIO_PORTB_DIR_R |= 0xF0; // Configura el bit del puerto B como salida
	GPIO_PORTB_DR8R_R |=0xF0; // se activa el driver de 8 mA .
	GPIO_PORTB_DEN_R |=0xF0; // se activa el pin como salida digital.

	GPIO_PORTA_DIR_R &= ~(0x3C); // Todos los switches (1,2,3,4) son de entrada en el puerto A
	GPIO_PORTA_AFSEL_R &= ~(0x3C); // No usamos función alternativa
	GPIO_PORTA_PUR_R |= 0x3C; // Activamos resistencia de pull-up en pines del puerto A
	GPIO_PORTA_DEN_R |= 0x3C; // Puertos A en digital
}

int main(void){
	configPuertos();
	unsigned int contA=0;
	unsigned int contB=0;

	while (1){

		//SW1
		if ((GPIO_PORTA_DATA_R&0x10)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x10)!=0);//Mientras este presionado

		}

		//SW2
		if ((GPIO_PORTA_DATA_R&0x08)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x08)!=0);//Mientras este presionado

		}

		//SW3
		if ((GPIO_PORTA_DATA_R&0x04)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x04)!=0);//Mientras este presionado

		}

		//SW4
		if ((GPIO_PORTA_DATA_R&0x20)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x20)!=0);//Mientras este presionado

		}




	}
}
