/* **************************************
 * TÍTULO: Tarea de LAB1 - H522
 * AUTOR: PABLO DÍAZ
 * CURSO: SISTEMAS DIGITALES
 * FECHA: 03/09/2017
 * -DESCRIPCIÓN: Adjunta con tarea
 * ***************************************/

#include <stdint.h>
#include "tm4c123gh6pm.h"

void config_pulsador_led(void){ //Configuramos puerto F sw1 sw2 LEDS
	unsigned long temp;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // Se habilita la señal de reloj del Puerto F
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
}

void main(void) {
	config_pulsador_led();//Configuramos SW1 SW2 leds
	uint32_t j;
	uint32_t bandera=0;
	GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds
	while(1){//Ejecutar siempre
		if ((GPIO_PORTF_DATA_R & 0x10)==0){//sw1 presionado
			while ((GPIO_PORTF_DATA_R & 0x10)==0);//Mientras este presionado	
			while(bandera==0){
				GPIO_PORTF_DATA_R=0x02;//LED ROJO
				for (j=0; j<100000;j++);//Retardo
				GPIO_PORTF_DATA_R=0x04;//LED AZUL
				for (j=0; j<100000;j++);//Retardo
				GPIO_PORTF_DATA_R=0x08;//LED VERDE
				for (j=0; j<100000;j++);//Retardo
				if ((GPIO_PORTF_DATA_R&0x01)==0){//Si sw2 espresionado
					bandera=1;
				}
			}
		}
		if ((GPIO_PORTF_DATA_R & 0x01)==0){//sw1 presionado
			while ((GPIO_PORTF_DATA_R & 0x01)==0);//Mientras este presionado
			while(bandera==0){
				GPIO_PORTF_DATA_R=0x08;//LED VERDE
				for (j=0; j<100000;j++);//Retardo
				GPIO_PORTF_DATA_R=0x02;//LED ROJO
				for (j=0; j<100000;j++);//Retardo
				GPIO_PORTF_DATA_R=0x04;//LED AZUL
				for (j=0; j<100000;j++);//Retardo
				if ((GPIO_PORTF_DATA_R&0x01)==0){//Si sw2 espresionado
					bandera=1;
				}
			}
		}
		bandera=0;//inicializamos denuevo
		GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds
		for (j=0; j<300000;j++);//Retardo para que revise el estado despues
	} //fin de while
} // fin main
