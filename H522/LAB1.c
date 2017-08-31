/* **************************************
 * TÍTULO: Tarea de LAB1- Diferentes Horarios
 * AUTOR: PABLO DÍAZ
 * CURSO: SISTEMAS DIGITALES
 * FECHA: 31/08/2017
 * -DESCRIPCIÓN: El led RGB debe mostrar los
 * siguientes colores de manera cíclica: rojo y cian
 * Para que el led cambie de color debe
 * presionarse (y soltarse) dos veces el pulsador SW1
 * ***************************************/

#include <stdint.h>
#include "tm4c123gh6pm.h"
void config_pulsadores_led(void){ //Configuramos puerto F
	unsigned long temp;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // Se habilita la señal de reloj del Puerto F
	temp = SYSCTL_RCGC2_R; // Espera a que se estabilice el reloj
	GPIO_PORTF_LOCK_R = 0x4C4F434B; //Quitamos proteccion
	// Habilitar el bit0 (PF0 - pin protegido)
	GPIO_PORTF_CR_R |= 0X1; // Se habilita la modificación de varios registros del PF0
	GPIO_PORTF_LOCK_R = 0; // Se activa la proteccion
	GPIO_PORTF_AMSEL_R &= ~(0X1F); // Se deshabilitan las funciones analóg. de los bits 0 al 4
	GPIO_PORTF_PCTL_R &= ~(0X000FFFFF); // Se asegura el uso GPIO
	GPIO_PORTF_DIR_R &= ~(0X11); //Se configura las entradas
	GPIO_PORTF_DIR_R |= 0X0E;//Se configura las salidas
	GPIO_PORTF_AFSEL_R &= ~(0X1F); // Se configuran los bits 0 al 4 como GPIO
	GPIO_PORTF_PUR_R |= 0X11; // Se activan la resist. pull-up de las entradas bits 0 y 4
	GPIO_PORTF_DR8R_R |= 0X0E; //Se activa salida de 8mA
	GPIO_PORTF_DEN_R |= 0X1F; // Se habilitan las salidas digitales de los bits 0 al 4
}


void main(void) {
	config_pulsadores_led();
	uint32_t n;
	uint32_t PresionadoYSoltado=0;//Usaremos una variable como bandera
	while(1){//Ejecutar siempre
		if (GPIO_PORTF_DATA_R==0x01){
			for(n=0;n<100000;n++);//Retardo para efecto rebote
			while (GPIO_PORTF_DATA_R!=0x11);//Mientras no este soltado
			//Esperar a ser soltado
			while(PresionadoYSoltado==0){
				GPIO_PORTF_DATA_R=0x08;//Prender led verde
				for(n=0;n<1000000;n++);//Retardo
				GPIO_PORTF_DATA_R=0x00;//Apago led verde //USAMOS ESTE PUNTO PARA PONER BREAK
				GPIO_PORTF_DATA_R=0x06;//Prender led rojo
				for(n=0;n<2000000;n++);//Retardo
				GPIO_PORTF_DATA_R=0x04;//Prender led azul
				for(n=0;n<3000000;n++);//Retardo
				GPIO_PORTF_DATA_R&=~0xE;//Apagamos todos los leds
				if (GPIO_PORTF_DATA_R==0x01){
					while (GPIO_PORTF_DATA_R!=0x11);//Mientras no este soltado
					PresionadoYSoltado=1;//Terminamos el while
				}
			}
		}
		for(n=0;n<100000;n++);//Un retardo
		PresionadoYSoltado=0;//Bandera activada denuevo
	} //fin de while
} // fin main ()
