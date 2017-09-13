
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
	uint32_t i=0,j,p,estado,bandera=0;
	uint32_t estados[7]={0,2,4,8,6,10,12,14};//La secuencia de colores
	uint32_t almacenar[3]={2,0,4};//La secuencia preconfigurada
	GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds

	while(1){

		GPIO_PORTF_DATA_R=estados[i];
		//for (j=0;j<400000;j++);//retardo para led
		while (bandera!=1){//bandera 1 nos indica que se tomo alguna accion
			estado=GPIO_PORTF_DATA_R;
			switch (!(estado&0x11)){
			case 0x1:
				almacenar[p]=estados[i];
				p++;
				bandera=1;
				break;
			case 0x10:

				i++;
				bandera=1;
				break;
			case 0x11:

				for(p=0;p<3;p++){
					GPIO_PORTF_DATA_R=almacenar[p];
					//for (j=0;j<400000;j++);//retardo para led
				}//mostrar todos los colores
				p=0;//reiniciamos p
				GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds
				while (GPIO_PORTF_DATA_R&0x11!=0);//Mientras no se presione
				bandera=1;
				break;
			default:
				break;
			}
		}//sale de bandera
		bandera=0;//reiniciamos bandera para la siguiente lectura
		GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds
		if (i==7){
			i=0;
		}
		p=0;
		j=0;

	}//fin while


} // fin main
