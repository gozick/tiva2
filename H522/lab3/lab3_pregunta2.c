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
int random(void){
	//Este valor de random esta entre 1 y 6
	//Entonces esta función genera ese valor y lo devuelve
	return ( rand() % (6+1));
}
unsigned int MCD (unsigned int contA,unsigned int contB){
	unsigned int temp;
	int bandera=1;
	if (contB==contA){
		temp=contB;//o puede ser temp=contA o puede ser temp=contA|contB
	}else{//Sino (*)
		while (bandera!=0){
			if (contA>contB){
				contA=contA-contB;//A=A-B
			}
			else{//contB<=contA
				contB=contB-contA;
			}
			if (contB==contA){//verificacion
				temp=contB;//o puede ser temp=contA o puede ser temp=contA|contB
				bandera=0;//rompemos el while tambien se puede hacer con break;
			}
		}
	}
	return temp;
}

int main(void){
	configPuertos();
	unsigned int contA=0;
	unsigned int contB=0;
	unsigned int mcd=0;
	long int i =0,j=0;
	int esAzul=0x04;//color de led puertoF

	while (1){

		//SW1
		if ((GPIO_PORTA_DATA_R&0x10)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x10)!=0);//Mientras este presionado
			if (contA<6){
				contA++;//Aumentamos el valor de contA;
			}//caso contrario no se ejecuta nada
		}

		//SW2
		if ((GPIO_PORTA_DATA_R&0x08)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x08)!=0);//Mientras este presionado
			if (contB<6){
				contB++;//Aumentamos el valor de contA;
			}//caso contrario no se ejecuta nada
		}

		//SW3
		if ((GPIO_PORTA_DATA_R&0x04)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x04)!=0);//Mientras este presionado
			contA=contA+2;//Aumentamos los valores de los contadores de A
			contB=contB+2;//Aumentamos los valores de los contadores de B
			for (i=0;i<contA;i++)//Haremos los parpadeos de los leds
			{
				//Prender
				GPIO_PORTF_DATA_R=esAzul;//LED AZUL
				//Retardo
				for (j=0;j<400000;j++);
				//Apagar
				GPIO_PORTF_DATA_R=0x00;//LED AZUL
				//Retardo
				for (j=0;j<400000;j++);
			}
			//Luego de que terminó de hacer los parpadeos de A
			//ocurre un retardo
			for (j=0;j<800000;j++);//el doble que el que demora entre parpadeo de A
			//Ahora el parpadeo de B
			for (i=0;i<contB;i++)//Haremos los parpadeos de los leds
			{
				//Prender
				GPIO_PORTF_DATA_R=esAzul;//LED AZUL
				//Retardo
				for (j=0;j<400000;j++);
				//Apagar
				GPIO_PORTF_DATA_R=0x00;//LED AZUL
				//Retardo
				for (j=0;j<400000;j++);
			}
		}

		//SW4
		if ((GPIO_PORTA_DATA_R&0x20)!=0){//si se presiona
			while ((GPIO_PORTA_DATA_R&0x20)!=0);//Mientras este presionado
			do{
			contA=random();
			contB=random();
			}while (contA<1&&contA>6&&contB<1&&contB>6);

			if (contA>contB){
				//Mostramos en forma ascendente
				//primero sería contB
				for (i=0;i<contB;i++)//Haremos los parpadeos de los leds
				{
					//Prender
					GPIO_PORTF_DATA_R=esAzul;//LED AZUL
					//Retardo
					for (j=0;j<400000;j++);
					//Apagar
					GPIO_PORTF_DATA_R=0x00;//LED AZUL
					//Retardo
					for (j=0;j<400000;j++);
				}
				for (j=0;j<800000;j++);	//Ahora el parpadeo de B
				for (i=0;i<contA;i++)//Haremos los parpadeos de los leds
				{
					//Prender
					GPIO_PORTF_DATA_R=esAzul;//LED AZUL
					//Retardo
					for (j=0;j<400000;j++);
					//Apagar
					GPIO_PORTF_DATA_R=0x00;//LED AZUL
					//Retardo
					for (j=0;j<400000;j++);
				}
			}else{//Caso contrario a contA>contB
				mcd=MCD(contA,contB);//llamamos a la función MCD
				for (i=0;i<mcd;i++)//Haremos los parpadeos de los leds
				{
					//Prender
					GPIO_PORTF_DATA_R=esAzul;//LED AZUL
					//Retardo
					for (j=0;j<400000;j++);
					//Apagar
					GPIO_PORTF_DATA_R=0x00;//LED AZUL
					//Retardo
					for (j=0;j<400000;j++);
				}
			}
		}
	}
}
