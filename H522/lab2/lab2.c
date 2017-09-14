/* **************************************
 * TÍTULO: Tarea de LAB2 - H522
 * AUTOR: PABLO DÍAZ
 * CURSO: SISTEMAS DIGITALES
 * FECHA: 03/09/2017
 * DESCRIPCIÓN: Adjunta con tarea
 * Módulo: Tiva Launchpad EK-TM4C123GXL
 * Fecha: 14/09/17
 * Semestre: 2017-2
 * ENLACE DE PUERTOS:
 * D1 esta en el puerto PB4
 * D2 esta en el puerto PB5
 * D3 esta en el puerto PB6
 * D4 esta en el puerto PB7
 * D5 esta en el puerto PB0
 * D6 esta en el puerto PB1
 * D7 esta en el puerto PB2
 * D8 esta en el puerto PB3
 *SW1 esta en el puerto PA4
 *SW2 esta en el puerto PA3
 *SW3 esta en el puerto PA2
 *SW4 esta en el puerto PA5
 *
 * LEDS ORDENADOS DE LA SIGUIENTE MANERA
 * D1 D2 D3   	 || PB4  PB5  PB6
 * D4 D5         || PB7  PC4
 * D6 D7 D8	     || PC5  PC6  PC7
 *
 *********************************************/
#include <stdint.h>
#include "tm4c123gh6pm.h"
void config_pulsador_led(void){ //Configuramos puerto F sw1 sw2 LEDS
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
}
void config_switches(void){
	//SYSCTL_RCGCGPIO_R |= 0xFF;//El reloj debe prenderse para los puertos A y B
	while((SYSCTL_PRGPIO_R &  0x0F )==0) { } // Realmente activado
	GPIO_PORTA_DIR_R &= ~(0x3C); // Todos los switches (1,2,3,4) son de entrada en el puerto A
	GPIO_PORTA_AFSEL_R &= ~(0x3C); // No usamos función alternativa
	GPIO_PORTA_PUR_R |= 0x3C; // Activamos resistencia de pull-up en pines del puerto A
	GPIO_PORTA_DEN_R |= 0x3C; // Puertos A en digital
}
void config_portB(void){
	//SYSCTL_RCGCGPIO_R |= 0xFF;// Esperamos la real activación
	while( (SYSCTL_PRGPIO_R & 0x0F)==0);	//Configuramos puertos de E/S para PB
	GPIO_PORTB_DIR_R |= 0xF0; // Configura el bit del puerto B como salida
	GPIO_PORTB_DR8R_R |=0xF0; // se activa el driver de 8 mA .
	GPIO_PORTB_DEN_R |=0xF0; // se activa el pin como salida digital.
}
void config_portE(void){
	//SYSCTL_RCGCGPIO_R |= 0xFF;
	while(!(SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5));
	GPIO_PORTE_DIR_R |= 0xFF; // Configura el bit del puerto B como salida
	GPIO_PORTE_DR8R_R |=0xFF; // se activa el driver de 8 mA .
	GPIO_PORTE_DEN_R |=0xFF; // se activa el pin como salida digital.
}

void prenderLeds(int numero_led){//para poder prender leds necesitamos solo un numero
	switch (numero_led){
	case 1://LED 1
		GPIO_PORTB_DATA_R=(1<<4);
		break;
	case 2:// LED 1 Y 2
		GPIO_PORTB_DATA_R=(1<<4)|(1<<5);
		break;
	case 3: //LED 1, 2 Y 3
		GPIO_PORTB_DATA_R=(1<<4)|(1<<5);
		GPIO_PORTE_DATA_R=(1<<6);
		break;
	case 4://LOS 4 LEDS
		GPIO_PORTB_DATA_R=(1<<4)|(1<<5);
		GPIO_PORTE_DATA_R=(1<<6)|(1<<7);
	}
}


void main(void) {
	config_pulsador_led();//Configurar puerto F
	config_switches();//puerto A
	config_portB();//puerto B
	config_portE();//puerto E
	uint32_t i=0,j,p,estado,bandera=0,bandera1=0;

	int numero_led=2;//Va a seguir el numero de led que prendio o apago

	uint32_t estados[4]={0,0,0,0};//LD1/LD2/LD3/LD4

	uint32_t almacenar[3]={1,0,1};//La secuencia preconfigurada

	GPIO_PORTB_DATA_R&=~0xFF;//apagamos todos los leds
	GPIO_PORTE_DATA_R&=~0xFF;//apagamos todos los leds

	while(1){

		if (numero_led>4) numero_led=4;//Mantenemos en 4 prendidos si se pasa de 4

		while (bandera!=1){
			estado=GPIO_PORTA_DATA_R;//estado de los puertos A
			switch (estado){

			case 0x10://SW1 PA4 prendido
				while (GPIO_PORTA_DATA_R!=0);//Mientras este presionado algún switch
				if (p<3) break;
				else{
					while(1){//Mientras que sw1 no se aplaste denuevo
						for (j=0;j<400000;j++);//retardo
						GPIO_PORTA_DATA_R=almacenar[p];//Prendemos los leds
						GPIO_PORTE_DATA_R=almacenar[p];//necesarios
						for (j=0;j<400000;j++);//retardo
						GPIO_PORTB_DATA_R&=~0xFF;//apagamos todos los leds
						GPIO_PORTE_DATA_R&=~0xFF;//apagamos todos los leds
						if (GPIO_PORTA_DATA_R&0x10==1){//Si se aplasta sw1
							while (GPIO_PORTA_DATA_R!=0);//Mientras este presionado algún switch
							break;
						}
					}
				}
				bandera=1;// se ejecutó un caso
				break;
			case 0x08://SW2 PA3 presionado
				while (GPIO_PORTA_DATA_R!=0);//Mientras este presionado algún switch

				if (numero_led!=0){//Mientras este algun led prendido norma
					numero_led=numero_led-1;
				}//si no hay prendidos no se hace nada
				prenderLeds(numero_led);//prendemos en el que se encuentra
				bandera=1;// se ejecutó un caso
				break;
			case 0x04://Sw3 PA2presionado
				while (GPIO_PORTA_DATA_R!=0);//Mientras este presionado algún switch
				if (p==3) p=0;//Si se pasó del arreglo que reemplace el primero
				almacenar[p]=estados[numero_led-1];//Almacenar el estado actual
				p++;
				bandera=1;// se ejecutó un caso
				break;
			case 0x20://Sw4 presionado
				while (GPIO_PORTA_DATA_R!=0);//Mientras este presionado algún switch
				if (numero_led<=4){
					GPIO_PORTA_DATA_R=numero_led+1;
				}
				bandera=1;// se ejecutó un caso
				break;
			default: //Otra opcion no hace nada
				break;
			}

			bandera=0;//Reiniciamos bandera para siguiente decision

			//		GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds


		}


		//		if (i==8){//Si se pasa que se reinicie
		//			i=0;
		//		}
		//		GPIO_PORTF_DATA_R=estados[i];
		//		for (j=0;j<400000;j++);//retardo para led
		//		while (bandera!=1){//bandera 1 nos indica que se tomo alguna accion
		//			estado=GPIO_PORTF_DATA_R;
		//			switch ((estado&0x11)){
		//			case 0x00://Switch sw1y sw2 presionados
		//				while ((GPIO_PORTF_DATA_R&0x11)==0){
		//					if ((GPIO_PORTF_DATA_R&0x01)==0) break;//Para que pueda detectar que otra tecla se aplastó
		//					if ((GPIO_PORTF_DATA_R&0x10)==0) break;
		//				}//Mientras este presionado
		//				while ((GPIO_PORTF_DATA_R&0x11)==0);
		//				for(p=0;p<3;p++){
		//					GPIO_PORTF_DATA_R=almacenar[p];
		//					for (j=0;j<800000;j++);//retardo para led
		//					GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds
		//					for (j=0;j<800000;j++);//retardo para led
		//				}//mostrar todos los colores
		//				p=0;//reiniciamos p
		//				while (GPIO_PORTF_DATA_R&0x11!=0);//Mientras no se presione
		//				while ((GPIO_PORTF_DATA_R&0x11)==0);//Mientras este presionado
		//				bandera=1;
		//				break;
		//			case 0x01://Sw1 presionado
		//				while ((GPIO_PORTF_DATA_R&0x10)==0){
		//					if ((GPIO_PORTF_DATA_R&0x11)==0) break;//Para romper el
		//					if ((GPIO_PORTF_DATA_R&0x01)==0) break;//case
		//				}
		//				i++;
		//				bandera=1;
		//				break;
		//			case 0x10://sw2 presionado
		//				while ((GPIO_PORTF_DATA_R&0x01)==0){
		//					if ((GPIO_PORTF_DATA_R&0x11)==0) break;//Para romper el
		//					if ((GPIO_PORTF_DATA_R&0x10)==0) break;//case
		//				}
		//				if (p==3){
		//					p=0;
		//				}
		//				almacenar[p]=estados[i];
		//
		//				p++;
		//				bandera=1;
		//				break;
		//			default:
		//				break;
		//			}
		//		}//sale de bandera
		//		bandera=0;//reiniciamos bandera para la siguiente lectura
		//		GPIO_PORTF_DATA_R&=~0x0E;//apagamos todos los leds
	}//fin while
} // fin main
