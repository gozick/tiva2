/***************************************************************************/
/*********************************ESCLAVO***********************************/
/***************************************************************************/
/*************** Proyecto: Comunicación Inalambrica      *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz -          ******************************/
/*************** Fecha: Octubre 2017 ***************************************/
/*************** Enunciado: Configuracion PC- HC05              ************/
/***************************************************************************/
/*
9600	104	10
19200	52	5.3333
38400	26	2.666
57600	17	23.1111
115200	8	43.555

PE4 RX EN TIVA | TX EN HC05
PE5 TX EN TIVA | RX EN HC05

VCC->3.3V
GND->GND
PE4-> TX EN HC05
PE5-> RX EN HC05

SOLO 4 PINES, CUANDO KEY ESTA ACTIVADO
FUNCIONA PARA CONFIGURAR MEDIANTE COMANDOS AT
*/
//Declaración de librerias
#include <stdint.h>
#include "tm4c123gh6pm.h"
//***************CONFIGURAR UART1********************//
void ConfiguraUART_HC05(void){ // Configuramos puerto UART1
	unsigned long temp;
	SYSCTL_RCGCUART_R |= (1<<5);// Habilitamos UART1
	temp = SYSCTL_RCGC1_R;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;	// Habilitamos GPIOB
	temp = SYSCTL_RCGC2_R;
	UART5_CTL_R &= ~UART_CTL_UARTEN;// Inhabilitamos el UART1
	//Configuramos a 34800 8 N 1 la velocidad de comunicación
	UART5_IBRD_R = (UART5_IBRD_R & ~UART_IBRD_DIVINT_M)|104; // IBRD =int(16,000,000/(16*115,200)) = int(8.68055)
	UART5_FBRD_R = (UART5_FBRD_R & ~UART_FBRD_DIVFRAC_M)|round(10); // FBRD = round(0.68055 * 64)= 43.55
	//UART5_LCRH_R = (UART5_LCRH_R & 0xFFFFFF00) | 0x70;// 8, N, 1, FIFOs habilitados
	UART5_LCRH_R = ((UART5_LCRH_R & ~0x000000FF)|(UART_LCRH_WLEN_8)|(UART_LCRH_FEN));
	UART5_CTL_R |= UART_CTL_UARTEN;	// Habilitamos el UART1
	GPIO_PORTE_AMSEL_R &= ~(0x30);// Desactivamos modo analógico en PB0 y PB1
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF)|0x00110000; // Conectamos UART0 a PB0 y PB1
	GPIO_PORTE_AFSEL_R |= 0x30;// Activamos funciones alternas en PB0 y PB1
	GPIO_PORTE_DEN_R |= 0x30;// Activamos funciones digitales en PB0 y PB1
}
void txcar_uart_HC05(uint32_t car){ //Transimir un caracter por uart1
	while ((UART5_FR_R & UART_FR_TXFF)!=0); //Espera que esté disponible para transmision
	UART5_DR_R = car;//enviar caracter
}
uint8_t rxcar_uart_HC05(void){//recepcion de un caracter
	uint8_t temp;
	while ((UART5_FR_R & UART_FR_RXFE)!=0); // Se espera que llegue un dato
	temp= UART5_DR_R&0xFF; // Se toman solo 8 bits
	return temp; // se retorna valor al ser llamada la funcion
}
void txmens_uart_HC05(uint8_t mens[]){ // Funcion para poder enviar cadena
	uint8_t letra;
	uint8_t i=0; // indice
	letra= mens[i++];
	while (letra != '\0'){ //Se envían todos los caracteres hasta el fin de cadena
		txcar_uart_HC05(letra);
		letra= mens[i++]; //siguiente indice
	}
}
//*************FIN CONFIGURAR UART1********************//
//***************CONFIGURAR UART0********************//
void ConfiguraUART_PC (void){
	unsigned long temp;
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // Se activa el reloj del UART
	temp = SYSCTL_RCGC1_R; // Espera de unos ciclos de reloj
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // Se activa el reloj del puerto A PA0 (U0Rx) PA1( U0Tx)
	temp = SYSCTL_RCGC2_R; // Espera de unos ciclos de reloj
	UART0_CTL_R &= ~ UART_CTL_UARTEN; // Se desactiva el UART
	UART0_IBRD_R = (UART0_IBRD_R & ~UART_IBRD_DIVINT_M)|104; // 16MHz/(16*9600) Parte entera
	UART0_FBRD_R = (UART0_FBRD_R & ~UART_FBRD_DIVFRAC_M)|round(10); //Parte fraccionaria*64
	UART0_LCRH_R = ((UART0_LCRH_R & ~0x000000FF)|(UART_LCRH_WLEN_8)|(UART_LCRH_FEN));
	// Se configuran los bits de datos, 1 bit de parada, sin paridad y habilita el FIFO
	UART0_CTL_R |= UART_CTL_UARTEN; // Se habilita el UART
	GPIO_PORTA_AFSEL_R |= 0x03; // Se activan las funciones alternas de PA0 y
	GPIO_PORTA_DEN_R |= 0x03; // Habilitación PA0 y PA1
}
void txcar_uart_PC(uint32_t car){//enviar caracter
	while ((UART0_FR_R & UART_FR_TXFF)!=0); //Espera que esté disponible para
	UART0_DR_R = car;
}
uint8_t rxcar_uart_PC(void){//recibir caracter
	uint8_t temp;
	while ((UART0_FR_R & UART_FR_RXFE)!=0); // Se espera que llegue un dato
	temp= UART0_DR_R&0xFF; // Se toman solo 8 bits
	return temp;
}
void txmens_uart_PC(uint8_t mens[]){//Similiar a uart1
	uint8_t letra;
	uint8_t i=0;
	letra= mens[i++];
	while (letra != '\0'){ //Se envían todos los caracteres hasta el fin de cadena
		txcar_uart_PC(letra);

		letra= mens[i++];
	}
}
//***********FIN*CONFIGURAR UART0********************//
//________________________MAIN____________________________//

void main(void){
	uint8_t dato;
	long int i;
	long long jmensaje=0;
	uint8_t mensaje[]="] ESPERANDO .... :     ";
	uint8_t enter[]="\r\n";
	uint8_t mensaje2[]="\n\r[";
	uint8_t empezar[]="AT\r\n";
	uint8_t holamundo[]="0";

	ConfiguraUART_PC();
	//config_pulsador_led();
	ConfiguraUART_HC05();
	//Tiempo muerto para dejar prender correctamente
	for (i=0;i<100000;i++);
//	txmens_uart_HC05(empezar);
	for (i=0;i<100000;i++);

	while(1){
		dato=rxcar_uart_HC05();
		///////////PARA EL NUMERO DE MENSAJE
		txmens_uart_PC(mensaje2);
		txcar_uart_PC(jmensaje+48);//para poner el numero 0 en ascii
		txmens_uart_PC(mensaje);
		txcar_uart_PC(dato);
		jmensaje++;
		if (jmensaje==70) jmensaje=0;
	}
}
//_____________________FIN_MAIN____________________________//
