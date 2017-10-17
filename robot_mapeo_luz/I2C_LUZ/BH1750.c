/***************************************************************************/
/*************** Proyecto: SENSOR DE LUZ                 *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz *****************************************/
/*************** Fecha: Octubre2017 ****************************************/
/*************** Enunciado: Se programa el sensor de presion    ************/
/****************y su respectivo i2c el cual es i2c1 ***********************/
/****************hyperterminal a 9600 8 N 1*********************************/
/****************determinación de la altura           **********************/
/***************************************************************************/
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define NumeroIntentosMax 5				// Numero maximo de intentos al enviar
//#define I2C_MCS_ACK		0x8				// Acknowledge prendido
//#define I2C_MCS_DATACK	0x8				// Acknowledge data
//#define I2C_MCS_ADRACK	0x4				// Acknowledge Address
//#define I2C_MCS_STOP	0x4				// Generar Stop
//#define I2C_MCS_START	0x2				// Generate Start
//#define I2C_MCS_ERROR	0x2				// Error
//#define I2C_MCS_RUN		0x1				// I2C prendido
//#define I2C_MCS_BUSY	0x1				// I2C ocupado
//#define I2C_MCR_MFE		0x10			// I2C Master funcion prendido

/*********************COMUNICACION CON PC************************************/
/*
9600	104	10
19200	52	5.3333
38400	26	2.666
57600	17	23.1111
115200	8	43.555
 */
void ConfiguraUART_PC(void){
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;	// Habilitamos reloj para el UART0
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;	// Habilitamos reloj para GPIOA
	UART0_CTL_R &= ~UART_CTL_UARTEN;	// Inhabilitamos el UART0
	UART0_IBRD_R = (UART0_IBRD_R & 0xFFFF0000) | 104;// Velocidad 9600bps (Fsysclk = 16MHz)
	UART0_FBRD_R = (UART0_FBRD_R & 0xFFFFFFC0) | 11;// Velocidad 9600bps (Fsysclk = 16MHz)
	UART0_LCRH_R = (UART0_LCRH_R & 0xFFFFFF00) | 0x70;	// 8, N, 1, FIFOs habilitados
	UART0_CTL_R |= UART_CTL_UARTEN;// Habilitamos el UART0
	GPIO_PORTA_AMSEL_R &= ~(0x03);	// Desactivamos modo analógico en PA0 y PA1
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)|0x00000011;	// Conectamos UART0 a PA0 y PA1
	GPIO_PORTA_AFSEL_R |= 0x03;	// Activamos funciones alternas en PA0 y PA1
	GPIO_PORTA_DEN_R |= 0x03;	// Activamos funciones digitales en PA0 y PA1
}
void txcar_uart_PC(uint32_t car){
	while ((UART0_FR_R & UART_FR_TXFF)!=0); //Espera que esté disponible para transmitir
	UART0_DR_R = car;
}
void txmens_uart_PC(uint8_t mens[]){
	uint8_t letra;
	uint8_t i=0;
	letra= mens[i++];
	while (letra != '\0'){ //Se envían todos los caracteres hasta el fin de cadena
		txcar_uart_PC(letra);
		letra= mens[i++];

	}
}
/*********************FIN COMUNICACION CON PC*********************************/

/*********************PRENDER I2C1 PARA BH1750*********************************/
void I2C1_BH1750(void){
	SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R1;					// habilita el reloj del I2C1
	while((SYSCTL_PRI2C_R & SYSCTL_PRI2C_R1)==0);
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 ;				// habiilita el reloj del puerto A
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0)==0);
	GPIO_PORTA_AFSEL_R |= 0xC0;								// se activan funciones alternativas
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~0xFF000000) + 0x33000000;
	GPIO_PORTA_ODR_R |= 0x80; 								// se configura el sda como open-drain
	GPIO_PORTA_ODR_R &= ~0x40;								// se desactiva el scl
	GPIO_PORTA_AMSEL_R |= 0xC0;								// desactiva funcion analogica
	GPIO_PORTA_DEN_R |= 0xC0;								// Función digital activada
	I2C1_MCR_R |= 0x10;										// se incializa el maestro
	I2C1_MTPR_R = (24);									// configure for 100 kbps clock;
	// se configura la velocidad MTPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;
	I2C1_MCS_R &= ~0x10;									// Se pone el bit HS en 0, se descativa high speedmode
}//fin I2C1
/********************* FIN PRENDER I2C1    ************************************/

/********************    TRANSMISION POR I2C  ************************************/
uint16_t leer_I2C_BH1750(uint8_t direccion_esclavo){
	/*
	 * Esta función recibe dos bytes y retorna un numero
	 * sin signo de 16 bits que representa la medicion del sensor
	 * pero aun falta modificar este dato para que sea real
	 */
	uint8_t datoH=0,datoL=0;
	uint8_t intento=1;
	//I2C1_MCS_R= (I2C1_MCS_R & ~0x17)| 0x1;					// Bit RUN 1
	do{															// MCS modo lectura
		while(I2C1_MCS_R&I2C_MCS_BUSY); 						// Esperar I2C
		I2C1_MSA_R =  (I2C1_MSA_R & ~0xFF) +					// Limpiamos registro
				(direccion_esclavo<<1);							// Direccion de esclavo +W/R=1
		I2C1_MSA_R|=0x1;										// Como lectura
		I2C1_MCS_R=(0
				|I2C_MCS_ACK									// ACK positivo activado
				|I2C_MCS_START									// Generar start/restart
				|I2C_MCS_RUN);									// Modo master prendido
		while(I2C1_MCS_R&I2C_MCS_BUSY); 						// Esperar I2C
		datoH=(I2C1_MDR_R&0xFF);								// Recibo Hight Byte del sensor
		I2C1_MCS_R=(0
				|I2C_MCS_STOP									// ACK positivo activado
				//|I2C_MCS_START								// Generar start/restart
				|I2C_MCS_RUN);									// Modo master prendido
		while(I2C1_MCS_R&I2C_MCS_BUSY); 						// Esperar I2C
		datoL=(I2C1_MDR_R&0xFF);								// Recibo Hight Byte del sensor
		intento++;												// Aumentar el contador
	}while(((I2C1_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR))!=0)&&(intento<=NumeroIntentosMax));//Mientras sea menor el intento
	return (datoH<<8)+datoL;										// Devolvemos valor medido
}// fin leer_I2C
uint32_t escribir_I2C_BH1750 (uint8_t direccion_esclavo ,uint8_t opecode){
	/* Esta función los Measurement code = opecode
	 * según datasheet de BH1750
	 */
	while(I2C1_MCS_R&I2C_MCS_BUSY); 								// Esperar a I2C libre
	I2C1_MSA_R =  (I2C1_MSA_R & ~0xFF) + (direccion_esclavo<<1);	// Direccion de esclavo con W/R=0
	I2C1_MSA_R&=~0x1;												// Modo escritura
	I2C1_MDR_R=opecode&0xFF;										// Enviamos opecode
	I2C1_MCS_R =(0
			|I2C_MCS_STOP											// Generar un stop
			|I2C_MCS_START											// Generar Start/ Restart
			|I2C_MCS_RUN);											// Prender modo Master
	while(I2C1_MCS_R&I2C_MCS_BUSY); 								// Esperar a I2C libre
	return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));	// Retornar algun error
}//fin escribir_I2C_BH1750
/******************** FIN TRANSMISION POR I2C  ************************************/

/******************** CONVERSION DE DATOS ************************************/
unsigned char Datos[5]; // 5 digitos para expresar la medida de luz
void NumerotoString(uint16_t n){

	//	Datos[0] = n/1000000 + 0x30; // Centenas
	//	n = n%1000000; // n ahora esta entre 0 y 999999
	//	Datos[1] = n/100000 + 0x30; // Centenas
	//	n = n%100000; // n ahora esta entre 0 y 99999
	Datos[0] = n/10000 + 0x30; 					// Decenas de miles
	n = n%10000; 								// n ahora esta entre 0 y 9999
	Datos[1] = n/1000 + 0x30; 					// Miles
	n = n%1000;									// n ahora esta entre 0 y 999
	Datos[2] = n/100 + 0x30; 					// Centenas
	n = n%100;									// n ahora esta entre 0 y 99
	Datos[3] = n/10 + 0x30; 					// Decenas
	n = n%10; 									// n ahora esta entre 0 y 9
	Datos[4] = n + 0x30;						// Unidades
	return Datos;
}
/********************FIN CONVERSION DE DATOS **********************************/
void retardo_ms (uint32_t milisegundos) {
	/* Esta función genera un retardo exacto contado en milisegundos
	 * solo basta ingresar y generará un retardo con error de +/- 1/16millones
	 */
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; 										//Deshabilitamos Systick
	NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R & ~0xFFFFFF) + (16000*milisegundos - 1); 	//Programamos valor de recarga
	NVIC_ST_CURRENT_R = (NVIC_ST_CURRENT_R & ~0xFFFFFF) + 1; 						//Borramos la cuenta
	NVIC_ST_CTRL_R |= (NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC); 				//Habilitamos Systick
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT)==0);
}
/********************          MAIN          **********************************/
void main (void){
	ConfiguraUART_PC(); 								// COMUNICACION UART CON PC
	I2C1_BH1750(); 										// CONFIGURA EL I2C PA6 CLOCK Y PA7 DATA
	uint16_t luz=0;										// Esta variable almacena valor leido I2C
	int i,contador=0;									// Se usara para retardo
	uint8_t direccion_esclavo=0b0100011;				// 7 digitos
	uint8_t power_on=0b00000001;						// Prender el sensor
	uint8_t MeasurementCode=0b00010000;					// Mesaurement Command
	uint8_t unidad_luz[]=" [ lx ]\n\r"	;				// Unidades del sensor de luz
	uint8_t lectura[]="Lectura [";						// TEXTO
	uint8_t lectura3[]="] :";							// TEXTO
	uint32_t error=0;
	error=escribir_I2C_BH1750(direccion_esclavo, power_on);	// Prendemos el sistema
	error=escribir_I2C_BH1750(direccion_esclavo, MeasurementCode);	// Prendemos el sistema
	while(1){												// Siempre ejecutandose

		//retardo_ms(120);									// Retardo para siguiente lectura
		//escribir_I2C_BH1750(direccion_esclavo, MeasurementCode);	// Comando Measurement Command
		//retardo_ms(10);									// Retardo para siguiente lectura
		luz=leer_I2C_BH1750(direccion_esclavo);				// Leemos Hight y Low Byte
		luz=luz/1.2;										// Calculamos la real medicion
		/////////////////////////////////////////////////////////////////////////
		NumerotoString(luz);								// Convierto luz a arreglo
		txmens_uart_PC(lectura);							// TEXTO
		txcar_uart_PC(contador+48);							// +48 o +0x30 por ser codigo ASCII
		txmens_uart_PC(lectura3);							// TEXTO
		txmens_uart_PC(Datos);								// Se envia datos con el arreglo
		txmens_uart_PC(unidad_luz);							// TEXTO : Unidades de la medicion
		Datos[0]=0;											// Se Borra valores dentro del
		Datos[1]=0;											// Arreglo
		Datos[2]=0;
		Datos[3]=0;
		Datos[4]=0;
		/////////////////////////////////////////////////////////////////////////
		contador++;											// PARA TEXTO
		if (contador==10)contador=0;						// TEXTO : Contador
	}
}/********************   FIN  MAIN     **********************************/
