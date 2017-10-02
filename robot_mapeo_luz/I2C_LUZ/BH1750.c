/***************************************************************************/
/*************** Proyecto: SENSOR DE LUZ                 *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz ** **************************************/
/*************** Fecha: Octubre017 *****************************************/
/*************** Enunciado: Se programa el sensor de presion    ************/
/****************y su respectivo i2c el cual es i2c1 ***********************/
/****************hyperterminal a 9600 8 N 1*********************************/
/****************determinación de la altura           **********************/
/***************************************************************************/
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <math.h>

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
//-------------------------------------------------------------------------------
/*********************PRENDER I2C1 PARA BH1750*********************************/
void I2C1_BH1750(void){
	SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R1;			// habilita el reloj del I2C1
	while((SYSCTL_PRI2C_R & SYSCTL_PRI2C_R1)==0);
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 ;// habiilita el reloj del puerto A
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0)==0);
	GPIO_PORTA_AFSEL_R |= 0xC0;				// se activan funciones alternativas
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~0xFF000000) + 0x33000000;
	GPIO_PORTA_ODR_R |= 0x80; 				// se configura el sda como open-drain
	GPIO_PORTA_ODR_R &= ~0x40;				// se desactiva el scl
	GPIO_PORTA_AMSEL_R |= 0xC0;				// desactiva funcion analogica
	GPIO_PORTA_DEN_R |= 0xC0;				// Función digital activada
	I2C1_MCR_R |= 0x10;						// se incializa el maestro
	I2C1_MTPR_R = (7<<0); // configure for 100 kbps clock;
	// se configura la velocidad MTPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;
	I2C1_MCS_R &= ~0x10;					// Se pone el bit HS en 0, se descativa high speedmode
}//fin I2C1
/********************* FIN PRENDER I2C1    ************************************/

/********************    TRANSMISION POR I2C  ************************************/
uint16_t leer_I2C_BH1750(uint8_t direccion_esclavo, uint8_t registro){
	uint8_t dato1,dato2,estado;
	uint16_t medidaluz;
	//I2C1_MCS_R= (I2C1_MCS_R & ~0x17)| 0x7;							// Condicion de inicio
	do{

		while(I2C1_MCS_R&0x00000001){}; // esperar I2C ready
		I2C1_MSA_R =  (I2C1_MSA_R & ~0xFF) +						// se escribe la direccion
				(direccion_esclavo<<1);								// del esclavo y se pone como lectura
		//ACK
		I2C1_MDR_R = (I2C1_MDR_R & ~0xFF) + registro;				//se escribe la direccion del registro que se va a leer
		I2C1_MCS_R= (I2C1_MCS_R & ~0x17)| 0x3;						// condicion de inicio
		do{
			estado= I2C1_MCS_R;
		}while(estado & 0x01 != 0); 								//Se espera que el bit busy se ponga en 0
	}while( (I2C1_MCS_R & 0x02) != 0);								// si hay error vuelve a realizar
	//todos desde el inicio, sino sigue
	do{
		I2C1_MSA_R =  (I2C1_MSA_R & ~0xFF) + (direccion_esclavo<<1) + 1; // se escribe la direccion del esclavo y se pone como lectura
		I2C1_MCS_R= (I2C1_MCS_R & ~0x1F)| 0x7;						//Condicion de parada
		do{
			estado= I2C1_MCS_R;
		}while(estado & 0x01 != 0); 					// se espera que el bit busy se ponga en 0
	}while( (I2C1_MCS_R & 0x02) != 0);					// si hay error vuelve a realizar todos desde el inicio, sino sigue
	dato1 = I2C1_MDR_R & 0xFF; 							// se lee la data del registro escogido
	//ACK
	dato2 = I2C1_MDR_R & 0xFF;
	//I2C1_MCS_R= (I2C1_MCS_R & ~0x1F)| 0x4;			// condicion de parada;
	medidaluz=(dato1<<8)+dato2;
	return medidaluz;
}// fin leer_I2C

void escribir_I2C_BH1750 (uint8_t direccion_esclavo , uint8_t registro , uint8_t valor){
	char estado;
	do{
		//I2C1_MCS_R= (I2C1_MCS_R & ~0x1F)| 0x7;		// condicion de inicio
		I2C1_MSA_R =  (I2C1_MSA_R & ~0xFF) + (direccion_esclavo<<1); // se escribe la direccion del esclavo y se pone como escritura
		I2C1_MDR_R = (I2C1_MDR_R & ~0xFF) + registro;	//se escribe la direccion del registro que se va a escribir
		I2C1_MCS_R= (I2C1_MCS_R & ~0x17)| 0x3;			// condicion de inicio
		do{
			estado= I2C1_MCS_R;
		}while((estado & 0x01) != 0); 					// se espera que el bit busy se ponga en 0
	}while( (I2C1_MCS_R & 0x02) != 0);					// si hay error vuelve a realizar todos desde el inicio, sino sigue

	do{
		I2C1_MDR_R = (I2C1_MDR_R & ~0xFF) + valor;		//se escribe el valor deseado
		I2C1_MCS_R= (I2C1_MCS_R & ~0x1F)| 0x5;			// condicion de parada
		do{
			estado= I2C1_MCS_R;
		}while(estado & 0x01 != 0); 					// se espera que el bit busy se ponga en 0
	}while( (I2C1_MCS_R & 0x02) != 0);					// si hay error vuelve a realizar todos desde el inicio, sino sigue
}//fin escribir_I2C
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
/********************          MAIN          **********************************/
void main (void){
	ConfiguraUART_PC(); 			// COMUNICACION UART CON PC
	I2C1_BH1750(); 					//CONFIGURA EL I2C PA6YPA7
	uint16_t luz;
	int i,contador=0;							//Se usara para retardo
	uint8_t direccion_esclavo=0b0100011;				//7 digitos
	uint8_t registro=0b00010001;						//8 digitos
	uint8_t power_on=0b00000001;						//prender el sensor

	uint8_t lectura2[]="\n\r";								//Enter
	uint8_t unidad_luz[]=" [ lx ]\n\r"	;				//Unidades del sensor de luz
	uint8_t lectura[]="Lectura [";
	uint8_t lectura3[]="] :";

	while(1){											//Siempre ejecutandose
		luz=leer_I2C_BH1750(direccion_esclavo, power_on);	//Escribimos el esclavo
		luz=leer_I2C_BH1750(direccion_esclavo, registro);	//Escribimos el esclavo
		for (i=0;i<400000;i++);
		luz=luz/1.2;
		/////////////////////////////////////////////////////////////////////////
		NumerotoString(luz);							//la variable datos es global
		txmens_uart_PC(lectura);
		txcar_uart_PC(contador+48);						//offset por ser ascii
		txmens_uart_PC(lectura3);						//Lectura numero 3
		txmens_uart_PC(Datos);
		txmens_uart_PC(unidad_luz);						//Fin de todo el mensaje
		Datos[0]=0;
		Datos[1]=0;
		Datos[2]=0;
		Datos[3]=0;
		Datos[4]=0;
		/////////////////////////////////////////////////////////////////////////
		contador++;
		if (contador==10)contador=0;					//Solo secuendia de 0 a 9
	}
}
/********************      FIN  MAIN          **********************************/
