/*
 * main.c
 * Programa que muestra el uso de interrupciones por systick
 * Las Interrupciones ocurren cada 500ms.
 * Hacemos uso del UART (9600,8,N,1) para sondear un caracter,
 * adicionalmente se desactivan las interruciones de systick para modificar una
 * cuenta regresiva la cual evita que el programa espere indefinidamente la recepcion de un dato
 *
 * Se utiliza la libreria stdbool que permite trabajar con variables booleanas
 */
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define NumeroIntentosMax 5				// Numero maximo de intentos al enviar
uint32_t timeout;
float posX=0;
float posY=0;
uint8_t msj1[]="POSICION X: ";
uint8_t msj2[]=" cm, POSICION Y: ";
uint8_t msj3[]=" , LUMINOSIDAD: ";
uint8_t msj4[]=" lux\n\r";
uint8_t enter[]="\n\r";
int angulo=45;
uint16_t luz=0;
uint8_t power_on=0b00000001;						// Prender el sensor
uint8_t MeasurementCode=0b00010000;					// Mesaurement Command
unsigned char Datos[5]; // 5 digitos para expresar la medida de luz
void configuraPuertosIO(void)
{
	SYSCTL_RCGC2_R|=0x20;  // clock para el PORTF
	while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)==0); // esperamos a que realmente se active
	GPIO_PORTF_DIR_R |= 0x06; // PF2 Y F1 como Salida
	GPIO_PORTF_AFSEL_R &= ~(0x6); 	// PF2 Y PF1 sin funciones alternas
	GPIO_PORTF_DR8R_R |=0x06; 		// Activamos driver de 8mA para PF2 Y PF1
	GPIO_PORTF_DEN_R |= 0x06; 		// Habilitar PF2 Y F1 CON funcion digital
	GPIO_PORTF_DATA_R &= ~(0x06);	// PF2=0 y PF1=0

}

void configuraSystick(void)
{
	NVIC_ST_CTRL_R&=~(0x01);
	NVIC_ST_RELOAD_R=4000000-1;  // periodo de 500ms
	NVIC_ST_CTRL_R|=NVIC_ST_CTRL_CLK_SRC|NVIC_ST_CTRL_INTEN|NVIC_ST_CTRL_ENABLE;
}


void ConfiguraUART0(void)
{
	SYSCTL_RCGC1_R|=0x01;  // clock para el uart0
	SYSCTL_RCGC2_R|=0x01;  // clock para el PORTA0
	while( (SYSCTL_PRUART_R & SYSCTL_PRUART_R0)==0); // esperamos a que realmente se active
	UART0_CTL_R&=0xFFFFFFFE;  // uarten=0;
	UART0_IBRD_R=104;  // BR=9600
	UART0_FBRD_R=11;
	UART0_LCRH_R=0x60;
	UART0_CTL_R=0x0301;  // RXEN=1, TXEN=1 uarten=1;
	GPIO_PORTA_AMSEL_R &= ~(0x03); // no analogico
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)|0x00000011;	// Activamos funciones alternas en PA0 y PA1
	GPIO_PORTA_AFSEL_R |= 0x03;	// Activamos funciones digitales en PA0 y PA1
	GPIO_PORTA_DEN_R |= 0x03;
}


void TxDato(uint8_t dato)
{
	// Espera a que el modulo de transmision del UART esté disponible para enviar un dato de 8 bits
	// in:  Dato a transmitir
	// out: none
	while ((UART0_FR_R & 0x20)==0x20); //Espera que esté disponible para transmitir (TXFF=0)
	UART0_DR_R = dato;
}

uint8_t RxDato(void)
{
	// Espera la recepcion de un dato del UART
	// in:  none
	// out: Dato Recibido
	uint8_t dato;
	while ((UART0_FR_R & 0x10)==0x10); // Esperamos a que llegue algún dato (RXFE=0)
	dato = UART0_DR_R;
	return dato;
}


uint8_t NewRxDato(void)
{
	// Espera la recepcion de un dato del UART, sale luego de 4 segundos
	// in:  none
	// out: Dato Recibido o 0xFF (Tiempo Fuera)
	uint8_t dato;

	timeout=8;
	while ((UART0_FR_R & 0x10)==0x10) // Esperamos a que llegue algún dato (RXFE=0)}
	{
		if(timeout==0) break;
	}
	if(timeout!=0)
	{
		dato = UART0_DR_R;
	}
	else dato=0xFF;   // Devuelve 0xFF si sale por TimeOut!
	return dato;
}
void txmens_uart0(uint8_t mens[])
{
	uint8_t letra;
	uint8_t i=0;
	letra= mens[i++];
	while (letra != '\0')
	{ //Se envían todos los caracteres hasta el fin de cadena
		TxDato(letra);
		letra= mens[i++];
	}
}

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
uint32_t escribir_I2C_BH1750 (uint8_t direccion_esclavo ,uint8_t opecode){
	/* Esta función los Measurement code = opecode
	 * según datasheet de BH1750
	 */
	while(I2C1_MCS_R&I2C_MCS_BUSY){}; 								// Esperar a I2C libre
	I2C1_MSA_R =  (I2C1_MSA_R & ~0xFF) + (direccion_esclavo<<1);	// Direccion de esclavo con W/R=0
	I2C1_MSA_R&=~0x1;												// Modo escritura
	I2C1_MDR_R=opecode&0xFF;										// Enviamos opecode
	I2C1_MCS_R =(0
			|I2C_MCS_STOP											// Generar un stop
			|I2C_MCS_START											// Generar Start/ Restart
			|I2C_MCS_RUN);											// Prender modo Master
	while(I2C1_MCS_R&I2C_MCS_BUSY){}; 								// Esperar a I2C libre
	return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));	// Retornar algun error
}//fin escribir_I2C_BH1750
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
void InterrucionSystick(void)   // RUTINA DE INTERRUPCION , ocurre cada 500ms
{
	uint8_t dato3;
	dato3=NewRxDato();
	if (dato3=='A')
	{
		GPIO_PORTF_DATA_R^=0x02;  	// Conmuta estado del LED AZUL


		posX=posX+cos(angulo)*0.4;			// Posicion relativade x
		posY=posY+sin(angulo)*0.4;			// Posicion relativa de y


		txmens_uart0(msj1);
		NumerotoString(posX);				// POS X
		txmens_uart0(Datos);

		txmens_uart0(msj2);
		NumerotoString(posY);				// POS Y
		txmens_uart0(Datos);

		txmens_uart0(msj3);

		luz=leer_I2C_BH1750(0b0100011);				// Leemos Hight y Low Byte
		luz=luz/1.2;
		NumerotoString(luz);								// Convierto luz a arreglo
		txmens_uart0(Datos);
		txmens_uart0(msj4);
		txmens_uart0(enter);
	}

	if(timeout>0) timeout--;

}
void main(void) {

	uint32_t dato1,dato2;
	configuraPuertosIO();	// ACTIVO PUERTO F2 y F3
	configuraSystick(); 	// Systick @ T=250ms
	ConfiguraUART0(); 		// UART @ 9600N,8,1
	I2C1_BH1750();
	dato1=0x41;
	uint32_t error;
	error=escribir_I2C_BH1750(0b0100011, power_on);				// Prendemos el sistema
		error=escribir_I2C_BH1750(0b0100011, MeasurementCode);		// Ponemos el codigo para leer continuamente
	while(1)
	{
		GPIO_PORTF_DATA_R^=0x04;  	// Conmuta estado del LED AZUL para compararr
		dato2=NewRxDato();			// entre interrupcion y proceso principal
	}

}
