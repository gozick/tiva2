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
uint8_t leer_I2C_BH1750(uint8_t direccion_esclavo, uint8_t registro){
	uint8_t dato,estado;
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
		I2C1_MCS_R= (I2C1_MCS_R & ~0x1F)| 0x7;
		do{
			estado= I2C1_MCS_R;
		}while(estado & 0x01 != 0); 					// se espera que el bit busy se ponga en 0
	}while( (I2C1_MCS_R & 0x02) != 0);					// si hay error vuelve a realizar todos desde el inicio, sino sigue
	dato = I2C1_MDR_R & 0xFF; 							// se lee la data del registro escogido
	//ACK
	
	//I2C1_MCS_R= (I2C1_MCS_R & ~0x1F)| 0x4;			// condicion de parada;
	return dato;
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
//unsigned char Datos[7]; // 3 digitos para enviar
//void NumToChar(long n){
//
//	Datos[0] = n/1000000 + 0x30; // Centenas
//	n = n%1000000; // n ahora esta entre 0 y 999999
//	Datos[1] = n/100000 + 0x30; // Centenas
//	n = n%100000; // n ahora esta entre 0 y 99999
//	Datos[2] = n/10000 + 0x30; // Centenas
//	n = n%10000; // n ahora esta entre 0 y 9999
//	Datos[3] = n/1000 + 0x30; // Centenas
//	n = n%1000; // n ahora esta entre 0 y 999
//	Datos[4] = n/100 + 0x30; // Centenas
//	n = n%100; // n ahora esta entre 0 y 99
//	Datos[5] = n/10 + 0x30; // Decenas
//	n = n%10; // n ahora esta entre 0 y 9
//	Datos[6] = n + 0x30; // Unidades
//	return Datos;
//}
/********************FIN CONVERSION DE DATOS **********************************/
//_________________________________MAIN__________________________________//
void main (void){
	ConfiguraUART_PC(); 			// COMUNICACION UART CON PC
	I2C1_BH1750(); 					//CONFIGURA EL I2C PA6YPA7
	uint8_t dato;
	uint8_t direccion_esclavo=0b0100011;				//7 digitos
	uint8_t registro=0b00010001;						//8 digitos
	dato=leer_I2C_BH1750(direccion_esclavo, registro);	//Escribimos el esclavo
														//escribimos el registro
														//para leer data

//	I2C1(); // PRENDER I2C1
//	//_____________________________1.CALIBRACION_________________________________//
//	//Declaracion de variables
//	uint32_t i,seg;
//	short dato1,dato2,datoreal;
//	short AC1,AC2,AC3;
//	short B1,B2,MB,MC,MD;
//	short oss; // oss =0,1,2,3
//	unsigned short AC4,AC5,AC6;
//	long UT,UP;
//	long altitud, presion;
//	long x1,x2,x3,B5,T;
//	long B6,B3,p;
//	unsigned long B4,B7;
//	uint8_t mensaje2[]= "Presion(Pa): ";
//	uint8_t mensaje3[]= "\n\r ";
//	uint8_t espacio[]= " ";
//	uint8_t mensaje4[]= "Altitud(msnm): ";
//	dato1=0;
//	dato2=0;
//	datoreal=0;
//	AC1=0;
//	AC2=0;
//	AC3=0;
//	AC4=0;
//	AC5=0;
//	AC6=0;
//	B1=0;
//	B2=0;
//	MB=0;
//	MC=0;
//	MD=0;
//	UT=0;
//	UP=0;
//	altitud=0;
//	presion=0;
//	x1=0;
//	x2=0;
//	x3=0;
//	B5=0;
//	B3=0;
//	T=0;
//	B6=0;
//	p=0;
//	B4=0;
//	B7=0;
//	seg=1;
//	oss=0;
//	//Leer datos de calibracion AC1
//	dato1=leer_I2C(0x77,0xAA);
//	dato2=leer_I2C(0x77,0xAB);
//	AC1=(dato1<<8)+dato2;
//	//Leer datos de calibracion AC2
//	dato1=leer_I2C(0x77,0xAC);
//	dato2=leer_I2C(0x77,0xAD);
//	AC2=(dato1<<8)+dato2;
//	//Leer datos de calibracion AC3
//	dato1=leer_I2C(0x77,0xAE);
//	dato2=leer_I2C(0x77,0xAF);
//	AC3=(dato1<<8)+dato2;
//	//Leer datos de calibracion AC4
//	dato1=leer_I2C(0x77,0xB0);
//	dato2=leer_I2C(0x77,0xB1);
//	AC4=(dato1<<8)+dato2;
//	//Leer datos de calibracion AC5
//	dato1=leer_I2C(0x77,0xB2);
//	dato2=leer_I2C(0x77,0xB3);
//	AC5=(dato1<<8)+dato2;
//	//Leer datos de calibracion AC6
//	dato1=leer_I2C(0x77,0xB4);
//	dato2=leer_I2C(0x77,0xB5);
//	AC6=(dato1<<8)+dato2;
//	//Leer datos de calibracion B1
//	dato1=leer_I2C(0x77,0xB6);
//	dato2=leer_I2C(0x77,0xB7);
//	B1=(dato1<<8)+dato2;
//	//Leer datos de calibracion B2
//	dato1=leer_I2C(0x77,0xB8);
//	dato2=leer_I2C(0x77,0xB9);
//	B2=(dato1<<8)+dato2;
//	//Leer datos de calibracion MB
//	dato1=leer_I2C(0x77,0xBA);
//	dato2=leer_I2C(0x77,0xBB);
//	MB=(dato1<<8)+dato2;
//	//Leer datos de calibracion MC
//	dato1=leer_I2C(0x77,0xBC);
//	dato2=leer_I2C(0x77,0xBD);
//	MC=(dato1<<8)+dato2;
//	//Leer datos de calibracion MD
//	dato1=leer_I2C(0x77,0xBE);
//	dato2=leer_I2C(0x77,0xBF);
//	MD=(dato1<<8)+dato2;
//
//	while(1){
//		//___________LEER TEMPERATURA UT_____________//
//
//		escribir_I2C(0x77,0xF4,0x2E);
//		for (i=0;i<100000;i++); //wait 4.5ms
//		dato1=leer_I2C(0x77,0xF6);
//		dato2=leer_I2C(0x77,0xF7);
//		UT=((dato1<<8)+dato2);
//		//______________LEER PRESION UP______________//
//		escribir_I2C(0x77,0xF4,(0x34+(oss<<6)));
//		for (i=0;i<450000;i++); //wait 4.5ms
//		dato1=leer_I2C(0x77,0xF6);
//		dato2=leer_I2C(0x77,0xF7);
//		UP=(((dato1<<16)+(dato2<<8))>>(8-oss));
//		//_______CALCULAR LA REAL TEMPERATURA________//
//		x1=(UT-AC6)*AC5/pow(2,15);
//		x2=MC*pow(2,11)/(x1+MD);
//		B5=x1+x2;
//		T=(B5+8)/pow(2,4);
//
//		//__________TRANSFORMAR PRESION_____________//
//		B6=B5-4000;
//		x1=(B2*(B6*B6/pow(2,12)))/pow(2,11);
//		x2=AC2*B6/pow(2,11);
//		x3=x1+x2;
//		B3=(((AC1*4+x3)<<oss)+2)/4;
//		x1=AC3*B6/pow(2,13);
//		x2=(B1*(B6*B6/pow(2,12)))/pow(2,16);
//		x3=((x1+x2)+2)/pow(2,2);
//		B4=AC4*(unsigned long)(x3+32768)/pow(2,15);
//		B7=((unsigned long)UP-B3)*(50000>>oss);
//		if (B7<0x80000000){
//			p=(B7*2)/B4;
//		}
//		else {
//			p=(B7/B4)*2;
//		}
//		x1=(p/pow(2,8))*(p/pow(2,8));
//		x1=(x1*3038)/pow(2,16);
//		x2=(-7357*p)/pow(2,16);
//		p=p+(x1+x2+3791)/pow(2,4);
//		//__________TRANSFORMAR MSNM_____________//
//		double altitud;
//		double v1,v2;
//		v1=(p*10000)/101325;
//		v1=v1/10000;
//		v2=1/5.225;
//		altitud=44330*(1-pow(v1,v2));
//		//____TRANSFORMAR PARA ENVIAR POR UART______//
//		//______________LIMPIAR REGISTRO____________//
//		for (i=0;i<7;i++){
//			Datos[i]=0;
//		}
//		NumToChar(p);
//		txmens_uart0(mensaje2);//PRESION
//		txmens_uart0(Datos);//VALOR DE PRESION
//		txmens_uart0(espacio);//espacio
//		txmens_uart0(mensaje4);//ALTITUD
//		NumToChar(altitud);
//		txmens_uart0(Datos);//VALOR DE ALTITUD
//		txmens_uart0(mensaje3);//SIGUIENTE LINEA
////
////		for (i=0;i<100000*seg;i++); //wait 4.5ms
//	}//FIN WHILE 1
}//______________FIN__MAIN___________________//
