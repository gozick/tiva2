/********************************************************************/
/*************** LABORATORIO N° 4   H522          *******************/
/********************************************************************/
/*************** Microcontrolador: TM4C123GH6PM *********************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad ***************/
/*************** Autor: Pablo Díaz **********************************/
/*************** Fecha: Noviembre 2017 ******************************/
/*************** Descripción: Maquina de corte mediante**************/
/*************** comandos enviados por computadora     **************/
/********************************************************************/
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <math.h>

unsigned char Datos[5];

void ConfiguraUART_PC (void){//***************CONFIGURAR UART0********************//
	unsigned long temp;
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;		// Se activa el reloj del UART
	temp = SYSCTL_RCGC1_R;						// Espera de unos ciclos de reloj
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;		// Se activa el reloj del puerto A PA0 (U0Rx) PA1( U0Tx)
	temp = SYSCTL_RCGC2_R;						// Espera de unos ciclos de reloj
	UART0_CTL_R &= ~ UART_CTL_UARTEN;			// Se desactiva el UART
	UART0_IBRD_R = (UART0_IBRD_R & ~UART_IBRD_DIVINT_M)|104; // 16MHz/(16*9600) Parte entera
	UART0_FBRD_R = (UART0_FBRD_R & ~UART_FBRD_DIVFRAC_M)|10; //Parte fraccionaria*64
	UART0_LCRH_R = ((UART0_LCRH_R & ~0x000000FF)|(UART_LCRH_WLEN_8)|(UART_LCRH_FEN));
	// Se configuran los bits de datos, 1 bit de parada, sin paridad y habilita el FIFO
	UART0_CTL_R |= UART_CTL_UARTEN;				// Se habilita el UART
	GPIO_PORTA_AFSEL_R |= 0x03;					// Se activan las funciones alternas de PA0 y
	GPIO_PORTA_DEN_R |= 0x03;					// Habilitación PA0 y PA1
}
void txcar_uart_PC(uint32_t car){				// Enviar caracter
	while ((UART0_FR_R & UART_FR_TXFF)!=0);		// Espera que esté disponible para
	UART0_DR_R = car;
}
uint8_t rxcar_uart_PC(void){					// Recibir caracter
	uint8_t temp;
	while ((UART0_FR_R & UART_FR_RXFE)!=0);		// Se espera que llegue un dato
	temp= UART0_DR_R&0xFF;						// Se toman solo 8 bits
	return temp;
}
void txmens_uart_PC(uint8_t mens[]){			// Similar a UART1
	uint8_t letra;
	uint8_t i=0;
	letra= mens[i++];
	while (letra != '\0'){						//Se envían todos los caracteres hasta el fin de cadena
		txcar_uart_PC(letra);

		letra= mens[i++];
	}
}//***********FIN*CONFIGURAR UART0********************//

int recibirNumero(void)
{
	uint32_t caracterRecibido;
	uint32_t contador=0;
	uint32_t numero=0;
	uint8_t msg[]=" No se ingreso un numero! \n \r";
	do{
		caracterRecibido=rxcar_uart_PC();
		if ((caracterRecibido<'0')&&(caracterRecibido>'9'))
		{
			txmens_uart_PC(msg);
			break; 										// Si recibe cualquier otro valor
		}else{
			caracterRecibido-=30;						// Codigo ASCII
			numero=numero*pow(10,contador)+caracterRecibido;// Algoritmo
			contador++;
		}
	}while((caracterRecibido!='\n')&&(caracterRecibido!='\r'));

	return numero;
}

void Config_SW_LEDS_PUERTOF(void){ 			//Configuramos puerto F sw1 sw2 LEDS
	unsigned long temp;
	SYSCTL_RCGCGPIO_R |= 0xFF;				// Se habilita la señal de reloj del Puerto F
	temp = SYSCTL_RCGC2_R;					// Espera a que se estabilice el reloj
	GPIO_PORTF_LOCK_R = 0x4C4F434B;			//Quitamos proteccion
	// Habilitar el bit0 (PF0 - pin protegido)
	GPIO_PORTF_CR_R |= 0X1;					// Se habilita la modificación de varios registros del PF0
	GPIO_PORTF_LOCK_R = 0;					// Se activa la proteccion
	GPIO_PORTF_AMSEL_R &= ~(0X1F);			// Se deshabilitan las fun. Analog. de los bits 0 al 4
	GPIO_PORTF_PCTL_R &= ~(0X000FFFFF);		// Se asegura el uso GPIO
	GPIO_PORTF_DIR_R &= ~(0X11);			//Se configura las entradas
	GPIO_PORTF_DIR_R |= 0X0E;				//Se configura las salidas
	GPIO_PORTF_AFSEL_R &= ~(0X1F);			// Se configuran los bits 0 al 4 como GPIO
	GPIO_PORTF_PUR_R |= 0X11;				// Se activan la resist. pull-up de las entradas bits 0 y 4
	GPIO_PORTF_DR8R_R |= 0X0E;				//Se activa salida de 8mA
	GPIO_PORTF_DEN_R |= 0X1F;				// Se habilitan las salidas digitales de los bits 0 al 4
}
void retardo_ms (uint32_t milisegundos){
	/* Esta función genera un retardo exacto contado en milisegundos
	 * solo basta ingresar y generará un retardo con error de +/- 1/16millones
	 */
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; 										//Deshabilitamos Systick
	NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R & ~0xFFFFFF) + (16000*milisegundos - 1); 	//Programamos valor de recarga
	NVIC_ST_CURRENT_R = (NVIC_ST_CURRENT_R & ~0xFFFFFF) + 1; 						//Borramos la cuenta
	NVIC_ST_CTRL_R |= (NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC); 				//Habilitamos Systick
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT)==0);
}

void NumerotoString(uint16_t n){
	/*
	 * Esta función convierte un número a caracter ascii guardados en un arreglo
	 */
	Datos[0] = n/10000 + 0x30; 					// Decenas de miles
	n = n%10000; 								// n ahora esta entre 0 y 9999
	Datos[1] = n/1000 + 0x30; 					// Miles
	Datos[2] = n/100 + 0x30; 					// Centenas
	n = n%1000;									// n ahora esta entre 0 y 999
	n = n%100;									// n ahora esta entre 0 y 99
	Datos[3] = n/10 + 0x30; 					// Decenas
	n = n%10; 									// n ahora esta entre 0 y 9
	Datos[4] = n + 0x30;						// Unidades
}
void main (void)
{
	ConfiguraUART_PC();							// Configura la comunicación PC
	Config_SW_LEDS_PUERTOF();					// Configuramos las salidas

	uint8_t msg1[]=" ¡BIENVENIDO! MAQUINA XY DE CORTE EN ESPERA DE COMANDO \n\r";
	uint8_t msg2_1[]=" INTRODUZCA LA POSICIÓN ";
	uint8_t msg2_2[]=" (X, Y) DEL POLÍGONO DE CORTE EN CM: \n\r";
	uint8_t msg3_1[]=" SE INTRODUJO LA COORDENADA X = ";
	uint8_t msg3_2[]=" CM, FALTA LA COORDENADA Y \n\r";
	uint8_t msg4_1[]=" COORDENADA X INCORRECTA, VUELVA A INTRODUCIRLA \n\r";
	uint8_t msg4_2[]=" COORDENADA Y INCORRECTA, VUELVA A INTRODUCIRLA \n\r";
	uint8_t msg5_1[]=" POSICION ";
	uint8_t msg5_2[]=" (";
	uint8_t msg5_3[]=",";
	uint8_t msg5_4[]=") CORRECTAMENTE INGRESADA \r\n";
	uint8_t msg6[]=" NUMERO DE POSICIONES INSUFICIENTES PARA EJECUTAR EL CORTE\n\r";

	uint8_t msg7[]=" PROCESO DE CORTE INICIADO, CODIGO G: \n\r";
	uint8_t msg8[]=" M03 \n\r";
	uint8_t msg9[]=" M00 \n\r";
	uint8_t msg11_1[]=" G00 X";
	uint8_t msg11_2[]=" Y";
	uint8_t msg11_3[]="\n\r";
	uint8_t msg11_4[]=" G01 X";
	uint8_t msg12_1[]=" G01 Z-1";
	uint8_t msg12_2[]=" G01 Z1";
	uint8_t msg13[]="N";
	uint8_t fin[]=" PROCESO DE CORTE COMPLETADO CON EXITO\n\r";

	uint32_t contadorCorte=1;
	uint32_t banderaX=0,banderaY=0;
	uint32_t cantMaxPosic=20;
	int numeroX=0,numeroY=0;
	uint32_t m=0;
	uint32_t arregloX[cantMaxPosic],arregloY[cantMaxPosic];


	while(1)
	{
		GPIO_PORTF_DATA_R=0x8;					// Prender led azul
		txmens_uart_PC(msg1);					// Texto Bienvenidos
		while (GPIO_PORTA_DATA_R&0x10!=0);		// Mientras este presionado SW1
		while (GPIO_PORTA_DATA_R&0x10==0x10);	// Mientras este suelto SW1
		GPIO_PORTF_DATA_R=0x4;					// Prender led verde
		while(contadorCorte<=cantMaxPosic&&contadorCorte>=2)
		{

			txmens_uart_PC(msg2_1);					// Texto INTRODUZCA POS
			txcar_uart_PC(contadorCorte);			// Texto X
			txmens_uart_PC(msg2_2);					// Texto termina msg2

			while (banderaX==0&&banderaY==0)
			{
				//	COORDENADA X
				numeroX=recibirNumero();				// Recibimos numero
				if ((numeroX>=0)&&(numeroX<=100))
				{
					txmens_uart_PC(msg3_1);				// Numero X entre 0 y 100
					NumerotoString(numeroX);
					txmens_uart_PC(Datos);
					txmens_uart_PC(msg3_2);
					banderaX=1;
					//	COORDENADA Y
					numeroY=recibirNumero();			// Recibimos numero
					if ((numeroY>=0)&&(numeroY<=120))
					{
						banderaY=1;
					}else{
						txmens_uart_PC(msg4_2);			// Cordenada incorrecta
					}
				}else{
					txmens_uart_PC(msg4_1);				// Cordenada incorrecta
				}

			}
			banderaX=0;									// Reiniciamos bandera
			banderaY=0;
			// Guardamos en arreglos las posiciones X e Y

			arregloX[contadorCorte--]=numeroX;
			arregloY[contadorCorte--]=numeroY;


			txmens_uart_PC(msg5_1);						// Posicion correctamente
			NumerotoString(contadorCorte);
			txmens_uart_PC(Datos);
			txmens_uart_PC(msg5_2);
			NumerotoString(numeroX);
			txmens_uart_PC(Datos);
			txmens_uart_PC(msg5_3);
			NumerotoString(numeroY);
			txmens_uart_PC(Datos);
			txmens_uart_PC(msg5_4);
			contadorCorte++;							// Siguiente posicion
			retardo_ms(2000);
			if (GPIO_PORTF_DATA_R&0x1==1){
				while (GPIO_PORTF_DATA_R&0x1==1);
				while(GPIO_PORTF_DATA_R&0x1!=0);
				if (contadorCorte<=2)
				{
					txmens_uart_PC(msg6);				// Insuficiente
				}else{
					GPIO_PORTF_DATA_R=0x2;
					break;
					// Salimos del bucle
				}
			}
		}

		txmens_uart_PC(msg7);
		//Empieza proceso de corte
		retardo_ms(10000);

		/////////////////////////////////////////////
		txmens_uart_PC(msg13);						//N
		NumerotoString(m);
		txmens_uart_PC(Datos);

		txmens_uart_PC(msg11_1);					//G00 X
		NumerotoString(arregloX[m]);
		txmens_uart_PC(Datos);
		txmens_uart_PC(msg11_2);					//G00 Y
		NumerotoString(arregloY[m]);
		txmens_uart_PC(Datos);
		txmens_uart_PC(msg11_3);					// ENTER

		retardo_ms(1000);

		txmens_uart_PC(msg13);						//N
		NumerotoString(m++);
		txmens_uart_PC(Datos);

		txmens_uart_PC(msg8);						// M03

		retardo_ms(1000);

		txmens_uart_PC(msg13);						//N
		NumerotoString(m++);
		txmens_uart_PC(Datos);

		txmens_uart_PC(msg12_1);

		retardo_ms(1000);
		//////////////////INICIO DEL CORTE//////////////////

		for (m=3;m<=contadorCorte;m++)
		{
			txmens_uart_PC(msg13);						//N
			NumerotoString(m);
			txmens_uart_PC(Datos);

			txmens_uart_PC(msg11_4);

			NumerotoString(arregloX[m-3]);;
			txmens_uart_PC(Datos);
			txmens_uart_PC(msg11_2);

			NumerotoString(arregloY[m-3]);;
			txmens_uart_PC(Datos);

			txmens_uart_PC(msg11_3);					// ENTER

			retardo_ms(1000*round(sqrt(pow(arregloX[m-3],2)+
					pow(arregloY[m-3],2))));
		}

		txmens_uart_PC(msg13);						//N
		NumerotoString(m++);
		txmens_uart_PC(Datos);

		txmens_uart_PC(msg12_2);

		retardo_ms(1000);

		//N
		NumerotoString(m++);
		txmens_uart_PC(Datos);

		txmens_uart_PC(msg9);	//PRENULTIMO


		m=0;					// LLEVAMOS AL INICIO DE COORDENADAS

		NumerotoString(m);
		txmens_uart_PC(Datos);
		txmens_uart_PC(msg9);	//PRENULTIMO

		txmens_uart_PC(msg11_1);					// G00 X
		NumerotoString(00);
		txmens_uart_PC(Datos);
		txmens_uart_PC(msg11_2);					// G00 Y
		NumerotoString(00);
		txmens_uart_PC(Datos);
		txmens_uart_PC(msg11_3);					// ENTER

		txmens_uart_PC(fin);						// TERMINADO


		contadorCorte=1;
		banderaX=0;
		banderaY=0;
		cantMaxPosic=20;
		numeroX=0;
		numeroY=0;
	}
}