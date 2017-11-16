//=============================================
//=============================================
//				LABORATORIO 5
// Autor:	Pablo Díaz V.
// Horario: H522
// Puerto usado: PB4 PWM
//=============================================

#include "stdio.h"
#include "math.h"
#include "stdint.h"
#include "tm4c123gh6pm.h"
void Configurar_TIMER1AYB_PWM(void)
{
	//--------------------TIMER1 A------------------------//
	SYSCTL_RCGC1_R |= (1<<17); 							// 1. Reloj Timer1
	while((SYSCTL_PRTIMER_R&0x2)!=0x2);					// 2. Esperamos a real prendido
	TIMER1_CTL_R&=~TIMER_CTL_TAEN;						// 3. Desactivamos TIMER A
	TIMER1_CFG_R = (TIMER1_CFG_R & ~0x7) + 0x04;		// 4. Configuramos
	TIMER1_TAMR_R = (TIMER1_TAMR_R &~0xFFF) + 0x50A;	// 5. Modo PWM
	TIMER1_CTL_R |= TIMER_CTL_TAPWML;					// 6. PWM

	SYSCTL_RCGC2_R|=0x2;								// Puerto B4 prendido
	while((SYSCTL_PRGPIO_R&0X2)!=0x2);					// Espera al reloj prendido
	GPIO_PORTB_AFSEL_R|=(1<<4);							// Activamos funcion alternativa
	GPIO_PORTB_PCTL_R=(GPIO_PORTB_PCTL_R&0xFFF0FFFF)+0x00070000;// Funcion PWM
	GPIO_PORTB_DIR_R|=(1<<4);							// Esta como entrada
	GPIO_PORTB_DR8R_R|=(1<<4);							// Habilitamos driver de corriente 8mA
	GPIO_PORTB_DEN_R|=(1<<4);							// Activamos pin como digital

	TIMER1_CTL_R|=TIMER_CTL_TAEN;						// Activamos TIMER1 A

}

void DetenerPWM(void)
{
	TIMER1_TAILR_R = 0;									// Valor de tope
	TIMER1_TAPR_R = 0;									// Del contador
	TIMER1_TAMATCHR_R = 0;								// Valor de comparacion
	TIMER1_TAPMR_R = 0;									// Del contador
}

void cambiarPWM(frecuencia, dutycycle)
{
	TIMER1_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TAMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TAPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void ConfiguraUART0(void)
{

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

void txcar_uart0(uint32_t car)
{
	while ((UART0_FR_R & UART_FR_TXFF)!=0); //Espera que esté disponible para transmitir
	UART0_DR_R = car;
}
uint8_t rxcar_uart0(void)
{
	uint8_t temp;
	while ((UART0_FR_R & UART_FR_RXFE)!=0); // Se espera que llegue un dato
	temp= UART0_DR_R&0xFF; // Se toman solo 8 bits
	return temp;
}
void txmens_uart0(uint8_t mens[])
{
	uint8_t letra;
	uint8_t i=0;
	letra= mens[i++];
	while (letra != '\0')
	{ //Se envían todos los caracteres hasta el fin de cadena
		txcar_uart0(letra);
		letra= mens[i++];
	}
}
int main (void)
{
	uint8_t msj1[]="INTRODUZCA EL ANGULO A MOVER: \n\r";
	uint8_t msj2[]="DIRECCIONANDO AL ANGULO ";
	uint8_t enter[]="\n\r";
	ConfiguraUART0();
	Configurar_TIMER1AYB_PWM();
	int esAngulo=0;
	int x;
	int frecuencia=50;
	DetenerPWM();
	while(1)
	{
		txmens_uart0(msj1);		// Enviamos mensaje
		while (esAngulo==0)
		{
			x=rxcar_uart0();
			switch (x)
			{
			case '1':
				cambiarPWM(frecuencia, 5);
				esAngulo=1;
				break;
			case '2':
				cambiarPWM(frecuencia, 6);
				esAngulo=1;
				break;
			case '3':
				cambiarPWM(frecuencia, 7);
				esAngulo=1;
				break;
			case '4':
				cambiarPWM(frecuencia, 8);
				esAngulo=1;
				break;
			case '5':
				cambiarPWM(frecuencia, 10);
				esAngulo=1;
				break;
			default:
				//cambiarPWM(1, 50);
				break;
			}
		}
		txmens_uart0(enter);
		txmens_uart0(msj2);
		txcar_uart0(x);
		txmens_uart0(enter);
		esAngulo=0;
	}
}
