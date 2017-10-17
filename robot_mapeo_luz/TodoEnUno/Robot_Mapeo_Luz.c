/***************************************************************************/
/*************** Proyecto: ROBOT MAPEO DE LUZ            *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz | Jose **********************************/
/*************** Fecha: Octubre2017 ****************************************/
/*************** Descripción del proyecto adjunto con el    ****************/
/*************** informe final                              ****************/
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
/* Descripción: PWM
 * El TIMER1 controla el avance hacia el frente, el TIMER3 controla el retroceso
 * Estas ondas PWM interactúan con el driver LN298 que controla a los motores
 * mediante la configuracion de sus pines IN1|IN2|IN3|IN4
 * La fuente del driver esta conectado a +12V
 * IN1|IN2 ------ TIMER1A|TIMER3A
 * IN3|IN4 ------ TIMER1B|TIMER3B
 */
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define NumeroIntentosMax 5							// Numero maximo de intentos al enviar
int frecuencia =10;									// Frecuencia de la Onda PWM
int dutycycle =50;									// DutyCycle empleado por los motores
int TiempoDeGiro= 50
		;								// Tiempo empleado para girar motores 90 grados
uint8_t direccion_esclavo=0b0100011;				// 7 digitos
uint8_t power_on=0b00000001;						// Prender el sensor
uint8_t MeasurementCode=0b00010000;					// Mesaurement Command
unsigned char Datos[5]; 							// 5 digitos para expresar la medida de luz
uint8_t unidad_luz[]=" [ lx ]\n\r"	;				// TEXTO Unidades del sensor de luz
uint8_t lectura[]="Lectura [";						// TEXTO
uint8_t lectura3[]="] :";							// TEXTO

void ConfiguraUART_HC05(void){ //***************CONFIGURAR UART1********************//
	unsigned long temp;
	SYSCTL_RCGCUART_R |= (1<<5);							// Habilitamos UART1
	temp = SYSCTL_RCGC1_R;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;					// Habilitamos GPIOB
	temp = SYSCTL_RCGC2_R;
	UART5_CTL_R &= ~UART_CTL_UARTEN;						// Inhabilitamos el UART1
	//Configuramos a 34800 8 N 1 la velocidad de comunicación
	UART5_IBRD_R = (UART5_IBRD_R & ~UART_IBRD_DIVINT_M)|104; // IBRD =int(16,000,000/(16*115,200)) = int(8.68055)
	UART5_FBRD_R = (UART5_FBRD_R & ~UART_FBRD_DIVFRAC_M)|10; // FBRD = round(0.68055 * 64)= 43.55
	//UART5_LCRH_R = (UART5_LCRH_R & 0xFFFFFF00) | 0x70;// 8, N, 1, FIFOs habilitados
	UART5_LCRH_R = ((UART5_LCRH_R & ~0x000000FF)|(UART_LCRH_WLEN_8)|(UART_LCRH_FEN));
	UART5_CTL_R |= UART_CTL_UARTEN;							// Habilitamos el UART1
	GPIO_PORTE_AMSEL_R &= ~(0x30);							// Desactivamos modo analógico en PB0 y PB1
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF)|0x00110000; // Conectamos UART0 a PB0 y PB1
	GPIO_PORTE_AFSEL_R |= 0x30;								// Activamos funciones alternas en PB0 y PB1
	GPIO_PORTE_DEN_R |= 0x30;								// Activamos funciones digitales en PB0 y PB1
}
void txcar_uart_HC05(uint32_t car){ 			// Transimir un caracter por uart1
	while ((UART5_FR_R & UART_FR_TXFF)!=0); 	// Espera que esté disponible para transmision
	UART5_DR_R = car;							// Enviar caracter
}
uint8_t rxcar_uart_HC05(void){					// Recepcion de un caracter
	uint8_t temp;
	while ((UART5_FR_R & UART_FR_RXFE)!=0); 	// Se espera que llegue un dato
	temp= UART5_DR_R&0xFF;						// Se toman solo 8 bits
	return temp;								// Se retorna valor al ser llamada la funcion
}
void txmens_uart_HC05(uint8_t mens[]){			// Funcion para poder enviar cadena
	uint8_t letra;
	uint8_t i=0; 								// indice
	letra= mens[i++];
	while (letra != '\0'){						//Se envían todos los caracteres hasta el fin de cadena
		txcar_uart_HC05(letra);
		letra= mens[i++];						//siguiente indice
	}
}//*************FIN CONFIGURAR UART1********************//
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
void Configurar_TIMER1AYB_TIMER3AYB_PWM(void)
{
	//--------------------TIMER1 A------------------------//
	// El TIMER 1 Nos servirá para avanzar y girar ambos lados
	// mientras que el TIMER 3 nos servirá para ir en reversa
	// y girar ambos lados pero en dirección contraria
	SYSCTL_RCGC1_R |= (1<<17); 							// 1. Reloj Timer1
	while((SYSCTL_PRTIMER_R&0x2)!=0x2);					// 2. Esperamos a real prendido
	TIMER1_CTL_R&=~TIMER_CTL_TAEN;						// 3. Desactivamos TIMER A
	TIMER1_CFG_R = (TIMER1_CFG_R & ~0x7) + 0x04;		// 4. Configuramos
	TIMER1_TAMR_R = (TIMER1_TAMR_R &~0xFFF) + 0x50A;	// 5. Modo PWM
	TIMER1_CTL_R |= TIMER_CTL_TAPWML;					// 6. PWM
	/* Las siguientes instrucciones se realizarian si se quiere una onda fija sin modificar
	TIMER0_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF; 			//Valor tope de contador
	TIMER0_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER0_TAMATCHR_R = ((16000*dutycycle)/frecuencia) & 0xFFFF; 	//Valor de comparación
	TIMER0_TAPMR_R = (((16000*dutycycle)/frecuencia) >> 16);
	 */
	SYSCTL_RCGC2_R|=0x2;								// Puerto B4 prendido
	while((SYSCTL_PRGPIO_R&0X2)!=0x2);					// Espera al reloj prendido
	GPIO_PORTB_AFSEL_R|=(1<<4);							// Activamos funcion alternativa
	GPIO_PORTB_PCTL_R=(GPIO_PORTB_PCTL_R&0xFFF0FFFF)+0x00070000;// Funcion PWM
	GPIO_PORTB_DIR_R|=(1<<4);							// Esta como entrada
	GPIO_PORTB_DR8R_R|=(1<<4);							// Habilitamos driver de corriente 8mA
	GPIO_PORTB_DEN_R|=(1<<4);							// Activamos pin como digital

	TIMER1_CTL_R|=TIMER_CTL_TAEN;						// Activamos TIMER1 A

	//--------------------TIMER1 B------------------------//
	//SYSCTL_RCGC1_R |= (1<<17); 							// 1. Reloj Timer1
	//while((SYSCTL_PRTIMER_R&0x2)!=0x2);					// 2. Esperamos a real prendido
	TIMER1_CTL_R&=~TIMER_CTL_TBEN;							// 3. Desactivamos TIMER A
	TIMER1_CFG_R = (TIMER1_CFG_R & ~0x7) + 0x04;			// 4. Configuramos
	TIMER1_TBMR_R = (TIMER1_TBMR_R &~0xFFF) + 0x50A;		// 5. Modo PWM
	TIMER1_CTL_R |= TIMER_CTL_TBPWML;						// 6. PWM

	//SYSCTL_RCGC2_R|=0x2;									// Puerto B5 prendido
	//while((SYSCTL_PRGPIO_R&0X2)!=0x2);					// Espera al reloj prendido
	GPIO_PORTB_AFSEL_R|=(1<<5);								// Activamos funcion alternativa
	GPIO_PORTB_PCTL_R=(GPIO_PORTB_PCTL_R&0xFF0FFFFF)+0x00700000;// Funcion PWM
	GPIO_PORTB_DIR_R|=(1<<5);								// Esta como entrada
	GPIO_PORTB_DR8R_R|=(1<<5);								// Habilitamos driver de corriente 8mA
	GPIO_PORTB_DEN_R|=(1<<5);								// Activamos pin como digital

	TIMER1_CTL_R|=TIMER_CTL_TBEN;						// Activamos TIMER1 B

	//--------------------TIMER3 A------------------------//
	SYSCTL_RCGC1_R |= (1<<19); 							// 1. Reloj Timer3
	while((SYSCTL_PRTIMER_R&(1<<3))!=(1<<3));			// 2. Esperamos a real prendido
	TIMER3_CTL_R&=~TIMER_CTL_TAEN;						// 3. Desactivamos TIMER A
	TIMER3_CFG_R = (TIMER3_CFG_R & ~0x7) + 0x04;		// 4. Configuramos
	TIMER3_TAMR_R = (TIMER3_TAMR_R &~0xFFF) + 0x50A;	// 5. Modo PWM
	TIMER3_CTL_R |= TIMER_CTL_TAPWML;					// 6. PWM
	//SYSCTL_RCGC2_R|=0x2;								// Puerto B2 prendido
	//while((SYSCTL_PRGPIO_R&0X2)!=0x2);					// Espera al reloj prendido
	GPIO_PORTB_AFSEL_R|=(1<<2);							// Activamos funcion alternativa
	GPIO_PORTB_PCTL_R=(GPIO_PORTB_PCTL_R&0xFFFFF0FF)+0x00000700;// Funcion PWM
	GPIO_PORTB_DIR_R|=(1<<2);							// Esta como entrada
	GPIO_PORTB_DR8R_R|=(1<<2);							// Habilitamos driver de corriente 8mA
	GPIO_PORTB_DEN_R|=(1<<2);							// Activamos pin como digital

	TIMER3_CTL_R|=TIMER_CTL_TAEN;						// Activamos TIMER1 A

	//--------------------TIMER3 B------------------------//
	//SYSCTL_RCGC1_R |= (1<<17); 							// 1. Reloj Timer1
	//while((SYSCTL_PRTIMER_R&0x2)!=0x2);					// 2. Esperamos a real prendido
	TIMER3_CTL_R&=~TIMER_CTL_TBEN;							// 3. Desactivamos TIMER A
	TIMER3_CFG_R = (TIMER3_CFG_R & ~0x7) + 0x04;			// 4. Configuramos
	TIMER3_TBMR_R = (TIMER3_TBMR_R &~0xFFF) + 0x50A;		// 5. Modo PWM
	TIMER3_CTL_R |= TIMER_CTL_TBPWML;						// 6. PWM

	//SYSCTL_RCGC2_R|=0x2;									// Puerto B5 prendido
	//while((SYSCTL_PRGPIO_R&0X2)!=0x2);					// Espera al reloj prendido
	GPIO_PORTB_AFSEL_R|=(1<<3);								// Activamos funcion alternativa
	GPIO_PORTB_PCTL_R=(GPIO_PORTB_PCTL_R&0xFFFF0FFF)+0x00007000;// Funcion PWM
	GPIO_PORTB_DIR_R|=(1<<3);								// Esta como entrada
	GPIO_PORTB_DR8R_R|=(1<<3);								// Habilitamos driver de corriente 8mA
	GPIO_PORTB_DEN_R|=(1<<3);								// Activamos pin como digital

	TIMER3_CTL_R|=TIMER_CTL_TBEN;						// Activamos TIMER1 B
}
void DetenerPWM_DERECHO(void){
	/* Esta función detiene totalmente la onda PWM para motor DERECHO
	 * Es decir pone en 0 lógico tanto el b1|b0
	 * para el controlador haciendo que el driver entre
	 * en estado de reposo para los motores
	 */
	//Hacia el frente
	TIMER1_TAILR_R = 0;									// Valor de tope
	TIMER1_TAPR_R = 0;									// Del contador
	TIMER1_TAMATCHR_R = 0;								// Valor de comparacion
	TIMER1_TAPMR_R = 0;									// Del contador
	//Hacia atrás
	TIMER3_TAILR_R = 0;									// Valor de tope
	TIMER3_TAPR_R = 0;									// Del contador
	TIMER3_TAMATCHR_R = 0;								// Valor de comparacion
	TIMER3_TAPMR_R = 0;									// Del contador
}
void DetenerPWM_IZQUIERDO(void){
	/* Esta función detiene totalmente la onda PWM para motor IZQUIERDO
	 * Es decir pone en 0 lógico tanto el b1|b0
	 * para el controlador haciendo que el driver entre
	 * en estado de reposo para los motores
	 */
	//Hacia el frente
	TIMER1_TBILR_R = 0;									// Valor de tope
	TIMER1_TBPR_R = 0;									// Del contador
	TIMER1_TBMATCHR_R = 0;								// Valor de comparacion
	TIMER1_TBPMR_R = 0;									// Del contador
	//Hacia atrás
	TIMER3_TBILR_R = 0;									// Valor de tope
	TIMER3_TBPR_R = 0;									// Del contador
	TIMER3_TBMATCHR_R = 0;								// Valor de comparacion
	TIMER3_TBPMR_R = 0;									// Del contador
}
void OndaPWM_DERECHO_TIMER1(int frecuencia, int dutycycle){
	/* TIMER1
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor DERECHO pero lo hace segun el driver LN298
	 */
	TIMER1_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TAMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TAPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_IZQUIERDO_TIMER1(int frecuencia,int dutycycle){
	/* TIMER1
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor IZQUIERDO pero lo hace segun el driver LN298
	 */
	TIMER1_TBILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TBPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TBMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TBPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_DERECHO_TIMER3(int frecuencia,int dutycycle){
	/* TIMER3
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor DERECHO pero lo hace segun el driver LN298
	 */
	TIMER3_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER3_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER3_TAMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER3_TAPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_IZQUIERDO_TIMER3(int frecuencia,int dutycycle){
	/* TIMER3
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor IZQUIERDO pero lo hace segun el driver LN298
	 */
	TIMER3_TBILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER3_TBPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER3_TBMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER3_TBPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void GirarIzquierda(int frecuencia,int dutycycle,int TiempoDeGiro){
	DetenerTodo();										// Detenemos cualquier movimiento
	OndaPWM_DERECHO_TIMER1(frecuencia,dutycycle);		// Movemos Motor derecho
	OndaPWM_IZQUIERDO_TIMER3(frecuencia, dutycycle);	// Retrocedemos Motor Izquierdo
	retardo_ms(TiempoDeGiro);							// Cercano a 90°
	DetenerTodo();										// Detenemos cualquier movimiento
}
void GirarDerecha(int frecuencia,int dutycycle,int TiempoDeGiro){
	DetenerTodo();										// Detenemos cualquier movimiento
	OndaPWM_IZQUIERDO_TIMER1(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER3(frecuencia, dutycycle);		// Retrocedemos Motor Derecho
	retardo_ms(TiempoDeGiro);							// Cercano a 90°
	DetenerTodo();										// Detenemos cualquier movimiento
}
void Avanzar(int frecuencia,int dutycycle,int TiempoDeGiro){
	OndaPWM_IZQUIERDO_TIMER1(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER1(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	retardo_ms(TiempoDeGiro);							// Cercano a 90°
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
}
void Retroceder(int frecuencia,int dutycycle,int TiempoDeGiro){
	/* Esta función hacer que retroceda generando
	 * la onda al inverso, es decir que IN2 ahora es IN1
	 * y IN1 ahora es IN2
	 */
	DetenerTodo();										// Detenemos ambos motores
	OndaPWM_IZQUIERDO_TIMER3(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER3(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	DetenerTodo();										// Detenemos ambos motores
}
void DetenerTodo(void){
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
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
	/*
	 * Esta función convierte un número a caracter ascii guardados en un arreglo
	 */
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
void HacerLecturaLuzIx_BH1750(void)
{
	uint16_t luz=0;										// Esta variable almacena valor leido I2C
	int contador=0;										// Se usará para retardo
	luz=leer_I2C_BH1750(direccion_esclavo);				// Leemos Hight y Low Byte
	luz=luz/1.2;										// Calculamos la real medicion
	/////////////////////////////////////////////////////////////////////////
	NumerotoString(luz);								// Convierto luz a arreglo
	txmens_uart_HC05(lectura);							// TEXTO
	txcar_uart_HC05(contador+48);							// +48 o +0x30 por ser codigo ASCII
	txmens_uart_HC05(lectura3);							// TEXTO
	txmens_uart_HC05(Datos);								// Se envia datos con el arreglo
	txmens_uart_HC05(unidad_luz);							// TEXTO : Unidades de la medicion
	Datos[0]=0;											// Se Borra valores dentro del
	Datos[1]=0;											// Arreglo
	Datos[2]=0;
	Datos[3]=0;
	Datos[4]=0;
	/////////////////////////////////////////////////////////////////////////
	contador++;											// PARA TEXTO
	if (contador==10)contador=0;						// TEXTO : Contador
}
void main (void){

	ConfiguraUART_PC(); 								// COMUNICACION UART CON PC
	Configurar_TIMER1AYB_TIMER3AYB_PWM();				// CONFIGURA ONDA PWM PARA LOS MOTORES
	ConfiguraUART_PC();									// CONFIGURA LA COMUNICACION VIA USB-PC-UART0
	ConfiguraUART_HC05();								// CONFIGURA LA COMUNICACION CON EL BLUETOOTH
	I2C1_BH1750(); 										// CONFIGURA EL I2C PA6 CLOCK Y PA7 DATA
	uint32_t error=0;
	uint16_t TeclasPresionadas;							// TeclasPresionadas son una primera tecla
	uint8_t PrimeraTecla=0,SegundaTecla=0;			// Teclas que almacenarán las letras
	// presionada junto con la segunda tecla
	error=escribir_I2C_BH1750(direccion_esclavo, power_on);				// Prendemos el sistema
	error=escribir_I2C_BH1750(direccion_esclavo, MeasurementCode);		// Ponemos el codigo para leer continuamente

	while(1)
	{
		PrimeraTecla=rxcar_uart_HC05();					// Primera tecla que recibe
		SegundaTecla=rxcar_uart_HC05();					// Segunda tecla que recibe
		TeclasPresionadas=(PrimeraTecla<<8)+SegundaTecla;	// Recibimos Segunda Tecla
		HacerLecturaLuzIx_BH1750();							// Realizamos lectura del sensor de Luz
		switch (TeclasPresionadas){
		case (0x7777):	// ww
		case (0x5757):	// Avanzar WW or ww
			Avanzar(frecuencia,dutycycle,TiempoDeGiro);		// Avanzar el robot
			break;
		case (0x5353):	// SS
		case (0x7373):	// Retroceder ss
			Retroceder(frecuencia,dutycycle,TiempoDeGiro);	// Retroceder el robot
			break;
		case 0x4141:// Girar Izquierda AA
		case 0x6161:
			GirarIzquierda(frecuencia,dutycycle,TiempoDeGiro);	// Girar hacia la izquierda
			break;
		case 0x4444:
		case 0x6464:// Girar Derecha dd
			GirarDerecha(frecuencia,dutycycle,TiempoDeGiro);	// Girar hacia la derecha
			break;
		case 'AW':
			break;
		default:
			DetenerTodo();									// Caso contrario detener todo
			break;
		}
	}

}/********************   FIN  MAIN     **********************************/
