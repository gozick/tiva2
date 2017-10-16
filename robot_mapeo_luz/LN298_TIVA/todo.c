/***************************************************************************/
/*************** Proyecto: CONTROLADOR DE DRIVER LN298   *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz ** **************************************/
/*************** Fecha: Octubre 2017 ***************************************/
/*************** Enunciado:Controlar 2 motores con driver*******************/
/***************************************************************************/
/* Descripción:
 * El TIMER1 controla el avance hacia el frente, el TIMER3 controla el retroceso
 * Estas ondas PWM interactúan con el driver LN298 que controla a los motores
 * mediante la configuracion de sus pines IN1|IN2|IN3|IN4
 * La fuente del driver esta conectado a +12V
 * IN1|IN2 ------ TIMER1A|TIMER3A
 * IN3|IN4 ------ TIMER1B|TIMER3B
 */
#include <stdint.h>
#include "tm4c123gh6pm.h"
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
void OndaPWM_DERECHO_TIMER1(frecuencia, dutycycle){
	/* TIMER1
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor DERECHO pero lo hace segun el driver LN298
	 */
	TIMER1_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TAMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TAPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_IZQUIERDO_TIMER1(frecuencia, dutycycle){
	/* TIMER1
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor IZQUIERDO pero lo hace segun el driver LN298
	 */
	TIMER1_TBILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TBPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TBMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TBPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_DERECHO_TIMER3(frecuencia, dutycycle){
	/* TIMER3
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor DERECHO pero lo hace segun el driver LN298
	 */
	TIMER3_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER3_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER3_TAMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER3_TAPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_IZQUIERDO_TIMER3(frecuencia, dutycycle){
	/* TIMER3
	 * Esta función genera una onda PWM para prender el pin que enciende
	 * el motor IZQUIERDO pero lo hace segun el driver LN298
	 */
	TIMER3_TBILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER3_TBPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER3_TBMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER3_TBPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void GirarDerecha(frecuencia,dutycycle,TiempoDeGiro){
	retardo_ms(10);										// Retardo
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER1(frecuencia,dutycycle);		// Movemos Motor derecho
	OndaPWM_IZQUIERDO_TIMER3(frecuencia, dutycycle);	// Retrocedemos Motor Izquierdo
	retardo_ms(TiempoDeGiro);							// Cercano a 90°
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
}
void GirarIzquierda(frecuencia,dutycycle,TiempoDeGiro){
	retardo_ms(10);										// Retardo
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
	OndaPWM_IZQUIERDO_TIMER1(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER3(frecuencia, dutycycle);		// Retrocedemos Motor Derecho
	retardo_ms(TiempoDeGiro);							// Cercano a 90°
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
}
void Avanzar(frecuencia,dutycycle,TiempoDeGiro){
	retardo_ms(10);										// Retardo
	OndaPWM_IZQUIERDO_TIMER1(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER1(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	retardo_ms(TiempoDeGiro);							// Cercano a 90°
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
}
void Retroceder(frecuencia,dutycycle,TiempoDeGiro){
	/* Esta función hacer que retroceda generando
	 * la onda al inverso, es decir que IN2 ahora es IN1
	 * y IN1 ahora es IN2
	 */
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
	retardo_ms(10);										// Retardo
	OndaPWM_IZQUIERDO_TIMER3(frecuencia,dutycycle);		// Movemos Motor Izquierdo
	OndaPWM_DERECHO_TIMER3(frecuencia,dutycycle);		// Movemos Motor Izquierdo
}
void DetenerTodo(void){
	DetenerPWM_IZQUIERDO();								// Detenemos Motor Izquierdo
	DetenerPWM_DERECHO();								// Detenemos Motor Derecho
}
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
int main(void) {
	Configurar_TIMER1AYB_TIMER3AYB_PWM();				// Configuramos el PWM TIMER1 A Y B, TIMER3 A Y B
	retardo_ms(100);									// Retardo
	int dutycycle=70;									// DutyCycle
	int frecuencia=10;									// Frecuencia
	int TiempoDeGiro=500;								// Tiempo usado para girar 90° en ms
	while(1){
		GirarDerecha(frecuencia,dutycycle,TiempoDeGiro);
		retardo_ms(1000);
		GirarIzquierda(frecuencia,dutycycle,TiempoDeGiro);
		retardo_ms(1000);
		Avanzar(frecuencia,dutycycle,TiempoDeGiro);
		retardo_ms(1000);
		DetenerTodo();
		retardo_ms(400);
		Retroceder(frecuencia,dutycycle,TiempoDeGiro);
		retardo_ms(1000);
	}
}
