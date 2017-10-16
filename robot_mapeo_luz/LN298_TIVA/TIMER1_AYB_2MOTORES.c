/***************************************************************************/
/*************** Proyecto: CONTROLADOR DE DRIVER LN298   *******************/
/***************************************************************************/
/*************** Microcontrolador: TM4C123GH6PM ****************************/
/*************** Tiva C Series TM4C123GH6PM LaunchPad **********************/
/*************** Autor: Pablo Díaz ** **************************************/
/*************** Fecha: Octubre 2017 ***************************************/
/*************** Enunciado:Controlar 2 motores con driver*******************/
/***************************************************************************/
#include <stdint.h>
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
	/*
	TIMER0_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF; //Valor tope de contador
	TIMER0_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER0_TAMATCHR_R = ((16000*dutycycle)/frecuencia) & 0xFFFF; //Valor de comparación
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
}
void DetenerPWM_DERECHO(void){
	TIMER1_TAILR_R = 0;									// Valor de tope
	TIMER1_TAPR_R = 0;									// Del contador
	TIMER1_TAMATCHR_R = 0;								// Valor de comparacion
	TIMER1_TAPMR_R = 0;									// Del contador
}
void DetenerPWM_IZQUIERDO(void){
	TIMER1_TBILR_R = 0;									// Valor de tope
	TIMER1_TBPR_R = 0;									// Del contador
	TIMER1_TBMATCHR_R = 0;								// Valor de comparacion
	TIMER1_TBPMR_R = 0;									// Del contador
}
void OndaPWM_DERECHO(frecuencia, dutycycle){
	TIMER1_TAILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TAPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TAMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TAPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void OndaPWM_IZQUIERDO(frecuencia, dutycycle){
	TIMER1_TBILR_R = (16000000/frecuencia - 1) & 0xFFFF;		//Valor tope de contador
	TIMER1_TBPR_R = ((16000000/frecuencia - 1) >> 16);
	TIMER1_TBMATCHR_R = (160000/frecuencia)*dutycycle & 0xFFFF;	//Valor de comparación
	TIMER1_TBPMR_R = (((160000/frecuencia)*dutycycle) >>16);
}
void retardo_ms (uint32_t milisegundos) {
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; 										//Deshabilitamos Systick
	NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R & ~0xFFFFFF) + (16000*milisegundos - 1); 	//Programamos valor de recarga
	NVIC_ST_CURRENT_R = (NVIC_ST_CURRENT_R & ~0xFFFFFF) + 1; 						//Borramos la cuenta
	NVIC_ST_CTRL_R |= (NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC); 				//Habilitamos Systick
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT)==0);
}
void GirarDerecha(void){
	retardo_ms(10);
	
}
int main(void) {
	Configurar_TIMER1AYB_PWM();							// Configuramos el PWM TIMER1 A Y B
	retardo_ms(100);
	int dutycycle=70;									// DutyCycle
	while(1){
		OndaPWM_DERECHO(10,dutycycle);
		retardo_ms(400);
		DetenerPWM_DERECHO();
		retardo_ms(1400);
		OndaPWM_IZQUIERDO(10,dutycycle);
		retardo_ms(400);
		DetenerPWM_IZQUIERDO();
		retardo_ms(1400);

	}
}
