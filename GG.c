#include "tm4c123gh6pm.h"
#include <stdint.h>

void timer_pwm (void){
    unsigned long temp;

    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0;
    while((SYSCTL_PRTIMER_R & 0x01)==0);
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;
    temp = SYSCTL_RCGC2_R;

    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER0_CFG_R = (TIMER0_CFG_R &~0x07)+0x04;
    TIMER0_TAMR_R = (TIMER0_TAMR_R &~0xFFF)+0x50A;
    TIMER0_CTL_R |= TIMER_CTL_TAPWML;
    TIMER0_TAILR_R = ( 159999 & 0xFFFF);
    TIMER0_TAPR_R = (TIMER0_TAPR_R &~0xFF) + 159999>>16;
    TIMER0_TAMATCHR_R = 0 & 0xFFFF;
    TIMER0_TAPMR_R = (TIMER0_TAPMR_R &~0xFF) + 0>>16;

    GPIO_PORTB_DIR_R |= 0x40;
    GPIO_PORTB_AFSEL_R |= 0x40;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R &~0x0F000000)+0x07000000;
    GPIO_PORTB_PUR_R &=~0x40;
    GPIO_PORTB_PDR_R &=~0x40;
    GPIO_PORTB_DR8R_R |= 0x40;
    GPIO_PORTB_AMSEL_R &=~0x40;
    GPIO_PORTB_DEN_R |= 0x40;

    TIMER0_CTL_R |= TIMER_CTL_TAEN;
}

void modifica_dc (unsigned char dc){
    TIMER0_TAMATCHR_R = dc*1600 & 0xFFFF;
    TIMER0_TAPMR_R = (TIMER0_TAPMR_R &~0xFF) + dc*1600>>16;
}
void configuart(void){
 unsigned long temp1;
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;
    temp1 = SYSCTL_RCGC1_R;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;
    temp1 = SYSCTL_RCGC2_R;

    UART0_CTL_R &= ~UART_CTL_UARTEN;
    UART0_IBRD_R = (UART0_IBRD_R &~0xFFFF)+104;
    UART0_FBRD_R = (UART0_FBRD_R &~0x3F)+11;
    UART0_LCRH_R |= 0x70;
    UART0_CTL_R |= UART_CTL_UARTEN;

   GPIO_PORTA_AFSEL_R |= 0x03;
   GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R &~0x000000FF)+0x00000011;
   GPIO_PORTA_PUR_R &=~0x03;
   GPIO_PORTA_PDR_R &=~0x03;
   GPIO_PORTA_AMSEL_R &=~0x03;
   GPIO_PORTA_DEN_R |= 0x03;
}
void txcar (char c){
    while ((UART0_FR_R & UART_FR_TXFF)!=0);
    UART2_DR_R = c;
}
char rxcar (void){
    char car;
    while((UART0_FR_R & UART_FR_RXFE)!=0);
    car = UART0_DR_R & 0xFF;
    return car;
}
void txmns (char mns[]){
    uint8_t i=0;
    char letratx;
        letratx = mns[i++];
        while(letratx != '\0'){
             txcar(letratx);
             letratx = mns[i++];
        }
}
void rxmns (char cadena[]){
    uint8_t j=0;
    char letrarx;
    while(letrarx != '\r'){
        letrarx = rxcar();
        cadena[j]=letrarx;
        j++;
    }
}
char ascii_bin (char arr[]){
    char a, b ,s;
    a = (arr[0]-48)*10;
    b = (arr[1]-48);
    s = a+b;
    return s;
}
void main (void){
    char mInicio[]="Bienvenido a la interfaz del ventilador\r\n";
    char mIngresodc[]="Ingrese porcentaje deseado\r\n";
    char mEspera[]="Procesando datos....\r\n";
    char mError[]="Valor erroneo, intentelo de nuevo\r\n";
    char ct[2];
    char duty_cycle;

    configuart();
    txmns(mInicio);
    txmns(mIngresodc);
    rxmns(ct);
    txmns(ct);
    duty_cycle = ascii_bin(ct);
    if ((duty_cycle < 10)||(duty_cycle > 100)){
    txmns(mEspera);
    modifica_dc(duty_cycle);
    }
    else{
    txmns(mError);
    }

}
