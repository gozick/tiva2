#include <stdint.h>
#include "tm4c123gh6pm.h"
void config_pulsadores_led(void){ //Configuramos puerto F
    unsigned long temp;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // Se habilita la señal de reloj del Puerto F
    temp = SYSCTL_RCGC2_R; // Espera a que se estabilice el reloj
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //Quitamos proteccion
    // Habilitar el bit0 (PF0 - pin protegido)
    GPIO_PORTF_CR_R |= 0X1; // Se habilita la modificación de varios registros del PF0
    GPIO_PORTF_LOCK_R = 0; // Se activa la proteccion
    GPIO_PORTF_AMSEL_R &= ~(0X1F); // Se deshabilitan las funciones analóg. de los bits 0 al 4
    GPIO_PORTF_PCTL_R &= ~(0X000FFFFF); // Se asegura el uso GPIO
    GPIO_PORTF_DIR_R &= ~(0X11); //Se configura las entradas
    GPIO_PORTF_DIR_R |= 0X0E;//Se configura las salidas
    GPIO_PORTF_AFSEL_R &= ~(0X1F); // Se configuran los bits 0 al 4 como GPIO
    GPIO_PORTF_PUR_R |= 0X11; // Se activan la resist. pull-up de las entradas bits 0 y 4
    GPIO_PORTF_DR8R_R |= 0X0E; //Se activa salida de 8mA
    GPIO_PORTF_DEN_R |= 0X1F; // Se habilitan las salidas digitales de los bits 0 al 4
}
void main(void) {
    config_pulsadores_led();
    uint32_t contador=0;
    uint32_t n=0;
    
    while (1)
    {
        while (contador>100000)
        {
            if (GPIO_PORTF_DATA_R&0x10==0x10)//cuando presione
            {
                while(GPIO_PORTF_DATA_R&0x10!=0x10);//Mientras este sin soltar
                n++; // aumentar contador de pulsos
                contador=0;
            }
            
            if (GPIO_PORTF_DATA_R&0x01==0x01) //Sw2 presionado sin soltar
            {
                while (GPIO_PORTF_DATA_R&0x01!=0x01); // mientras SW2 este sin soltar
                n++;
                contador=0;
            }
            
            contador++; // aumentar ciclo de chequeo
        }//Fin de espera de pulsacion de algun switch 1 o 2
        contador=0;
        //Ahora vamos a mostrar los pulsos de leds
        for (int i=0;i=n;i++) // Este for va a prender N veces amarillo
        {
        for (int j=0;j<400000;j++); // espera de tiempo similar a 1 seg
        GPIO_PORTF_DATA_R=(amarillo);
        for (int j=0;j<400000;j++); // espera de tiempo similar a 1 seg
        GPIO_PORTF_DATA_R&=~0xFF; // apagar todos los leds
        }
        

              
    }//fin while 1

    
} // fin main()
