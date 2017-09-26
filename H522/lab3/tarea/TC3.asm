;*****************************************************
; Nombre del programa: DiazV_TC3.asm
; Semestre: 2017.2
; Descripción: Adjunta en el informe
; Módulo: Tiva LaunchPad TM4C123GH6PM
; Autor: Pablo Díaz
; Conexiones: SW2 SW1 LEDS RGB
;*****************************************************
; Desplazamientos (Offset) de cada registro
DATA .equ 0x000003FC
DIR .equ 0x00000400
AFSEL .equ 0x00000420
DR8R .equ 0x00000508
PUR .equ 0x00000510
DEN .equ 0x0000051C
LOCK .equ 0x00000520
CR .equ 0x00000524
AMSEL .equ 0x40025528
; Numero de elementos del arreglo
ELEMENTOS .equ 4
.text ; El programa debe estar en la sección de código
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
arreglo_espacio:	.bss ARREGLOS,40
; Asignamos espacio para las variables
arreglos .word ARREGLOS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	.thumb ; Se programa con instrucciones THUMB y se
	; usa sintaxis UAL
	.global main ; Dirección de inicio la etiqueta main
main:

	ldr r2, SYSCTL_RCGC2;Prendemos señal del reloj
	ldr r3, GPIO_PORTF_BASE; guardamos la base en puerto F
	ldr r1, [r2]; leo el dato
	orr r0, r1, #0x00000020;activo los puertos que deseo
	str r0, [r2] ; Señal de reloj para GPIOF
	nop;espero
	nop;espero
	nop;espero
	ldr r0, KEYLOCK
	str r0, [r3, #LOCK] ; se escribe la llave para desbloquear PD7
	ldr r0, [r3, #CR]
	orr r0, r0, #0x00000001 ; desbloquea pf0, habilita como GPIO
	str r0, [r3, #CR]
	mov r0,#0;bloquear denuevo
	str r0, [r3, #LOCK] ;bloqueamos el lock
	mov r0, #0x000000E; configuración GPIOF
	str r0,[r3,#DIR]
	str r0, [r3, #DR8R] ; leemos dato de drenador 8mA
	mov r0, #0x0000001F;activamos los puertos 1,2 y 3
	str r0, [r3, #DEN];guardamos los datos
	ldr r0,[r3,#DIR]
	ldr r0,[r3,#AFSEL];Activamos el AFSEL
	and r0,r0,#0xE0
	str r0,[r3,#AFSEL]
	ldr r0, [r3, #PUR]
	orr r0, r0, #0x011 ; activa resistencias de pull-up
	str r0, [r3, #PUR]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
guardar_arreglos:
	ldr r7,arreglos ;GUARDAMOS LOS VALORES DEL ARREGLO
	mov r6,#1			;#1
	str r6,[r7]
	mov r6,#3			;#3
	str r6,[r7,#0x1]
	mov r6,#5			;#5
	str r6,[r7,#0x2]
	mov r6,#7			;#7
	str r6,[r7,#0x3]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
sw2_presionado:
	ldr r1,[r3,#DATA]
	and r1, #0x01		;ESTADO SW2
	cbz r1, sw2_soltado ;MIENTRAS NO SE PRESIONE SW2
	b sw2_presionado;bucle
sw2_soltado:
	ldr r1,[r3,#DATA]
	and r1,r1, #0x01;PF4
	cbnz r1, prender_led_azul;MIENTRAS NO SE SUELTE SW2
	b sw2_soltado;bucle
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
prender_led_azul:
	ldr r1, [r3, #DATA]
	mov r1,#0x4			;PRENDEMOS LED AZUL
	str r1, [r3, #DATA]
	mov r1,#100
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
retardo_led_1:
	mov r2,#10000			;GENERAMOS UN RETARDO
retardo_led_2:
	sub r2,r2,#0x1
	cmp r2,#0x0
	bne retardo_led_2
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_1
	ldr r7,arreglos; LEEMOS EL PUNTERO DEL ARREGLO
	mov r0,#4		; DECIMOS EL NUMERO DE ELEMENTOS DEL ARREGLO
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
sw1_presionado:
	ldr r4,[r3,#DATA]
	and r4, #0x10		;VEMOS EL ESTADO DEL SW1
	cbz r4, sw1_soltado ;SI NO ES PRESIONADO
	b sw1_presionado;bucle
sw1_soltado:

	ldr r4,[r3,#DATA]
	and r4, #0x10;PF4
	cbnz r4, j_arreglo  ;SI NO ES SOLTADO
	b sw1_soltado
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
j_arreglo:
	ldr r5,[r7] 		;LEE EL ESTADO DEL ARREGLO
j_pregunta:
	and r5,#0XFF		;SOLO TOMAMOS 1 BYTE
	cmp r5,#0				;R5 SIRVE PARA LOS PULSOS
	bne prender_led_rojo	;SI TODAVIA HAY PULSOS POR HACER
	beq siguiente_i			;SIGUIENTE NUMERO EN EL ARREGLO
	nop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
prender_led_rojo:			;PRENDEMOS
	ldr r4,[r3,#DATA]		;LED ROJO Y APAGAMOS
	mov r4,#0x02			;LOS OTROS
	str r4,[r3,#DATA]		;LEDS
	mov r1,#100
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
retardo_led_11:				;GENERAMOS
	mov r2,#10000			;RETARDOS
retardo_led_22:
	sub r2,r2,#0x1
	cmp r2,#0x0
	bne retardo_led_22
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_11
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
apagar_led_rojo:
	mov r4, #0x0			;APAGAMOS
	str r4,[r3,#DATA]		;TODOS LOS LEDS
	mov r1,#100
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
retardo_led_111:
	mov r2,#10000			;RETARDO
retardo_led_222:			;ANTES DE APAGAR
	sub r2,r2,#0x1
	cmp r2,#0x0
	bne retardo_led_222
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_111
	subs r5,#1			;RESTAMOS R5 Y VEMOS SI
	b j_pregunta		;YA SE TERMINO LOS PULSOS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
siguiente_i:			;AUMENTAMOS EL VALOR
	add r7,#1			;DEL PUNTERO DEL ARREGLO
	subs r0,#1			;RESTAMOS LA CANTIDAD DE
	cmp r0,#0			;DATOS QUE HAY EN EL
	beq reiniciar_i		;ARREGLO
	bne sw1_presionado
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
reiniciar_i:
	mov r0, #4			;REINICIAMOS
	ldr r7,arreglos		;EL NUMERO DE DATOS DEL ARREGLO
	b sw1_presionado	;VOLVEMOS A VER ESTADO SW1
; Registros base
GPIO_PORTE_BASE: .word 0x40024000
GPIO_PORTF_BASE: .word 0x40025000
SYSCTL_RCGC2: .word 0x400FE108
KEYLOCK: .word 0x4c4f434b
	.end
