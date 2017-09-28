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


;DECLARACION DE LAS VARIABLES


.text ; El programa debe estar en la sección de código
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

	;Ahora configuramos el puerto A


	ldr r3,GPIO_PORTA_BASE;guardamos la direccion
	ldr r0,[r3,#DIR]
	bic r0,#0x3C
	str r0,[r3,#DIR]

	ldr r0,[r3,#DR8R]
	bic r0,#0x3C
	str r0,[r3,#DR8R]

	ldr r0,[r3,#PUR]
	orr r0,r0,#0x3C
	str r0,[r3,#PUR]

	ldr r0,[r3,#DEN]
	orr r0,r0,#0x3C
	str r0,[r3,#DEN]

	;Ahora configuramos el puerto B

	ldr r3,GPIO_PORTB_BASE;guardamos la direccion

	ldr r0,[r3,#DIR]
	orr r0,#0xF0
	str r0,[r3,#DIR]

	ldr r0,[r3,#DR8R]
	orr r0,#0xF0
	str r0,[r3,#DR8R]

	ldr r0,[r3,#DEN]
	orr r0,#0xF0
	str r0,[r3,#DEN]
					;SE TERMINO DE CONFIGURAR TODOS LOS PEURTOS A, B Y F

asignar_valores:
	ldr r1,VARA
	ldr r2,VARB
	ldr r3,VARC
	ldr r4,VARD
						;r0 tendra la suma
	add r0,r1
	add r0,r0,r2
	add r0,r0,r3
	add r0,r0,r4
						;r5 tendra la multiplicacion
	mul r5,r1,r2
	mul r5,r5,r3
	mul r5,r5,r4

pregunta1:
	cmp r5,#99
	bls pregunta2
	b ledazulindef
pregunta2:
	cmp r0,#100
	bls sicumple
	b ledazulindef


sicumple:
	and r5,#0xF
	ldr r3,GPIO_PORTB_BASE
prender:
	str r5,[r3,#DATA]
;retardo
retardo_led_1:
	mov r2,#10000			;GENERAMOS UN RETARDO
retardo_led_2:
	sub r2,r2,#0x1
	cmp r2,#0x0
	bne retardo_led_2
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_1
apagar:
	mov r1,#0
	str r1,[r3,#DATA]
retardo_led_11:
	mov r2,#10000			;GENERAMOS UN RETARDO
retardo_led_22:
	sub r2,r2,#0x1
	cmp r2,#0x0
	bne retardo_led_22
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_11


prender:
	and r0,r0,#0xF
	str r0,[r3,#DATA]

retardo_led_111:
	mov r2,#10000;GENERAMOS UN RETARDO
retardo_led_222:
	sub r2,r2,#0x1
	cmp r2,#0x0
	bne retardo_led_222
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_111

apagar:
	and r1,r1,#0
	str r1,[r3,#DATA]
;retardo

ledazulindef:
	ldr r3,GPIO_PORTF_BASE
	ldr r0,[r3,#DATA]
	eor r0,#0x04
	str r0,[r3,#DATA]
	b ledazulindef



GPIO_PORTE_BASE: .word 0x40024000
GPIO_PORTF_BASE: .word 0x40025000
GPIO_PORTA_BASE: .word 0x40004000
GPIO_PORTB_BASE: .word 0x40005000
VARA: .word 0x1
VARB: .word 0x2
VARC: .word 0x3
VARD: .word 0x4
SYSCTL_RCGC2: .word 0x400FE108

KEYLOCK: .word 0x4c4f434b
	.end
