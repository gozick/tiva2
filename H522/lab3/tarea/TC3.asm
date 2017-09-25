; Nombre del programa: led_on_off.asm
; Semestre: 2015.1./actualizado en 2017.1
; Descripción: Se lograr hacer parpadear un led
; Módulo: LaunchPad TIVA
; Autor: Ing. Freri Orihuela
; Conexiones: Led AZUL en pin PF2
;********************************************************************************
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
ELEMENTOS .equ 4


arreglo_espacio:
	.space 4; solo necesitamos 4 variables
	.text ; El programa debe estar en la sección de código
	.thumb ; Se programa con instrucciones THUMB y se
	; usa sintaxis UAL
	.global main ; Dirección de inicio la etiqueta main
main:
	ldr r2, SYSCTL_RCGC2;Prendemos señal del reloj
	ldr r3, GPIO_PORTF_BASE; guardamos la base en puerto F
	ldr r1, [r2]; leo el dato
	orr r0, r1, #0x00000020;activo los puertos que deseo
	str r0, [r2] ; Señal de reloj para GPIOF
	nop;espero a real activacion
	nop
	nop
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


	ldr r0,[r3,#AFSEL]
	and r0,r0,#0xE0
	str r0,[r3,#AFSEL]

	ldr r0, [r3, #PUR]
	orr r0, r0, #0x00000011 ; activa resistencias de pull-up
	str r0, [r3, #PUR]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
establecer_arreglo:
	ldr r7,arreglo; llamamos la palabra arreglo
	mov r6, #1
	str r6,[r7]
	add r7,#1
	mov r6, #3
	str r6,[r7]
	add r7,#1
	mov r6, #5
	str r6,[r7]
	add r7,#1
	mov r6, #7
	str r6,[r7]
	add r7,#1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
sw2_presionado:
	ldr r1,[r3,#DATA]
	and r1,r1, #0x01;PF4
	cbz r1, sw2_soltado; si no es cero enviar a sw1 presionado
	b sw2_presionado;bucle
sw2_soltado:
	ldr r1,[r3,#DATA]
	and r1,r1, #0x01;PF4
	cbNz r1, prender_led_azul; si no es cero enviar a sw1 presionado
	b sw2_soltado;bucle

prender_led_azul:
	; Conmutar LED Azul (PF2)
	ldr r1, [r3, #DATA] ; r1 <-- [dirección base + offset(DATA)]
	mov r1,#0x4;
	str r1, [r3, #DATA]
	mov r1,#100
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
retardo_led_1:
	mov r2,#10000;10000 por cada 1 de r1
retardo_led_2:
	sub r2,r2,#0x1
	;cbz r2,retardo_led_2
	cmp r2,#0x0
	bne retardo_led_2
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_1
	;cbz r1,sw1_presionado
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
sw1_presionado:
	ldr r4,[r3,#DATA]
	and r4, #0x10;PF4
	cbz r4, sw1_soltado; si no es cero enviar a sw1 presionado
	b sw1_presionado;bucle
sw1_soltado:
	ldr r4,[r3,#DATA]
	and r4, #0x10;PF4
	cbnz r4, j_arreglo; si no es cero enviar a sw1 presionado
	b sw1_soltado;bucle
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
j_arreglo:
	ldr r6,[r7]; j=Arreglo[i]
	add r7,#1;siguiente registro de arreglo
j_diferente_cero:
	cbnz r6, prender_led_rojo
	cbz r6, reiniciar_i
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
prender_led_rojo:
	ldr r4,[r3,#DATA]
	mov r4,#0x0E
	str r4,[r3,#DATA]
	mov r1,#100
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
retardo_led_11:
	mov r2,#10000;10000 por cada 1 de r1
retardo_led_22:
	sub r2,r2,#0x1
	;cbz r2,retardo_led_2
	cmp r2,#0x0
	bne retardo_led_22
	sub r1,r1,#1
	cmp r1,#0
	bne retardo_led_11
	mov r4, #0x0
	str r4,[r3,#DATA]
	;cbz r1,sw1_presionado
	sub r6,#1
	b j_arreglo
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
reiniciar_i:
	mov r7,#0x0
	b sw1_presionado
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
fin: b fin;
; Registros base
GPIO_PORTE_BASE: .word 0x40024000
GPIO_PORTF_BASE: .word 0x40025000
SYSCTL_RCGC2: .word 0x400FE108
KEYLOCK: .word 0x4c4f434b
arreglo: .word arreglo_espacio
	.end
