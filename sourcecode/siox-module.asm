/*
 * siox-module.asm
 *
 *  Created: 2016/08/04
 *   Author: lynf
 */ 
;
;
;######################################################################################
; This software is Copyright by Francis Lyn and is issued under the following license:
;
; Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License
;
;######################################################################################
;
;
;
; ATmega328P cpu, 16.0000 MHz external crystal.
; L: FF, H: DA, E: 05
;
; Interrupt driven serial UART receiver handler with 32 byte receiver buffer
; Include this file with main program code. Make sure the equates, interrupt
; vectors, RAM buffers are defined in the main program code.
;
;
; =====
;.list
; =====
;
;
; Constant equates (definitions)
;
; UART definitions
;
;.equ	BAUD = 19200		; Baud rate
;.equ	BAUD_PRE = 51		; Baud rate prescaler - 16.00 MHz clock
;
; ---
;
.equ	sblen = 0x20		; Serial input buffer length
;
;
;
; ============================================
;       S R A M   D E F I N I T I O N S
; ============================================
;
;.DSEG
;.ORG	0X0100
;
; RTC clock module data buffers
;
;uart_st:
;
;sinb:						; Serial input buffer
;.byte		sblen
;
;getcnt:
;.byte		1				; Input buffer getbyte counter
;
;putcnt:
;.byte		1				; Input buffer putbyte counter
;
;
;uart_end:
;
;
; ============================================
;   R E S E T   A N D   I N T   V E C T O R S
; ============================================
;
;.CSEG
;.ORG	$0000
;		rjmp		start		; Int vector 1 - Reset vector
;
;.ORG	URXCaddr
;		jmp			URXCint		;  USART Rx Complete
;
;
; End of interrupt vectors, start of program code space
;
;.ORG	0x0034
;
;
;
;
;
; Zero RAM data buffers
;
;zbuf:
;		ldi		rgb,(uart_end-uart_st)
;		clr		rmp
;		ldxptr	uart_st
;zbuf1:
;		st		X+,rmp
;		dec		rgb
;		brne	zbuf1
;		ret
;
;
;
;		rcall	inzuart			; Disable UART when using PD0 and PD1 pins

;
;
;###########################################################################
;
; UART serial receive interrupt handler.
;
; USART Receive Complete interrupt handler. Interrupt invoked by RXC0 set
; when UDRE buffer has received character ready. Routine puts the received
; character into the SINB serial input buffer and increments the PUTCNT counter.
; When SBLEN+1 counts reached, PUTCNT is rolled back to 0 counts. SINB acts
; as a circular buffer.
;
URXCint:
		in		SRsav,SREG		; Save SREG
		push	rga
		push	rmp
		push	XH
		push	XL
;
		ldxptr	sinb			; Sinb base address
		lds		rmp,putcnt		; Get current putcnt offset
		clr		rga
		add		XL,rmp			; Add putcnt offset to sinb pointer
		adc		XH,rga			; Update pointer (16 bit addition)
		lds		rga,UDR0		; rga <-- UDR0
		st		X,rga			; Store to SINB buffer
		inc		rmp				; Increment putcnt
		cpi		rmp,(sblen+1)	; Past end of buffer?
		brcs	URXCint1		;	No, continue
		clr		rmp				;	Yes, reset putcnt
URXCint1:
		sts		putcnt,rmp		; Update putcnt to next free location
;
		pop		XL
		pop		XH
		pop		rmp
		pop		rga
		out		SREG,SRsav			; Restore SREG
		reti

;
;
;
;
;###########################################################################
;
;
; ============================================
;	U A R T Serial I/O Moulde
; ============================================
;
;
; Initialize the UART for 19200 baud asynchronous operation
;
inzuart:
		cli							; Clear global interrupts
		ldi		rmp,high(BAUD_PRE)
		sts		UBRR0H,rmp			; Load baud rate register high
		ldi		rmp,low(BAUD_PRE)
		sts		UBRR0L,rmp			; Load baud rate register low
;
; Setup frame for 1 start, 8 data, 1 stop and no parity
;
		ldi		rmp,(1<<UCSZ00)|(1<<UCSZ01)
		sts		UCSR0C,rmp
;
; Enable the UART RX, TX, and RXC interrupt
;
		ldi		rmp,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0) ; |(1<<TXCIE0)|(1<<UDRIE0)
		sts		UCSR0B,rmp			; Enable RX, TX and RXC interrupt
;
		sei							; Set global interrupts
		ret

;
; Scan for console character and return with character if any,
; else return with rga = 0. Data is available in sinb when putcnt
; is greater than getcnt.
;
getc:
		lds		rga,getcnt
		lds		rmp,putcnt
		cp		rga,rmp			; Compare getcnt to putcnt
		breq	getc2			; Same, no new data
getc0:
		push	XH
		push	XL				; Save X registers
		ldxptr	sinb			; sinb base address
		lds		rmp,getcnt		; Get current getcnt offset
		clr		rga
		add		XL,rmp			; Add putcnt offset to sinb pointer
		adc		XH,rga			; Update pointer (16 bit addition)
;		
		ld		rga,X			; rga <-- @XHL
		pop		XL
		pop		XH				; Restore X registers
		inc		rmp				; Increment getcnt
		cpi		rmp,(sblen+1)	; Past end of buffer?
		brcs	getc1			;	No, continue
		clr		rmp				;	Yes, reset getcnt
getc1:
		sts		getcnt,rmp		; Update getcnt to next free location
		ret
getc2:
		clr		rga
		ret
;
; Wait for a received data byte, return received data in rga.
;
ci:	
		lds		rgc,getcnt
		lds		rgd,putcnt
		cp		rgc,rgd			; Compare getcnt to putcnt
		breq	ci				; Wait for new received data
		rjmp	getc0
;
; Load UDR0 from rga. Wait until transmitter is empty before loading.		(OK)
;
co:	
		lds		rmp,UCSR0A		; Get UART control status register
		sbrs	rmp,UDRE0		; Test if UDR0 is empty
		rjmp	co
;
; Send data
;
		sts		UDR0,rga		; UDR0 <-- rga
		ret
;
;
;###########################################################################
;
;
;
; End of source code
;
; ======================================================================
.exit
; ======================================================================
;





