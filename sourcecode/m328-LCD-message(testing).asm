/*
 * m328_LCD_message.asm
 *
 *  Created: 10/7/2015 9:28:39 AM
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
; All Versions of m328_LCD_message.asm runs at 9600 baud.
;
; Copyright F.Lyn 2016.07.26, all rights reserved
;
; NOTE: The 3.3 V version of the Pro-mini controller board uses an 8.000 MHz
;		clock compared to the 5 V version, so the parameters affected by the
;		Fosc have to be changed accordingly. This is done by setting the equate
;		Vcc_low = true for 3.3 V, else Vcc_low = false for 5 V devices.
;
; This project is for a stand-alone LCD messaging centre with communications
; to a uTile host controller via an nRF24L01+ ISM radio link. The host controller
; sends commands to invoke stored messages on the 2x16 line, a JHD 162A module
; with a TWI backpack interface. The LCD messaging centre, or annunciator, uses
; an ATmega328P controller to store up to 8 (or more) lines of text messages, 16
; characters/line. Stored messages are printed on the annunciator when commands
; are received from the uTile host controller.
;
; A built in editor is used to compose and store User Messages in EEPROM non-volatile
; memory.
;
; Change List
;=============
; 1. Read incoming serial into 4 byte buffer if @ character seen. Current
;	 sequential read and processing may be too slow to capture the input
;	 messge command string.
; 2. Alternative is to implement an interrupt driven serial handler. This
;	 may be best alternative.
; 3. Fixed <glin1:>, brne	glin5 changed to glin2.
; 4. DEL is acting like a CR, change to emulate BS
; 5. Other control keys, ignore, <glin7:> modified. Check for ^S key.
;
; ATmega328P cpu, 16.0000 MHz external crystal * Depends on controller board
;
; PC5 = SCL
; PC4 = SDA
;

.list		; Listing on

;
; General equates
;
.equ	FALSE = 0x0			; Logical 0
.equ	TRUE = !FALSE		; Logical 1
.equ	Vcc_low = false		; False for 5 V controller board
;.equ	Vcc_low = true		; True for 3.3 V controller board
.equ	debug = true		; Turn on debugging routines
;.equ	debug = false		; Turn on debugging routines
;.equ	baud_low = true		; 9600 baud for later boards
.equ	baud_low = false	; 19200 baud for 1st board
;
;
;
; Message parameters
;
.equ	msg_sz = 16			; Length of annunciator message
.equ	msg_num = 8			; Number of messages stored
.equ	msg_en = PC0		; Enable message editor, ground A0 input pin
;
; Processor operating voltage
;

.if	Vcc_low
.equ F_CPU = 8000000
.else
.equ F_CPU = 16000000
.endif

;
;
;
; TWI specific equates
;
.equ	STARTC = 0x08		; START condition sent
.equ	RSTART = 0x10		; REPEATED START condition sent
.equ	SLAw_ACK = 0x18		; Slave address write sent, ACK returned
.equ	SLAw_NAK = 0x20		; Slave address write sent, NAK returned
.equ	SLAr_ACK = 0x40		; Slave address write sent, ACK returned
.equ	SLAr_NAK = 0x48		; Slave address write sent, NAK returned
.equ	DATAr_ACK = 0x50	; Data byte received, ACK returned
.equ	DATAw_ACK = 0x28	; Data byte sent, ACK returned
.equ	DATAw_NAK = 0x30	; Data byte sent, NAK received
.equ	DATAr_NAK = 0x58	; Data byte received, NAK returned
.equ	RTC_MSLA = 0xfe		; Master's own slave address
;
;
; LCD backpack slave twi
;
.equ	LCD_SLA = 0b01001110	; LCD slave address (0x27), 7 bits + R/W bit
.equ	LCD_RS = 0				; LCD register select control
.equ	LCD_RW = 1				; LCD read/write* control
.equ	LCD_E = 2				; LCD enable control
.equ	LCD_BT = 3				; LCD backlight control
;
;
;
; UART definitions
;
;
.if		baud_low
.equ	BAUD = 9600			; Baud rate
.if		Vcc_low
.equ	BAUD_PRE = 51		; Baud rate prescaler - 8.00 MHz clock
.else
.equ	BAUD_PRE = 103		; Baud rate prescaler - 16.00 MHz clock
.endif
;
.else
;
.equ	BAUD = 19200		; Baud rate
.if		Vcc_low
.equ	BAUD_PRE = 25		; Baud rate prescaler - 8.00 MHz clock
.else
.equ	BAUD_PRE = 51		; Baud rate prescaler - 16.00 MHz clock
.endif
;
.endif
;
.equ	NULL = 0x0			; Null terminator
.equ	BELL = 0x07			; Bell
.equ	BS = 0x08			; Backspace
.equ	HT = 0x09			; Tab
.equ	LF = 0x0a			; Linefeed
.equ	CR = 0x0d			; Carriage return
.equ	ctlS = 0x13			; Control S
.equ	ctlW = 0x17			; Control W
.equ	ctlX = 0x18			; Control X
.equ	ctlZ = 0x1a			; Control Z
.equ	SP = 0x20			; Space
.equ	ESC = 0x1b			; Escape
.equ	DEL = 0x7f			; Delete
.equ	CMA	= 0x2c			; Comma
.equ	at = 0x40			; '@'
;


.if		Vcc_low

; Timer0, Timer1 and Timer2 parameters for 8.00 MHz clock
;
; Prescaler:	1		8		64			256			1024
; TCNTn clk:	8 MHz	1 MHz	125 kHz		31.25 kHz	7.8125 kHz
; Period:				1 us	8 us		32 us		128 us
;
; TCNT0 prescaler = 1024, clk_T0 = 8 MHz/1024 = 7.8125 kHz, 128 us
; TCNT1 prescaler = 256, clk_T1 = 8 MHz/1024 = 7.8125 kHz, 128 us
; TCNT2 prescaler = 1024, clk_T2 = 8 MHz/1024 = 7.8125 kHz, 128 us
;
;
.equ	OCR0Aload = 32		; OCR0A 8 bit register, 32 x 128 us = 4.096 ms
.equ	OCR1A64us = 1		; OCR1A 16 bit register, 128 us
.equ	OCR2Aload = 39		; OCR2A 8 bit register, 39 x 128 us = 4.992 ms
.equ	OCR1A1728us = 14	; OCR1A 16 bit register, 1562 x 128 us = 1792 us
.equ	OCR1A5ms = 39		; OCR1A 16 bit register, 39 x 128 us = 5 ms
.equ	OCR1A100ms = 781	; OCR1A 16 bit register, 781 x 128 us = 100 ms
;

.else

;
; Timer0, Timer1 and Timer2 parameters for 16.00 MHz clock
;
; Prescaler:	1		8		64			256			1024
; TCNTn clk:	16 MHz	2 MHz	250 kHz		62.5 kHz	15.625 kHz
; Period:				0.5 us	4 us		16 us		64 us
;
; TCNT0 prescaler = 1024, clk_T0 = 16 MHz/1024 = 15.625 kHz, 64 us
; TCNT1 prescaler = 256, clk_T1 = 16 MHz/1024 = 15.625 kHz, 64 us
; TCNT2 prescaler = 1024, clk_T2 = 16 MHz/1024 = 15.625 kHz, 64 us
;
;
.equ	OCR0Aload = 64		; OCR0A 8 bit register, 64 x 64 us = 4.096 ms
.equ	OCR1A64us = 1		; OCR1A 16 bit register, 64 us
.equ	OCR2Aload = 78		; OCR2A 8 bit register, 78 x 64 us = 4.992 ms
.equ	OCR1A1728us = 28	; OCR1A 16 bit register, 1562 x 64 us = 1728 us
.equ	OCR1A5ms = 78		; OCR1A 16 bit register, 78 x 64 us = 5 ms
.equ	OCR1A100ms = 1562	; OCR1A 16 bit register, 1562 x 64 us = 100 ms

.endif
;
;	
;
.equ	sblen = 0x1f		; Serial input buffer length
;
;
;
;
; Flag register flaga
;
.equ	numfl = 0			; Valid byte number flaga
.equ	crf = 1				; Carriage return key flaga
.equ	escf = 2			; Escape key flaga
.equ	kyf	= 3				; Control key flaga
.equ	xclinf = 4			; Delayed line clear flaga
.equ	tcnt1fa = 5			; TCNT1 flaga software timer flag
;
; Flag register flagb
;
.equ	LCD_LEDb = 0		; LCD backlight control flagb
.equ	paceb = 1			; Clock display pace flagb
.equ	pacec = 2			; LCD update pace flag
.equ	twi_erfb = 4		; TWI write error flagb
.equ	t100mf = 5			; 200ms timer flag
.equ	blnkmf = 6			; Blank line flag
;
;
;
.equ	maxbits = 0xff		; High byte mask
.equ	nbits = 16			; Convert 16 bits
.equ	ndig = 3			; Digit pair bytes
.equ	ndec = 5			; Digits to display/convert
;
;
; --- Line input buffer ---
;
.equ	linsz = 0x10		; Line buffer size
.equ	vtri = 3			; y = 0 position
.equ	hzri = 7			; x = 0 position
;
; --- Register definitions ---
;
; Low registers
;
.def	count = R2			; Counter for line buffer
.def	asav = R3			; rga save register
.def	SRsav = R4			; SREG save
.def	res0 = R5			; result register 0
.def	res1 = R6			; result register 1
.def	res2 = R7			; result register 2
.def	rtmr = R8			; Temporary timer
;
;
; High registers
;
.def	rmp = R16			; Multipurpose register
.def	rga = R17			; GP register RGA
.def	rgb = R18			; GP register RGB
.def	rgc = R19			; GP register RGC
.def	rgd = R20			; GP register RGD
.def	rge	= R21			; GP register RGE
.def	rgv	= R22			; Variable register
.def	flaga = R23			; Flag A register, 8 flags
.def	flagb = R24			; Flag B register, 8 flags
;
;
;
; --- Macro definitions ---
;
.macro	ldzptr				; Load ZH:ZL pointer with address*2
		ldi		ZH,high(@0*2)
		ldi		ZL,low(@0*2)
.endm
;
.macro	ldxptr				; Load XH:XL pointer with address to access data memory
		ldi		XH,high(@0)
		ldi		XL,low(@0)
.endm
;
.macro	ldyptr					; Load YH:YL pointer with address to access data memory
		ldi		YH,high(@0)
		ldi		YL,low(@0)
.endm
;
; Exchange contents of registers
;
.macro	xchreg					; Exchange registers
		push	@0
		push	@1
		pop		@0
		pop		@1
.endm
;
;
; --- SRAM Data Segment ---
;
.DSEG
.ORG	0X0100				; 2 Kb SRAM space
;
;
; Message buffers
;
msg_st:						; Space for GPS data buffers
;
msg0:
.byte		16				; Message 0 text buffer
;
msg1:
.byte		16				; Message 1 text buffer
;
;
msg2:
.byte		16				; Message 2 text buffer
;
;
msg3:
.byte		16				; Message 3 text buffer
;
;
msg4:
.byte		16				; Message 4 text buffer
;
;
msg5:
.byte		16				; Message 5 text buffer
;
;
msg6:
.byte		16				; Message 6 text buffer
;
;
msg7:
.byte		16				; Message 7 text buffer
;
;
msg8:
.byte		16				; Message 8 text buffer
;
;
msg_end:
;
linbuf:
.byte	linsz				; Character input line buffer
;
offs:
.byte		1				; Offset buffer
;
linxb:
.byte		1				; Line number buffer
;
msgxb:
.byte		1				; Message number buffer
;
; Data buffer for word and byte
;
wdbuf:
.byte	2					; Word byte buffer
;
dba:
.byte	1					; Data byte buffer (Keep this buffer below wdbuf)
;
;
;
; UART serial input buffer
;
uart_st:
sinb:						; Serial input buffer
.byte		sblen
;
getcnt:
.byte		1				; Input buffer getbyte counter
;
putcnt:
.byte		1				; Input buffer putbyte counter
;
;
uart_end:
;
;
; ============================================
;   R E S E T   A N D   I N T   V E C T O R S
; ============================================
;
;
; --- Code Segment ---
;
.CSEG
.ORG	$0000						; Interrupt vectors go here
;
		jmp			start			; Reset vector
;
.ORG	OC0Aaddr
		jmp			Timer0_COMPA	; Timer 0 Output Compare A handler
;
.ORG	OC1Aaddr
		jmp			Timer1_COMPA	; Timer 1 Output Compare A handler
;
;
.ORG	URXCaddr
		jmp			URXCint		;  USART Rx Complete
;
;
; End of interrupt vectors, start of program code space
;
;
.ORG	0x0034					; Program begins here
;
;
;###########################################################################
;
;
;
; ============================================
;     I N T E R R U P T   S E R V I C E S
; ============================================
;
;
; --- Timer 0 interrupt handler ---
;
; Used for I/O scanning
; 
; TCNT0 run in Output Compare mode, using OCR0A register to
; generate output compare interrupt every 64 x 64 us = 4.096 ms.
;
; TCNT0 operates in Clear Timer on Compare Match (WGM02:0 = 2).
; On Compare Match, TCNT0 counter is cleared.
; OCR0A (set to 64) defines the counter's TOP value.
;
; Clk_T0 = 16 MHz/1024 = 15.625 kHz, 64 us period, 
;
Timer0_COMPA:
;
		push	rmp					; Save registers
		in		SRsav,SREG
;
; 100 ms timer, 
;
t_100m:
		sbrc	flagb,t100mf		; Flag set?
		rjmp	t_100m1				;	Yes, exit timer
		dec		rtmr				;	No, run timer
		brne	t_100m1
		sbr		flagb,(1<<t100mf)	; Set flag at end of timer count down
t_100m1:
;
		out		SREG,SRsav			; Restore SREG
		pop		rmp
		reti
;
; --- Timer 1 interrupt handler ---
;
; Used to generate LCD time delays
;
; TCNT1 (16 bit) run in output compare mode, using OCR1A register to
; generate output compare interrupt every 64 us.
;
; TCNT1 operates in mode 5 CTC (WGM13:10 = 4).
; On compare match, TCNT1 is cleared.
; OCR1A (set to 1) define the counter's TOP value.
;
; Clk_tb = 16 MHz/1024 = 15.625 kHz, 64 us period, 
;
Timer1_COMPA:
;
		push	rmp
		in		SRsav,SREG					; Save SREG
;
		lds		rmp,TCCR1B					; Fetch control register
		cbr		rmp,(1<<CS12)|(1<<CS10)		; Stop timer by clearing CS12 & CS10
		sts		TCCR1B,rmp
;
		sbr		flaga,(1<<tcnt1fa)			; Foreground flag
;
		out		SREG,SRsav					; Restore SREG
		pop		rmp
		reti		
;
;
;
; USART Receive complete interrupt handler. Interrupt invoked by RXC0 set
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
		cpi		rmp,sblen		; Past end of buffer?
		brne	URXCint1
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
;         Initialization routines
; ============================================
;
;
; Turn off watchdog
;
wdt_off:
		cli							; Clear global interrupts
;
; Reset WD timer
;
		wdr
;
		in		rmp,MCUSR				; Clear WDRF bit
		andi	rmp,(0xff & (0<<WDRF))	; WDRF bit = 0
		out		MCUSR,rmp
;
; Set WDCE and WDE bits, keep old prescaler setting
;
		lds		rmp,WDTCSR
		ori		rmp,(1<<WDCE)|(1<<WDE)
		sts		WDTCSR,rmp
;
; Turn off WDT
;
		ldi		rmp,(0<<WDE)			; Clear WD system reset enable
		sts		WDTCSR,rmp
;
		sei								; Set global interrupts
		ret
;
; --- Initialization Routines ---
;
initz:
		rcall	zmsgb			; Clear data space buffers
;
		rcall	zregs			; Clear lower registers R0,..,R15
		clr		flaga			; Clear flag registers
		clr		flagb
;
; Enable Message Editor routine
;
		sbi		PORTC,PC0			; Enable PC0 (A0 input) pull-up
;
;
; SPI module, enable SPI, master mode, fosc/64
;
		ldi		rmp,(1<<SPE)|(1<<MSTR)|(1<<SPR1)	; fosc/64
		out		SPCR,rmp
;
.if		Vcc_low

		ldi		rmp,(1<<SPI2X)		; Enable SPI double speed if 8 MHz clock
		out		SPSR,rmp			; on 3.3 V controller board

.endif
;
;
;
; --- Timers Initialization ----
;
;
; === TCNT0 Initialization === 
;
; Setup TCNT0 prescaler = 1024, clock period = 64 us
;
InitTimer0:
		ldi		rmp,(1<<CS02)|(1<<CS00)	; Divide by 1024 prescaler, Fclk = 15.625 kHz
		out		TCCR0B,rmp				; Timer/Counter0 control register B
;
; Setup TCNT0 for CTC mode
;
		ldi		rmp,(1<<WGM01)			; CTC mode
		out		TCCR0A,rmp				; Timer/Counter0 control register A
;
; Initialize OCR0A output compare register
;
		ldi		rmp,OCR0Aload			; Set OCR0A = 64 for 4.096 ms period
		out		OCR0A,rmp
;
; Enable Timer/Counter0 Compare A Match Interrput in TIMSK0
;
		lds		rmp,TIMSK0
		sbr		rmp,(1<<OCIE0A)			; Enable Timer/Counter0 Output Compare A Match Interrupt
		sts		TIMSK0,rmp
;
;
; === TCNT1 Initialization === (OK)
;
; Setup 16 bit Timer/Counter1 Compare A in CTC mode. This timer
; runs when started and stops at end of timer cycle when TCNT1 = OCR1A
; and OC1A interrupt occurs. Timer is stopped inside interrupt handler.
;
; Setup TCNT1 prescaler for 1024, clock period = 64 us and CTC mode
; Note: User to explicitly start TCNT1 by setting CS12 in TCCR1B
; 
InitTimer1:
		ldi		rmp,(1<<WGM12)
		sts		TCCR1B,rmp				; Set CTC1 mode
;
; Initialize OCR1A output compare register
;
		ldi		rmp,high(OCR1A64us)		; Set OCR1A = 64 us period
		ldi		rga,low(OCR1A64us)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
; Enable TCNT1 Compare A Match Interrputs in TIMSK1
;
		lds		rmp,TIMSK1
		sbr		rmp,(1<<OCIE1A)		; Enable Output Compare A Match Interrupt
		sts		TIMSK1,rmp
;
;
; === INT0 Initialization ===
;
;
; Set up INT0 falling edge interrupt for use with nRF24L01+ module
;
initINT0:
		lds		rmp,EICRA			; rmp <-- EICRA
		sbr		rmp,(1<<ISC01)		; Falling edge INT0
		sts		EICRA,rmp			; EICRA <-- rmp
		in		rmp,EIMSK			; rmp <-- EIMSK
		sbr		rmp,(1<<INT0)		; Enable INT0
		out		EIMSK,rmp			; EIMSK <-- rmp 
;
;
; --- Enable GLobal Interrupts
;
		sei							; Set global interrupts
		ret
;
;
; Initialize TWI module - set up as bus master
;
TWI_init:
;
		ldi		rmp,48			; 250 kHz TWI bus rate
		sts		TWBR,rmp		; Load bit rate generator divider
		ret
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
;
;
;###########################################################################
;
;     M A I N    P R O G R A M    S T A R T
;
;###########################################################################
;
;
;
; Controller startup and initialization
;
start:
;
; Initialize the stack pointer to end of SRAM
;
		ldi		rmp,high(RAMEND)	; Init MSB stack
		out		SPH,rmp
		ldi		rmp,low(RAMEND)		; Init LSB stack
		out		SPL,rmp
;
; Initialize the engine
;
		rcall	wdt_off			; Disable watchdog. Must be done soon after a reset
		rcall	initz			; Initialize engine
		rcall	TWI_init		; Initialize the TWI module		* LCD *
		call	LCD_ledon		; Turn on LCD backlight bit
		rcall	inzuart			; Initialize the UART
;
; The LCD is a slow start-up device, add 1 s delay after power-up
;
		rcall	d1s				;		* LCD *

;
; LCD module start-up. Home, clear display, turn off cursor
;
	;;	rcall	LCD_start		; Open LCD TWI slave
	;;	rcall	LCD_home
	;;	rcall	LCD_clear
	;;	rcall	LCD_cur_off		; Cursor off
;
		rcall	ld_msg			; Load messages stored in EEPROM
;
;
;
; Main program control loop
;
main:
		rcall	prscn			; Print main screen and menu

tst0:
	;	rcall	ci
	;	rcall	co
	;	rjmp	tst0

;
; Check if msg_en input pin grounded to enable TZ offset data entry. This
; routine is run only if GPS module TX line is disconnected from RX in pin
; since the UART RX input is connected to both the USB RX line as well as
; the GPS Tx line.
;

		sbis	PINC,msg_en		; Test if msg_en pin jumpered to ground
		rjmp	gtmsg_ed		;	Yes, go to message editor

;
;
; Command input processor for LCD messages monitors serial
; input line for annunciator control string of format:
; @, linxb, msgxb, CR, LF	ASCII printable string sent
; from host controller uTile over serial link.
;
; Using LCD line number data saved in linxb, LCD cursor is
; positioned to specified line1 or line2. Using the LCD message
; number data saved in linxb, OFFS buffer is loaded with target
; message to display, then message is sent to LCD display.
;


cmi:
		rcall	pxy
		.db		3,2
		rcall	ceol
;
		rcall	ci				; Get incoming character
		cpi		rga,at			; '@' ?
		brne	cmi				;	No, keep scanning
		rcall	ci				; Get incoming character
		ldi		rmp,'0'			; Ascii '0'
		sub		rga,rmp			; Convert to binary
		sts		linxb,rga		; Save LCD target line number
;
; Check for 'f' or regular message number 
;
		rcall	ci				; Get incoming character
		rcall	case			; Upper case letter
		cbr		flagb,(1<<blnkmf)	; Clear the flag
		cpi		rga,'F'			; Check if message 'f'
		brne	cmi4				;	No, process number and store
		sbr		flagb,(1<<blnkmf)	; Set blank line flag
		rjmp	cmi4a				; Continue
;
cmi4:
		ldi		rmp,'0'			; Ascii '0'
		sub		rga,rmp			; Convert to binary
		sts		msgxb,rga		; Save LCD target message number
;
cmi4a:
		rcall	ci				; Get incoming character
		cpi		rga,CR			; CR ?
		breq	cmi1			;	Yes, continue processing
		rjmp	cmi				;	No, error loop
;
;
; Possible good LCD control string is received, target LCD line number
; and message number are stored in respective linxb and msgxb.
; Issue specified message to LCD display. If message number = 'F', 
; position cursor to line number and printing blank line message
;
cmi1:
		lds		rmp,linxb		; Get target LCD line
		andi	rmp,0b00000011	; Mask to lines 1 or 2
		cpi		rmp,1
		brne	cmi2			; Go to line 1
		rcall	LCD_line1
		rjmp	cmi3
;
cmi2:
		cpi		rmp,2
		brne	cmix
		rcall	LCD_line2		; Go to line 2
;
; Print target message to LCD
;
cmi3:
		sbrc	flagb,blnkmf
		rjmp	cmiblnk			; Blank the line
		lds		rmp,msgxb		; Get target LCD message number
		andi	rmp,0b00000111	; Mask to 0 to 7 message numbers
		sts		offs,rmp
		rcall	LCD_prmsg
;
cmix:
		rjmp	cmi				; Loop to start
;
cmiblnk:
		ldzptr	blankm			; Erase current line
		rcall	LCD_wrln
		rjmp	cmi				; Loop to start
;
;
; Print main screen
;
prscn:
		call	clrscn			; Home cursor and clear screen
		ldzptr	scrnm			; Print main banner
		call	pptr
		ret
;
;
;###########################################################################
;
; Message control routines
;
; Get message line and save in RAM  message buffer
;
; Entry:	offs = message index number
;

gtmsg_ed:
		rcall	prmsg_pg		; Show all messages on screen

	;;	rcall	LCD_line1
	;;	rcall	LCD_prmsg		; Show on LCD

gtmsg:
		rcall	slinb
		rcall	prmsg_hdr		; Show message ruler line
;
gtmsg1:
		rcall	slinc			; Position cursor, blank line
		rcall	glin
;
; Possible good line input
;
gtmsg2:
		sbrs	flaga,crf		; Test for a <CR> exit
		rjmp	gtmsg3
		tst		count			; Check for a blank line
		breq	gtmsg2a			;	Yes, move to next line
		rjmp	gtmsg4			;	No, check line input
;
; Blank line <CR> only, same as nxlin
;
gtmsg2a:
		rcall	msg_lo			; Normal video
		rcall	nxln			; Move to next line
		rcall	msg_hi			; Hilight video
	;;	rcall	LCD_line1
	;;	rcall	LCD_prmsg
		rjmp	gtmsg1			; Redo line
;
;
; Test for cursor keys
;
;       Up arrow        ESC [ A   -->   Previous Line
;       Down  "         ESC [ B   -->   Next Line
;
gtmsg3:
		sbrs	flaga,escf		; Escape char?
		rjmp	gtmsg4			;	No, continue processing text line
		rcall	ci
		cpi		rga,'['			; '['?
		brne	gtmsg1			;	No, redo line
		rcall	ci				;	Yes, process rest of cursor string
		cpi		rga,'B'			; DN arrow?
		brne	gtmsg3a
		rcall	msg_lo			; Normal video
		rcall	nxln			; Move to next line
		rcall	msg_hi			; Hilight video
	;;	rcall	LCD_line1
	;;	rcall	LCD_prmsg
		rjmp	gtmsg1			; Redo line
;
gtmsg3a:
		cpi		rga,'A'			; UP key
		brne	gtmsg1
		rcall	msg_lo			; Normal video
		rcall	prvln			; Up a line
		rcall	msg_hi			; Hilight video
	;;	rcall	LCD_line1
	;;	rcall	LCD_prmsg
		rjmp	gtmsg1			; Redo line
;
; Check for a ^S key for storing messages option
;
gtmsg4:
		cpi		rga,ctlS			; ^S key?
		brne	gtmsg4a				;	No, continue
		rcall	store_msg			;	Yes, store messages menu
		clr		rmp
		sts		offs,rmp			; msg0
		rjmp	gtmsg1
;
; Line is text message string
;
gtmsg4a:
		ldxptr	linbuf			; Reset linbuf pointer
		rcall	msg_pntr		; Setup YHL message pointer
		rcall	clr_msgb		; Clear current line buffer
;
; Transfer linbuf text to message buffer
;
gtmsg5:
		ld		rmp,X+
		st		Y+,rmp
		dec		rgb
		brne	gtmsg5			; Move whole line to RAM  buffer
;
		rcall	msg_hi			; Hilight current line
	;;	rcall	LCD_line1
	;;	rcall	LCD_prmsg
;
		rjmp	gtmsg			; Loop to start


;
;
; Go to next message line, index in offs
;
nxln:
		lds		rmp,offs		; Get offs
		inc		rmp				; Increment offs to next timer
		cpi		rmp,(msg_num)	; offs = (msg_num)?
		brne	nxln1			;	No, done
		clr		rmp				;	Yes, reset to 0
nxln1:
		sts		offs,rmp
		ret
;
; Go to previous message line, index in offs
;
prvln:
		lds		rmp,offs
		dec		rmp				; Decrement offs to previous timer
		cpi		rmp,0xff		; offs = -1?
		brne	prvln1			;	No
		ldi		rmp,(msg_num-1)	;	Yes, go to max message number
prvln1:
		sts		offs,rmp
		ret
;
; Highlight current message indexed by OFFS.
;
msg_hi:
		rcall	vrev			; Hilight attribute ON
msg_hi1:
		rcall	prmsg_ln		; Print indexed message
		rcall	vlo				; Normal on
		ret
;
; Show as normal attribute current message indexed by OFFS.
;
msg_lo:
		rcall	vlo
		rjmp	msg_hi1
;
; Set pointer YHL to message indexed by offs content. Used to
; adjust YHL prior to printing target message
;
msg_pntr:
		ldyptr	msg0			; Base address of message buffers
		lds		rmp,offs		; Get offs content, which message
		tst		rmp				; Check if offs = 0
		brne	msg_pntr1		;	No, continue
		ret						;	Yes, exit
msg_pntr1:
		adiw	YH:YL,msg_sz	; Move YHL to next message
		dec		rmp
		brne	msg_pntr1
		ret
;
; Print message line (offs = line #) to correct line position on screen
;
prmsg_ln:
		rcall	msg_pntr		; Setup YHL message pointer per offs
		rcall	lin_pos			; Position cursor to line start
		lds		rga,offs		; Get message number
		rcall	pahex			; Print message number
		rcall	dblsp			; Print a double space
		rcall	prmsg_rb		; Print a message line
		ret
;
; Print RAM buffer message <-- YHL
;
; Entry:	msg buffer <-- YHL
;
;
prmsg_rb:
		push	rgd
		ldi		rgd,msg_sz		; Message character counter
prmsg_rb1:
		ld		rga,Y+			; Get a character
		call	co
		dec		rgd
		brne	prmsg_rb1
		pop		rgd
		ret
;
; Clear RAM message buffer (fill with ACSII spaces) <-- YHL
;
; Entry:	msg buffer <-- YHL
;
;
clr_msgb:
		push	rgd
		push	YH
		push	YL
;
		rcall	msg_pntr		; Setup YHL message pointer
		ldi		rgd,msg_sz		; Message character counter
		ldi		rmp,' '			; Fill buffer with space character
clr_msgb1:
		st		Y+,rmp
		dec		rgd
		brne	clr_msgb1
;
		pop		YL
		pop		YH
		pop		rgd
		ret
;
; Print message ruler line
;
prmsg_hdr:
		ldzptr	msg_rlm
		rcall	pptr
		ret
;
msg_rlm:
	.db		"|..............|",ctlZ
;
;
; Print msgb <-- YHL to LCD.
; Caller to preset LCD cursor to line 1 or line 2
;
; Entry:	msg buffer <-- YHL
;
LCD_prmsg:
		push	rgd
		ldi		rgd,msg_sz		; Message character counter
		rcall	msg_pntr		; Setup YHL message pointer
LCD_prmsg1:		
		ld		rmp,Y+			; Get message character
		rcall	LCD_wrdat		; Data to LCD slave
		dec		rgd
		brne	LCD_prmsg1
		pop		rgd
		ret
;
;
; Print page of msg_num messages, highlight first message
; Print message number, then message text, one message per line
;
; Registers:	rga, rgd, rmp, YHL, XHL
;
;
prmsg_pg:
		push	rga
		push	rgd
		clr		rmp				; Start at message 0
		sts		offs,rmp		; offs = 0, line 0
		ldi		rgd,msg_num		; Number of messages
		rcall	msg_pntr		; Setup YHL message pointer
;
prmsg_pg1:
		rcall	prmsg_ln		; Print message line 
		lds		rmp,offs
		inc		rmp				; Next line
		sts		offs,rmp
		dec		rgd				; Count messages printed
		brne	prmsg_pg1		; Continue
		clr		rmp				; Point to message 0
		sts		offs,rmp		; offs = 0, line 0
;
		rcall	msg_hi			; Hilight first line
	;;	rcall	LCD_line1
	;;	rcall	LCD_prmsg
;
		pop		rgd
		pop		rga
		ret
;
; Position cursor to position of start of selected message
;
; Entry:	offs = selected line number
;
lin_pos:
		push	YH
		push	YL
		ldi		YL,hzri			; Initial horizontal offset
		ldi		YH,vtri			; Initial horizontal offset
		lds		rmp,offs		; Get offset and process row info
		andi	rmp,0b00001111	; Isolate bits for col 0..8
		add		YH,rmp			; Add initial vertical offset
;
		rcall	gotoxy			; Position the cursor
		pop		YL
		pop		YH
		ret
;
;
; Store messages to EEPROM. Y to store, else <CR> to exit
;
store_msg:
		rcall	slind			; Status message line	
		ldzptr	storem			; Storage message
		rcall	pptr
;	
store_msg1:
		rcall	getc			; Scan for any console input
		cpi		rga,CR			; <CR>?
		brne	store_msg2
		rcall	slind
		ret
;
store_msg2:
		rcall	case
		cpi		rga,'Y'			; 'Y' response?
		brne	store_msg1		; No, redo input
;
		rcall	slind
		ldzptr	savEEms
		rcall	pptr
;
		rcall	str_msg			; Write messages buffer to EEPROM
;
		rcall	d500ms			; Short delay to see message
		rcall	slind
		ret
;
;
;
;
;###########################################################################
;
;
;************************************************
;
; Store and load TZ offset to EEPROM
;
;************************************************
;
; Registers: rga, rgb, rmp, Y, Z
;
str_eeprm:
;
		ldi		rmp,0xff
		out		EEARL,rmp				; EEPROM start address = -1 as
		out		EEARH,rmp				;  EEAR is pre-decremented
;
		rcall	EEWrSeq					; Write to EEPROM
		ret
;
; Load EEPROM TZ 0ffset setting to TZb.
;
ld_eeprm:
		ldi		rmp,0xff
		out		EEARL,rmp				; EEPROM start address = -1
		out		EEARH,rmp
;
ld_eeprm1:
		rcall	EERdSeq					; Get source byte from EEPROM
		ret
;
;
;
;
;
;###########################################################################
;
;	L C D   P O L L E D   D R I V E R   M O D U L E   R O U T I N E S
;
;###########################################################################
;
; The LCD routines interface to the TWI interface via <twi_wrbyt> routine
;
; ==== LCD Support Routines ====
;
;
; ==================================================
;	Setup TWI Master to slave write to LCD backpack	
; ==================================================
;
; Setup master to slave write to LCD backpack TWI interface
; Master sends START then SLA+W 7 bit slave address plus write bit.
;
; Registers:	rga, rmp
;
; Entry:		rgb = slave address, 7 bits + R/W bit (SLA value)
; Exit:			C= 0 OK, else C=1 on error
;
LCD_start:
		rcall	twi_S			; Send a START
		brcs	LCD_S_err		; Error
		ldi		rgb,LCD_SLA		; Slave address + W
		rcall	LCD_SLAw		; Send slave address + write
		brcs	LCD_S_err		; Error
		ret
;
; On S or SLA+W error, send S, set error flag and exit
;
LCD_S_err:
		ret
;
; Send Slave Address + W. Based on <twi_SLAw> but does not check for NAK.
; This is needed to prevent the LCD driver from hanging up the TWI bus if
; the slave is not on-line.
;
; Entry:		rgb = slave address, 7 bits + R/W bit (SLA value)
;
LCD_SLAw:
		mov		rmp,rgb			; Slave address + W bit, 0
		sts		TWDR,rmp		; Load twi data register
		ldi		rmp,(1<<TWINT)|(1<<TWEN)	; Clear TWINT, enable bus
		sts		TWCR,rmp
LCD_SLAw1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating START sent
		sbrs	rmp,TWINT
		rjmp	LCD_SLAw1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,SLAw_ACK	; Slave address write sent?
		brne	LCD_SLAwerr		; Not sent, error condition
		clc						; C=0 if OK
		ret
LCD_SLAwerr:
		sec						; C=1 on error
		ret	
;
; ==== LCD Data Write Routines ====
;
;
; Write data byte in rmp to LCD backpack TWI interface.
;
; Data is written as two 4 bit nibbles, high followed by low nibble.
;
; LCD_RS = 0
; LCD_RW = 0
; LCD_E is toggled high then low to write to LCD.
;
; Registers:	rmp, rgd temporary storage
; Entry:		rmp has data to transmit
;
; Call <LCD_start> to enable TWI to LCD backpack communications.
;
; At end of transmission to session LCD, invoke the TWI STOP <twi_P>
; to release the backpack slave from TWI bus.
;
LCD_wrins:
		push	rmp				; Save data
		andi	rmp,0b11110000	; Mask off control bits
		rcall	LCD_wrnib		; Send high nibble
;	
		pop		rmp				; Fetch data
		swap	rmp				; Swap nibbles
		andi	rmp,0b11110000	; Mask off control bits
		rcall	LCD_wrnib		; Send low nibble
		ret
;
; Write rmp to LCD RAM via LCD backpack TWI interface.
;
; LCD_RS = 1
; LCD_RW = 0
; LCD_E is toggled high then low to write to LCD.
;
; Registers:	rmp, rgd temporary storage
; Entry:		rmp has data to transmit
;
;
LCD_wrdat:
		push	rmp				; Save data
		andi	rmp,0b11110000	; Mask off control bits
		sbr		rmp,(1<<LCD_RS)	; Set RS bit for write to CGRAM
		rcall	LCD_wrnib		; Send high nibble
;	
		pop		rmp				; Fetch data
		swap	rmp				; Swap nibbles
		andi	rmp,0b11110000	; Mask off control bits
		sbr		rmp,(1<<LCD_RS)	; Set RS bit for write to CGRAM
		rcall	LCD_wrnib		; Send low nibble
		ret
;
; Write the byte passed in rmp to TWI in two nibbles, high nibble
; followed by low nibble. Data passed in (rmp) to LCD_wrnib TWI
; low nibble is ignored, only high nibble is used. The low nibble is
; replaced by control bits for LCD, LCD_RS, LCD_RW, LCD_E and BT bits.
;
; Registers:	rmp, rgd
; Entry:		rmp = (data_nibble.00000)
;
LCD_wrnib:
		bst		flagb,LCD_LEDb	; T <-- LCD_LEDb backlight flag
		bld		rmp,LCD_BT		; rmp(LCD_BT) <-- T
		push	rmp				; Save a copy
		rcall	twi_wrbyt		; Write high nibble to TWI bus
		pop		rmp
		push	rmp
		sbr		rmp,(1<<LCD_E)	; Set E bit
		rcall	twi_wrbyt		; Write to TWI bus
;
		pop		rmp
		cbr		rmp,(1<<LCD_E)	; Clear E
		rcall	twi_wrbyt		; Write to TWI bus
		ret
;
;
; ==== LCD Control Routines ====
;
;
; Turn on LCD LED backlight by flag setting flag LCD_LEDb = 1
;
LCD_ledon:
		sbr		flagb,(1<<LCD_LEDb)	; Set backlight flag
		ret
;
; Turn off LCD LED backlight by flag clearing flag LCD_LEDb = 0
;
LCD_ledoff:
		cbr		flagb,(1<<LCD_LEDb)	; Clear backlight flag
		ret
;
;
; Home cursor on LCD display (1.64 ms execution time max).
;
LCD_home:
		ldi		rmp,0b00000010		; Home cursor command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d1728us
		ret
;
;  Clear display and home cursor
;
LCD_clear:
		ldi		rmp,0b00000001		; Clear and home cursor command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d1728us
		ret
;
;  Display OFF
;
LCD_off:
		ldi		rmp,0b00001000		; Display off command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
;  Display ON
;
LCD_on:
		ldi		rmp,0b00001100		; Display on command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
;  Shift LCD display left
;
LCD_left:
		ldi		rmp,0b00011000		; Display shift left command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
;  Shift LCD display right
;
LCD_right:
		ldi		rmp,0b00011100		; Display shift right command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 100 us delay
		ret
;
; Set display RAM address to 00, line 1 start
;
LCD_line1:
		ldi		rmp,0b10000000		; Display address 00 command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 64 us delay
		ret
;
; Set display RAM address to 64, line 2 start
;  
LCD_line2:
		ldi		rmp,0b11000000		; Display address 64 command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 64 us delay
		ret
;
; Set display RAM address to 78, line
;  
LCD_c75:
		ldi		rmp,0b11001110		; Display address 78 command
		rcall	LCD_wrins			; Master to LCD slave
		rcall	d64us				; 64 us delay
		ret
;
; Write message pointed to by Z, ctrlZ terminated, to LCD display.
;
LCD_wrln:
		push	rmp
LCD_wrln1:
		lpm		rmp,Z+			; String byte to rga, Z+
		cpi		rmp,ctlZ		; byte ^Z?
		brne	LCD_wrln2		; Print if not ^Z
		pop		rmp
		ret
LCD_wrln2:
		cpi		rmp,NULL		; Skip any nulls in string
		breq	LCD_wrln1
		rcall	LCD_wrdat
		rjmp	LCD_wrln1
;
; Display character passed in rmp as two hexadecimal digits
;
; Registers:	rmp, rga
;
LCD_pahex:
		push	rmp				; Save data
		swap	rmp				; Show high nibble (MSD) first
		rcall	LCD_pahex1
		pop		rmp
;
LCD_pahex1:
		andi	rmp,0x0f		; Mask off high nibble
		ldi		rga,0x30		; '0'
		add		rmp,rga			; Convert hex number to character
		cpi		rmp,0x3a		; Subtract ':' to check if > 9?
		brcs	LCD_pahex2		;  No, it is 0 ... 9
		ldi		rga,7			;  Yes, convert to A ... F
		add		rmp,rga
LCD_pahex2:
		rcall	LCD_wrdat		; Write to LCD hex digit
		ret
;
; Send a ':' character to LCD display
;
LCD_colon:
		ldi		rmp,0x3a		; ':' character
		rcall	LCD_wrdat
		ret
;
; Position cursor right to column (1 to 20). Cursor position specified in
; register rga.
;
; User calls HOME or sets line 1 or 2 by SLN1 or SLN2 prior to calling LCD_cur.
;
; Registers:	rmp, rga
;
LCD_cur:
		dec		rga				; Cursor starts at position 1
LCD_cur1:
		rcall	LCD_right
		dec		rga				; Count cursor position
		brne	LCD_cur1
		ret
;
; Send a space character to LCD display
;
LCD_space:
		ldi		rmp,0x20		; Space character
		rcall	LCD_wrdat
		ret
;
; Send 3 spaces to LCD display
;
LCD_3space:
		ldi		rga,3
LCD_3sp:
		ldi		rmp,0x20		; Space character
		rcall	LCD_wrdat
		dec		rga
		brne	LCD_3sp
		ret
;
; Turn off LCD cursor
;
LCD_cur_off:
		ldi		rmp,0x0c		; No cursor
		rcall	LCD_wrins		; Master to LCD slave
		ret
;
;
; Clean up LCD lines 1 and 2
;
LCD_clean:
		rcall	LCD_line2
		ldzptr	blankm				; Erase line
		rcall	LCD_wrln
		rcall	LCD_line1
		ldzptr	blankm				; Erase line
		rcall	LCD_wrln
		ret
;
;
;###########################################################################
;
;		T W I   P O L L E D   D R I V E R   M O D U L E
;
;###########################################################################
;
;
; ============
;	TWI Start
; ============
;
; Send start condition. Atomic operation. Usually the first twi operation to
; start a twi master read/write sequence.
;
twi_S:
		ldi		rmp,(1<<TWINT)|(1<<TWSTA)|(1<<TWEN)	; Clear TWINT, set START, enable bus
		sts		TWCR,rmp
twi_S1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating START sent
		sbrs	rmp,TWINT
		rjmp	twi_S1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,STARTC		; START condition sent?
		breq	twi_S2			;	Yes
		cpi		rmp,RSTART		; Repeated start condition sent?
		brne	twi_error		; Not sent, error condition
twi_S2:
		clc						; C=0 if OK
		ret
;
twi_error:
		sec						; C=1 on error
		ret
;
; ===========================
;	TWI Repeated Start	(OK)
; ===========================
;
; Send repeated start condition. Atomic operation. 
;
twi_RS:
		ldi		rmp,(1<<TWINT)|(1<<TWSTA)|(1<<TWEN)	; Clear TWINT, set START, enable bus
		sts		TWCR,rmp
twi_RS1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating RSTART sent
		sbrs	rmp,TWINT
		rjmp	twi_RS1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,RSTART		; START condition sent?
		brne	twi_RSerr		; Not sent, error condition
		clc						; C=0 if OK
		ret
;
twi_RSerr:
		sec						; C=1 on error
		ret
;
		ret	
;
; =========================
;	TWI Stop	(OK)
; =========================
;
; Send stop condition. Atomic operation
;
twi_P:
		ldi		rmp,(1<<TWINT)|(1<<TWSTO)|(1<<TWEN)
		sts		TWCR,rmp
		ret
;
; Send one data byte to slave. Atomic operation.
; Sends a byte, waits for and checks for ACK, C=1 on error
; else C=0 if all OK.
;
; Entry:	rmp = data to write to TWDR
; 
twi_wrbyt:
		sts		TWDR,rmp		; Load twi data register
		ldi		rmp,(1<<TWINT)|(1<<TWEN)	; Clear TWINT, enable bus
		sts		TWCR,rmp
twi_wrbyt1:
		lds		rmp,TWCR		; Wait for TWINT flag set, indicating byte sent
		sbrs	rmp,TWINT
		rjmp	twi_wrbyt1
;
		lds		rmp,TWSR		; Check status code in status register
		andi	rmp,0xf8		; Mask off prescaler bits
		cpi		rmp,DATAw_ACK	; Data sent?
		brne	twi_wrerr		; Not sent, error condition
		clc						; C=0 if OK
		ret
;
; On TWI read error, set RTC_error flag bit and exit
;
twi_wrerr:
		sec
		ret		
;
;
;
;
;###########################################################################
;
; Timers - general purpose
;
;
; Start 100 ms timer
;
start_t100m:
		ldi		rmp,24
		mov		rtmr,rmp
		cbr		flagb,(1<<t100mf)
		ret
;
; 10 us delay	(OK)
;
d_10u:
;
.if		Vcc_low
		ldi		rgc,24				; 10.1 us delay
.else
		ldi		rgc,50
.endif
;
d_10u1:
		dec		rgc
		brne	d_10u1
		ret
;
; 15 us delay	(OK)
;
d_15u:
;
.if		Vcc_low
		ldi		rgc,38
.else
		ldi		rgc,75
.endif
;
d_15u1:
		dec		rgc
		brne	d_15u1
		ret
;
; 130 us delay	(OK)
;
d_130u:
		ldi		rgb,13
d_130u1:
		rcall	d_10u
		dec		rgb
		brne	d_130u1
		ret
;
; 1 ms delay
;
d_1m:
		ldi		rgb,100
d_1m1:
		rcall	d_10u
		dec		rgb
		brne	d_1m1
		ret
;
; Various time delays used while accessing the LCD
; One-shot Timers based on TCNT1 interrupt handler, 64 us interrupt rate.
; 
;
d64us:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A64us)		; Set OCR1A = 64 us period
		ldi		rga,low(OCR1A64us)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d64us1:
		sbrs	flaga,tcnt1fa
		rjmp	d64us1
		ret
;
; 1.73 ms delay timer 
;
d1728us:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A1728us)		; Set OCR1A = 1728 us period
		ldi		rga,low(OCR1A1728us)
		sts		OCR1AH,rmp					; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d1728us1:
		sbrs	flaga,tcnt1fa
		rjmp	d1728us1
		ret
;
; 5 ms delay timer 
;
d5ms:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A5ms)			; Set OCR1A = 100 ms period
		ldi		rga,low(OCR1A5ms)
		sts		OCR1AH,rmp					; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d5ms1:
		sbrs	flaga,tcnt1fa
		rjmp	d5ms1
		ret
;
; 100 ms delay timer 
;
d100ms:
;
; Initialize OCR1A output compare register with new reload values
;
		ldi		rmp,high(OCR1A100ms)	; Set OCR1A = 100 ms period
		ldi		rga,low(OCR1A100ms)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
		cbr		flaga,(1<<tcnt1fa)			; Clear timer flag in case it is set
		lds		rmp,TCCR1B					; Fetch control register
		sbr		rmp,(1<<CS12)|(1<<CS10)		; Start timer by setting CS12 & CS10
		sts		TCCR1B,rmp
d100ms1:
		sbrs	flaga,tcnt1fa
		rjmp	d100ms1
		ret
;
; General 0.5 s timer
;
d500ms:
		ldi		rgb,5			; 5 x 0.1 s = 0.5 s
		rjmp	d1s1
d1s:
		ldi		rgb,10			; 10 x 0.1 s = 1.0 s
d1s1:
		call	d100ms
		dec		rgb
		brne	d1s1
		ret
;
;
; Clear RAM data space message buffers
;
zmsgb:
		ldi		rgb,(msg_end-msg_st)
		ldxptr	msg_st
		ldi		rmp,' '			; Space character
zmsgb1:
		st		X+,rmp
		dec		rgb
		brne	zmsgb1
;
; Clear linbuf and pointers
;
zbuf:
		ldi		rgb,sblen
		ldxptr	uart_st
		clr		rmp
zbuf1:
		st		X+,rmp
		dec		rgb
		brne	zbuf1
		sts		getcnt,rgb
		sts		putcnt,rgb
		ret
;
; Zero lower registers R0...R15
;
zregs:
		ldi		rga,16
		clr		rmp
		ldxptr	0x0			; Register file base address
zregs1:
		st		X+,rmp
		dec		rga
		brne	zregs1
		ret
;
; Print 5 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p5dg:
		mov		rga,res2		; Fetch 1st of 3 bytes
		rcall	pdg				; Show digit 5 only
;
; Print 4 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p4dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pacc			; Show digits 4,3
;
; Print 2 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p2dg:
		mov		rga,res0		; Fetch 3rd of 3 bytes
		call	pacc
		ret
;
; Print 3 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p3dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pdg				; Show digit 3 only
		rjmp	p2dg			; Show digits 2,1
;
; Convert packed BCD digits in rga to ascii and display		(OK)
;
pacc:
		mov		asav,rga		; Save the data
		swap	rga				; Hi nibble first
		call	pdg				; Convert to ascii and display
		mov		rga,asav
		call	pdg				; Show lo nibble
		ret
;
; Display bcd low nibble as ascii digit			(OK)
;
pdg:
		andi	rga,0x0f		; Mask hi nibble
		ldi		rmp,'0'			; Convert by adding '0'
		add		rga,rmp
		call	co				; Show the digit
		ret
;
;
;###########################################################################
;
; --- Conversion and testing routines ---
;
;  Convert rga to upper case
;
case:
		cpi		rga,0x61		; Ascii 'a'
		brcs	case1			; < a
		cpi		rga,0x7b		; Ascii 'z'
		brcc	case1			; > z
		clc
		sbci	rga,SP			; Subtract SP to convert to UC
case1:
		ret
;
;
; gnum used to process possible good ascii number from linbuf, convert it to		(OK)
; binary number in YH:YL using gdbn.
;
; Save data byte in DBA if number less than 256, or data word
; in WDBUF if number is 256 to 65,535. The numfl is set if the input
; data is good byte value. User to clear numfl after test.
;
; Registers:	flaga, rgv, YH, YL, rmp, XH:XL
;
gnum:
		cbr		flaga,(1<<numfl)	; Clear the good number flag
		mov		rgv,count			; Load counter with number of characters in linbuf
		cpi		rgv,ndec+1			; Compare to limit
		brcc	gnum2				; Too many digits error
		call	gdbn				; Convert to binary in YH:YL
		brcc	gnum2				; Error
		ldxptr	wdbuf				; Point to data word buffer
		st		X+,YH				; Save result high byte
		st		X+,YL				; Save result low byte
		st		X,YL				; Save result low to data byte buffer
		tst		YH					; Test result high byte if zero
		breq	gnum1				;	Yes, word value
		ldi		rmp,0xff			;	No, cap byte value to 0xff
		st		X,rmp				;   Save in dba
gnum1:
		sbr		flaga,(1<<numfl)	; Mark as byte value
gnum2:
		ret
;
; Convert ASCII decimal string in LINBUF to binary number in  YH:YL.		(OK)
; The number to be converted is maximum allowed 5 ascii digits long, 
; set by ndec, equivalent to (0xffff) binary number.
;
; This routine called by gnum to convert ascii numbers for data entry
;
; Entry: rgv = ndec, number of ascii digits to convert
; Exit:  YH:YL <-- 16 bit result if ok, C = 1
;        YH:YL <-- 00, C = 0 if error
; Regs:  rga, rgb, rgc, YH, YL, rgv, XH:XL
;
gdbn:
		clr		YH			; Clear result registers
		clr		YL
		ldxptr	linbuf		; Setup line buffer pointer
gdbn1:
		ld		rga,X+		; Fetch a character
		call	decdg		; Convert to BCD
		brcc	gdbnx		; Error exit
		mov		asav,rga	; Save character
		call	dex10		; Value * 10, result in YH:YL
		brcs	gdbnov		; Overflow
;
		mov		rgc,asav	; Add original digit in
		clr		rgb
		call	adebc		; YH:YL = YH:YL + rgb:rgc
		brcs	gdbnov		; Overflow error
		dec		rgv			; All characters processed?
		brne	gdbn1		;	No, continue
		sec
		ret					;	Yes, normal exit
;
gdbnx:
		clc					; Error exit
		clr		YH
		clr		YL
		ret
;
gdbnov:
		sec					; Overflow condition
		ldi		YH,0xff
		ldi		YL,0xff	; Limit to 0xFFFF
		ret
;
; Convert ASCII 0.....9 to BCD, C = 1 if ok, else		(OK)
; C = 0 and rga unchanged
; Registers:	rga, asav
;
decdg:
		mov		asav,rga	; Save ascii digit
		subi	rga,'0'		; Char less than char '0'?
		brcs	ddgx		;	Yes, error exit
		cpi		rga,LF		;  Char from 0...9?
		brcc	ddgx		;	No, error exit
		ret					; Is 0...9
ddgx:
		clc					; Not 0...9
		mov		rga,asav
		ret
;
; Convert 16 bit binary in YH:YL to packed bcd in res2:res1:res0		(OK)
;
; Registers: rgb, rgc, YH, YL, res0, res1, res2
;
bn2bcd:
		ser		rgc					; rgc = 0xff
		mov		res2,rgc
;
; Process 10,000's digit
;
cvde_L10k:
		inc		res2
		subi	YL,low(10000)
		sbci	YH,high(10000)
		brcc	cvde_L10k			; Loop until C set
		subi	YL,low(-10000)		; Correct last subtraction
		sbci	YH,high(-10000)
		ldi		rgb,(256-16)
;
; Process 1000's digit
;
cvde_L1k:
		subi	rgb,(-16)
		subi	YL,low(1000)
		sbci	YH,high(1000)
		brcc	cvde_L1k			; Loop until C set
		subi	YL,low(-1000)		; Correct last subtraction
		sbci	YH,high(-1000)
		mov		res1,rgc
;
; Process 100's digit
;
cvde_L100:
		inc		res1
		subi	YL,low(100)
		sbci	YH,high(100)
		brcc	cvde_L100			; Loop until C set
		subi	YL,low(-100)		; Correct last subtraction
		or		res1,rgb
		ldi		rgb,(256-16)
;
; Process 10's digit
;
cvde_L10:
		subi	rgb,(-16)
		subi	YL,10
		brcc	cvde_L10			; Loop until C set
		subi	YL,-10				; Correct last subtraction
		mov		res0,rgb
		or		res0,YL
		ret
;
;
; Convert rga to hex digit and set C, else clear C if character
; not an ascii hexadecimal digit. On error, return with character in rga
;
; Registers:	rga
;
hexdg:
		mov		asav,rga	; Save char
		rcall	case		; Fold to UC
		subi	rga,'0'		; rga < '0'?
		brcs	hexdg1		;	Yes, exit
		cpi		rga,LF		; rga from 0...9?
		brcs	hexdg2		;	Yes
		subi	rga,7		; Dump funny chars
		cpi		rga,LF		; Char from 9...A?
		brcs	hexdg1
		cpi		rga,0x10	; Char above F?
		brcc	hexdg1		;	Yes
hexdg2:
		ret					; Normal exit, C=1
hexdg1:
		mov		rga,asav	; Restore char
		clc
		ret
;
; Convert packed BCD number (2 digits) into two ascii digits
;
cv2asc:
		andi	rga, 0x0f			; Mask off higher nibble
		ldi		rgv, 0x30 			; Add ascii '0' to convert
		add		rga, rgv			; Convert to ascii in rga
		st		Y+,rga				; Update LT_hourb buffer
		ret
;		
;
;
; --- Decimal Adjust after Addition ---
;
; rga has result of 2 packed packed BCD digits after addition on entry.
; DAA routine adjusts rga for proper BCD representation. C flag has result
; of carry-out to allow for multiple precision additions.
;
; Registers: rga
; Entry:	rga = result of prior packed bcd addition
; Exit:		rga = decimal adjusted result for 2 digits
;  
daa:
		push	rmp
		ldi		rmp,0x66	; Adjustment for both BCD digits
		add		rga,rmp		; Add adjustment to BCD pair
		brcc	danoc		; C=0
		andi	rmp,0x0f	; C=1, high nibble adjustment removed
danoc:
		brhc	danoh		; H=0
		andi	rmp,0xf0	; H=1, low nibble adjustment removed
danoh:
		sub		rga,rmp		; Final adjustment
		pop		rmp
		ret
;
; DAS decimal adjust subtraction of two packed BCD numbers
; Based on Intel IA-32 instruction code for DAS
;
das:
		push	rgb					; Save working registers
		push	rgc
;
; Save entry data
;
		mov		rgb,rga				; copy of rga for lo nibble testing
		mov		rgc,rga				; rgc has rga for hi nibble testing
;
; Test low nibble. If ((rga & 0x0f) > 9)) --> daa_adjlo
;
das_testlo:
		andi	rgb,0x0f
		cpi		rgb,9+1
		brcc	das_adjlo		; Low nibble is > 9
		rjmp	das_testhi
;
das_adjlo:
		ldi		rgb,0x06		; Decimal adjust for low nibble
		sub		rga,rgb			; Add 0x06 to entry_rga
;
; Test high nibble. If ((entry_rga > 0x99) or (entry_C = 1)) --> daa_adjhi
;
das_testhi:
		cpi		rgc,0x99+1		; Test entry_rga
		brcc	das_adjhi		; entry_rga > 0x99
		rjmp	das_x
;
das_adjhi:
		ldi		rgb,0x60		; Decimal adjust for high nibble
		sub		rga,rgb			; Add 0x60 to entry_rga
;
das_x:
;
		pop		rgc
		pop		rgb
		ret
;
;
;
;###########################################################################
;
;
; --- Line input and initialization routines ---	(OK)
;
;  A 20 byte line input buffer is supported. The buffer is initially
;  cleared to zeroes, and pointed to by XH:XL. COUNT maintains a
;  count of characters entered. Entry is terminated by <'CR'>, <^X> 
;  erases current line and starts over, and <BS> or <DEL> erases
;  previous character. XH:XL is reserved for use as LINBUF pointer
;  to allow multiple GCHR calls.
;
;	Registers used:
;	rmp, rga, rgb, rgc, X
;
glin:
		rcall	inzln			; Zero the line buffer and count register
glin1:
		rcall	ci				; Get a character
		cpi		rga,CR			; Test if <CR>
		brne	glin2			;	No, look for next special key
		ldxptr	linbuf			;	Yes, reset linbuf pointer
		sbr		flaga,(1<<crf)	; And set CR flag
		ret
;
; Look for a ^X key, if so do a line delete
;
glin2:
		cpi		rga,ctlX		; Test if <^X>
		brne	glin3			;	No, look for next special key
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin
glin2a:
		call	bksp			; Move cursor back one space
		dec		rgb
		brne	glin2a			; back to start
		rjmp	glin			; Restart
;
; Look for a BS key, if so do a delete character at cursor
;
glin3:
		cpi		rga,BS			; Test if backspace
		brne	glin3b			;	No, look for next special key
glin3a:
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin1			;	Yes, fetch another character
		dec		rgb
		mov		count,rgb
		call	bksp			; Move cursor back one space
		ldi		rmp,0			; Backup pointer and insert 0 
		st		-X,rmp
		rjmp	glin1
;
; Look for a DEL key, if so do a backspace
;
glin3b:
		cpi		rga,DEL			; Test if DEL
		brne	glin5			;	No, look for next special key
		rjmp	glin3a
;
; Look for a Tab key, if so expand tab to spaces
;
glin5:
		cpi		rga,HT			; Test if tab
		brne	glin6			;	No,  look for next special key
		ldi		rgc,7			; Temp counter
		ldi		rga,SP			; Space character
glin5a:
		rcall	ldlin
		dec		rgc
		brne	glin5a
		rjmp	glin1
;
; Look for a Escape key, if so set escf
;
glin6:
		cpi		rga,ESC			; Test if esc
		brne	glin7			;	No, look for other control key
		sbr		flaga,(1<<escf)	; Set esc flag
		ret
;
; Look for other control key. Check for ^S key and do return if
; found.
;
glin7:
		cpi		rga,ctlS		; ^S?
		brne	glin7a			;	No, continue
		ret						;	Yes, exit
glin7a:
		rcall	fctl			; Test for other control key
		sbrs	flaga,kyf
		rjmp	glin8			;	kyf = 0
		rjmp	glin1			; Ignore other control keys
;
; Arrive here is valid key entry
;
glin8:
		rcall	ldlin			; Load the input buffer and show
		rjmp	glin1
;
; Load character in rga to LINBUF, update pointer and character counter		(OK)
;
ldlin:
		mov		rgb,count		; Get current count
		cpi		rgb,linsz		; End of buffer?
		brne	ldlin1			;	No
		ret						;	Yes, exit
ldlin1:
		inc		rgb
		mov		count,rgb		; Update count
		st		X+,rga			; Store entered key to buffer
		rcall	co				; Show it
		ret
;
;  Get linbuf character, increment XH:XL pointer and set C if
;  not 'CR', else clear C, rga = 0. 
;
gchr:
		ld		rga,X+			; Get character from line buffer, advance pointer
		cpi		rga,0			; Test for 0
		brne	gchr1			;	rga >= 0, means ascii printable character
		clc
		ret
gchr1:
		sec
		ret
;
; Clear input line buffer	(OK)
;
inzln:
		clr		rmp				; Fill byte
		clr		count			; Initialize count to 0
		ldi		rgb,linsz		; Buffer size
		ldxptr	linbuf			; Point to line buffer
inzln1:
		st		X+,rmp
		dec		rgb
		brne	inzln1
		ldxptr	linbuf			; Point to line buffer
		cbr		flaga,(1<<crf)|(1<<escf)	; Clear exit flags
		ret
;
;  Test rga for control key, 0...19H, 7FH..FFH, and set KYF		(OK)
;  if true, else clear KYF. rga preserved
;
fctl:
		sbr		flaga,(1<<kyf)
		cpi		rga,SP				; rga < SP?
		brcs	fctl1				;	Yes
		cpi		rga,DEL				; rga >= SP?
		brcc	fctl1				;	No
		cbr		flaga,(1<<kyf)		; Clear kyf
fctl1:
		ret
;
;
; --- Data Buffer control and Math routines ---
;
; Multiply YH:YL by 10, called by gdbn ascii to binary converter routine		(OK)
; YH:YL = YH:YL * 10, C = 0 if ok, C = 1 on error
; Registers:	rga, rgb, rgc, YH, YL
;
dex10:
		call	dex2		; YH:YL * 2
		brcs	dexx		; Error exit, overflow and C=1
		push	YH			; Copy YH:YL to rgb:rgc
		pop		rgb
		push	YL
		pop		rgc
;
		rcall	dex2		; * 4
		brcs	dexx
		rcall	dex2		; * 8
		brcs	dexx
		call	adebc		; YH:YL = YH:YL + rgb:rgc
dexx:
		ret
;
; YH:YL: = YH:YL * 2
; 
dex2:
		clc
		rol		YL
		rol		YH
		ret
;
; YH:YL = YH:YL + rgb:rgc, C = 0 if ok else C = 1 on overflow
;
adebc:
		add		YL,rgc
		adc		YH,rgb
		ret
;
; Decrement XH:XL pointer by 1		(OK)
;
decxptr:
		push	rmp
		ldi		rmp,-1
		add		XL,rmp
		adc		XH,rmp
		pop		rmp
		ret
;
; "div8u" - 8/8 Bit Unsided Division				(OK)
;
; This subroutine divides the two register variables "rga" (dividend) and 
; "rgb" (divisor). The result is placed in "rga" and the remainder in "rgb".
;  
; High registers used:	4 (rga,rgb,rgc,rgv)
;
;                                  
; Register Variables:
;	rgc	remainder
;	rga	dividend & result
;	rgb divisor
;	rgv	loop counter
;
; Entry:	(rga) = dividend
;			(rgb) = divisor
; Exit:		(rga) = integer part of quotient
;			(rgb) = integer remainder 
;                                    
div8u:	
		push	rgc
		push	rgv
		sub		rgc,rgc			; clear remainder and carry
        ldi		rgv,9			; init loop counter
d8u_1:	rol		rga				; shift left dividend
        dec		rgv				; decrement counter
        brne	d8u_2			; if done
		mov		rgb,rgc			; move remainder to rgb
		pop		rgv
		pop		rgc
        ret						;    return
;
d8u_2:	rol		rgc				; shift dividend into remainder
        sub		rgc,rgb			; remainder = remainder - divisor
        brcc	d8u_3			; if result negative
        add		rgc,rgb			;    restore remainder
        clc						;    clear carry to be shifted into result
        rjmp	d8u_1			; else
d8u_3:	sec						;    set carry to be shifted into result
        rjmp	d8u_1
;
;
;###########################################################################
;
;
; --- General Screen Routines ---
;
slina:
		call	pxy
		.db		3,10
		rjmp	slin
slinb:
		call	pxy				; Cursor to messagess line B
		.db		14,10			; Cursor to status line A
		rjmp	slin
slinc:
		call	pxy				; Cursor to messagess line C
		.db		15,10
		rjmp	slin
slind:
		call	pxy				; Cursor to messagess line C
		.db		18,2
;
slin:
		call	ceol			; Clear the rest of line
		ret
;
; Clear display screen
;
clean:
		call	pxy				; Wipe screen clean
		.db		2,1
		call	clin			; Clear lines
		.dw		15
		call	pxy
		.db		2,1
		ret
;
; Clear lines specified immediately following rcall to clin
;
clin:
	pop		ZH				; Point to data word
	pop		ZL
	lsl		ZL				; Z*2 for word address
	rol		ZH
	andi	ZL,0xfe			; Fetch lower byte of word
	lpm		rgb,Z+			; Get word
	adiw	Z,1
	lsr		ZH
	ror		ZL				; Z/2
	push	ZL				; Return address to stack
	push	ZH
clin1:
	call	ceol			; Clear lines
	call	cdown			; Move cursor down 1 row
	dec		rgb
	brne	clin1
	ret
;
; Display 'Enter:' prompt
;
enter:
		call	slinb
		call	ceol
		ldzptr	entm
		call	pptr
		ret
;
sak:
		call	slind
		ldzptr	sakm			; Strike any key
		call	pptr
		call	ci				; Wait for any key
		ret
;
; --- Video routines ---
;
;
; --- Low level video drivers ---
;
; Register rga used to pass data to console output routine
;
; Print rga data as two hexadecimal digits.			(OK)
;
pahex:
	push	rga
	swap	rga				; Show MSD nibble first
	rcall	pahex1
	pop		rga
pahex1:
	andi	rga, 0x0f		; Mask off higher nibble
	ldi		rgv, 0x30 		; Add ascii '0' to convert
	add		rga, rgv		; Convert to ascii
	cpi		rga, 0x3a		; Check if > 9
	brcs	pahex2			;  No, it is 0 ... 9
	ldi		rgv, 0x07		;  Yes, convert to A ... F
	add		rga, rgv
pahex2:
	call	co
	ret
;
; Print rga contents as decimal (0...255). Leading			(OK)
; zero suppression is provided only on the 100's
; digit, so at least two digits are always printed.
;
; Registers rga, rgb not saved
;
pdec:
	ldi		rgb,100			; Get 100's digit
	call	div8u
	tst		rga				; Do leading zero suppression
	breq	pdec1
	call	pnum
pdec1:
	ldi		rga,10			; Get 10's digit
	xchreg	rga,rgb
	call	div8u			; rgb has units
	call	pnum
	xchreg	rga,rgb
pnum:
	ori		rga,0x30		; Ascii "0"
	call	co				; Show ascii decimal
	ret
;
;###########################################################################
;
;
; Scan for console character and return with character if any,
; else return with rga = 0. Data is available in sinb when putcnt
; is greater than getcnt.
;
; Registers:
;	rmp, rga, rgb, XHL (preserved across routine)
;
; Exit:	rga <-- character, if any, else 0
;
getc:
		lds		rmp,getcnt
		lds		rga,putcnt
		cp		rga,rmp			; Compare getcnt to putcnt
		breq	getc2			; Same, no new data
getc0:
		push	XH
		push	XL				; Save X registers
		ldxptr	sinb			; sinb base address
		lds		rmp,getcnt		; Get current getcnt offset
		clr		rga
		add		XL,rmp			; Add getcnt offset to sinb pointer
		adc		XH,rga			; Update pointer (16 bit addition)
;		
		ld		rga,X			; rga <-- @XHL
		pop		XL
		pop		XH				; Restore X registers
		inc		rmp				; Increment getcnt
		cpi		rmp,sblen		; Past end of buffer?
		brne	getc1
		clr		rmp				;	Yes, reset getcnt
getc1:
		sts		getcnt,rmp		; Update getcnt to next buffer location
		ret
getc2:
		clr		rga
		ret
;
; Wait for a received data byte, return received data in rga.
;
ci:	
		lds		rmp,getcnt
		lds		rga,putcnt
		cp		rga,rmp			; Compare getcnt to putcnt
		breq	ci				; No incoming data
		rjmp	getc0			; Get new data
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
; Print CR and LFs	(OK)
;
crllf:
	rcall	crlf			; Two CRLF
crlf:
	push	rga
	ldi		rga,CR			; Carriage return
	call	co
	ldi		rga,LF			; Linefeed
	call	co
	rjmp	cco
;
; Print spaces	(OK)
;
dblsp:
	call	space
space:
	push	rga
	ldi		rga,SP			; Space
cco:
	call	co
	pop		rga
	ret
;
; Print comma	(OK)
;
prcma:
	push	rga
	ldi		rga,cma
	rjmp	cco
;
; Print delete character at cursor	(OK)
;
bksp:
	push	rga
	call	cbak			; Delete character at cursor
	call	ceol			; Clear cursor to end of line
	pop		rga
	ret
;
; Print message string, ^Z terminated. Routine is called with		(OK)
; code address of string loaded in ZH:ZL.
;
pptr:
	push	rga
pptr1:
	lpm		rga,Z+			; String byte to rga, Z+
	cpi		rga,ctlZ		; byte ^Z?
	brne	pptr2			; Print if not ^Z
	pop		rga
	ret
pptr2:
	cpi		rga,NULL		; Skip any nulls in string
	breq	pptr1
	rcall	co
	rjmp	pptr1
;
;
; --- Video and Cursor control routines ---
;
; Clear screen	(OK)
;
clrscn:
	push	zh
	push	zl
	ldzptr	scrn		; Home cursor
	call	pptr
	ldzptr	clrs		; Clear entire screen
	rjmp	video
;
; --- Move cursor down ---
;
cdown:
	push	zh
	push	zl
	ldzptr	cudn		; Cursor down one row
	rjmp	video
;
; --- Clear to end of line ---
;
ceol:
	push	zh
	push	zl
	ldzptr	eol			; Clear to end of line
	rjmp	video
;
; --- Cursor back one column ---
;
cbak:
	push	zh
	push	zl
	ldzptr	cubk			; Cursor back 1 column
	rjmp	video
;
; --- Highlight on ---
;
vhi:
	push	zh
	push	zl
	ldzptr	hi			; Highlight on
	rjmp	video
;
; --- Normal ---
;
vlo:
	push	zh
	push	zl
	ldzptr	lo			; Normal - attributes off
	rjmp	video
;
; --- Reverse ---	(OK)
;
vrev:
	push	zh
	push	zl
	ldzptr	rev			; Reverse on
video:
	rcall	pptr
	pop		zl
	pop		zh
	ret
;
; --- Video position cursor sequences ---
; Lead-in sequence
;
vpxy1:
	push	zh
	push	zl
	ldzptr	pxy1			; Lead-in sequence
	rjmp	video
;
; Middle sequence
;
vpxy2:
	push	zh
	push	zl
	ldzptr	pxy2			; Middle sequence
	rjmp	video
;
; End sequence
;
vpxy3:
	push	zh
	push	zl
	ldzptr	pxy3			; Trailing sequence
	rjmp	video
;
; --- Save cursor position ---
;
vscp:
	push	zh
	push	zl
	ldzptr	scp			; Save cursor position
	rjmp	video
;
; --- Restore cursor position ---
;
vrcp:
	push	zh
	push	zl
	ldzptr	rcp					; Restore cursor position
	rjmp	video
;
; --- Position cursor at row, column immediately following rcall to pxy ---
;
; Row & column values must be given as ascii decimal.			(OK)
;
pxy:
	call	vpxy1			; Lead-in sequence
	pop		ZH				; Point to string start address
	pop		ZL
	clc
	rol		ZL				; 16 bit multiply by 2 for word address
	rol		ZH
;
	lpm		rga,Z+			; Pick up row value
	call	pdec			; Print it and ..
	call	vpxy2			; Middle sequence		+++++ Uses Z pointer, must save Z +++
	lpm		rga,Z+			; Pick up column value
	call	pdec			; Print it and ..
	call	vpxy3			; End sequence
;
	clc
	ror		ZH
	ror		ZL
	push	ZL				; Return to caller
	push	ZH
	ret
;
; Position cursor at (YH)-->row, (YL)-->col		(OK)
;
gotoxy:
	call	vpxy1			; Send lead-in string
	mov		rga,YH			; Get row value
	call	pdec			; Send row
	call	vpxy2			; Send middle string
	mov		rga,YL			; Get col value
	call	pdec			; Send col
	call	vpxy3			; Send trailing string
	ret
;
;
; --- Message strings data area ---
;
; Terminal control sequences
;
cudn:	.db	ESC,"[B",ctlZ		; Move cursor down
cubk:	.db	ESC,"[D",ctlZ		; Cursor back one column
scrn:	.db	ESC,"[H",ctlZ		; Home cursor
eos:	.db	ESC,"[0J",ctlZ		; Clear from cursor to end of screen
clrs:	.db	ESC,"[J",ctlZ		; Clear entire screen
eol:	.db	ESC,"[K",ctlZ		; Erase to end of line
hi:		.db	ESC,"[1m",ctlZ		; Highlight on
lo:		.db	ESC,"[m",ctlZ		; Normal - attributes off
rev:	.db	ESC,"[7m",ctlZ		; Reverse on
pxy1:	.db	ESC,"[",ctlZ		; Lead-in sequence
pxy2:	.db	";",ctlZ			; Middle sequence
pxy3:	.db	"H",ctlZ			; Trailing sequence
dlc:	.db	ESC,"[1M",ctlZ		; Delete line at cursor
scp:	.db	ESC,"7",ctlZ		; Save cursor position
rcp:	.db	ESC,"8",ctlZ		; Restore cursor position
;
;
;
; --- Message strings data area ---
;
;
;###########################################################################
;
.include	"eeprom_module.asm"
;
;###########################################################################
;
;
;******************************************
; Store and load messages to EEPROM
;******************************************
;
; Registers: rmp, rga, Y, X, Z
;
str_msg:
;
		ldi		rmp,0xff
		out		EEARL,rmp					; EEPROM start address = -1 as
		out		EEARH,rmp					;  EEAR is pre-decremented
		ldi		rmp,high(msg_end-msg_st)		; Buffer size byte counter
		mov		XH,rmp
		ldi		rmp,low(msg_end-msg_st)		; Buffer size byte counter
		mov		XL,rmp						; X = bytes to move
;
		ldi		ZH,high(msg0)				; ton0 <-- Z, source
		ldi		ZL,low(msg0)
;
str_msg1:
		ld		rmp,Z+				; Get source byte from data buffer
		rcall	EEWrSeq				; Write to EEPROM
;
		sbiw	XH:XL,1				; Decrement XHL counter
		brne	str_msg1			;	No, loop for next write
;
		ret
;
; Load EEPROM messages image to message buffer space.
; If EEPROM erased (contains all 0xff), use default 'SP'
; character instead 0f 0xff.
;
ld_msg:
		ldi		rmp,0xff
		out		EEARL,rmp				; EEPROM start address = -1
		out		EEARH,rmp
		ldi		rmp,high(msg_end-msg_st)	; Buffer size byte counter
		mov		XH,rmp
		ldi		rmp,low(msg_end-msg_st)	; Buffer size byte counter
		mov		XL,rmp					; X = bytes to move
;
		ldi		ZH,high(msg0)				; ton0 <-- Z, source
		ldi		ZL,low(msg0)
;
ld_msg1:
		rcall	EERdSeq					; Get source byte from EEPROM
		cpi		rmp,0xff				; Check for 0xff, EEPROM  erased?
		brne	ld_msg2					;	No, data valid
		ldi		rmp,SP					;	Yes, load a space char instead
ld_msg2:
		st		Z+,rmp					; Write to message buffer
		sbiw	XH:XL,1					; Decrement XHL counter
		brne	ld_msg1					;	No, loop for next read
;
		ret
;
;
; --- Screen displays ---
;
scrnm:
			.db		"  ======<<< m328-LCD Annunciator >>>======",cr,lf,ctlZ
linem:
			.db		"--------------------------------------------",ctlZ
;
entm:		.db		"Enter: ",ctlZ
err1:		.db		"  *** Unrecognized Input! ***",ctlZ
sakm:		.db		"  Strike any key --> ",ctlZ
;
blankm:		.db		"                                  ",ctlZ		; Erase line on LCD
;
storem:		.db		"^S store in EEPROM, Y to save, <CR> to exit",ctlZ 
savEEms:	.db		"*** Saving to EEPROM ... ",ctlZ
;
.exit
;
;
; --- End of source code ---
;
;

