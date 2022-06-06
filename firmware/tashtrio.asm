;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashTrio: ADB Keyboard, Mouse, and Modem
;;;
;


;;; License ;;;

;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <https://www.gnu.org/licenses/>.


;;; Connections ;;;

;;;                                                                 ;;;
;                               .--------.                            ;
;                       Supply -|01 \/ 08|- Ground                    ;
;              ADB <-->    RA5 -|02    07|- RA0    ---> UART TX       ;
;       PS/2 Mouse <-->    RA4 -|03    06|- RA1    <--- UART RX       ;
;    PS/2 Keyboard --->    RA3 -|04    05|- RA2    <--> PS/2 Clock    ;
;                               '--------'                            ;
;;;                                                                 ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1840, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1840.inc
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

;WARNING: do NOT use RA2 for ADB, the Schmitt Trigger takes too long to react
AP_APIN	equ	RA5	;Pin on PORTA where ADB bus is connected
PP_CPIN	equ	RA2	;Pin on PORTA where PS/2 clock is connected
PP_KPIN	equ	RA3	;Pin on PORTA where PS/2 keyboard is connected
PP_MPIN	equ	RA4	;Pin on PORTA where PS/2 mouse is connected

			;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when last byte sent successfully, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too

			;AU_FLAG:
AU_TXON	equ	7	;Set when the selected device wants TX events
AU_SEMD	equ	6	;Set when SRQ is enabled for modem
AU_SEMS	equ	5	;Set when SRQ is enabled for mouse
AU_SEKB	equ	4	;Set when SRQ is enabled for keyboard
AU_SRMD	equ	2	;Set when the modem is requesting service
AU_SRMS	equ	1	;Set when the mouse is requesting service
AU_SRKB	equ	0	;Set when the keyboard is requesting service

			;AK_R2H:
K2H_DEL	equ	6	;Delete
K2H_CAP	equ	5	;CapsLock
K2H_RST	equ	4	;Reset
K2H_CTL	equ	3	;Control
K2H_SHF	equ	2	;Shift
K2H_OPT	equ	1	;Option
K2H_CMD	equ	0	;Command

			;AK_R2L:
K2L_CLR	equ	7	;Clear
K2L_SLK	equ	6	;ScrollLock

			;AK_MOD:
KMD_LCT	equ	7	;Left Control
KMD_RCT	equ	6	;Right Control
KMD_LSH	equ	5	;Left Shift
KMD_RSH	equ	4	;Right Shift
KMD_LOP	equ	3	;Left Option
KMD_ROP	equ	2	;Right Option

			;AM_BTN:
AMB_2A	equ	5	;Clear when second button is down
AMB_1A	equ	4	;Clear when first button is down
AMB_2S	equ	1	;Clear when second button is down, set when up
AMB_1S	equ	0	;Clear when first button is down, set when up

			;PP_FLAG:
PP_RXKI	equ	7	;Set when keyboard byte in PP_BUF, user clears
PP_RXMI	equ	6	;Set when mouse byte in PP_BUF, user clears
PP_TXKI	equ	5	;Set to transmit PP_BUF to keyboard, interrupt clears
PP_TXMI	equ	4	;Set to transmit PP_BUF to mouse, interrupt clears
PP_TOUT	equ	3	;Set when the bus has been idle a while, user clears

			;PU_FLAG:
PU_MSIN	equ	7	;Set if the mouse has been initialized
PU_SCTL	equ	6	;Set if the nonexistent special control key was pressed
PU_APPS	equ	5	;Set if the apps key is down
PU_CAPS	equ	4	;Set if the caps lock key is pretending to be locked
PU_TXKE	equ	3	;Set when the keyboard FSA wants to know when TX done
PU_TXME	equ	2	;Set when the mouse FSA wants to know when TX done


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB state machine
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	PP_FLAG	;PS/2 flags
	PP_FSAP	;Pointer to where to resume PS/2 state machine
	PP_SR	;PS/2 shift register
	PP_BUF	;PS/2 buffer
	X6
	X5
	X4
	X3
	X2
	X1
	X0
	
	endc

	cblock	0x140	;Upper half of bank 2 registers
	
	;ADB user program registers
	AU_FLAG	;Flags
	AU_FSA	;Current device state machine (upper byte of PC)
	AU_FSAP	;Pointer to where to resume device state machine
	AU_TEMP	;Temporary holding variable between states
	
	;ADB keyboard registers
	AK_PUSH	;Push pointer for keyboard queue (0x2080-0x20BF)
	AK_POP	;Pop pointer for keyboard queue (0x2080-0x20BF)
	AK_R2H	;Register 2 high byte
	AK_R2L	;Register 2 low byte
	AK_MOD	;Keyboard modifier key state
	AK_R3H	;Register 3 high byte
	AK_R3L	;Register 3 low byte
	
	;ADB mouse registers
	AM_DYH	;Delta Y high byte
	AM_DYL	;Delta Y low byte
	AM_DXH	;Delta X high byte
	AM_DXL	;Delta X low byte
	AM_BTN	;Mouse button state
	AM_R3H	;Register 3 high byte
	AM_R3L	;Register 3 low byte
	
	;ADB modem registers
	AD_RPSH	;Push pointer for modem RX (to Mac) queue (0x2000-0x203F)
	AD_RPOP	;Pop pointer for modem RX (to Mac) queue (0x2000-0x203F)
	AD_TPSH	;Push pointer for modem TX (from Mac) queue (0x2040-0x207F)
	AD_TPOP	;Pop pointer for modem TX (from Mac) queue (0x2040-0x207F)
	AD_TLEN	;Length of modem TX (from Mac) queue (0x2040-0x207F)
	AD_R3H	;Register 3 high byte
	AD_R3L	;Register 3 low byte
	
	;PS/2 user program registers
	PU_FLAG	;Flags
	PK_FSAP	;Pointer to where to resume PS/2 keyboard state machine
	PK_RPT	;Last keycode pressed, to suppress typematic repeating
	PM_FSAP	;Pointer to where to resume PS/2 mouse state machine
	PM_TEMP	;Temporary holding variable between mouse states
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	;TODO should the ADB timer enabled-or-not be handled like the PS/2 one?
	movlp	0		;Copy the Timer0 flag into the carry bit so it
	bcf	STATUS,C	; doesn't change on us mid-stream
	btfsc	INTCON,TMR0IF	; "
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlb	7		;If the ADB pin has had a negative or positive
	movlp	0		; edge, handle it as an event for the ADB state
	btfsc	IOCAF,AP_APIN	; machine
	call	IntAdbEdge	; "
	movlb	0		;If the Timer2 flag is set, handle it as an
	movlp	0		; event for the PS/2 state machine (it returns
	btfsc	PIR1,TMR2IF	; immediately if the timer is not enabled)
	call	IntPs2Timer	; "
	movlb	7		;If the PS/2 clock pin has had a negative edge,
	movlp	0		; handle it as an event for the PS/2 state
	btfsc	IOCAF,PP_CPIN	; machine
	call	IntPs2Edge	; "
	movlb	0		;If the UART receiver has a byte, handle it
	movlp	0		; "
	btfsc	PIR1,RCIF	; "
	call	IntRx		; "
	movlb	0		;If the UART transmitter wants a byte, handle
	btfsc	PIR1,TXIF	; it
	call	IntTx		; "
	retfie

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << AP_APIN	;Toggle the edge that the IOC interrupt catches
	xorwf	IOCAN,F		; "
	xorwf	IOCAP,F		; "
	bcf	IOCAF,AP_APIN	;Clear the interrupt flag
	btfsc	IOCAN,AP_APIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	clrf	AP_FSAP		;Reset the ADB state machine
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return

IntPs2Timer
	movlb	1		;If the Timer2 interrupt isn't enabled, return
	btfss	PIE1,TMR2IE	; "
	return			; "
	bcf	PIE1,TMR2IE	;Disable the Timer2 interrupt
	movlb	0		; "
	bcf	PIR1,TMR2IF	;Clear the Timer2 interrupt flag
	movf	PP_FSAP,W	;Resume the PS/2 state machine
	movlp	high Ps2Fsa	; "
	callw			; "
	movwf	PP_FSAP		;On return, save the address returned in W
	return

IntPs2Edge
	bcf	IOCAF,PP_CPIN	;Clear the IOC interrupt flag
	movlb	1		;If we're here because we just pulled the clock
	btfss	TRISA,PP_CPIN	; pin low, reset the state machine
	clrf	PP_FSAP		; "
	movlb	0		;If Timer2 has overflowed, reset the state
	btfsc	PIR1,TMR2IF	; machine and set the timeout flag so the user
	clrf	PP_FSAP		; program knows about it
	btfsc	PIR1,TMR2IF	; "
	bsf	PP_FLAG,PP_TOUT	; "
	bcf	PIR1,TMR2IF	;Clear Timer2 and its interrupt
	clrf	TMR2		; "
	movf	PP_FSAP,W	;Resume the PS/2 state machine
	movlp	high Ps2Fsa	; "
	callw			; "
	movwf	PP_FSAP		;On return, save the address returned in W
	return

IntRx
	movlb	2
	movf	AD_RPSH,W	;Load the RX push pointer into FSR0
	movwf	FSR0L		; "
	incf	AD_RPSH,W	;If the queue is full, jump ahead to dump the
	andlw	B'00111111'	; received byte
	xorwf	AD_RPOP,W	; "
	btfsc	STATUS,Z	; "
	bra	IntRx0		; "
	incf	AD_RPSH,F	;Increment and wrap the RX push pointer
	bcf	AD_RPSH,6	; "
	movlb	3		;Push the contents of the receiver register
	movf	RCREG,W		; into the RX queue
	movwf	INDF0		; "
	movlb	2		; "
	bsf	AU_FLAG,AU_SRMD	;Raise the flag so the modem requests service
	return
IntRx0	movlb	3		;Dump the received byte, we have no space for
	movf	RCREG,W		; it
	return

IntTx
	movlb	2
	movf	AD_TPOP,W	;If the queue is empty, disable the TX
	xorwf	AD_TPSH,W	; interrupt and return
	btfss	STATUS,Z	; "
	bra	IntTx0		; "
	movlb	1		; "
	bcf	PIE1,TXIE	; "
	return			; "
IntTx0	movf	AD_TPOP,W	;Load the TX pop pointer into FSR0
	movwf	FSR0L		; "
	incf	AD_TPOP,F	;Increment and wrap the pointer
	bsf	AD_TPOP,6	; "
	bcf	AD_TPOP,7	; "
	decf	AD_TLEN,F	;Decrement the queue length
	movf	INDF0,W		;Pop the top byte off the TX queue and load it
	movlb	3		; for transmission
	movwf	TXREG		; "
	return


;;; Mainline ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON
	
	banksel	RCSTA		;UART async mode, 2400 kHz
	movlw	B'01001000'
	movwf	BAUDCON
	movlw	13
	movwf	SPBRGH
	movlw	4
	movwf	SPBRGL
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10010000'
	movwf	RCSTA
		
	banksel	IOCAN		;ADB and PS/2 clock set IOCAF on negative edge
	movlw	(1 << AP_APIN) | (1 << PP_CPIN)
	movwf	IOCAN
	
	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:32 prescaler,
	movlw	B'01010100'	; thus ticking every 4 us; weak pull-ups on
	movwf	OPTION_REG
	
	banksel	T1CON		;Timer1 ticks once per instruction cycle
	movlw	B'00000001'
	movwf	T1CON
	
	banksel	T2CON		;Timer2 overflows after 3.072 ms
	movlw	B'00101110'
	movwf	T2CON
	
	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA
	
	banksel	LATA		;Ready to pull PORTA lines low when outputs
	clrf	LATA
	
	banksel	TRISA		;TX out, rest are open-collector outputs,
	movlw	B'00111110'	; currently off
	movwf	TRISA
	
	banksel	PIE1		;RX peripheral interrupt on
	movlw	B'00100000'
	movwf	PIE1
	
	clrf	AP_FLAG		;Set initial values of key globals
	bsf	AP_FLAG,AP_RST
	clrf	AP_FSAP
	clrf	PP_FLAG
	clrf	PP_FSAP
	movlb	2
	clrf	AU_FLAG
	movlw	0x80
	movwf	AK_PUSH
	movwf	AK_POP
	movlw	0x40
	movwf	AD_TPSH
	movwf	AD_TPOP
	clrf	AD_TLEN
	clrf	AM_DYH
	clrf	AM_DYL
	clrf	AM_DXH
	clrf	AM_DXL
	movlw	0xFF
	movwf	AM_BTN
	movwf	PK_RPT
	clrf	AD_RPSH
	clrf	AD_RPOP
	clrf	PU_FLAG
	clrf	PK_FSAP
	clrf	PM_FSAP
	
	movlw	0x20		;Set up FSRs to point more or less permanently
	movwf	FSR0H		; to linear memory
	movwf	FSR1H
	
	movlw	B'11001000'	;On-change interrupt, peripheral interrupts so
	movwf	INTCON		; Timer2 works, and interrupt subsystem on

Main
	call	SvcAdb		;Service the ADB user program
	call	SvcPs2		;Service the PS/2 user program
	bra	Main		;Loop

SvcAdb
	movlb	2		;If the reset flag is set, handle it
	btfsc	AP_FLAG,AP_RST	; "
	call	SvcAdbReset	; "
	btfsc	AP_FLAG,AP_RXCI	;If the received-command flag is set, handle it
	call	SvcAdbCommand	; "
	btfss	AP_FLAG,AP_RXDI	;If either the received-data or collision flag
	btfsc	AP_FLAG,AP_COL	; is set, handle it
	call	SvcAdbData	; "
	btfsc	AP_FLAG,AP_DONE	;If the transmission-done flag is set, handle
	call	SvcAdbData	; it
	btfss	AU_FLAG,AU_TXON	;If the transmit-buffer-ready flag is set and
	return			; the state machine wishes to be notified of
	btfss	AP_FLAG,AP_TXI	; this, handle it
	call	SvcAdbData	; "
	return			; "

SvcAdbReset
	movlw	0x62		;Keyboard register 3 puts it at address 0x2,
	movwf	AK_R3H		; with SRQ enabled and handler ID 2 (Extended
	movlw	0x02		; Keyboard)
	movwf	AK_R3L		; "
	movlw	B'11111111'	;Keyboard register 2 has all modifier keys up
	movwf	AK_R2H		; and reserved bits set to 1
	movlw	B'11111111'	; "
	movwf	AK_R2L		; "
	movlw	B'11111100'	; "
	movwf	AK_MOD		; "
	movlw	0x63		;Mouse register 3 puts it at address 0x3, with
	movwf	AM_R3H		; SRQ enabled and handler ID 1 (100 cpi mouse)
	movlw	0x01		; "
	movwf	AM_R3L		; "
	movlw	0x65		;Modem register 3 puts it at address 0x5, with
	movwf	AD_R3H		; SRQ enabled and handler ID 0x36
	movlw	0x36		; "
	movwf	AD_R3L		; "
	movlw	B'01110000'	;Set the SRQ enable bits for all peripherals
	iorwf	AU_FLAG,F	; "
	bcf	AP_FLAG,AP_RST	;Clear the reset flag, if it was set
	return

SvcAdbCommand
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	movf	AP_BUF,W	;If the low four bits of the command are zero,
	andlw	B'00001111'	; this is a SendReset command and should be
	btfsc	STATUS,Z	; treated the same as a reset pulse
	bra	SvcAdbReset	; "
	movlp	0		;Set PCLATH to 0 in case no device matches
	swapf	AK_R3H,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the keyboard, load the high byte
	andlw	B'11110000'	; of the keyboard's state machine into PCLATH
	btfsc	STATUS,Z	; "
	movlp	high AKFsa	; "
	swapf	AM_R3H,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the mouse, load the high byte of
	andlw	B'11110000'	; the mouse's state machine into PCLATH
	btfsc	STATUS,Z	; "
	movlp	high AMFsa	; "
	swapf	AD_R3H,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the modem, load the high byte of
	andlw	B'11110000'	; the modem's state machine into PCLATH
	btfsc	STATUS,Z	; "
	movlp	high ADFsa	; "
	movf	PCLATH,W	;Save the matched state machine for later use
	movwf	AU_FSA		; "
	btfsc	STATUS,Z	;If none of the devices had an address match,
	bra	SvcAdC0		; don't enter a state machine
	movlw	0		;Call into the initial state of the selected
	callw			; device's state machine
	movwf	AU_FSAP		;On returning, save the address returned in W
SvcAdC0	bcf	AP_FLAG,AP_SRQ	;If any device is calling for service, set the
	swapf	AU_FLAG,W	; ADB peripheral's SRQ flag
	andwf	AU_FLAG,W	; "
	btfss	STATUS,Z	; "
	bsf	AP_FLAG,AP_SRQ	; "
	btfsc	AD_TLEN,5	;If the TX queue is half full, raise SRQ to
	bsf	AP_FLAG,AP_SRQ	; try and stall for time to empty it...
	movlp	0		;Return PCLATH to 0 for normal operation
	return

SvcAdbData
	movf	AU_FSA,W	;If the last command byte selected a state
	btfsc	STATUS,Z	; machine, resume it at the point from which
	bra	SvcAdD0		; it was last left; if not, don't
	movwf	PCLATH		; "
	movf	AU_FSAP,W	; "
	callw			; "
	movwf	AU_FSAP		;On returning, save the address returned in W
	movlp	0		;Return PCLATH to 0 for normal operation
SvcAdD0	bcf	AP_FLAG,AP_RXDI	;Clear the flags that could have brought us
	bcf	AP_FLAG,AP_COL	; here
	bcf	AP_FLAG,AP_DONE	; "
	return

SvcPs2
	;TODO clear TXKE/TXME on timeout too?
	movlb	2		;If the timer has expired on the bus, reset
	btfsc	PP_FLAG,PP_TOUT	; both state machines
	clrf	PK_FSAP		; "
	btfsc	PP_FLAG,PP_TOUT	; "
	clrf	PM_FSAP		; "
	bcf	PP_FLAG,PP_TOUT	; "
	btfsc	PP_FLAG,PP_RXKI	;If a byte has been received for the keyboard,
	bra	SvcPs2K		; jump into its state machine
	btfsc	PP_FLAG,PP_RXMI	;If a byte has been received for the mouse,
	bra	SvcPs2M		; jump into its state machine
	btfss	PP_FLAG,PP_TXKI	;If ready to send a byte for the keyboard and
	btfss	PU_FLAG,PU_TXKE	; the state machine has set itself to receive
	bra	$+2		; such events, jump into its state machine
	bra	SvcPs2L		; "
	btfss	PP_FLAG,PP_TXMI	;If ready to send a byte for the mouse and the
	btfss	PU_FLAG,PU_TXME	; state machine has set itself to receive such
	return			; events, jump into its state machine
	bra	SvcPs2N		; "
SvcPs2K	bcf	PP_FLAG,PP_RXKI	;Clear the receive flag
SvcPs2L	movf	PK_FSAP,W	;Resume keyboard state machine
	movlp	high PKFsa	; "
	callw			; "
	movwf	PK_FSAP		;On return, save the address returned in W
	movlp	0		;Return PCLATH to 0 for normal operation
	return
SvcPs2M	bcf	PP_FLAG,PP_RXMI	;Clear the receive flag
SvcPs2N	movf	PM_FSAP,W	;Resume mouse state machine
	movlp	high PMFsa	; "
	callw			; "
	movwf	PM_FSAP		;On return, save the address returned in W
	movlp	0		;Return PCLATH to 0 for normal operation
	return


;;; Lookup Tables ;;;

PKLut	org	0x800

	retlw	0xFF
	retlw	0x65
	retlw	0xFF
	retlw	0x60
	retlw	0x63
	retlw	0x7A
	retlw	0x78
	retlw	0x6F
	retlw	0xFF
	retlw	0x6D
	retlw	0x64
	retlw	0x61
	retlw	0x76
	retlw	0x30
	retlw	0x32
	retlw	0xFF
	retlw	0xFF
	retlw	0x3A
	retlw	0x38
	retlw	0xFF
	retlw	0x36
	retlw	0x0C
	retlw	0x12
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x06
	retlw	0x01
	retlw	0x00
	retlw	0x0D
	retlw	0x13
	retlw	0xFF
	retlw	0xFF
	retlw	0x08
	retlw	0x07
	retlw	0x02
	retlw	0x0E
	retlw	0x15
	retlw	0x14
	retlw	0xFF
	retlw	0xFF
	retlw	0x31
	retlw	0x09
	retlw	0x03
	retlw	0x11
	retlw	0x0F
	retlw	0x17
	retlw	0xFF
	retlw	0xFF
	retlw	0x2D
	retlw	0x0B
	retlw	0x04
	retlw	0x05
	retlw	0x10
	retlw	0x16
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x2E
	retlw	0x26
	retlw	0x20
	retlw	0x1A
	retlw	0x1C
	retlw	0xFF
	retlw	0xFF
	retlw	0x2B
	retlw	0x28
	retlw	0x22
	retlw	0x1F
	retlw	0x1D
	retlw	0x19
	retlw	0xFF
	retlw	0xFF
	retlw	0x2F
	retlw	0x2C
	retlw	0x25
	retlw	0x29
	retlw	0x23
	retlw	0x1B
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x27
	retlw	0xFF
	retlw	0x21
	retlw	0x18
	retlw	0xFF
	retlw	0xFF
	retlw	0x39
	retlw	0x7B
	retlw	0x24
	retlw	0x1E
	retlw	0xFF
	retlw	0x2A
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x33
	retlw	0xFF
	retlw	0xFF
	retlw	0x53
	retlw	0xFF
	retlw	0x56
	retlw	0x59
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x52
	retlw	0x41
	retlw	0x54
	retlw	0x57
	retlw	0x58
	retlw	0x5B
	retlw	0x35
	retlw	0x47
	retlw	0x67
	retlw	0x45
	retlw	0x55
	retlw	0x4E
	retlw	0x43
	retlw	0x5C
	retlw	0x6B
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x7C
	retlw	0xFF
	retlw	0xFF
	retlw	0x7D
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x37
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x37
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x7F
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x4B
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x4C
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x77
	retlw	0xFF
	retlw	0x3B
	retlw	0x73
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x72
	retlw	0x75
	retlw	0x3D
	retlw	0xFF
	retlw	0x3C
	retlw	0x3E
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x79
	retlw	0xFF
	retlw	0x69
	retlw	0x74
	retlw	0xFF
	retlw	0xFF


;;; State Machines ;;;

PKFsa	org	0x900

PKFsaStart
	btfsc	PU_FLAG,PU_APPS	;If the apps key is held down, incoming bytes
	bra	PKFsaAp		; are treated quite differently, skip ahead
	movf	PP_BUF,W	;If the incoming byte is 0x77, it's Num Lock,
	addlw	-119		; which is treated specially depending on the
	btfsc	STATUS,Z	; state of the nonexistent special control key
	bra	PKFSta3		; "
	addlw	-12		;If the incoming byte is 0x83, it's F7, the ONE
	btfsc	STATUS,Z	; keycode for which the MSB is set; who the
	bra	PKFSta2		; hell invented this stupid protocol?
	addlw	-93		;If the incoming byte is 0xE0, this signals an
	btfsc	STATUS,Z	; extended code, so transition accordingly and
	retlw	low PKFsaE0	; wait for the keycode it's escaping
	addlw	-1		;If the incoming byte is 0xE1, this signals an
	btfsc	STATUS,Z	; extended extended code which is only used
	retlw	low PKFsaE1	; when the user pushes the Pause (F15) key
	addlw	-15		;If the incoming byte is 0xF0, this signals the
	btfsc	STATUS,Z	; release of a key, transition accordingly and
	retlw	low PKFsaF0	; wait for the keycode being released
	btfsc	PP_BUF,7	;If it's none of those and the MSB is set, just
	retlw	low PKFsaStart	; ignore it
	movf	AK_PUSH,W	;Load the push point of the ADB keyboard queue
	movwf	FSR0L		; into FSR0
	movf	PP_BUF,W	;Convert the keycode from PS/2 to ADB
	movlp	high PKLut	; "
	callw			; "
	incf	WREG,W		;If it's 0xFF, it's invalid, so ignore it
	btfsc	STATUS,Z	; "
	retlw	low PKFsaStart	; "
	decf	WREG,W		; "
	xorlw	0x39		;If the key being pressed is caps lock and the
	btfss	STATUS,Z	; caps-lock-locked bit is set, suppress the key
	bra	PKFSta0		; press event
	btfsc	PU_FLAG,PU_CAPS	; "
	retlw	low PKFsaStart	; "
PKFSta0	xorlw	0x39		; "
PKFSta1	xorwf	PK_RPT,W	;If this keycode is the same as the one we most
	btfsc	STATUS,Z	; recently sent, suppress it
	retlw	low PKFsaStart	; "
	xorwf	PK_RPT,W	;Otherwise, keep this one as the most recent
	movwf	PK_RPT		; keycode
	movwf	INDF0		;Push the converted keycode onto the ADB
	incf	AK_PUSH,F	; keyboard queue and increment and wrap the
	bcf	AK_PUSH,6	; pointer
	bsf	AU_FLAG,AU_SRKB	;Set flag so ADB keyboard requests service
	retlw	low PKFsaStart	;Wait for the next byte
PKFSta2	movf	AK_PUSH,W	;The ADB keycode for F7 is 0x62, so load that
	movwf	FSR0L		; to push on top of the keyboard queue and
	movlw	0x62		; continue above
	bra	PKFSta1		; "
PKFSta3	movf	AK_PUSH,W	;The ADB keycode for Num Lock/Clear is 0x47,
	movwf	FSR0L		; but if the nonexistent special control key is
	movlw	0x47		; down, this is a press of Pause/F15 instead so
	btfsc	PU_FLAG,PU_SCTL	; load the correct byte to push on top of the
	movlw	0x71		; keyboard queue and continue above
	bra	PKFSta1		; "
PKFsaAp	movf	PP_BUF,W	;With the apps key held down...
	addlw	-4		;If the incoming byte is 0x04, it's F3, so set
	btfsc	STATUS,Z	; serial baud rate to 2400
	bra	PKFABR2		; "
	addlw	-1		;If the incoming byte is 0x05, it's F1, so set
	btfsc	STATUS,Z	; serial baud rate to 300
	bra	PKFABR3		; "
	addlw	-1		;If the incoming byte is 0x06, it's F2, so set
	btfsc	STATUS,Z	; serial baud rate to 1200
	bra	PKFABR1		; "
	addlw	-6		;If the incoming byte is 0x0C, it's F4, so set
	btfsc	STATUS,Z	; serial baud rate to 4800
	bra	PKFABR4		; "
	addlw	-107		;If the incoming byte is 0x77, it's Num Lock,
	btfsc	STATUS,Z	; so check if the nonexistent special control
	bra	PKFAClr		; key is down and send power key if it is
	addlw	-105		;If the incoming byte is 0xE0, this signals an
	btfsc	STATUS,Z	; extended code, so transition accordingly and
	retlw	low PKFsaAppsE0	; wait for the keycode it's escaping
	addlw	-1		;If the incoming byte is 0xE1, this signals an
	btfsc	STATUS,Z	; extended extended code which is only used
	retlw	low PKFsaAppsE1	; when the user pushes the Pause (F15) key
	addlw	-15		;If the incoming byte is 0xF0, this signals the
	btfsc	STATUS,Z	; release of a key, transition accordingly and
	retlw	low PKFsaAppsF0	; wait for the keycode being released
	retlw	low PKFsaStart	;If it's none of those, ignore it
PKFABR2	movlb	3		;Set UART baud rate to the default of 2400
	movlw	13		; "
	movwf	SPBRGH		; "
	movlw	4		; "
PKFABRF	movwf	SPBRGL		; "
	movlb	2		; "
	retlw	low PKFsaStart	; "
PKFABR3	movlb	3		;Set UART baud rate to 300
	movlw	104		; "
	movwf	SPBRGH		; "
	movlw	42		; "
	bra	PKFABRF		; "
PKFABR1	movlb	3		;Set UART baud rate to 1200
	movlw	26		; "
	movwf	SPBRGH		; "
	movlw	10		; "
	bra	PKFABRF		; "
PKFABR4	movlb	3		;Set UART baud rate to 4800
	movlw	6		; "
	movwf	SPBRGH		; "
	movlw	130		; "
	bra	PKFABRF		; "
PKFAClr	btfss	PU_FLAG,PU_SCTL	;If the nonexistent special control key is not
	retlw	low PKFsaStart	; down, ignore this keycode
	movf	AK_PUSH,W	;Load the push point of the ADB keyboard queue
	movwf	FSR0L		; into FSR0
	movlw	0x7F		;Push 0x7F, the keycode indicating a press of
	movwf	INDF0		; the power key, onto the ADB keyboard queue,
	incf	AK_PUSH,F	; then increment and wrap the pointer
	bcf	AK_PUSH,6	; "
	bsf	AU_FLAG,AU_SRKB	;Set flag so ADB keyboard requests service
	retlw	low PKFsaStart	;Wait for the next byte

PKFsaE0
	movf	PP_BUF,W	;If the incoming byte is 0xF0, this signals the
	xorlw	0xF0		; release of an 0xE0-escaped key, so transition
	btfsc	STATUS,Z	; accordingly and wait for the escaped keycode
	retlw	PKFsaE0F0	; being released
	xorlw	0xDF		;If the incoming byte is 0x2F, this is the apps
	btfsc	STATUS,Z	; key, which is used as a modifier key for
	bra	PKFsE00		; special control commands
	btfsc	PP_BUF,7	;If it's anything else with the MSB set, just
	bra	PKFsaStart	; ignore it
	movf	AK_PUSH,W	;Load the push point of the ADB keyboard queue
	movwf	FSR0L		; into FSR0
	movf	PP_BUF,W	;Convert the keycode from PS/2 to ADB (setting
	iorlw	B'10000000'	; the MSB so it looks at the upper half of the
	movlp	high PKLut	; LUT)
	callw			; "
	incf	WREG,W		;If it's 0xFF, it's invalid, so ignore it
	btfsc	STATUS,Z	; "
	retlw	low PKFsaStart	; "
	decf	WREG,W		; "
	xorwf	PK_RPT,W	;If this keycode is the same as the one we most
	btfsc	STATUS,Z	; recently sent, suppress it
	retlw	low PKFsaStart	; "
	xorwf	PK_RPT,W	;Otherwise, keep this one as the most recent
	movwf	PK_RPT		; keycode
	movwf	INDF0		;Push the converted keycode onto the ADB
	incf	AK_PUSH,F	; keyboard queue and increment and wrap the
	bcf	AK_PUSH,6	; pointer
	bsf	AU_FLAG,AU_SRKB	;Set flag so ADB keyboard requests service
	retlw	low PKFsaStart	;Wait for the next byte
PKFsE00	bsf	PU_FLAG,PU_APPS	;Set the apps-key-down bit and wait for the
	retlw	low PKFsaStart	; next byte

PKFsaE1
	movf	PP_BUF,W	;If the incoming byte is 0xF0, this signals the
	xorlw	0xF0		; release of an 0xE1-escaped key, so transition
	btfsc	STATUS,Z	; accordingly and wait for the escaped keycode
	retlw	low PKFsaE1F0	; being released
	xorlw	0xE4		;If the incoming byte is 0x14, this signals the
	btfsc	STATUS,Z	; press of the nonexistent special control key
	bsf	PU_FLAG,PU_SCTL	; used in the pressing of the Pause (F15) key
	retlw	low PKFsaStart	;Wait for the next byte

PKFsaF0
	movf	PP_BUF,W	;If the incoming byte is 0x77, it's Num Lock,
	addlw	-119		; which is treated specially depending on the
	btfsc	STATUS,Z	; state of the nonexistent special control key
	bra	PKFF03		; "
	addlw	-12		;If the incoming byte is 0x83, it's F7, blah
	btfsc	STATUS,Z	; blah stupid protocol
	bra	PKFF02		; "
	btfsc	PP_BUF,7	;If it's anything else with the MSB set, just
	bra	PKFsaStart	; ignore it
	movf	AK_PUSH,W	;Load the push point of the ADB keyboard queue
	movwf	FSR0L		; into FSR0
	movf	PP_BUF,W	;Convert the keycode from PS/2 to ADB
	movlp	high PKLut	; "
	callw			; "
	incf	WREG,W		;If it's 0xFF, it's invalid, so ignore it
	btfsc	STATUS,Z	; "
	retlw	low PKFsaStart	; "
	decf	WREG,W		; "
	xorlw	0x39		;If the key being released is caps lock, toggle
	btfss	STATUS,Z	; the caps-lock-locked bit and if it's set,
	bra	PKFF00		; suppress the key release event; otherwise,
	movlw	1 << PU_CAPS	; let the event go through
	xorwf	PU_FLAG,F	; "
	movlw	0		; "
	btfsc	PU_FLAG,PU_CAPS	; "
	retlw	low PKFsaStart	; "
PKFF00	xorlw	0x39		; "	
	iorlw	B'10000000'	;Set its MSB before pushing it onto the ADB
PKFF01	movwf	INDF0		; keyboard queue to indicate a released key,
	incf	AK_PUSH,F	; increment and wrap the pointer
	bcf	AK_PUSH,6	; "
	movlw	0xFF		;If a key's been released, typematic repeat is
	movwf	PK_RPT		; not a concern anymore
	bsf	AU_FLAG,AU_SRKB	;Set flag so ADB keyboard requests service
	retlw	low PKFsaStart	;Wait for the next byte
PKFF02	movf	AK_PUSH,W	;The ADB keycode for F7 is 0x62, or 0xE2 with
	movwf	FSR0L		; its MSB set for a release code, so load that
	movlw	0xE2		; to push on top of the keyboard queue and
	bra	PKFF01		; continue above
PKFF03	movf	AK_PUSH,W	;The ADB keycode for Num Lock/Clear is 0x47,
	movwf	FSR0L		; but if the nonexistent special control key
	movlw	0xC7		; was pressed, this is a release of Pause/F15
	btfsc	PU_FLAG,PU_SCTL	; instead so load the correct byte to push on
	movlw	0xF1		; top of the keyboard queue, clear the flag for
	bcf	PU_FLAG,PU_SCTL	; the nonexistent special control key, and
	bra	PKFF01		; continue above

PKFsaE0F0
	btfsc	PP_BUF,7	;If the MSB of the byte is set, just ignore it
	bra	PKFsaStart	; "
	movf	AK_PUSH,W	;Load the push point of the ADB keyboard queue
	movwf	FSR0L		; into FSR0
	movf	PP_BUF,W	;Convert the keycode from PS/2 to ADB (setting
	iorlw	B'10000000'	; the MSB so it looks at the upper half of the
	movlp	high PKLut	; LUT)
	callw			; "
	incf	WREG,W		;If it's 0xFF, it's invalid, so ignore it
	btfsc	STATUS,Z	; "
	retlw	low PKFsaStart	; "
	decf	WREG,W		; "
	iorlw	B'10000000'	;Set its MSB before pushing it onto the ADB
	movwf	INDF0		; keyboard queue to indicate a released key,
	incf	AK_PUSH,F	; increment and wrap the pointer
	bcf	AK_PUSH,6	; "
	movlw	0xFF		;If a key's been released, typematic repeat is
	movwf	PK_RPT		; not a concern anymore
	bsf	AU_FLAG,AU_SRKB	;Set flag so ADB keyboard requests service
	retlw	low PKFsaStart	;Wait for the next byte

PKFsaE1F0
	retlw	low PKFsaStart	;Wait for the next byte, we don't care

PKFsaAppsE0
	movf	PP_BUF,W	;If the incoming byte is 0xF0, this signals the
	addlw	-240		; release of an extended key, maybe apps, so
	btfsc	STATUS,Z	; transition accordingly, otherwise return to
	retlw	low PKFsaAppE0F0; get the next keycode
	retlw	low PKFsaStart	; "

PKFsaAppsE1
	movf	PP_BUF,W	;If the incoming byte is 0xF0, this signals the
	xorlw	0xF0		; release of an 0xE1-escaped key, so transition
	btfsc	STATUS,Z	; accordingly and wait for the escaped keycode
	retlw	low PKFsaAppE1F0; being released
	xorlw	0xE4		;If the incoming byte is 0x14, this signals the
	btfsc	STATUS,Z	; press of the nonexistent special control key
	bsf	PU_FLAG,PU_SCTL	; used in the pressing of the Pause (F15) key
	retlw	low PKFsaStart	;Wait for the next byte

PKFsaAppsF0
	movf	PP_BUF,W	;If the released key is anything but the Clear/
	xorlw	0x77		; Num Lock key, we don't care
	btfss	STATUS,Z	; "
	retlw	low PKFsaStart	; "
	btfss	PU_FLAG,PU_SCTL	;If the nonexistent special control key wasn't
	retlw	low PKFsaStart	; pressed, we don't care
	bcf	PU_FLAG,PU_SCTL	;Release the nonexistent special control key
	movf	AK_PUSH,W	;Load the push point of the ADB keyboard queue
	movwf	FSR0L		; into FSR0
	movlw	0xFF		;Push 0xFF, the keycode indicating a release of
	movwf	INDF0		; the power key, onto the ADB keyboard queue,
	incf	AK_PUSH,F	; then increment and wrap the pointer
	bcf	AK_PUSH,6	; "
	bsf	AU_FLAG,AU_SRKB	;Set flag so ADB keyboard requests service
	retlw	low PKFsaStart	;Wait for the next byte

PKFsaAppE0F0
	movf	PP_BUF,W	;If the key being released is the apps key,
	addlw	-47		; clear the relevant flag, else do nothing
	btfsc	STATUS,Z	; "
	bcf	PU_FLAG,PU_APPS	; "
	retlw	low PKFsaStart	; "

PKFsaAppE1F0
	retlw	low PKFsaStart	;Wait for the next byte, we don't care


PMFsa	org	0xA00

PMFsaStart
	btfss	PU_FLAG,PU_MSIN	;If the mouse hasn't been initialized yet, this
	bra	PMFsaS0		; might be the device saying it's ready for it
	movf	PP_BUF,W	;First byte in a mouse packet contains the MSBs
	movwf	PM_TEMP		; of the bytes to come, so save it for later
	bsf	AM_BTN,AMB_1S	;Copy the button state to the AND and set mouse
	btfsc	PP_BUF,0	; button bits for the ADB mouse, then wait for
	bcf	AM_BTN,AMB_1S	; the X delta byte
	btfsc	PP_BUF,0	; "
	bcf	AM_BTN,AMB_1A	; "
	bsf	AM_BTN,AMB_2S	; "
	btfss	PP_BUF,1	; "
	retlw	low PMFsaX	; "
	bcf	AM_BTN,AMB_2S	; "
	bcf	AM_BTN,AMB_2A	; "
	retlw	low PMFsaX	; "
PMFsaS0	movf	PP_BUF,W	;If the byte we just got is the mouse device ID
	btfss	STATUS,Z	; byte, 0x00, then continue, else wait for the
	retlw	low PMFsaStart	; next byte
	movlw	0xF4		;Load an 0xF4 byte to tell the mouse to begin
	movwf	PP_BUF		; to send us data packets
	movlb	1		;Pull the clock line low to begin sending
	bcf	TRISA,PP_CPIN	; "
	movlb	7		;Set the IOC interrupt bit just in case the
	bsf	IOCAF,PP_CPIN	; clock line is already low
	movlb	2		; "
	bsf	PP_FLAG,PP_TXMI	;Set the flags saying we have a byte to send
	bsf	PU_FLAG,PU_TXME	; and want to be notified when it's finished
	retlw	low PMFsaSent	;Transition and wait for byte to finish sending

PMFsaSent
	bcf	PU_FLAG,PU_TXME	;Byte sent, we no longer want to be notified
	retlw	low PMFsaAck	; for TX done events, wait for acknowledge byte

PMFsaAck
	movf	PP_BUF,W	;If the byte we just got is an 0xFA acknowledge
	xorlw	0xFA		; byte, set the flag that the mouse is now
	btfsc	STATUS,Z	; initialized, so start accepting its packets
	bsf	PU_FLAG,PU_MSIN	; and stop waiting for the ID byte
	retlw	low PMFsaStart	; "

PMFsaX
	lslf	PP_BUF,W	;Double and sign-extend the X delta value while
	rlf	PP_BUF,F	; getting its MSB from the first byte of the
	addwf	AM_DXL,F	; packet and add it to the ADB mouse's X delta
	movlw	0		; "
	btfsc	PP_BUF,0	; "
	iorlw	B'00000001'	; "
	btfsc	PM_TEMP,4	; "
	iorlw	B'11111110'	; "
	addwfc	AM_DXH,F	; "
	retlw	low PMFsaY	;Transition to wait for the Y delta

PMFsaY
	lslf	PP_BUF,W	;Double and sign-extend the Y delta value while
	rlf	PP_BUF,F	; getting its MSB from the first byte of the
	subwf	AM_DYL,F	; packet and subtract it from the ADB mouse's Y
	movlw	0		; delta; we do this because on a PS/2 mouse,
	btfsc	PP_BUF,0	; a positive Y delta means up, whereas on an
	iorlw	B'00000001'	; ADB mouse, it means down
	btfsc	PM_TEMP,5	; "
	iorlw	B'11111110'	; "
	subwfb	AM_DYH,F	; "
	bsf	AU_FLAG,AU_SRMS	;Set flag so the ADB mouse calls for service
	retlw	low PMFsaStart	;Wait for the next byte


Ps2Fsa	org	0xB00

Ps2FsaStart
	movlb	1		;If we're here while the clock is being pulled
	btfss	TRISA,PP_CPIN	; low by us, it must be because we want to send
	bra	P2FsaS0		; a byte, so skip ahead
	movlb	0		;Otherwise, initialize the shift register to
	movlw	0x80		; take a byte, LSB first
	movwf	PP_SR		; "
	btfsc	PORTA,PP_MPIN	;Judge who the clock came from based on which
	retlw	low Ps2FsaKbBitP; data line is pulled low and resume from the
	retlw	low Ps2FsaMsBitP; appropriate state next time clock goes low
P2FsaS0	bsf	PIE1,TMR2IE	;Set the next timer interrupt to bring us back
	movlb	0		; so we keep the clock low long enough to let
	retlw	low Ps2FsaTxStrt; the keyboard/mouse know to back off

Ps2FsaIgnore
	retlw	low Ps2FsaIgnore;Utility state to ignore bus until timeout

Ps2FsaKbBitP
	bcf	STATUS,C	;Copy the state of the keyboard data pin into
	btfsc	PORTA,PP_KPIN	; the carry bit
	bsf	STATUS,C	; "
	rrf	PP_SR,F		;Rotate it into shift register from the left
	btfsc	STATUS,C	;If we rotated a 1 out of the shift register,
	bra	P2FKBP0		; we've completed a byte
	btfsc	PP_SR,7		;In this state, before the bit we just got, we
	retlw	low Ps2FsaKbBitN; had an even number of 1 bits, so the (odd)
	retlw	low Ps2FsaKbBitP; parity bit would be high if we ended there,
P2FKBP0	btfsc	PP_SR,7		; but we just got a bit, so go to a state
	retlw	low Ps2FsaKbParN; (either to get another data bit or to expect
	retlw	low Ps2FsaKbParP; the parity bit) based on the bit we just got

Ps2FsaMsBitP
	bcf	STATUS,C	;(Same as Ps2FsaKbBitP but for mouse)
	btfsc	PORTA,PP_MPIN	; "
	bsf	STATUS,C	; "
	rrf	PP_SR,F		; "
	btfsc	STATUS,C	; "
	bra	P2FMBP0		; "
	btfsc	PP_SR,7		; "
	retlw	low Ps2FsaMsBitN; "
	retlw	low Ps2FsaMsBitP; "
P2FMBP0	btfsc	PP_SR,7		; "
	retlw	low Ps2FsaMsParN; "
	retlw	low Ps2FsaMsParP; "

Ps2FsaKbBitN
	bcf	STATUS,C	;(Same as Ps2FsaKbBitP but with an odd number
	btfsc	PORTA,PP_KPIN	; of 1 bits prior to bit just received)
	bsf	STATUS,C	; "
	rrf	PP_SR,F		; "
	btfsc	STATUS,C	; "
	bra	P2FKBN0		; "
	btfsc	PP_SR,7		; "
	retlw	low Ps2FsaKbBitP; "
	retlw	low Ps2FsaKbBitN; "
P2FKBN0	btfsc	PP_SR,7		; "
	retlw	low Ps2FsaKbParP; "
	retlw	low Ps2FsaKbParN; "

Ps2FsaMsBitN
	bcf	STATUS,C	;(Same as Ps2FsaKbBitN but for mouse)
	btfsc	PORTA,PP_MPIN	; "
	bsf	STATUS,C	; "
	rrf	PP_SR,F		; "
	btfsc	STATUS,C	; "
	bra	P2FMBN0		; "
	btfsc	PP_SR,7		; "
	retlw	low Ps2FsaMsBitP; "
	retlw	low Ps2FsaMsBitN; "
P2FMBN0	btfsc	PP_SR,7		; "
	retlw	low Ps2FsaMsParP; "
	retlw	low Ps2FsaMsParN; "

Ps2FsaKbParP
	btfsc	PORTA,PP_KPIN	;We expect a 1 parity bit, so go on to expect
	retlw	low Ps2FsaKbStop; the stop bit if we got it, or ignore this
	retlw	low Ps2FsaIgnore; transaction if we didn't

Ps2FsaMsParP
	btfsc	PORTA,PP_MPIN	;(Same as Ps2FsaKbParP but for mouse)
	retlw	low Ps2FsaMsStop; "
	retlw	low Ps2FsaIgnore; "

Ps2FsaKbParN
	btfss	PORTA,PP_KPIN	;(Same as Ps2FsaKbParP but expecting a 0 parity
	retlw	low Ps2FsaKbStop; bit)
	retlw	low Ps2FsaIgnore; "

Ps2FsaMsParN
	btfss	PORTA,PP_MPIN	;(Same as Ps2FsaKbParN but for mouse)
	retlw	low Ps2FsaMsStop; "
	retlw	low Ps2FsaIgnore; "

Ps2FsaKbStop
	btfss	PORTA,PP_KPIN	;We expect the stop bit to be 1, so go on if we
	retlw	low Ps2FsaIgnore; get it, ignore this transaction if we don't
	movf	PP_SR,W		;Move the filled shift register's contents into
	movwf	PP_BUF		; the buffer
	bsf	PP_FLAG,PP_RXKI	;Set flag that we've received a keyboard byte
	retlw	low Ps2FsaStart	;Return to expect the next byte

Ps2FsaMsStop
	btfss	PORTA,PP_MPIN	;(Same as Ps2FsaKbStop but for mouse)
	retlw	low Ps2FsaIgnore; "
	movf	PP_SR,W		; "
	movwf	PP_BUF		; "
	bsf	PP_FLAG,PP_RXMI	; "
	retlw	low Ps2FsaStart	; "

Ps2FsaTxStrt
	movlb	1		;Pull the appropriate data line low for the
	btfsc	PP_FLAG,PP_TXKI	; peripheral that we want to send a byte to
	bcf	TRISA,PP_KPIN	; "
	btfsc	PP_FLAG,PP_TXMI	; "
	bcf	TRISA,PP_MPIN	; "
	bsf	TRISA,PP_CPIN	;Release the clock line
	movlb	0		;Transition to the appropriate state for the
	btfsc	PP_FLAG,PP_TXKI	; peripheral that we want to send a byte to or
	retlw	low Ps2FsaTxKbSt; back to start if neither and we somehow got
	btfsc	PP_FLAG,PP_TXMI	; here by mistake
	retlw	low Ps2FsaTxMsSt; "
	retlw	low Ps2FsaStart	; "

Ps2FsaTxKbSt
	movf	PP_BUF,W	;Load the shift register from the buffer
	movwf	PP_SR		; "
	bsf	STATUS,C	;Rotate the LSB out of the right, rotating a 1
	rrf	PP_SR,F		; into the MSB
	movlb	1		;Pull the data line low if the bit we rotated
	btfss	STATUS,C	; out is a 0, release it for a 1
	bcf	TRISA,PP_KPIN	; "
	btfsc	STATUS,C	; "
	bsf	TRISA,PP_KPIN	; "
	movlb	0		;If the bit we just sent was a 1, that gives us
	btfsc	STATUS,C	; an odd number of 1 bits, so our current (odd)
	retlw	Ps2FsaTxKbBN	; parity bit is a 0, likewise it's 1 if we just
	retlw	Ps2FsaTxKbBP	; sent a 0; transition to the appropriate state

Ps2FsaTxKbBN
	lsrf	PP_SR,F		;Rotate the next bit out the right of the shift
	movlb	1		; register
	btfsc	STATUS,Z	;If we just shifted out the placeholder bit,
	bcf	TRISA,PP_KPIN	; pull the line low to send our 0 parity bit
	movlb	0		; and get ready to send our stop bit
	btfsc	STATUS,Z	; "
	retlw	low Ps2FsaTxKbSp; "
	movlb	1		;Pull the data line low if we shifted out a 0,
	btfss	STATUS,C	; release it if we shifted out a 1
	bcf	TRISA,PP_KPIN	; "
	btfsc	STATUS,C	; "
	bsf	TRISA,PP_KPIN	; "
	movlb	0		;If the bit we just sent was a 1, that gives us
	btfsc	STATUS,C	; an even number of 1 bits, so our current
	retlw	Ps2FsaTxKbBP	; parity bit is a 1, likewise it's 0 if we just
	retlw	Ps2FsaTxKbBN	; sent a 0; transition to the appropriate state

Ps2FsaTxKbBP
	lsrf	PP_SR,F		;(Same as Ps2FsaTxKbBP but current parity bit
	movlb	1		; is inverted)
	btfsc	STATUS,Z	; "
	bsf	TRISA,PP_KPIN	; "
	movlb	0		; "
	btfsc	STATUS,Z	; "
	retlw	low Ps2FsaTxKbSp; "
	movlb	1		; "
	btfss	STATUS,C	; "
	bcf	TRISA,PP_KPIN	; "
	btfsc	STATUS,C	; "
	bsf	TRISA,PP_KPIN	; "
	movlb	0		; "
	btfsc	STATUS,C	; "
	retlw	Ps2FsaTxKbBN	; "
	retlw	Ps2FsaTxKbBP	; "

Ps2FsaTxKbSp
	movlb	1		;Release the data line to send our stop bit
	bsf	TRISA,PP_KPIN	; "
	movlb	0		; "
	retlw	low Ps2FsaTxKbAc;Transition to wait for the acknowledge bit

Ps2FsaTxKbAc
	btfsc	PORTA,PP_KPIN	;If the data line is pulled low, the peripheral
	retlw	low Ps2FsaIgnore; acknowledged the byte we just sent, so wait
	bcf	PP_FLAG,PP_TXKI	; for the next, else ignore this transaction
	retlw	low Ps2FsaStart	; "

Ps2FsaTxMsSt
	movf	PP_BUF,W	;(Same as Ps2FsaTxKbSt but for mouse)
	movwf	PP_SR		; "
	bsf	STATUS,C	; "
	rrf	PP_SR,F		; "
	movlb	1		; "
	btfss	STATUS,C	; "
	bcf	TRISA,PP_MPIN	; "
	btfsc	STATUS,C	; "
	bsf	TRISA,PP_MPIN	; "
	movlb	0		; "
	btfsc	STATUS,C	; "
	retlw	Ps2FsaTxMsBN	; "
	retlw	Ps2FsaTxMsBP	; "

Ps2FsaTxMsBN
	lsrf	PP_SR,F		;(Same as Ps2FsaTxKbBN but for mouse)
	movlb	1		; "
	btfsc	STATUS,Z	; "
	bcf	TRISA,PP_MPIN	; "
	movlb	0		; "
	btfsc	STATUS,Z	; "
	retlw	low Ps2FsaTxMsSp; "
	movlb	1		; "
	btfss	STATUS,C	; "
	bcf	TRISA,PP_MPIN	; "
	btfsc	STATUS,C	; "
	bsf	TRISA,PP_MPIN	; "
	movlb	0		; "
	btfsc	STATUS,C	; "
	retlw	Ps2FsaTxMsBP	; "
	retlw	Ps2FsaTxMsBN	; "

Ps2FsaTxMsBP
	lsrf	PP_SR,F		;(Same as Ps2FsaTxKbBP but for mouse)
	movlb	1		; "
	btfsc	STATUS,Z	; "
	bsf	TRISA,PP_MPIN	; "
	movlb	0		; "
	btfsc	STATUS,Z	; "
	retlw	low Ps2FsaTxMsSp; "
	movlb	1		; "
	btfss	STATUS,C	; "
	bcf	TRISA,PP_MPIN	; "
	btfsc	STATUS,C	; "
	bsf	TRISA,PP_MPIN	; "
	movlb	0		; "
	btfsc	STATUS,C	; "
	retlw	Ps2FsaTxMsBN	; "
	retlw	Ps2FsaTxMsBP	; "

Ps2FsaTxMsSp
	movlb	1		;(Same as Ps2FsaTxKbSp but for mouse)
	bsf	TRISA,PP_MPIN	; "
	movlb	0		; "
	retlw	low Ps2FsaTxMsAc; "

Ps2FsaTxMsAc
	btfsc	PORTA,PP_MPIN	;(Same as Ps2FsaTxKbAc but for mouse)
	retlw	low Ps2FsaIgnore; "
	bcf	PP_FLAG,PP_TXMI	; "
	retlw	low Ps2FsaStart	; "


AKFsa	org	0xC00

AKFsaCommand
	btfsc	AP_BUF,2	;Talk is the only command that sets bit 2
	bra	AKFsaTalk	; "
	btfsc	AP_BUF,3	;Only talk and listen set bit 3, so if it's not
	bra	AKFsaListen	; talk, it's listen
	retlw	low AKFsaIgnore	;Ignore flush

AKFsaIgnore
	retlw	low AKFsaIgnore	;Utility state to ignore until next command

AKFsaListen
	btfss	AP_BUF,1	;Keyboard only responds to listens on registers
	retlw	low AKFsaIgnore	; 2 and 3
	btfss	AP_BUF,0	;Go to the appropriate state and wait for a
	retlw	low AKFsaLstn2H	; data byte
	retlw	low AKFsaLstn3H	; "

AKFsaLstn2H
	retlw	low AKFsaLstn2L	;Keyboard listen 2 only has effect on low byte

AKFsaLstn2L
	;TODO mechanism to signal something that LEDs have changed?
	movlw	B'11111000'	;Accept the state of the keyboard LEDs from the
	andwf	AK_R2L,F	; listen command but ignore all other bits
	movlw	B'00000111'	; "
	andwf	AP_BUF,W	; "
	iorwf	AK_R2L,F	; "
	retlw	low AKFsaIgnore	; "

AKFsaLstn3H
	movf	AP_BUF,W	;We can't act on the high byte until we know
	movwf	AU_TEMP		; what the low byte (handler ID) is, so store
	retlw	low AKFsaLstn3L	; it in temporary space

AKFsaLstn3L
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	AKFL3L1		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	AKFL3L0		; detected
	addlw	-4		;If handler ID is not 0x02/0x03, ignore this
	andlw	B'11111110'	; command, we don't understand any other
	btfss	STATUS,Z	; handlers
	retlw	low AKFsaIgnore	; "
	movf	AP_BUF,W	;If handler ID is 0x02/0x03, accept it as our
	movwf	AK_R3L		; new handler ID, because we as an extended
	retlw	low AKFsaIgnore	; keyboard understand those
AKFL3L0	btfss	AK_R3H,7	;If a collision has not been detected, skip
	bra	AKFL3L2		; ahead to change the address; if one has been
	bcf	AK_R3H,7	; detected, clear it and ignore this command
	retlw	low AKFsaIgnore	; "
AKFL3L1	bcf	AU_FLAG,AU_SEKB	;Copy the state of the SRQ enable bit to the SRQ
	bcf	AK_R3H,5	; enable flag and to our copy of register 3
	btfsc	AU_TEMP,5	; "
	bsf	AU_FLAG,AU_SEKB	; "
	btfsc	AU_TEMP,5	; "
	bsf	AK_R3H,5	; "
AKFL3L2	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	AU_TEMP,F	; byte as our new address and we're done
	movf	AK_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	AU_TEMP,W	; "
	movwf	AK_R3H		; "
	retlw	low AKFsaIgnore	; "

AKFsaTalk
	movf	AP_BUF,W	;Branch to the appropriate handler for the
	andlw	B'00000011'	; register (0, 2, or 3) being ordered to talk;
	btfsc	STATUS,Z	; if it's 1, we have no register 1, so ignore
	bra	AKFsaTalk0H	; "
	addlw	-2		; "
	btfsc	STATUS,Z	; "
	bra	AKFsaTalk2H	; "
	addlw	-1		; "
	btfsc	STATUS,Z	; "
	bra	AKFsaTalk3H	; "
	retlw	low AKFsaIgnore	; "

AKFsaTalk0H
	bcf	AU_FLAG,AU_SRKB	;Clear the SRQ flag, will re-set it if need be
	movf	AK_PUSH,W	;If the pop pointer is equal to the push
	xorwf	AK_POP,W	; pointer, we have no data to return, so ignore
	btfsc	STATUS,Z	; the command
	retlw	low AKFsaIgnore	; "
	movf	AK_POP,W	;Get the first byte off the queue; we don't yet
	movwf	FSR0L		; increment the pop pointer here because it's
	movf	INDF0,W		; possible for us to collide while sending
	btfss	AK_R3L,0	;If we're handler 2, change the keycodes for
	call	AKSquashMods	; right shift/ctrl/option to the left ones
	movwf	AP_BUF		;Load keycode into the buffer
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	AU_FLAG,AU_TXON	; and we are interested in transmission events
	retlw	low AKFsaTalk0L	;Deal with what happened in the next state

AKFsaTalk0L
	bsf	AU_FLAG,AU_SRKB	;Set SRQ flag - if we're here, we have data
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AKFT0L0		; bit of register 3, clear the transmit events
	bsf	AK_R3H,7	; flag and we're done until we get another
	bcf	AU_FLAG,AU_TXON ; command
	retlw	low AKFsaIgnore	; "
AKFT0L0	movf	AP_BUF,W	;If the byte on top of the queue is an 0x7F or
	andlw	B'01111111'	; an 0xFF, i.e. the power key, we have to send
	xorlw	0x7F		; it twice, so skip ahead
	btfsc	STATUS,Z	; "
	bra	AKFT0L3		; "
	incf	AK_POP,W	;If there's only one event on the queue, load
	andlw	B'10111111'	; 0xFF as the second byte for transmission
	xorwf	AK_PUSH,W	; "
	btfsc	STATUS,Z	; "
	movlw	0xFF		; "
	btfsc	STATUS,Z	; "
	bra	AKFT0L2		; "
	xorwf	AK_PUSH,W	;Get the second byte off the queue
	movwf	FSR0L		; "
	movf	INDF0,W		; "
	btfss	AK_R3L,0	;If we're handler 2, change the keycodes for
	call	AKSquashMods	; right shift/ctrl/option to the left ones
AKFT0L2	movwf	AP_BUF		;Load keycode into the buffer
AKFT0L3	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	AU_FLAG,AU_TXON	; and that we no longer want transmit events
	retlw	low AKFsaTalk0E	;Deal with what happened in the next state

AKFsaTalk0E
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AKFT0E0		; bit of register 3 and we're done until we get
	bsf	AK_R3H,7	; another command
	retlw	low AKFsaIgnore	; "
AKFT0E0	movf	AK_POP,W	;Transmission successful, so update register 2
	movwf	FSR0L		; based on the first keycode sent and advance
	movf	INDF0,W		; the pop pointer
	call	AKUpdateR2	; "
	incf	AK_POP,F	; "
	bcf	AK_POP,6	; "
	movf	AP_BUF,W	;If the last byte we sent was 0x7F or 0xFF,
	andlw	B'01111111'	; we sent only one keycode
	xorlw	B'01111111'	; "
	btfsc	STATUS,Z	; "
	bra	AKFT0E1		; "
	movf	AK_POP,W	;If we sent a second keycode, update register 2
	movwf	FSR0L		; based on that one too and advance the pop
	movf	INDF0,W		; pointer
	call	AKUpdateR2	; "
	incf	AK_POP,F	; "
	bcf	AK_POP,6	; "
AKFT0E1	movf	AK_POP,W	;If the pop pointer has caught up with the push
	xorwf	AK_PUSH,W	; pointer, we no longer need service so clear
	btfsc	STATUS,Z	; our SRQ flag
	bcf	AU_FLAG,AU_SRKB	; "
	retlw	low AKFsaIgnore	;And we're done

AKFsaTalk2H
	movf	AK_R2H,W	;Load the high byte of register 2 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	AU_FLAG,AU_TXON	; and we are interested in transmission events
	retlw	low AKFsaTalk2L	;Deal with what happened in the next state

AKFsaTalk2L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AKFT2L0		; bit of register 3, clear the transmit events
	bsf	AK_R3H,7	; flag and we're done until we get another
	bcf	AU_FLAG,AU_TXON	; command
	retlw	low AKFsaIgnore	; "
AKFT2L0	movf	AK_R2L,W	;Load the low byte of register 2 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	AU_FLAG,AU_TXON	; and that we no longer want transmit events
	retlw	low AKFsaTalk23E;Deal with what happened in the next state

AKFsaTalk3H
	movf	AK_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	iorwf	AP_BUF,F	; collision detection
	movlb	2		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	AU_FLAG,AU_TXON	; and we are interested in transmission events
	retlw	low AKFsaTalk3L	;Deal with what happened in the next state

AKFsaTalk3L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AKFT3L0		; bit of register 3, clear the transmit events
	bsf	AK_R3H,7	; flag and we're done until we get another
	bcf	AU_FLAG,AU_TXON	; command
	retlw	low AKFsaIgnore	; "
AKFT3L0	movf	AK_R3L,W	;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	AU_FLAG,AU_TXON	; and that we no longer want transmit events
	retlw	low AKFsaTalk23E;Deal with what happened in the next state

AKFsaTalk23E
	btfsc	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bsf	AK_R3H,7	; bit of register 3
	retlw	low AKFsaIgnore	;Either way, we're done

AKSquashMods
	addlw	-123		;Change 0x7B (release right shift) to 0x38
	btfsc	STATUS,Z	; (release shift)
	retlw	0x38		; "
	addlw	-1		;Change 0x7C (release right option) to 0x3A
	btfsc	STATUS,Z	; (release option)
	retlw	0x3A		; "
	addlw	-1		;Change 0x7D (release right control) to 0x36
	btfsc	STATUS,Z	; (release control)
	retlw	0x36		; "
	addlw	-126		;Change 0xFB (press right shift) to 0xB8 (press
	btfsc	STATUS,Z	; shift)
	retlw	0xB8		; "
	addlw	-1		;Change 0xFC (press right option) to 0xBA
	btfsc	STATUS,Z	; (press option)
	retlw	0xBA		; "
	addlw	-1		;Change 0xFD (press right control) to 0xB6
	btfsc	STATUS,Z	; (press control)
	retlw	0xB6		; "
	addlw	-3		;Otherwise, leave as is
	return			; "

AKUpdateR2
	;TODO "exceptional event" for ADB keyboards means reset was pressed
	btfss	WREG,7		;If MSB of the key pressed is set (because the
	bra	AKUpR20		; key is being released), snuff it to zero and
	andlw	B'01111111'	; complement the register 2 registers before
	comf	AK_R2H,F	; and after setting them, this way we don't
	comf	AK_R2L,F	; have to copypaste code
	comf	AK_MOD,F	; "
	call	AKUpR22		; "
	comf	AK_R2H,F	; "
	comf	AK_R2L,F	; "
	comf	AK_MOD,F	; "
	bra	AKUpR21		; "
AKUpR20	call	AKUpR22		; "
AKUpR21	bsf	AK_R2H,K2H_CTL	;If either control key is down, reflect that in
	btfsc	AK_MOD,KMD_LCT	; the appropriate bits in keyboard register 2
	btfss	AK_MOD,KMD_RCT	; "
	bcf	AK_R2H,K2H_CTL	; "
	bsf	AK_R2H,K2H_SHF	;If either shift key is down, reflect that in
	btfsc	AK_MOD,KMD_LSH	; the appropriate bits in keyboard register 2
	btfss	AK_MOD,KMD_RSH	; "
	bcf	AK_R2H,K2H_SHF	; "
	bsf	AK_R2H,K2H_OPT	;If either option key is down, reflect that in
	btfsc	AK_MOD,KMD_LOP	; the appropriate bits in keyboard register 2
	btfss	AK_MOD,KMD_ROP	; "
	bcf	AK_R2H,K2H_OPT	; "
	return
AKUpR22	addlw	-51		;0x33 (delete/backspace)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_DEL
	addlw	-3		;0x36 (left control)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_LCT
	addlw	-1		;0x37 (command)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_CMD
	addlw	-1		;0x38 (left shift)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_LSH
	addlw	-1		;0x39 (caps lock)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_CAP
	addlw	-1		;0x3A (left option)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_LOP
	addlw	-13		;0x47 (clear/num lock)
	btfsc	STATUS,Z
	bcf	AK_R2L,K2L_CLR
	addlw	-36		;0x6B (F14/scroll lock)
	btfsc	STATUS,Z
	bcf	AK_R2L,K2L_SLK
	addlw	-16		;0x7B (right shift)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_RSH
	addlw	-1		;0x7C (right option)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_ROP
	addlw	-1		;0x7D (right control)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_RCT
	addlw	-2		;0x7F (reset)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_RST
	return


AMFsa	org	0xD00

AMFsaCommand
	btfsc	AP_BUF,2	;Talk is the only command that sets bit 2
	bra	AMFsaTalk	; "
	btfsc	AP_BUF,3	;Only talk and listen set bit 3, so if it's not
	bra	AMFsaListen	; talk, it's listen
	retlw	low AMFsaIgnore	;Ignore flush

AMFsaIgnore
	retlw	low AMFsaIgnore	;Utility state to ignore until next command

AMFsaListen
	btfsc	AP_BUF,0	;Mouse only listens on register 3
	btfss	AP_BUF,1	; "
	retlw	low AMFsaIgnore	; "
	retlw	low AMFsaLstn3H	; "

AMFsaLstn3H
	movf	AP_BUF,W	;We can't act on the high byte until we know
	movwf	AU_TEMP		; what the low byte (handler ID) is, so store
	retlw	low AMFsaLstn3L	; it in temporary space

AMFsaLstn3L
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	AMFL3L1		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	AMFL3L0		; detected
	addlw	-3		;If handler ID is not 0x01/0x02, ignore this
	andlw	B'11111110'	; command, we don't understand any other
	btfss	STATUS,Z	; handlers
	retlw	low AMFsaIgnore	; "
	movf	AP_BUF,W	;If handler ID is 0x01/0x02, accept it as our
	movwf	AM_R3L		; new handler ID, because we as a 100/200 cpi
	retlw	low AMFsaIgnore	; mouse understand those
AMFL3L0	btfss	AM_R3H,7	;If a collision has not been detected, skip
	bra	AMFL3L2		; ahead to change the address; if one has been
	bcf	AM_R3H,7	; detected, clear it and ignore this command
	retlw	low AMFsaIgnore	; "
AMFL3L1	bcf	AU_FLAG,AU_SEMS	;Copy the state of the SRQ enable bit to the SRQ
	bcf	AM_R3H,5	; enable flag and to our copy of register 3
	btfsc	AU_TEMP,5	; "
	bsf	AU_FLAG,AU_SEMS	; "
	btfsc	AU_TEMP,5	; "
	bsf	AM_R3H,5	; "
AMFL3L2	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	AU_TEMP,F	; byte as our new address and we're done
	movf	AM_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	AU_TEMP,W	; "
	movwf	AM_R3H		; "
	retlw	low AMFsaIgnore	; "

AMFsaTalk
	lsrf	AP_BUF,W	;Mouse only responds on registers 0 and 3
	xorwf	AP_BUF,W	; "
	btfsc	WREG,0		; "
	retlw	low AMFsaIgnore	; "
	btfsc	AP_BUF,0	; "
	bra	AMFsaTalk3H	; "
	bra	AMFsaTalk0H	; "

AMFsaTalk0H
	;TODO the mouse button is still sticky when the modem is active...
	btfss	AU_FLAG,AU_SRMS	;If we have anything to say for register 0, our
	retlw	low AMFsaIgnore	; service request flag would be set
	bcf	AU_FLAG,AU_SRMS	;Clear the SRQ flag, will re-set it if need be
	btfsc	AM_DYH,7	;We handle the Y delta differently depending on
	bra	AMFT0H1		; whether it's negative or positive
	movlw	0xC1		;If it's positive, subtract 63 from it
	addwf	AM_DYL,F	; "
	movlw	0xFF		; "
	addwfc	AM_DYH,F	; "
	btfsc	STATUS,C	;If it didn't borrow, leave the difference,
	bra	AMFT0H0		; it's more than we can convey in one delta
	movlw	B'00111111'	;If it borrowed, add 63 to the difference and
	addwf	AM_DYL,W	; that's our final delta
	movwf	AP_BUF		; "
	clrf	AM_DYH		; "
	clrf	AM_DYL		; "
	bra	AMFT0H3		; "
AMFT0H0	movlw	B'00111111'	;If it didn't borrow, send the delta 63 and
	movwf	AP_BUF		; signal that we need to be serviced again to
	bsf	AU_FLAG,AU_SRMS	; convey the rest of it
	bra	AMFT0H3		; "
AMFT0H1	movlw	0x40		;If it's negative, add 64 to it
	addwf	AM_DYL,F	; "
	movlw	0		; "
	addwfc	AM_DYH,F	; "
	btfss	STATUS,C	;If it didn't overflow, leave the sum, it's
	bra	AMFT0H2		; more than we can convey in one delta
	movlw	B'01000000'	;If it overflowed, add -64 to the sum and
	addwf	AM_DYL,W	; that's our final delta
	movwf	AP_BUF		; "
	clrf	AM_DYL		; "
	bra	AMFT0H3		; "
AMFT0H2	movlw	B'01000000'	;If it didn't overflow, send the delta -64 and
	movwf	AP_BUF		; signal that we need to be serviced again to
	bsf	AU_FLAG,AU_SRMS	; convey the rest of it
AMFT0H3	btfsc	AM_R3L,1	;If the handler ID is 0x01, halve the delta
	bra	AMFT0H4		; value to simulate a 100 cpi mouse
	lsrf	AP_BUF,F	; "
	btfsc	AP_BUF,5	; "
	bsf	AP_BUF,6	; "
AMFT0H4	bsf	AP_BUF,7	;Copy the mouse button state, ANDing together
	btfsc	AM_BTN,AMB_1S	; the AND state and the set state to ensure
	btfss	AM_BTN,AMB_1A	; clicks don't get lost
	bcf	AP_BUF,7	; "
	movf	AP_BUF,W	;Save first byte in case of collision and we
	movwf	AU_TEMP		; need to undo changes to mouse deltas
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	AU_FLAG,AU_TXON	; and we are interested in transmission events
	retlw	low AMFsaTalk0L	;Deal with what happened in the next state

AMFsaTalk0L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AMFT0L0		; bit of register 3, clear the transmit events
	bsf	AM_R3H,7	; flag and we're done until we get another
	bcf	AU_FLAG,AU_TXON ; command
	retlw	low AKFsaIgnore	; "
AMFT0L0	btfsc	AM_DXH,7	;We handle the X delta differently depending on
	bra	AMFT0L2		; whether it's negative or positive
	movlw	0xC1		;If it's positive, subtract 63 from it
	addwf	AM_DXL,F	; "
	movlw	0xFF		; "
	addwfc	AM_DXH,F	; "
	btfsc	STATUS,C	;If it didn't borrow, leave the difference,
	bra	AMFT0L1		; it's more than we can convey in one delta
	movlw	B'00111111'	;If it borrowed, add 63 to the difference and
	addwf	AM_DXL,W	; that's our final delta
	movwf	AP_BUF		; "
	clrf	AM_DXH		; "
	clrf	AM_DXL		; "
	bra	AMFT0L4		; "
AMFT0L1	movlw	B'00111111'	;If it didn't borrow, send the delta 63 and
	movwf	AP_BUF		; signal that we need to be serviced again to
	bsf	AU_FLAG,AU_SRMS	; convey the rest of it
	bra	AMFT0L4		; "
AMFT0L2	movlw	0x40		;If it's negative, add 64 to it
	addwf	AM_DXL,F	; "
	movlw	0		; "
	addwfc	AM_DXH,F	; "
	btfss	STATUS,C	;If it didn't overflow, leave the sum, it's
	bra	AMFT0L3		; more than we can convey in one delta
	movlw	B'01000000'	;If it overflowed, add -64 to the sum and
	addwf	AM_DXL,W	; that's our final delta
	movwf	AP_BUF		; "
	clrf	AM_DXL		; "
	bra	AMFT0L4		; "
AMFT0L3	movlw	B'01000000'	;If it didn't overflow, send the delta -64 and
	movwf	AP_BUF		; signal that we need to be serviced again to
	bsf	AU_FLAG,AU_SRMS	; convey the rest of it
AMFT0L4	btfsc	AM_R3L,1	;If the handler ID is 0x01, halve the delta
	bra	AMFT0L5		; value to simulate a 100 cpi mouse
	lsrf	AP_BUF,F	; "
	btfsc	AP_BUF,5	; "
	bsf	AP_BUF,6	; "
AMFT0L5	bsf	AP_BUF,7	;Copy the mouse button state, ANDing together
	btfsc	AM_BTN,AMB_2S	; the AND state and the set state to ensure
	btfss	AM_BTN,AMB_2A	; clicks don't get lost
	bcf	AP_BUF,7	; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	AU_FLAG,AU_TXON	; and that we no longer want transmit events
	retlw	low AMFsaTalk0E	;Deal with what happened in the next state

AMFsaTalk0E
	btfsc	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bsf	AM_R3H,7	; bit of register 3 and finish
	btfsc	AP_FLAG,AP_COL	; "
	retlw	low AMFsaIgnore	; "
	swapf	AM_BTN,W	;If the AND and set mouse button state differ,
	xorwf	AM_BTN,W	; signal that we need to be serviced to get
	andlw	B'00000011'	; the state again so that a click isn't lost
	btfss	STATUS,Z	; "
	bsf	AU_FLAG,AU_SRMS	; "
	movlw	B'11110000'	;Set all the bits of the AND mouse button
	iorwf	AM_BTN,F	; state for next time
	retlw	low AMFsaIgnore

AMFsaTalk3H
	movf	AM_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	iorwf	AP_BUF,F	; collision detection
	movlb	2		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	AU_FLAG,AU_TXON	; and we are interested in transmission events
	retlw	low AMFsaTalk3L	;Deal with what happened in the next state

AMFsaTalk3L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AMFT3L0		; bit of register 3, clear the transmit events
	bsf	AM_R3H,7	; flag and we're done until we get another
	bcf	AU_FLAG,AU_TXON	; command
	retlw	low AMFsaIgnore	; "
AMFT3L0	movf	AM_R3L,W	;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	AU_FLAG,AU_TXON	; and that we no longer want transmit events
	retlw	low AMFsaTalk3E	;Deal with what happened in the next state

AMFsaTalk3E
	btfsc	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bsf	AM_R3H,7	; bit of register 3
	retlw	low AMFsaIgnore	;Either way, we're done


ADFsa	org	0xE00

ADFsaCommand
	btfsc	AP_BUF,2	;Talk is the only command that sets bit 2
	bra	ADFsaTalk	; "
	btfsc	AP_BUF,3	;Only talk and listen set bit 3, so if it's not
	bra	ADFsaListen	; talk, it's listen
	;fall through		;Ignore flush

ADFsaIgnore
	retlw	low ADFsaIgnore	;Utility state to ignore until next command

ADFsaListen
	lslf	AP_BUF,W	;If register being told to listen is 1 or 2,
	xorwf	AP_BUF,W	; ignore
	btfsc	WREG,1		; "
	retlw	low ADFsaIgnore	; "
	btfsc	AP_BUF,1	;If it's 3, next byte goes to listen 3 handler
	retlw	low ADFsaLstn3H	; "
	retlw	low ADFsaLstn00	;If it's 0, next byte goes to listen 0 handler

ADFsaLstn00
	movf	AD_TLEN,W	;If the queue doesn't have enough space to take
	andlw	B'01111111'	; another 8 bytes, ignore this command
	addlw	-55		; "
	btfsc	STATUS,C	; "
	retlw	low ADFsaIgnore	; "
	movf	AD_TPSH,W	;Load the TX push point into FSR0
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn01	;Wait for the next byte

ADFsaLstn01
	incf	AD_TPSH,W	;Load the TX push point plus 1 into FSR0
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn02	;Wait for the next byte

ADFsaLstn02
	movf	AD_TPSH,W	;Load the TX push point plus 2 into FSR0
	addlw	2		; "
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn03	;Wait for the next byte

ADFsaLstn03
	movf	AD_TPSH,W	;Load the TX push point plus 3 into FSR0
	addlw	3		; "
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn04	;Wait for the next byte

ADFsaLstn04
	movf	AD_TPSH,W	;Load the TX push point plus 4 into FSR0
	addlw	4		; "
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn05	;Wait for the next byte

ADFsaLstn05
	movf	AD_TPSH,W	;Load the TX push point plus 5 into FSR0
	addlw	5		; "
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn06	;Wait for the next byte

ADFsaLstn06
	movf	AD_TPSH,W	;Load the TX push point plus 6 into FSR0
	addlw	6		; "
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the byte into the TX queue but don't
	movwf	INDF0		; advance the pointer yet
	retlw	low ADFsaLstn07	;Deal with everything when we get the last byte

ADFsaLstn07
	movf	AP_BUF,W	;If the last byte is a count rather than data,
	andlw	B'11111000'	; skip ahead
	xorlw	B'10000000'	; "
	btfsc	STATUS,Z	; "
	bra	ADFL070		; "
	movf	AD_TPSH,W	;Load the TX push point plus 7 into FSR0
	addlw	7		; "
	bsf	WREG,6		; "
	bcf	WREG,7		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Load the last byte into the TX queue
	movwf	INDF0		; "
	movlw	8		;All eight bytes were data, so increment the
	addwf	AD_TLEN,F	; queue length by 8 and increment the pointer
	addwf	AD_TPSH,F	; by 8 and wrap the pointer
	bsf	AD_TPSH,6	; "
	bcf	AD_TPSH,7	; "
	movlb	1		;Enable the TX interrupt so data gets sent
	bsf	PIE1,TXIE	; "
	movlb	2		; "
	retlw	low ADFsaIgnore
ADFL070	movf	AP_BUF,W	;Take the low three bits of the last byte as
	andlw	B'00000111'	; the count of how many of the preceding bytes
	addwf	AD_TLEN,F	; were valid and increase the queue length and
	addwf	AD_TPSH,F	; increment and wrap the pointer
	bsf	AD_TPSH,6	; "
	bcf	AD_TPSH,7	; "
	movlb	1		;Enable the TX interrupt so data gets sent
	bsf	PIE1,TXIE	; "
	movlb	2		; "
	retlw	low ADFsaIgnore

ADFsaLstn3H
	movf	AP_BUF,W	;We can't act on the high byte until we know
	movwf	AU_TEMP		; what the low byte (handler ID) is, so store
	retlw	low ADFsaLstn3L	; it in temporary space

ADFsaLstn3L
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	ADFL3L1		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	ADFL3L0		; detected
	retlw	low ADFsaIgnore	;Dummy ignores other handler IDs
ADFL3L0	btfss	AD_R3H,7	;If a collision has not been detected, skip
	bra	ADFL3L2		; ahead to change the address; if one has been
	bcf	AD_R3H,7	; detected, clear it and ignore this command
	retlw	low ADFsaIgnore	; "
ADFL3L1	bcf	AU_FLAG,AU_SEMD	;Copy the state of the SRQ enable bit to the SRQ
	bcf	AD_R3H,5	; enable flag and to our copy of register 3
	btfsc	AU_TEMP,5	; "
	bsf	AU_FLAG,AU_SEMD	; "
	btfsc	AU_TEMP,5	; "
	bsf	AD_R3H,5	; "
ADFL3L2	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	AU_TEMP,F	; byte as our new address and we're done
	movf	AD_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	AU_TEMP,W	; "
	movwf	AD_R3H		; "
	retlw	low ADFsaIgnore	; "

ADFsaTalk
	lslf	AP_BUF,W	;If register being told to talk is 1 or 2,
	xorwf	AP_BUF,W	; ignore
	btfsc	WREG,1		; "
	retlw	low ADFsaIgnore	; "
	btfsc	AP_BUF,1	;If it's 3, branch into talk 3 handler,
	bra	ADFsaTalk3H	; otherwise it's 0, so fall into talk 0 handler
	;fall through

ADFsaTalk0
	bcf	AU_FLAG,AU_SRMD	;Clear the SRQ flag, will re-set it if need be
	movf	AD_RPOP,W	;If the queue is empty, give no response
	xorwf	AD_RPSH,W	; "
	btfsc	STATUS,Z	; "
	retlw	low ADFsaIgnore	; "
	clrf	AU_TEMP		;Use temp variable as a counter of bytes sent
	bsf	AU_FLAG,AU_TXON	;We are interested in transmission events
	;fall through

ADFsaTalk0By
	movf	AD_RPOP,W	;Load the RX queue pop point into FSR0
	movwf	FSR0L		; "
	movf	INDF0,W		;Pop the next byte off the queue into the ADB
	movwf	AP_BUF		; buffer and set it to be sent
	bsf	AP_FLAG,AP_TXI	; "
	xorlw	0x95		;0x95 bytes have to be doubled up as the driver
	btfsc	STATUS,Z	; treats them specially; we use the MSB of the
	btfsc	AD_TLEN,7	; TX queue length as a flag since it's unused;
	bra	ADFT0B0		; if the byte is 0x95 and the flag is not set,
	bsf	AD_TLEN,7	; set it and skip over incrementing the pointer
	bra	ADFT0B1		; "
ADFT0B0	incf	AD_RPOP,F	;Increment and wrap the pointer
	bcf	AD_RPOP,6	; "
	bcf	AD_TLEN,7	;Clear the 0x95 flag if it was set
ADFT0B1	incf	AU_TEMP,F	;Increment the counter of bytes sent
	movf	AD_RPOP,W	;If the queue is empty, figure out how to pad
	xorwf	AD_RPSH,W	; the rest of the eight bytes
	btfsc	STATUS,Z	; "
	bra	ADFT0B2		; "
	incf	AU_TEMP,W	;If the queue isn't empty and we haven't yet
	btfss	WREG,3		; sent seven data bytes, go around again
	retlw	low ADFsaTalk0By; "
	movf	AD_RPOP,W	;Load the RX queue pop point into FSR0
	movwf	FSR0L		; "
	movf	INDF0,W		;If the upper nibble of the next byte on the
	andlw	B'11110000'	; queue is anything but 0b1000, we can send it
	xorlw	B'10000000'	; as our eighth byte and have it interpreted
	btfss	STATUS,Z	; as data
	retlw	low ADFsaTalk0Fn; "
	bsf	AU_FLAG,AU_SRMD	;Else, raise SRQ flag so it gets sent later
	retlw	low ADFsaTalk0Ct;Send a control byte as our eighth byte
ADFT0B2	movf	AU_TEMP,W	;Depending on how many bytes we've already
	brw			; sent when the queue comes up empty, choose:
	retlw	low ADFsaIgnore	;If none, send no data (shouldn't happen)
	retlw	low ADFsaTalk0D1;If 1, send 6 dummy bytes and a control byte
	retlw	low ADFsaTalk0D2;If 2, send 5 dummy bytes and a control byte
	retlw	low ADFsaTalk0D3;If 3, send 4 dummy bytes and a control byte
	retlw	low ADFsaTalk0D4;If 4, send 3 dummy bytes and a control byte
	retlw	low ADFsaTalk0D5;If 5, send 2 dummy bytes and a control byte
	retlw	low ADFsaTalk0D6;If 6, send 1 dummy byte and a control byte
	retlw	low ADFsaTalk0Ct;If 7, send a control byte

ADFsaTalk0D1
	bsf	AP_FLAG,AP_TXI	;Resend our last byte because it doesn't matter
	retlw	low ADFsaTalk0D2;Five more dummy bytes to send before control

ADFsaTalk0D2
	bsf	AP_FLAG,AP_TXI	;Resend our last byte because it doesn't matter
	retlw	low ADFsaTalk0D3;Four more dummy bytes to send before control

ADFsaTalk0D3
	bsf	AP_FLAG,AP_TXI	;Resend our last byte because it doesn't matter
	retlw	low ADFsaTalk0D4;Three more dummy bytes to send before control

ADFsaTalk0D4
	bsf	AP_FLAG,AP_TXI	;Resend our last byte because it doesn't matter
	retlw	low ADFsaTalk0D5;Two more dummy bytes to send before control

ADFsaTalk0D5
	bsf	AP_FLAG,AP_TXI	;Resend our last byte because it doesn't matter
	retlw	low ADFsaTalk0D6;One more dummy byte to send before control

ADFsaTalk0D6
	bsf	AP_FLAG,AP_TXI	;Resend our last byte because it doesn't matter
	retlw	low ADFsaTalk0Ct;Next byte is control byte

ADFsaTalk0Fn
	movf	AD_RPOP,W	;Load the RX queue pop point into FSR0
	movwf	FSR0L		; "
	movf	INDF0,W		;Pop the next byte off the queue into the ADB
	movwf	AP_BUF		; buffer and set it to be sent
	bsf	AP_FLAG,AP_TXI	; "
	xorlw	0x95		;0x95 bytes have to be doubled up as the driver
	btfsc	STATUS,Z	; treats them specially; we use the MSB of the
	btfsc	AD_TLEN,7	; TX queue length as a flag since it's unused;
	bra	ADFT0F0		; if the byte is 0x95 and the flag is not set,
	bsf	AD_TLEN,7	; set it and skip over incrementing the pointer
	bra	ADFT0F1		; "
ADFT0F0	incf	AD_RPOP,F	;Increment and wrap the pointer
	bcf	AD_RPOP,6	; "
	bcf	AD_TLEN,7	;Clear the 0x95 flag if it was set
ADFT0F1	movf	AD_RPOP,W	;If the queue is not yet empty, set the flag so
	xorwf	AD_RPSH,W	; the modem signals for service again
	btfss	STATUS,Z	; "
	bsf	AU_FLAG,AU_SRMD	; "
	bcf	AU_FLAG,AU_TXON	;We are no longer interested in transmit events
	retlw	low ADFsaIgnore	;Done

ADFsaTalk0Ct
	movf	AU_TEMP,W	;If it's not a data byte from 0x00-0x7F, last
	iorlw	B'10000000'	; byte should be a counter of how many of the
	movwf	AP_BUF		; preceding bytes are valid with the MSB set
	bsf	AP_FLAG,AP_TXI	; "
	bcf	AU_FLAG,AU_TXON	;We are no longer interested in transmit events
	retlw	low ADFsaIgnore	;Done

;Serial number "used to be": 44 43 D1 1C 52 E8 E1 E1

ADFsaTalk3H
	movf	AD_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	iorwf	AP_BUF,F	; collision detection
	movlb	2		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	AU_FLAG,AU_TXON	; and we are interested in transmission events
	retlw	low ADFsaTalk3L	;Deal with what happened in the next state

ADFsaTalk3L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ADFT3L0		; bit of register 3, clear the transmit events
	bsf	AD_R3H,7	; flag and we're done until we get another
	bcf	AU_FLAG,AU_TXON	; command
	retlw	low ADFsaIgnore	; "
ADFT3L0	movf	AD_R3L,W	;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	AU_FLAG,AU_TXON	; and that we no longer want transmit events
	retlw	low ADFsaTalk3E	;Deal with what happened in the next state

ADFsaTalk3E
	btfsc	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bsf	AD_R3H,7	; bit of register 3
	retlw	low ADFsaIgnore	;Either way, we're done


AdbFsa	org	0xF00

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	TRISA,AP_APIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	TRISA,AP_APIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	bsf	INTCON,TMR0IE	;Set timer to interrupt and bring us to the
	retlw	low AdbFsaTxBitD; transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	TRISA,AP_APIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	TRISA,AP_APIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	PORTA,AP_APIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	IOCAP,AP_APIN	; set by releasing the pin so the timer we just
	bsf	IOCAN,AP_APIN	; set doesn't immediately get reset
	bcf	IOCAF,AP_APIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	retlw	low AdbFsaRxBit	; "

AdbFsaRxBit	
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBit	; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a command has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBit	; "


;;; End of Program ;;;
	end
