;
; m328-LCD-message.asm
;
; Created: 7/26/2016 10:45:40 AM
; Author : lynf
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
; This file is used as a folder for all versions of m328-LCD-message.asm to
; assemble, link and test the program.
;
; Notes:
; ======
;
; Must have TWI pull-up termination resistors installed else
; interface will not work for slave units like LCD backpack module!
;
.include	"m328-LCD-message(testing).asm"
;
;
.exit
;
