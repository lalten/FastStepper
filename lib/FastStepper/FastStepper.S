; #include <avr/io.h>

; .extern FastStepper

; .global TIMER1_COMPA_vect
; .type   TIMER1_COMPA_vect, @function

; TIMER1_COMPA_vect:
;     push    r0
;     in      r0, _SFR_IO_ADDR(SREG)
;     push    r26

;     ; only execute on every second edge
;     lds     r26, FastStepper.is_rising_edge_isr
;     com     r26
;     sts     FastStepper.is_rising_edge_isr, r26
;     sbrs    r26, 0
;     rjmp    exit_isr

;     ; r1 is used as zero
;     push    r1
;     eor     r1, r1

;     push    r27
;     push    r28
;     push    r29

;     ; increment or decrement position, based on dir pin state
;     lds     r26, FastStepper.current_position
;     lds	    r27, FastStepper.current_position+1
;     lds	    r28, FastStepper.current_position+2
;     lds	    r29, FastStepper.current_position+3
;     sbic    _SFR_IO_ADDR(PORTB), 4
;     rjmp    decr_pos
; incr_pos:
;     adiw    r26, 1
;     adc     r28, r1
;     adc     r29, r1
;     rjmp    save_pos
; decr_pos:
;     sbiw    r26, 1
;     sbc     r28, r1
;     sbc     r29, r1
; save_pos:
;     sts     FastStepper.current_position,   r26
;     sts     FastStepper.current_position+1, r27
;     sts     FastStepper.current_position+2, r28
;     sts     FastStepper.current_position+3, r29
    
;     push    r22
;     lds     r22, FastStepper.has_target
;     sbrs    r22, 0
;     rjmp    exit_pop_2

;     push    r23
;     push    r24
;     push    r25
;     push    r30
;     push    r31

;     lds     r22, FastStepper.target_position
;     lds	    r23, FastStepper.target_position+1
;     lds	    r24, FastStepper.target_position+2
;     lds	    r25, FastStepper.target_position+3

;     cp      r26, r22
;     cpc     r27, r23
;     cpc     r28, r24
;     cpc     r29, r25
;     breq    reset_virtual_pos

; target_position_reached:
;     ; stop clock
;     lds     r26, TCCR1B
;     andi    r26, 0b11111000
;     sts     TCCR1B, r26
;     ; zero target soeed and current speed
;     sts     FastStepper.target_speed,   r1
;     sts     FastStepper.target_speed+1, r1
;     sts     FastStepper.target_speed+2, r1
;     sts     FastStepper.target_speed+3, r1
;     sts     FastStepper.current_speed,   r1
;     sts     FastStepper.current_speed+1, r1
;     sts     FastStepper.current_speed+2, r1
;     sts     FastStepper.current_speed+3, r1

; reset_virtual_pos:
;     sts     FastStepper.reset_current_position_virtual_offset, r1
;     call    millis
;     sts     FastStepper.time_of_last_step,   r22
;     sts     FastStepper.time_of_last_step+1, r23
;     sts     FastStepper.time_of_last_step+2, r24
;     sts     FastStepper.time_of_last_step+3, r25

; exit_pop_1:
;     pop     r31
;     pop     r30
;     pop     r25
;     pop     r24
;     pop     r23

; exit_pop_2:
;     pop     r22
;     pop     r29
;     pop     r28
;     pop     r27
;     pop     r1

; exit_isr:
;     pop     r26
; 	out	    _SFR_IO_ADDR(SREG), r0
;     pop     r0
; 	reti

; .size TIMER1_COMPA_vect, .-TIMER1_COMPA_vect
