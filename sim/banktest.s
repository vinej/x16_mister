; ============================================================================
; banktest.s -- CPU-in-the-loop test of the 256-ROM-bank / cart feature.
;
; Boots from ROM bank 0 at $C000, copies the worker to LowRAM $0400 (bank-
; switching code must not run from the switched window), then:
;   t0  ROM bank register at $0001 holds all 8 bits
;   t1  HiRAM SDRAM banks: r/w + bank isolation (regression of phase f)
;   t1b HiRAM BRAM bank 0 (regression)
;   t2  cart banks 32-255: isolated write/read + bank isolation ($C000/$FFF0)
;   t3  cart sequential indexed fill + verify (back-to-back window reads)
;   t4  EXECUTE code from a cart bank (sequential opcode fetches from SDRAM)
;   t5  unpopulated ROM banks 16-31 read $FF, writes ignored
;   t6  bank 0 system ROM still intact; cart data retained across all tests
;
; Markers (LowRAM): $0002 = $AA pass / $EE fail; $0003 = fail code;
;                   $0004 = actual value that failed the compare.
; ============================================================================
.setcpu "65C02"

RAMB  = $00           ; RAM bank register (also plain LowRAM byte)
ROMB  = $01           ; ROM bank register (also plain LowRAM byte)
DONE  = $02
FCODE = $03
FVAL  = $04
EXPT  = $10           ; scratch: expected value in t3

.import __LOWCODE_LOAD__, __LOWCODE_SIZE__

; compare A against an immediate; on mismatch record actual+code and fail
.macro CHECK val, code
    cmp #val
    beq :+
    sta FVAL
    lda #code
    sta FCODE
    jmp fail
:
.endmacro

; ----------------------------------------------------------------------------
.segment "CODE"
reset:
    sei                       ; $78 = ROM signature byte checked in t6
    ldx #$FF
    txs
    ldx #0
@cl:lda __LOWCODE_LOAD__+0,x
    sta $0400,x
    lda __LOWCODE_LOAD__+256,x
    sta $0500,x
    lda __LOWCODE_LOAD__+512,x
    sta $0600,x
    inx
    bne @cl
    jmp $0400

; ----------------------------------------------------------------------------
.segment "LOWCODE"
worker:
    ; ---- t0: ROM bank register holds all 8 bits ----
    lda #200
    sta ROMB
    lda ROMB
    CHECK 200, 10
    stz ROMB

    ; ---- t1: HiRAM SDRAM banks r/w + isolation ----
    lda #2
    sta RAMB
    lda #$11
    sta $A000
    lda #$77
    sta $BFFF
    lda #3
    sta RAMB
    lda #$22
    sta $A000
    lda #2
    sta RAMB
    lda $A000
    CHECK $11, 11
    lda $BFFF
    CHECK $77, 12
    lda #3
    sta RAMB
    lda $A000
    CHECK $22, 13

    ; ---- t1b: HiRAM BRAM bank 0 regression ----
    stz RAMB
    lda #$44
    sta $A000
    lda $A000
    CHECK $44, 14

    ; ---- t2: cart banks r/w + isolation ----
    lda #40
    sta ROMB
    lda #$5A
    sta $C000
    lda #$66
    sta $FFF0
    lda #41
    sta ROMB
    lda #$C3
    sta $C000
    lda #40
    sta ROMB
    lda $C000
    CHECK $5A, 21
    lda $FFF0
    CHECK $66, 22
    lda #41
    sta ROMB
    lda $C000
    CHECK $C3, 23

    ; ---- t3: sequential indexed fill + verify in cart bank 40 ----
    lda #40
    sta ROMB
    ldx #0
@f3:txa
    eor #$5C
    sta $C200,x
    inx
    cpx #32
    bne @f3
    ldx #0
@v3:txa
    eor #$5C
    sta EXPT
    lda $C200,x
    cmp EXPT
    beq :+
    sta FVAL
    lda #31
    sta FCODE
    jmp fail
:   inx
    cpx #32
    bne @v3

    ; ---- t4: execute code FROM a cart bank ----
    lda #40
    sta ROMB
    ldx #0
@f4:lda cartsub,x
    sta $C100,x
    inx
    cpx #(cartsub_end - cartsub)
    bne @f4
    stz $20
    jsr $C100
    lda $20
    CHECK $7B, 41             ; $77 + 4 x inc

    ; ---- t5: unpopulated ROM banks 16-31 ----
    lda #20
    sta ROMB
    lda $C123
    CHECK $FF, 51
    lda #$00
    sta $C123
    lda $C123
    CHECK $FF, 52             ; write must have been ignored

    ; ---- t6: bank 0 ROM intact + cart retention ----
    stz ROMB
    lda $C000
    CHECK $78, 61             ; sei = first ROM byte
    lda #40
    sta ROMB
    lda $C000
    CHECK $5A, 62
    lda #41
    sta ROMB
    lda $C000
    CHECK $C3, 63
    stz ROMB

    ; ---- t7: cart LOADER data (TB streamed "CX16"+patterns into bank 32
    ;          through the ld_* port before releasing the CPU) ----
    lda #32
    sta ROMB
    lda $C000
    CHECK $43, 71             ; 'C'
    lda $C001
    CHECK $58, 72             ; 'X'
    lda $C002
    CHECK $31, 73             ; '1'
    lda $C003
    CHECK $36, 74             ; '6'
    lda $C004
    CHECK $A1, 75             ; 4 ^ $A5
    lda $FEFF
    CHECK $99, 76             ; bank 32 offset $3EFF
    stz ROMB

    ; ---- t8: EXACT r49 ramtas RAM-bank count (memory.s) -> expect 256 ----
    ; phase 1: find first bank whose $A000 write wraps into bank 0
    stz RAMB
    ldx $A000       ;get value from 00:a000
    inx             ;use value + 1 as test value for other banks
    ldy #1          ;bank to test
@p1:sty RAMB
    lda $A000       ;save current value
    stx $A000       ;write test value
    stz RAMB
    cpx $A000       ;check if 00:a000 is affected = wrap-around
    beq @wrapd
    sty RAMB
    sta $A000       ;restore value
    iny             ;next bank
    bne @p1
    bra @p2
@wrapd:
    sty $05         ;diag: phase-1 wrap bank
@p2:stz RAMB        ;restore value in 00:a000
    dex
    stx $A000
    ; phase 2: 8-cell EOR test per bank, stop at wrap bank (Y) or first fail
    ldx #1          ;start testing from bank 1
    stx RAMB
@bk:ldx #8          ;test 8 addresses in each bank
@cl:lda $A000,x     ;read, xor, write, compare
    eor #$FF
    sta $A000,x
    cmp $A000,x
    bne @cellf      ;test failed, we are done
    eor #$FF        ;restore value
    sta $A000,x
    dex             ;test next address
    bne @cl
    inc RAMB        ;select next bank
    cpy RAMB        ;stop at last bank that does not wrap-around to bank0
    bne @bk
    lda #2          ;diag: exited via cpy-stop
    sta $06
    bra @t8done
@cellf:
    sta $07         ;diag: expected (eor'd) value
    lda $A000,x
    sta $08         ;diag: re-read actual
    stx $09         ;diag: failing X offset
    lda #1          ;diag: exited via cell fail
    sta $06
@t8done:
    lda RAMB        ;number of RAM banks (0 = wrapped = 256 banks)
    CHECK 0, 81
    stz RAMB

    lda #$AA
    sta DONE
@hp:bra @hp

fail:
    lda #$EE
    sta DONE
@hf:bra @hf

; runs at $C100 in cart bank 40 (copied there by t4); position-independent
cartsub:
    lda #$77
    sta $20
    ldx #4
@l: inc $20
    dex
    bne @l
    rts
cartsub_end:

; ----------------------------------------------------------------------------
.segment "VECTORS"
    .word reset, reset, reset
