; ============================================================================
; datetest.s -- the PRINT DATE$ crash hunt, run against the REAL RTL.
;
; Faithful transcription of the R49 KERNAL pieces involved in DATE$:
;   * i2c.s inline primitives: SDA/SCL bit-bang on VIA1 DDR-A ($9F03) with
;     the port read on $9F01, incl. scl_high's wait_for_clk poll
;   * i2c_read_byte's transaction shape: START,addr+W,ptr, STOP, START,
;     addr+R, byte, NACK, STOP -- with PHP/SEI/PLP around each byte like
;     the KERNAL (IRQs get windows BETWEEN bytes)
;   * rtc_get_date_time: regs 0..6 + the "seconds changed -> re-read" loop
; ...while a periodic IRQ (TB-driven) runs an SMC read transaction, like
; the VSYNC handler's kbd_scan does on real HW.
;
; Result markers (LowRAM):
;   $0002 = $AA all pass / $EE fail       $0003 = fail code
;   $0010..$0016 = last date/time regs    $0017 = retry count of last call
;   BRK vector -> monitor proxy: sets $0002=$EB (the "crash to monitor")
; ============================================================================
.setcpu "65C02"

DONE   = $02
FCODE  = $03
REGS   = $10        ; sec,min,hour,wkday,date,month,year
RETRY  = $17
SAVSEC = $18
TMP    = $19
ITER   = $1A

PR  = $9F01         ; VIA1 port A
DDR = $9F03         ; VIA1 DDR A
SDA = $01
SCL = $02

.macro sda_low
    lda #SDA
    tsb DDR
.endmacro
.macro sda_high
    lda #SDA
    trb DDR
.endmacro
.macro scl_low
    lda #SCL
    tsb DDR
.endmacro
.macro scl_high
.scope
    lda #SCL
    trb DDR
wfc:lda PR
    and #SCL
    beq wfc
.endscope
.endmacro
.macro send_bit          ; C = bit
.scope
    bcs one
    sda_low
    bra clk
one:sda_high
clk:scl_high
    scl_low
.endscope
.endmacro
.macro rec_bit           ; -> C
    sda_high
    scl_high
    lda PR
    lsr                  ; SDA = bit0 -> C
    scl_low
.endmacro

.segment "CODE"
reset:
    sei
    ldx #$FF
    txs
    stz DONE
    stz FCODE
    stz ITER
    ; i2c_init/restore: clear PA0/1 output latches, release both lines
    lda #SDA|SCL
    trb PR
    sda_high
    scl_high
    cli

main:
    jsr get_date_time
    ; verify against the TB's wall clock: 2026-07-05 21:58:40, wkday 4
    lda REGS+5
    cmp #$07
    beq :+
    lda #51
    jmp fail
:   lda REGS+6
    cmp #$26
    beq :+
    lda #52
    jmp fail
:   lda REGS+2
    cmp #$21
    beq :+
    lda #53
    jmp fail
:   inc ITER
    lda ITER
    cmp #8              ; several full reads with IRQ traffic in between
    bne main
    lda #$AA
    sta DONE
@h: bra @h

fail:
    sta FCODE
    lda #$EE
    sta DONE
@h: bra @h

; ---------------- rtc_get_date_time (rtc.s transcription) ------------------
get_date_time:
    stz RETRY
@again:
    inc RETRY
    lda RETRY
    cmp #6
    bcc :+
    lda #61             ; re-read loop never stabilized
    jmp fail
:   ldx #$6F
    ldy #0
    jsr i2c_read_byte
    sta SAVSEC
    sta REGS+0
    ldy #1
    jsr i2c_read_byte
    sta REGS+1
    ldy #2
    jsr i2c_read_byte
    sta REGS+2
    ldy #3
    jsr i2c_read_byte
    and #$07
    sta REGS+3
    ldy #4
    jsr i2c_read_byte
    sta REGS+4
    ldy #5
    jsr i2c_read_byte
    and #$1F
    sta REGS+5
    ldy #6
    jsr i2c_read_byte
    sta REGS+6
    ldy #0
    jsr i2c_read_byte
    cmp SAVSEC
    bne @again
    rts

; ---------------- i2c_read_byte (i2c.s shape, X=dev Y=ptr) -----------------
; preserves X and Y like the KERNAL version (callers rely on it)
DEVSV = $1B
PTRSV = $1C
i2c_read_byte:
    php
    sei
    stx DEVSV
    sty PTRSV
    ; START, addr+W, ptr
    jsr i2c_start
    lda DEVSV
    asl
    jsr i2c_write
    lda PTRSV
    jsr i2c_write
    jsr i2c_stop
    jsr i2c_start
    lda DEVSV
    asl
    inc                  ; addr+R
    jsr i2c_write
    jsr i2c_read
    ; NACK + STOP  (i2c_read_stop)
    sec
    send_bit
    jsr i2c_stop
    ldx DEVSV
    ldy PTRSV
    lda TMP
    plp
    rts

i2c_start:
    sda_low
    jsr brief
    scl_low
    rts

i2c_stop:
    sda_low
    jsr brief
    scl_high
    jsr brief
    sda_high
    jsr brief
    rts

i2c_write:               ; A = byte
    ldx #8
iwl:rol
    tay
    send_bit
    tya
    dex
    bne iwl
    rec_bit              ; slave ACK slot (ignored, like rtc.s)
    rts

i2c_read:                ; -> TMP
    stz TMP
    ldx #8
irl:rec_bit
    rol TMP
    dex
    bne irl
    rts

brief:                   ; ~5us at 8 MHz
    ldx #12
@l: dex
    bne @l
    rts

; ---------------- IRQ: one SMC read, like kbd_scan every VSYNC -------------
irq:
    pha
    phx
    phy
    tsx                  ; BRK detection: B flag in the pushed P
    lda $0104,x
    and #$10
    bne brkvec
    ldx #$42
    ldy #$18             ; SMC "get keyboard status"
    jsr i2c_read_byte
    sta $20              ; last SMC byte (TB checks it's sane)
    ply
    plx
    pla
    rti

; ---------------- BRK = "crash to monitor" proxy ----------------------------
brkvec:
    lda #$EB
    sta DONE
@h: bra @h

.segment "VECTORS"
    .word brkvec         ; NMI
    .word reset          ; RESET
    .word irq            ; IRQ; BRK detected inside via the B flag
