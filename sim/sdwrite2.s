.setcpu "65C02"
; ============================================================================
; BUG3 v2: SAVE write path with the REAL data source -- sector_buffer in
; banked HiRAM at $B9C6 (the actual R49 address, fat32.map), read back with
; the exact R49 loop shape (lda sector_buffer,y / two 256-byte halves, odd
; base => page crossings at y=$3A).  Image forensics showed odd-offset bytes
; arriving STALE (previous byte's value) -- hunting that here in RTL.
;
;   phase 1: fill $B9C6..$BBC5 with pattern (n^$5A / n^$A5 halves)
;   phase 2: CMD24 lba0, R49 slow write loop reading the HiRAM buffer
;   phase 3: CMD17 lba0 read-back -> $0600
;   $00=$AA done.  TB verifies the image AND the read-back.
; ============================================================================

SPI_DATA = $9F3E
SPI_CTRL = $9F3F
SECBUF   = $B9C6          ; the real R49 sector_buffer address (HiRAM bank 0)

.zeropage
done:   .res 1            ; $00
r24:    .res 1            ; $01  R1 of CMD24
r17:    .res 1            ; $02  R1 of CMD17
errc:   .res 1            ; $03
ptr:    .res 2            ; $04/$05
cnt:    .res 2            ; $06/$07
lbab:   .res 1            ; $08

.segment "CODE"
reset:
        sei
        ldx #$ff
        txs
        stz done
        stz errc

        ; ---- fill the HiRAM sector buffer with the pattern ----
        ldy #0
f1:     tya
        eor #$5A
        sta SECBUF,y
        iny
        bne f1
f2:     tya
        eor #$A5
        sta SECBUF+256,y
        iny
        bne f2

        ; ---- CMD24 lba0: the R49 slow write, reading HiRAM ----
        stz lbab
        lda #$58
        ldx #$01
        jsr send_cmd
        sta r24
        jsr wait_ready
        lda #$FE
        jsr spi_wr

        ldy #0
w1:     lda SECBUF,y      ; REAL R49 shape: abs,y from the odd HiRAM base
        sta SPI_DATA
:       bit SPI_CTRL
        bmi :-
        iny
        bne w1
w2:     lda SECBUF+256,y
        sta SPI_DATA
:       bit SPI_CTRL
        bmi :-
        iny
        bne w2

        lda #$00          ; dummy CRC
        jsr spi_wr
        lda #$00
        jsr spi_wr
        jsr desel         ; R49 quirk: immediate deselect

        ; ---- CMD17 lba0 read-back -> $0600 ----
        stz lbab
        jsr read_sector
        lda #$00
        sta ptr
        lda #$06
        sta ptr+1
        jsr read512
        jsr desel

        lda #$aa
        sta done
halt:   jmp halt

; ---------------------------------------------------------------------------
spi:    sta SPI_DATA
:       bit SPI_CTRL
        bmi :-
        lda SPI_DATA
        rts

spi_wr: sta SPI_DATA
:       bit SPI_CTRL
        bmi :-
        rts

spi_ff: lda #$ff
        bra spi

desel:  lda SPI_CTRL
        and #$FE
        sta SPI_CTRL
        bra spi_ff

sel:    lda SPI_CTRL
        ora #$01
        sta SPI_CTRL
        jsr spi_ff
wait_ready:
        ldx #200
@1:     ldy #0
@2:     jsr spi_ff
        cmp #$ff
        beq @ok
        dey
        bne @2
        dex
        bne @1
        lda #1
        sta errc
@ok:    rts

send_cmd:
        pha
        phx
        jsr desel
        jsr sel
        plx
        pla
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda lbab
        jsr spi
        txa
        jsr spi
        ldy #10
@r1:    jsr spi_ff
        bit #$80
        beq @got
        dey
        bne @r1
@got:   rts

read_sector:
        lda #$51
        ldx #$01
        jsr send_cmd
        sta r17
        ldx #200
@o:     ldy #0
@i:     jsr spi_ff
        cmp #$fe
        beq @tok
        dey
        bne @i
        dex
        bne @o
        lda #2
        sta errc
@tok:   rts

read512:
        lda #$00
        sta cnt
        lda #$02
        sta cnt+1
@l:     jsr spi_ff
        ldy #0
        sta (ptr),y
        inc ptr
        bne :+
        inc ptr+1
:       lda cnt
        bne :+
        dec cnt+1
:       dec cnt
        lda cnt
        ora cnt+1
        bne @l
        rts

.segment "VECTORS"
        .word reset
        .word reset
        .word reset
