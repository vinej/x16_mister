.setcpu "65C02"
; ============================================================================
; BUG3 (SAVE corrupts image) write-path test -- faithful R49 sdcard.s flow.
;
; Replicates the DOS read-modify-write pattern of a SAVE:
;   CMD17 lba0 (read FAT-like sector)          -> $0400
;   CMD24 lba0 (write it back, patternA)          <- R49 shape: send_cmd =
;       deselect+select+wait_ready, R1 poll, wait_ready, $FE token, 512 bytes,
;       2 dummy CRC, then IMMEDIATE DESELECT (no DRESP/busy wait -- R49 quirk!)
;   CMD24 lba1 (write dir-like sector, patternB)  <- back-to-back write
;   CMD17 lba0 -> $0600, CMD17 lba1 -> $0800 (read-back verify through sd_card)
;   $00=$AA done.
;
; ZP results: $03=R1 of CMD24#1  $04=R1 of CMD24#2  $0A=error code (0=ok)
; patternA byte = offset ^ $5A ; patternB byte = offset ^ $C3
; ============================================================================

SPI_DATA = $9F3E
SPI_CTRL = $9F3F

.zeropage
done:   .res 1          ; $00
r24a:   .res 1          ; $01  R1 of first CMD24
r24b:   .res 1          ; $02  R1 of second CMD24
r17a:   .res 1          ; $03  R1 of first CMD17
errc:   .res 1          ; $04  error code (1=wait_ready timeout, 2=token timeout)
ptr:    .res 2          ; $05/$06
cnt:    .res 2          ; $07/$08
lbab:   .res 1          ; $09  lba low byte for cmd (0 or 1)
xorv:   .res 1          ; $0A  pattern xor value
off:    .res 1          ; $0B  offset counter (low 8 bits)

.segment "CODE"
reset:
        sei
        ldx #$ff
        txs
        stz done
        stz errc

        ; --- CMD17 lba0 -> $0400 (the "read FAT sector" step) ---
        lda #0
        sta lbab
        jsr read_sector
        lda #$00
        sta ptr
        lda #$04
        sta ptr+1
        jsr read512
        jsr desel               ; R49 deselects after read too

        ; --- CMD24 lba0, pattern A (write the FAT sector back) ---
        lda #0
        sta lbab
        lda #$5A
        sta xorv
        jsr write_sector
        sta r24a

        ; --- CMD24 lba1, pattern B (write the dir sector) -- FAST no-poll loop ---
        lda #1
        sta lbab
        lda #$C3
        sta xorv
        jsr write_sector_fast
        sta r24b

        ; --- read back both through sd_card ---
        lda #0
        sta lbab
        jsr read_sector
        lda #$00
        sta ptr
        lda #$06
        sta ptr+1
        jsr read512
        jsr desel

        lda #1
        sta lbab
        jsr read_sector
        lda #$00
        sta ptr
        lda #$08
        sta ptr+1
        jsr read512
        jsr desel

        lda #$aa
        sta done
halt:   jmp halt

; ---------------------------------------------------------------------------
spi:                            ; xfer A -> returns received byte in A
        sta SPI_DATA
:       bit SPI_CTRL
        bmi :-
        lda SPI_DATA
        rts

spi_wr:                         ; R49 spi_write: sta + poll busy, NO read-back
        sta SPI_DATA
:       bit SPI_CTRL
        bmi :-
        rts

spi_ff: lda #$ff
        bra spi

desel:                          ; R49 deselect: clear bit0, one clock-out read
        lda SPI_CTRL
        and #$FE
        sta SPI_CTRL
        bra spi_ff

sel:                            ; R49 select: set bit0, clock, wait_ready
        lda SPI_CTRL
        ora #$01
        sta SPI_CTRL
        jsr spi_ff
        ; fall through to wait_ready

wait_ready:                     ; poll until $FF (R49: ~500ms budget)
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
        sta errc                ; timeout
@ok:    rts

; send command: A=cmd byte, lbab=lba low byte (arg = 0..255), X=crc
; R49 send_cmd shape: deselect, select(+wait_ready), 6 bytes, poll R1 (<=10)
send_cmd:
        pha
        phx
        jsr desel
        jsr sel
        plx
        pla
        jsr spi                 ; cmd
        lda #$00
        jsr spi                 ; arg 31:24
        lda #$00
        jsr spi                 ; arg 23:16
        lda #$00
        jsr spi                 ; arg 15:8
        lda lbab
        jsr spi                 ; arg 7:0
        txa
        jsr spi                 ; crc
        ldy #10
@r1:    jsr spi_ff
        bit #$80
        beq @got
        dey
        bne @r1
@got:   rts

read_sector:                    ; CMD17 lbab, wait token (leaves card SELECTED)
        lda #$51
        ldx #$01
        jsr send_cmd
        sta r17a
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
        sta errc                ; token timeout
@tok:   rts

read512:                        ; 512 bytes -> (ptr)
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

; write sector lbab with pattern (offset ^ xorv), R49 SLOW shape; R1 in A
write_sector:
        lda #$58                ; CMD24
        ldx #$01
        jsr send_cmd
        pha                     ; save R1
        jsr wait_ready          ; R49: wait for card ready before token
        lda #$FE
        jsr spi_wr              ; data token (R49 uses spi_write)
        ; 512 data bytes: two 256-byte halves, byte = off ^ xorv
        stz off
@h1:    lda off
        eor xorv
        jsr spi_wr              ; sta + poll, no read-back (R49 slow path)
        inc off
        bne @h1
@h2:    lda off
        eor xorv
        eor #$FF                ; second half: (off ^ xorv ^ FF) to stay distinct
        jsr spi_wr
        inc off
        bne @h2
        lda #$00                ; dummy CRC
        jsr spi_wr
        lda #$00
        jsr spi_wr
        jsr desel               ; R49 QUIRK: deselect IMMEDIATELY (no DRESP/busy)
        pla                     ; return R1
        rts

; write sector lbab, R49 FAST shape: back-to-back sta SPI_DATA, NO polling
; (sdcard.s @1f: lda buf,y / sta SPI_DATA / iny / bne = 13 cycles/byte on
; real HW; only spi_stall protects it on ours).  R1 in A.
write_sector_fast:
        lda #$58                ; CMD24
        ldx #$01
        jsr send_cmd
        pha
        jsr wait_ready
        lda #$FE
        jsr spi_wr              ; token (R49 uses spi_write even in fast mode)
        stz off
@f1:    lda off                 ; 3
        eor xorv                ; 3
        sta SPI_DATA            ; 4   <- NO poll: back-to-back like R49 fast
        inc off                 ; 5
        bne @f1                 ; 3
@f2:    lda off
        eor xorv
        eor #$FF
        sta SPI_DATA
        inc off
        bne @f2
        lda #$00                ; dummy CRC (R49 fast path also uses spi_write)
        jsr spi_wr
        lda #$00
        jsr spi_wr
        jsr desel               ; immediate deselect
        pla
        rts

.segment "VECTORS"
        .word reset             ; NMI
        .word reset             ; RESET
        .word reset             ; IRQ
