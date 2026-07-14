.setcpu "65C02"

SPI_DATA = $9F3E
SPI_CTRL = $9F3F

.zeropage
done:   .res 1          ; $00 = done marker (must NOT collide with ptr)
        .res 1
ptr:    .res 2          ; $02/$03
cnt:    .res 2          ; $04/$05

.segment "CODE"
reset:
        sei
        ldx #$ff
        txs
        lda #$01
        sta SPI_CTRL          ; select card

        ; CMD17 read block 0
        lda #$51
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$01
        jsr spi

        ; wait for 0xFE token (generous: ~200*256 polls, like the real ROM)
        ldx #200
waittok_o:
        ldy #255
waittok:
        lda #$ff
        jsr spi
        cmp #$fe
        beq gottok
        dey
        bne waittok
        dex
        bne waittok_o
gottok:
        ; read 512 bytes into $0400..$05FF
        lda #$00
        sta ptr
        lda #$04
        sta ptr+1
        lda #$00
        sta cnt
        lda #$02
        sta cnt+1            ; cnt = $0200 = 512
rdloop:
        lda #$ff
        jsr spi
        ldy #0
        sta (ptr),y
        inc ptr
        bne :+
        inc ptr+1
:
        lda cnt
        bne :+
        dec cnt+1
:
        dec cnt
        lda cnt
        ora cnt+1
        bne rdloop

        lda #$aa
        sta done             ; DONE marker (testbench watches $00)
halt:   jmp halt

; A = tx byte; returns received byte in A
spi:
        sta SPI_DATA
poll:   bit SPI_CTRL         ; N <- bit7 (busy)
        bmi poll
        lda SPI_DATA
        rts

.segment "VECTORS"
        .word reset          ; NMI
        .word reset          ; RESET
        .word reset          ; IRQ
