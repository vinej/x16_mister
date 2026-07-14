.setcpu "65C02"
; ============================================================================
; BUG1 wedge reproduction: the X16 ROM probes the SD card BEFORE the OSD image
; is mounted.  Without the vsd_sel ss-gating (the BBC/ZX idiom), sd_card
; accepts the pre-mount CMD17 and wedges read_state in RD_STATE_WAIT_IO with
; sd_rd pending.  This program replays that flow:
;   phase 1 (pre-mount): select, CMD17 lba0, SHORT token poll (times out like
;            the ROM), deselect, set $01=$11 -> testbench mounts the image.
;   wait:    poll $02 until the testbench sets it (mount done).
;   phase 2 (post-mount, NO core reset -- the user just types @$):
;            select, CMD0 (R1 -> $03), CMD8 (R1 -> $04, +4 echo bytes),
;            CMD17 lba0    -> 512 bytes to $0400,
;            CMD17 lba2048 -> 512 bytes to $0600,
;            $00=$AA (done).
; The testbench verifies both sectors against what the hps model served.
; ============================================================================

SPI_DATA = $9F3E
SPI_CTRL = $9F3F

.zeropage
done:   .res 1          ; $00  $AA when finished
p1d:    .res 1          ; $01  $11 when phase 1 (pre-mount probe) is done
mnt:    .res 1          ; $02  set to 1 BY THE TESTBENCH when image mounted
r1a:    .res 1          ; $03  R1 response to phase-2 CMD0  (expect $01)
r1b:    .res 1          ; $04  R1 response to phase-2 CMD8  (expect $01)
ptr:    .res 2          ; $05/$06
cnt:    .res 2          ; $07/$08

.segment "CODE"
reset:
        sei
        ldx #$ff
        txs

        ; ---------------- phase 1: pre-mount probe (like the ROM at boot) ---
        lda #$01
        sta SPI_CTRL          ; select card
        jsr cmd17_lba0

        ; short token poll -- times out (no image -> no fill -> no token)
        ldx #2
p1o:    ldy #32
p1i:    jsr spi_ff
        cmp #$fe
        beq p1done            ; (tolerated; not expected)
        dey
        bne p1i
        dex
        bne p1o
p1done:
        stz SPI_CTRL          ; deselect
        jsr spi_ff            ; clock out the deselect (like the ROM)
        lda #$11
        sta p1d               ; tell the TB: probe done, please mount

        ; ---------------- wait for the TB to mount the image ----------------
wmnt:   lda mnt
        beq wmnt

        ; ---------------- phase 2: what @$ does after the OSD mount ---------
        lda #$01
        sta SPI_CTRL          ; select

        lda #$40              ; CMD0 GO_IDLE
        ldx #$95
        jsr cmd_a0
        sta r1a

        lda #$48              ; CMD8 SEND_IF_COND
        ldx #$87
        jsr cmd_a0
        sta r1b
        jsr spi_ff            ; 4 echo bytes of R7
        jsr spi_ff
        jsr spi_ff
        jsr spi_ff

        ; ---- CMD17 lba 0 -> $0400 ----
        jsr cmd17_lba0
        jsr waittok
        lda #$00
        sta ptr
        lda #$04
        sta ptr+1
        jsr read512

        ; ---- CMD17 lba 2048 -> $0600 ----
        lda #$51
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$08
        jsr spi
        lda #$00
        jsr spi
        lda #$01
        jsr spi
        jsr waittok
        lda #$00
        sta ptr
        lda #$06
        sta ptr+1
        jsr read512

        stz SPI_CTRL          ; deselect
        lda #$aa
        sta done              ; DONE marker (testbench watches $00)
halt:   jmp halt

; ---------------------------------------------------------------------------
; A = tx byte; returns received byte in A
spi:
        sta SPI_DATA
poll:   bit SPI_CTRL          ; N <- bit7 (busy)
        bmi poll
        lda SPI_DATA
        rts

spi_ff: lda #$ff
        bra spi

; send command A with arg 0 and crc X, then poll R1 (<= 10 tries, ROM-style)
cmd_a0:
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        lda #$00
        jsr spi
        txa
        jsr spi
        ldy #10
r1p:    jsr spi_ff
        bit #$80
        beq r1got             ; bit7 clear -> R1 response
        dey
        bne r1p
r1got:  rts

cmd17_lba0:
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
        jmp spi

; generous token wait (like the real ROM's ~508 ms timeout)
waittok:
        ldx #200
wt_o:   ldy #0
wt_i:   jsr spi_ff
        cmp #$fe
        beq wt_got
        dey
        bne wt_i
        dex
        bne wt_o
wt_got: rts

; read 512 bytes to (ptr)
read512:
        lda #$00
        sta cnt
        lda #$02
        sta cnt+1             ; cnt = $0200 = 512
rdloop: jsr spi_ff
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
        rts

.segment "VECTORS"
        .word reset          ; NMI
        .word reset          ; RESET
        .word reset          ; IRQ
