@echo off
rem Launch the X16 emulator with the r49 ROM and the test SD image --
rem the reference setup for comparing ML apps against the MiSTer core.
cd /d C:\quartus\projects\x16_forth\emulator
start "" x16emu.exe -rom C:\quartus\projects\x16_mister\rom\r49.bin -sdcard C:\quartus\projects\x16_mister\rom\x16_rc3.img
