# mj65c02 -- 65C02 CPU core

Vendored 2026-07-08 from https://github.com/MJoergen/65c02 (MIT license,
see LICENSE).  Rockwell 65C02 instruction set (incl. RMB/SMB/BBR/BBS, no
undefined opcodes); WAI/STP are NOT implemented (rtl/wai_shim.sv provides
them at the system level).  Validated upstream against the Klaus Dormann
6502/65C02 functional test suite in hardware, and previously booted the
X16 ROM in the MJoergen/x16-nexys4ddr project.

Local integration: rtl/cpu/mj65c02/mj65c02_wrap.vhd (same port shape as
r65c02_wrap/p65c816_wrap).  Local modifications to the upstream sources
(all marked with `jyv 2026-07-11` comments):

1. cpu_65c02.vhd: added a sync_o output (opcode-fetch indicator, the exact
   expression the bundled debug module already received as sync_i).
2. Full STRICT VHDL-93 port (upstream is VHDL-2008; Quartus Prime
   Standard's 2008 support is too partial to rely on):
   * `use ieee.numeric_std_unsigned.all` -> `use ieee.std_logic_unsigned.all`
     in all synthesis files; the four `to_integer` calls became
     `conv_integer` (Quartus Std's ieee lacks the 2008 package, 10481).
   * cpu_65c02.vhd: out-mode ports (addr_o, wr_en_o, wr_data_o,
     ioport_dir_o, ioport_out_o) were read internally -- 2008-only
     (Quartus 10577/10600); rewritten via internal signals + final
     assignments.
   * mj65c02_wrap.vhd: `not x` port-map actuals (2008 expression actuals)
     moved onto intermediate signals.
   * control.vhd: the if/elsif/else-generate chain (2008) became three
     complementary VHDL-93 if-generates.
   sim/run.sh compiles the core with `vcom -93` precisely to prove
   93-cleanliness before Quartus ever sees the files.
3. cpu_65c02.vhd: the G_SIM debug generate (debug.vhd + fmt.vhd tracing)
   is removed; both files stay in the tree for reference but are compiled
   by neither Quartus nor ModelSim.  debug_o is tied to zero.

Build note: the core compiles into its own VHDL library `mj65c02` -- its
entity names (alu, pc, sr, ...) collide case-insensitively with the
P65C816's in `work`.  Only mj65c02_wrap.vhd lives in `work`.
