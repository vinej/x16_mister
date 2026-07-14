--============================================================================
-- p65c816_wrap.vhd  -  Drop-in replacement for r65c02_wrap / t65_wrap.
--
-- Wraps the P65C816 core (P65816 project) so it presents the exact same
-- port shape as the existing wrappers.  C5G_x16.sv only needs to change
-- the entity name in the instantiation:
--
--   BEFORE (65C02):
--     r65c02_wrap u_cpu (
--         .clk    (cpu_clk),
--         .enable (cpu_rdy),
--         .res_n  (cpu_reset_n),
--         .irq_n  (1'b1),
--         .nmi_n  (1'b1),
--         .rdy    (1'b1),
--         .r_w_n  (cpu_rwn),
--         .sync   (cpu_sync),
--         .addr   (cpu_a),
--         .din    (cpu_di),
--         .dout   (cpu_do),
--         .pc     (cpu_pc)
--     );
--
--   AFTER (65C816):
--     p65c816_wrap u_cpu (
--         .clk    (cpu_clk),
--         .enable (cpu_rdy),
--         .res_n  (cpu_reset_n),
--         .irq_n  (1'b1),
--         .nmi_n  (1'b1),
--         .rdy    (1'b1),
--         .r_w_n  (cpu_rwn),
--         .sync   (cpu_sync),
--         .addr   (cpu_a),
--         .din    (cpu_di),
--         .dout   (cpu_do),
--         .pc     (cpu_pc)
--     );
--
-- IMPORTANT NOTES FOR X16 EMULATION:
--
--   1. EMULATION MODE (E=1):  The 65C816 resets in emulation mode (E=1),
--      which makes it behave exactly like a 65C02.  The X16 KERNAL never
--      issues the XCE opcode to switch to native mode, so this wrapper is
--      safe to use as a direct replacement.  If native-mode support is
--      ever needed, expose the E flag from P65C816 as an extra output.
--
--   2. ADDRESS BUS:  P65C816 exposes a 24-bit address bus (A_OUT[23:0]).
--      In emulation mode (E=1) the upper byte is always the Program Bank
--      Register (PBR=0 at reset, stays 0 unless XCE is issued).  This
--      wrapper exposes only A_OUT[15:0] on the `addr` port, which is all
--      the X16 memory map requires.  The bank byte (A_OUT[23:16]) is
--      discarded -- it will be 0x00 in emulation mode.
--
--   3. SYNC signal:  The 65816 does not have a dedicated SYNC pin.  The
--      equivalent is VPA=1 AND VDA=1, which is asserted only on the opcode
--      byte fetch (first bus cycle of every instruction) -- identical
--      semantics to the 65C02 SYNC.  This wrapper reconstructs SYNC from
--      those two signals.
--
--   4. WE / R_W_N:  Despite the signal name, P65C816 drives WE low on
--      write cycles and high otherwise.  That matches the X16 wrapper
--      convention for R_W_N directly, so no inversion is required.
--
--   5. RDY / CE:  P65C816 has two stall inputs:
--        - CE  (clock enable)   - mapped to '1' (always clocked)
--        - RDY_IN (ready/stall) - mapped to the `enable` port, which is
--          the VERA read-stall signal cpu_rdy in C5G_x16.sv.
--      The `rdy` port of this wrapper is unused (tied to '1' internally),
--      matching the convention of r65c02_wrap.
--
--   6. ABORT_N:  Not used by the X16 hardware; tied to '1' here.  The
--      signal is only meaningful for WAI (Wait for Interrupt) on hardware
--      with an ABORT input.
--
--   7. PC reconstruction:  P65C816 does not expose a PC port.  The PC is
--      reconstructed by latching A_OUT[15:0] on every SYNC cycle, gated
--      by `enable`.  During the SYNC cycle itself the live address is
--      forwarded combinationally (same behaviour as r65c02_wrap) so that
--      equality probes like `cpu_sync && cpu_a == 16'hC028` trigger on
--      the correct cycle.
--
--   8. QUARTUS PROJECT:  Add all files from rtl/cpu/65C816/ to the
--      Quartus project and set the library to "work" (no separate library
--      partition is needed -- P65816_pkg, the sub-modules and P65C816
--      all use `library work`).  Also add this wrapper file.
--      Then replace r65c02_wrap with p65c816_wrap everywhere in the .qsf:
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/P65816_pkg.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/ALU.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/AddSubBCD.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/BCDAdder.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/AddrGen.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/MCode.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/P65C816.vhd
--        set_global_assignment -name VHDL_FILE rtl/cpu/65C816/p65c816_wrap.vhd
--============================================================================
library IEEE;
use IEEE.std_logic_1164.all;

library work;
use work.P65816_pkg.all;

entity p65c816_wrap is
    port (
        clk      : in  std_logic;
        enable   : in  std_logic;                       -- VERA stall / cpu_rdy
        res_n    : in  std_logic;
        irq_n    : in  std_logic;
        nmi_n    : in  std_logic;
        rdy      : in  std_logic;                       -- unused; tie to '1'
        r_w_n    : out std_logic;
        sync     : out std_logic;
        addr     : out std_logic_vector(15 downto 0);
        din      : in  std_logic_vector(7 downto 0);
        dout     : out std_logic_vector(7 downto 0);
        pc       : out std_logic_vector(15 downto 0);
        emu_mode : out std_logic;                       -- 1=emulation, 0=native
        i_flag   : out std_logic;                       -- current I (IRQ-disable) flag
        -- VPB, active low: asserted during interrupt vector pulls.  The X16
        -- platform (R44+ ROMs) REQUIRES vector pulls to SET the hardware ROM
        -- bank latch to 0 (the zp $01 shadow keeps the old value for the
        -- handler to save) -- the c816 KERNAL RAM stubs jump straight into
        -- bank-0 ROM and crash into whatever bank is live without this.
        vpb      : out std_logic;
        -- bus_valid = VDA or VPA: the address bus carries a real memory
        -- access this cycle.  On internal cycles P65C816's A_OUT holds
        -- in-flight address math (AA+inc etc.) -- ghost addresses that,
        -- undecoded, can hit IO with read side effects (VERA data-port
        -- auto-increment, VIA flag clears).  Qualify all chip selects.
        bus_valid : out std_logic
    );
end p65c816_wrap;

architecture rtl of p65c816_wrap is

    signal a24_int   : std_logic_vector(23 downto 0);
    -- KEEP the full 24-bit address: the wrapper only consumes [15:0], and
    -- Quartus's removal of the "dangling" a24_int[23:16] cone (fitter note:
    -- "logic that only feeds a dangling port will be removed") is the prime
    -- suspect for silicon-only corruption of the INDEXED-store address path
    -- (STA abs,X writes vanish on HW, work in RTL -- the check_rom boot
    -- crash).  Anchoring the net forbids the trim cascade.
    attribute keep : boolean;
    attribute keep of a24_int : signal is true;
    signal we_int    : std_logic;
    signal vpa_int   : std_logic;
    signal vda_int   : std_logic;
    signal sync_int  : std_logic;
    signal pc_reg    : std_logic_vector(15 downto 0) := (others => '0');

begin

    u_cpu : entity work.P65C816
        port map (
            CLK     => clk,
            RST_N   => res_n,
            CE      => '1',                 -- clock always enabled; stall via RDY_IN
            RDY_IN  => enable,              -- VERA stall: holds CPU when low
            NMI_N   => nmi_n,
            IRQ_N   => irq_n,
            ABORT_N => '1',                 -- not used by X16 hardware
            D_IN    => din,
            D_OUT   => dout,
            A_OUT   => a24_int,
            WE      => we_int,
            RDY_OUT => open,                -- WAI stall output; not wired to X16
            VPA     => vpa_int,
            VDA     => vda_int,
            MLB     => open,                -- memory lock; not needed for X16
            VPB     => vpb,                 -- vector pull -> bank-0 latch (X16 R44+)
            E_OUT   => emu_mode,            -- expose emulation/native flag
            I_OUT   => i_flag               -- expose I (IRQ-disable) flag
        );

    -- X16 only needs the 16-bit portion of the address.
    -- In emulation mode (E=1, reset default) A_OUT[23:16] = PBR = 0x00.
    addr  <= a24_int(15 downto 0);

    -- P65C816 drives WE low during write cycles and high during reads.
    -- That already matches the wrapper's R_W_N convention directly.
    r_w_n <= we_int;

    -- SYNC: VPA=1 AND VDA=1 is the 65816 equivalent of the 65C02 SYNC pin.
    -- It is asserted only on the opcode byte fetch (first bus cycle of
    -- every instruction), which is identical semantics to SYNC on the 65C02.
    sync_int <= vpa_int and vda_int;
    sync     <= sync_int;

    -- valid bus cycle: program fetch (VPA) or data access (VDA)
    bus_valid <= vpa_int or vda_int;

    -- PC reconstruction: latch address on every opcode fetch, gated by enable
    -- so the throttle stays accurate.  On the sync cycle itself the live
    -- address is forwarded combinationally so probe conditions such as
    --   (cpu_sync && cpu_a == 16'hC028)
    -- fire on the exact same cycle as they did with r65c02_wrap.
    process (clk)
    begin
        if rising_edge(clk) then
            if enable = '1' and sync_int = '1' then
                pc_reg <= a24_int(15 downto 0);
            end if;
        end if;
    end process;

    pc <= a24_int(15 downto 0) when sync_int = '1' else pc_reg;

end rtl;
