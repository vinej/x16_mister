--============================================================================
-- mj65c02_wrap.vhd  -  MJoergen 65c02 in the r65c02_wrap/p65c816_wrap port
-- shape (jyv 2026-07-08), so x16.sv can offer a runtime 65C02/65C816 choice.
--
-- Bus contract (established empirically in tb_wai): the core consumes
-- rd_data_i COMBINATIONALLY in the same ce-enabled cycle as the address --
-- exactly the X16 bus convention (cpu_di valid when cpu_rdy commits), so
-- enable/din wire straight through, like the other CPU wrappers.  ce_i is
-- a true global clock enable: when cpu_rdy drops (SDRAM/VERA/SPI stall)
-- the CPU freezes HOLDING the bus -- the P65C816 RDY semantics our memory
-- system is built for (writes included: ext_ram_sdram's edge-triggered
-- wpush handles held writes).  NOTE: ce must stay ACTIVE during reset;
-- the core implements reset as a forced BRK (vector fetch through the
-- interrupt microcode) and needs enabled cycles while rst_i is high.
--
-- sync = the core's opcode-fetch indicator (active HIGH during the fetch
-- cycle; note r65c02_wrap's sync is LOW on fetch -- x16.sv's wai_shim gets
-- this one directly, no inversion).  pc = addr latched on fetch, live
-- during the fetch cycle (same convention as r65c02_wrap).
--
-- bus_valid = rd_en or wr_en: the core marks internal (no-bus) cycles, so
-- x16.sv can qualify chip selects exactly like the '816's VDA|VPA -- ghost
-- reads of $9F3E (SPI auto-tx!) and other side-effecting addresses cannot
-- happen.
--
-- NMI: the core edge-detects internally; the stretched SMC pulse maps
-- straight onto nmi_i.  IRQ: level, maskable.  WAI/STP: not in this core;
-- rtl/wai_shim.sv provides them in front of this wrapper.
--============================================================================
library IEEE;
use IEEE.std_logic_1164.all;

-- The core compiles into its own library: its entity names (alu, pc, sr...)
-- collide with the P65C816's in `work` (VHDL names are case-insensitive:
-- alu = ALU -> Quartus 10430).  Internal `entity work.x` references resolve
-- to the compilation library, so the core sources need no edits.
library mj65c02;

entity mj65c02_wrap is
    port (
        clk       : in  std_logic;
        enable    : in  std_logic;                       -- cpu_rdy (bus-complete)
        res_n     : in  std_logic;
        irq_n     : in  std_logic;
        nmi_n     : in  std_logic;
        rdy       : in  std_logic;                       -- unused (shape compat)
        r_w_n     : out std_logic;
        sync      : out std_logic;
        addr      : out std_logic_vector(15 downto 0);
        din       : in  std_logic_vector(7 downto 0);
        dout      : out std_logic_vector(7 downto 0);
        pc        : out std_logic_vector(15 downto 0);
        bus_valid : out std_logic                        -- rd_en | wr_en
    );
end mj65c02_wrap;

architecture rtl of mj65c02_wrap is
    signal a_int      : std_logic_vector(15 downto 0);
    signal wr_en      : std_logic;
    signal rd_en      : std_logic;
    signal sync_int   : std_logic;
    signal pc_reg     : std_logic_vector(15 downto 0) := (others => '0');
    -- jyv 2026-07-11: expressions as port-map actuals (`not x`) are
    -- VHDL-2008; Quartus Standard's front end is 93-grade, so the
    -- inversions live on intermediate signals and everything compiles
    -- without -hdl_version games.
    signal rst_int    : std_logic;
    signal irq_int    : std_logic;
    signal nmi_int    : std_logic;
begin
    rst_int <= not res_n;
    irq_int <= not irq_n;
    nmi_int <= not nmi_n;
    u_cpu : entity mj65c02.cpu_65c02
        generic map (
            G_LOG_NAME      => "",
            G_SIM           => false,
            G_ENABLE_IOPORT => false,      -- $0000/$0001 are the X16 bank
                                           -- latches, plain memory to the CPU
            G_VARIANT       => "65C02",
            G_VERBOSE       => 0
        )
        port map (
            clk_i        => clk,
            rst_i        => rst_int,
            ce_i         => enable,
            nmi_i        => nmi_int,
            nmi_ack_o    => open,
            irq_i        => irq_int,
            addr_o       => a_int,
            wr_en_o      => wr_en,
            wr_data_o    => dout,
            rd_en_o      => rd_en,
            rd_data_i    => din,
            ioport_in_i  => (others => '0'),
            ioport_out_o => open,
            ioport_dir_o => open,
            sync_o       => sync_int,
            debug_o      => open
        );

    addr      <= a_int;
    r_w_n     <= not wr_en;
    sync      <= sync_int;
    bus_valid <= rd_en or wr_en;

    -- PC of the currently-fetching opcode: live during the fetch cycle,
    -- latched otherwise (r65c02_wrap convention for x16.sv debug triggers)
    process (clk)
    begin
        if rising_edge(clk) then
            if enable = '1' and sync_int = '1' then
                pc_reg <= a_int;
            end if;
        end if;
    end process;

    pc <= a_int when sync_int = '1' else pc_reg;

end rtl;
