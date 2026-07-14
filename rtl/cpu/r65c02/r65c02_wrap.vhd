--============================================================================
-- r65c02_wrap.vhd  -  Drop-in replacement for t65_wrap.
--
-- Wraps the r65c02_tc full 65C02 core (Jens Gutschmidt, OpenCores) so it
-- presents the exact same port shape as the old T65 wrapper.  C5G.sv only
-- needs to swap t65_wrap -> r65c02_wrap in the instantiation.
--
-- PC port: r65c02_tc does not expose its PC.  We re-synthesise it by
-- latching the address bus on cpu_sync (matching T65's behaviour: PC of
-- the currently-executing opcode).  During the sync cycle itself we
-- forward a_o combinationally so PC-equality checks (e.g. == $038B) fire
-- on the same cycle they did with T65.
--============================================================================
library IEEE;
use IEEE.std_logic_1164.all;

library r65c02_tc;

entity r65c02_wrap is
    port (
        clk      : in  std_logic;
        enable   : in  std_logic;                       -- mapped to RDY
        res_n    : in  std_logic;
        irq_n    : in  std_logic;
        nmi_n    : in  std_logic;
        rdy      : in  std_logic;                       -- unused (always 1 in C5G.sv)
        r_w_n    : out std_logic;
        sync     : out std_logic;
        addr     : out std_logic_vector(15 downto 0);
        din      : in  std_logic_vector(7 downto 0);
        dout     : out std_logic_vector(7 downto 0);
        pc       : out std_logic_vector(15 downto 0)
    );
end r65c02_wrap;

architecture rtl of r65c02_wrap is
    signal a_int    : std_logic_vector(15 downto 0);
    signal sync_int : std_logic;
    signal sync_fetch : std_logic;
    signal pc_reg   : std_logic_vector(15 downto 0) := (others => '0');
begin
    u_cpu : entity r65c02_tc.r65c02_tc
        port map (
            clk_clk_i   => clk,
            d_i         => din,
            irq_n_i     => irq_n,
            nmi_n_i     => nmi_n,
            rdy_i       => enable,
            rst_rst_n_i => res_n,
            so_n_i      => '1',
            a_o         => a_int,
            d_o         => dout,
            rd_o        => open,
            sync_o      => sync_int,
            wr_n_o      => r_w_n,
            wr_o        => open
        );

    addr <= a_int;
    -- r65c02_tc exposes SYNC active-low during opcode fetch.
    sync_fetch <= not sync_int;
    sync <= sync_fetch;

    -- Latch PC on opcode fetch (gated by enable so the throttle stays honest).
    process (clk)
    begin
        if rising_edge(clk) then
            if enable = '1' and sync_fetch = '1' then
                pc_reg <= a_int;
            end if;
        end if;
    end process;

    -- Forward live address during sync, latched value otherwise.  This makes
    -- C5G.sv's "cpu_en && cpu_sync && cpu_pc == 16'h038B" trigger on the
    -- exact fetch cycle, just like T65.
    pc <= a_int when sync_fetch = '1' else pc_reg;

end rtl;
