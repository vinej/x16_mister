library ieee;
use ieee.std_logic_1164.all;
-- jyv 2026-07-11: was ieee.numeric_std_unsigned (VHDL-2008) -- Quartus
-- Standard's ieee library does not ship that package (Error 10481).  The
-- Synopsys std_logic_unsigned covers every use here (slv arithmetic and
-- slv/integer comparison); to_integer calls were made conv_integer.
use ieee.std_logic_unsigned.all;

entity xr is
   port (
      clk_i    : in  std_logic;
      ce_i     : in  std_logic;
      wait_i   : in  std_logic;
      xr_sel_i : in  std_logic;
      alu_ar_i : in  std_logic_vector(7 downto 0);

      xr_o     : out std_logic_vector(7 downto 0)
   );
end entity xr;

architecture structural of xr is

   -- 'X' register
   signal xr : std_logic_vector(7 downto 0) := X"00";

begin

   -- 'X' register
   xr_proc : process (clk_i)
   begin
      if rising_edge(clk_i) then
         if ce_i = '1' then
            if wait_i = '0' then
               if xr_sel_i = '1' then
                  xr <= alu_ar_i;
               end if;
            end if;
         end if;
      end if;
   end process xr_proc;

   -- Drive output signal
   xr_o <= xr;

end architecture structural;

