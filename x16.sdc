# ============================================================================
# x16.sdc -- X16 core timing constraints (applied ON TOP of sys/sys_top.sdc).
#
# sys_top.sdc line 14 cuts the whole core PLL group from the HDMI/audio/system
# clocks, but its `*[*]` wildcard lumps all THREE core outputs into a single
# group, so they are analyzed as synchronous to each other:
#
#   outclk_0  general[0]  25.0 MHz  pix_clk  (VERA VGA)
#   outclk_1  general[1]  12.5 MHz  aud_mclk (IKAOPM)
#   outclk_2  general[2]   8.0 MHz  cpu_clk  (CPU + VIA + I2C)
#
# The cpu_clk <-> pix_clk crossing (VERA external bus) is handled in HARDWARE by
# the _q1/_q2/_q3 CPU-bus pipeline + per-domain reset synchronizers, so the three
# domains are genuinely asynchronous.  Declare them mutually async -- exactly as
# the working C5G reference (x16_monitor/C5G.sdc) does.  Without this the 8 MHz
# domain shows ~-18 ns worst slack on phantom cpu<->pix paths.
# ============================================================================
# general[3] = outclk_3 = 100 MHz SDRAM controller clock.  It is crossed to the
# 8 MHz cpu_clk only through the ext_ram_sdram req/ack toggle synchronizers, so
# declare it asynchronous to the core clocks as well.
set_clock_groups -asynchronous \
    -group [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -group [get_clocks {*|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -group [get_clocks {*|pll|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -group [get_clocks {*|pll|pll_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}]
