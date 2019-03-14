###################################################################

# Created by write_sdc on Wed Mar 13 11:17:54 2019

###################################################################
set sdc_version 2.0

set_units -time ns -resistance kOhm -capacitance pF -voltage V -current mA
set_load -pin_load 1 [get_ports {dout_low[15]}]
set_load -pin_load 1 [get_ports {dout_low[14]}]
set_load -pin_load 1 [get_ports {dout_low[13]}]
set_load -pin_load 1 [get_ports {dout_low[12]}]
set_load -pin_load 1 [get_ports {dout_low[11]}]
set_load -pin_load 1 [get_ports {dout_low[10]}]
set_load -pin_load 1 [get_ports {dout_low[9]}]
set_load -pin_load 1 [get_ports {dout_low[8]}]
set_load -pin_load 1 [get_ports {dout_low[7]}]
set_load -pin_load 1 [get_ports {dout_low[6]}]
set_load -pin_load 1 [get_ports {dout_low[5]}]
set_load -pin_load 1 [get_ports {dout_low[4]}]
set_load -pin_load 1 [get_ports {dout_low[3]}]
set_load -pin_load 1 [get_ports {dout_low[2]}]
set_load -pin_load 1 [get_ports {dout_low[1]}]
set_load -pin_load 1 [get_ports {dout_low[0]}]
set_load -pin_load 1 [get_ports {dout_high[15]}]
set_load -pin_load 1 [get_ports {dout_high[14]}]
set_load -pin_load 1 [get_ports {dout_high[13]}]
set_load -pin_load 1 [get_ports {dout_high[12]}]
set_load -pin_load 1 [get_ports {dout_high[11]}]
set_load -pin_load 1 [get_ports {dout_high[10]}]
set_load -pin_load 1 [get_ports {dout_high[9]}]
set_load -pin_load 1 [get_ports {dout_high[8]}]
set_load -pin_load 1 [get_ports {dout_high[7]}]
set_load -pin_load 1 [get_ports {dout_high[6]}]
set_load -pin_load 1 [get_ports {dout_high[5]}]
set_load -pin_load 1 [get_ports {dout_high[4]}]
set_load -pin_load 1 [get_ports {dout_high[3]}]
set_load -pin_load 1 [get_ports {dout_high[2]}]
set_load -pin_load 1 [get_ports {dout_high[1]}]
set_load -pin_load 1 [get_ports {dout_high[0]}]
set_load -pin_load 1 [get_ports zero]
set_load -pin_load 1 [get_ports error]
create_clock [get_ports clk]  -period 45  -waveform {0 22.5}
set_clock_latency -source 2.25  [get_clocks clk]
set_clock_uncertainty -setup 4.5  [get_clocks clk]
set_clock_transition -max -fall 9 [get_clocks clk]
set_clock_transition -max -rise 9 [get_clocks clk]
set_input_delay -clock clk  -max 18  [get_ports rst]
set_input_delay -clock clk  -max 18  [get_ports {cmdin[5]}]
set_input_delay -clock clk  -max 18  [get_ports {cmdin[4]}]
set_input_delay -clock clk  -max 18  [get_ports {cmdin[3]}]
set_input_delay -clock clk  -max 18  [get_ports {cmdin[2]}]
set_input_delay -clock clk  -max 18  [get_ports {cmdin[1]}]
set_input_delay -clock clk  -max 18  [get_ports {cmdin[0]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[15]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[14]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[13]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[12]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[11]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[10]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[9]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[8]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[7]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[6]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[5]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[4]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[3]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[2]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[1]}]
set_input_delay -clock clk  -max 18  [get_ports {din_1[0]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[15]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[14]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[13]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[12]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[11]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[10]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[9]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[8]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[7]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[6]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[5]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[4]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[3]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[2]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[1]}]
set_input_delay -clock clk  -max 18  [get_ports {din_2[0]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[15]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[14]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[13]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[12]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[11]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[10]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[9]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[8]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[7]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[6]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[5]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[4]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[3]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[2]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[1]}]
set_input_delay -clock clk  -max 18  [get_ports {din_3[0]}]
set_output_delay -max 22.5  [get_ports {dout_low[15]}]
set_output_delay -max 22.5  [get_ports {dout_low[14]}]
set_output_delay -max 22.5  [get_ports {dout_low[13]}]
set_output_delay -max 22.5  [get_ports {dout_low[12]}]
set_output_delay -max 22.5  [get_ports {dout_low[11]}]
set_output_delay -max 22.5  [get_ports {dout_low[10]}]
set_output_delay -max 22.5  [get_ports {dout_low[9]}]
set_output_delay -max 22.5  [get_ports {dout_low[8]}]
set_output_delay -max 22.5  [get_ports {dout_low[7]}]
set_output_delay -max 22.5  [get_ports {dout_low[6]}]
set_output_delay -max 22.5  [get_ports {dout_low[5]}]
set_output_delay -max 22.5  [get_ports {dout_low[4]}]
set_output_delay -max 22.5  [get_ports {dout_low[3]}]
set_output_delay -max 22.5  [get_ports {dout_low[2]}]
set_output_delay -max 22.5  [get_ports {dout_low[1]}]
set_output_delay -max 22.5  [get_ports {dout_low[0]}]
set_output_delay -max 22.5  [get_ports {dout_high[15]}]
set_output_delay -max 22.5  [get_ports {dout_high[14]}]
set_output_delay -max 22.5  [get_ports {dout_high[13]}]
set_output_delay -max 22.5  [get_ports {dout_high[12]}]
set_output_delay -max 22.5  [get_ports {dout_high[11]}]
set_output_delay -max 22.5  [get_ports {dout_high[10]}]
set_output_delay -max 22.5  [get_ports {dout_high[9]}]
set_output_delay -max 22.5  [get_ports {dout_high[8]}]
set_output_delay -max 22.5  [get_ports {dout_high[7]}]
set_output_delay -max 22.5  [get_ports {dout_high[6]}]
set_output_delay -max 22.5  [get_ports {dout_high[5]}]
set_output_delay -max 22.5  [get_ports {dout_high[4]}]
set_output_delay -max 22.5  [get_ports {dout_high[3]}]
set_output_delay -max 22.5  [get_ports {dout_high[2]}]
set_output_delay -max 22.5  [get_ports {dout_high[1]}]
set_output_delay -max 22.5  [get_ports {dout_high[0]}]
set_output_delay -max 22.5  [get_ports zero]
set_output_delay -max 22.5  [get_ports error]
set_input_transition -max 4.5  [get_ports rst]
set_input_transition -min 0.45  [get_ports rst]
set_input_transition -max 4.5  [get_ports {cmdin[5]}]
set_input_transition -min 0.45  [get_ports {cmdin[5]}]
set_input_transition -max 4.5  [get_ports {cmdin[4]}]
set_input_transition -min 0.45  [get_ports {cmdin[4]}]
set_input_transition -max 4.5  [get_ports {cmdin[3]}]
set_input_transition -min 0.45  [get_ports {cmdin[3]}]
set_input_transition -max 4.5  [get_ports {cmdin[2]}]
set_input_transition -min 0.45  [get_ports {cmdin[2]}]
set_input_transition -max 4.5  [get_ports {cmdin[1]}]
set_input_transition -min 0.45  [get_ports {cmdin[1]}]
set_input_transition -max 4.5  [get_ports {cmdin[0]}]
set_input_transition -min 0.45  [get_ports {cmdin[0]}]
set_input_transition -max 4.5  [get_ports {din_1[15]}]
set_input_transition -min 0.45  [get_ports {din_1[15]}]
set_input_transition -max 4.5  [get_ports {din_1[14]}]
set_input_transition -min 0.45  [get_ports {din_1[14]}]
set_input_transition -max 4.5  [get_ports {din_1[13]}]
set_input_transition -min 0.45  [get_ports {din_1[13]}]
set_input_transition -max 4.5  [get_ports {din_1[12]}]
set_input_transition -min 0.45  [get_ports {din_1[12]}]
set_input_transition -max 4.5  [get_ports {din_1[11]}]
set_input_transition -min 0.45  [get_ports {din_1[11]}]
set_input_transition -max 4.5  [get_ports {din_1[10]}]
set_input_transition -min 0.45  [get_ports {din_1[10]}]
set_input_transition -max 4.5  [get_ports {din_1[9]}]
set_input_transition -min 0.45  [get_ports {din_1[9]}]
set_input_transition -max 4.5  [get_ports {din_1[8]}]
set_input_transition -min 0.45  [get_ports {din_1[8]}]
set_input_transition -max 4.5  [get_ports {din_1[7]}]
set_input_transition -min 0.45  [get_ports {din_1[7]}]
set_input_transition -max 4.5  [get_ports {din_1[6]}]
set_input_transition -min 0.45  [get_ports {din_1[6]}]
set_input_transition -max 4.5  [get_ports {din_1[5]}]
set_input_transition -min 0.45  [get_ports {din_1[5]}]
set_input_transition -max 4.5  [get_ports {din_1[4]}]
set_input_transition -min 0.45  [get_ports {din_1[4]}]
set_input_transition -max 4.5  [get_ports {din_1[3]}]
set_input_transition -min 0.45  [get_ports {din_1[3]}]
set_input_transition -max 4.5  [get_ports {din_1[2]}]
set_input_transition -min 0.45  [get_ports {din_1[2]}]
set_input_transition -max 4.5  [get_ports {din_1[1]}]
set_input_transition -min 0.45  [get_ports {din_1[1]}]
set_input_transition -max 4.5  [get_ports {din_1[0]}]
set_input_transition -min 0.45  [get_ports {din_1[0]}]
set_input_transition -max 4.5  [get_ports {din_2[15]}]
set_input_transition -min 0.45  [get_ports {din_2[15]}]
set_input_transition -max 4.5  [get_ports {din_2[14]}]
set_input_transition -min 0.45  [get_ports {din_2[14]}]
set_input_transition -max 4.5  [get_ports {din_2[13]}]
set_input_transition -min 0.45  [get_ports {din_2[13]}]
set_input_transition -max 4.5  [get_ports {din_2[12]}]
set_input_transition -min 0.45  [get_ports {din_2[12]}]
set_input_transition -max 4.5  [get_ports {din_2[11]}]
set_input_transition -min 0.45  [get_ports {din_2[11]}]
set_input_transition -max 4.5  [get_ports {din_2[10]}]
set_input_transition -min 0.45  [get_ports {din_2[10]}]
set_input_transition -max 4.5  [get_ports {din_2[9]}]
set_input_transition -min 0.45  [get_ports {din_2[9]}]
set_input_transition -max 4.5  [get_ports {din_2[8]}]
set_input_transition -min 0.45  [get_ports {din_2[8]}]
set_input_transition -max 4.5  [get_ports {din_2[7]}]
set_input_transition -min 0.45  [get_ports {din_2[7]}]
set_input_transition -max 4.5  [get_ports {din_2[6]}]
set_input_transition -min 0.45  [get_ports {din_2[6]}]
set_input_transition -max 4.5  [get_ports {din_2[5]}]
set_input_transition -min 0.45  [get_ports {din_2[5]}]
set_input_transition -max 4.5  [get_ports {din_2[4]}]
set_input_transition -min 0.45  [get_ports {din_2[4]}]
set_input_transition -max 4.5  [get_ports {din_2[3]}]
set_input_transition -min 0.45  [get_ports {din_2[3]}]
set_input_transition -max 4.5  [get_ports {din_2[2]}]
set_input_transition -min 0.45  [get_ports {din_2[2]}]
set_input_transition -max 4.5  [get_ports {din_2[1]}]
set_input_transition -min 0.45  [get_ports {din_2[1]}]
set_input_transition -max 4.5  [get_ports {din_2[0]}]
set_input_transition -min 0.45  [get_ports {din_2[0]}]
set_input_transition -max 4.5  [get_ports {din_3[15]}]
set_input_transition -min 0.45  [get_ports {din_3[15]}]
set_input_transition -max 4.5  [get_ports {din_3[14]}]
set_input_transition -min 0.45  [get_ports {din_3[14]}]
set_input_transition -max 4.5  [get_ports {din_3[13]}]
set_input_transition -min 0.45  [get_ports {din_3[13]}]
set_input_transition -max 4.5  [get_ports {din_3[12]}]
set_input_transition -min 0.45  [get_ports {din_3[12]}]
set_input_transition -max 4.5  [get_ports {din_3[11]}]
set_input_transition -min 0.45  [get_ports {din_3[11]}]
set_input_transition -max 4.5  [get_ports {din_3[10]}]
set_input_transition -min 0.45  [get_ports {din_3[10]}]
set_input_transition -max 4.5  [get_ports {din_3[9]}]
set_input_transition -min 0.45  [get_ports {din_3[9]}]
set_input_transition -max 4.5  [get_ports {din_3[8]}]
set_input_transition -min 0.45  [get_ports {din_3[8]}]
set_input_transition -max 4.5  [get_ports {din_3[7]}]
set_input_transition -min 0.45  [get_ports {din_3[7]}]
set_input_transition -max 4.5  [get_ports {din_3[6]}]
set_input_transition -min 0.45  [get_ports {din_3[6]}]
set_input_transition -max 4.5  [get_ports {din_3[5]}]
set_input_transition -min 0.45  [get_ports {din_3[5]}]
set_input_transition -max 4.5  [get_ports {din_3[4]}]
set_input_transition -min 0.45  [get_ports {din_3[4]}]
set_input_transition -max 4.5  [get_ports {din_3[3]}]
set_input_transition -min 0.45  [get_ports {din_3[3]}]
set_input_transition -max 4.5  [get_ports {din_3[2]}]
set_input_transition -min 0.45  [get_ports {din_3[2]}]
set_input_transition -max 4.5  [get_ports {din_3[1]}]
set_input_transition -min 0.45  [get_ports {din_3[1]}]
set_input_transition -max 4.5  [get_ports {din_3[0]}]
set_input_transition -min 0.45  [get_ports {din_3[0]}]
