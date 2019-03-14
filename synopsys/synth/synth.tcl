set search_path ". ../ ../../../rtl ../libs"
set target_library "nldm_op_cond_typ.db"
set link_library "* $target_library"

analyze -format sverilog {pkg/operation_pkg.sv ALU.sv Control.sv Mux4.sv OneBitAdder.sv RegisterBank.sv Top.sv}
elaborate Top -parameters "WIDTH=16"
link

check_design

analyze_datapath_extraction

# Constraints
set period 45.0
create_clock -period $period [get_ports clk]
set_clock_uncertainty -setup [expr {10*$period/100}] [get_clocks clk]
set_clock_transition -max [expr {20*$period/100}] [get_clocks clk]
set_clock_latency -source [expr {5*$period/100}] [get_clocks clk]
set_input_delay -clock [get_clocks clk] -max [expr {40*$period/100}] [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay -max [expr {50*$period/100}] [all_outputs]
set_load -max 1 [all_outputs]
set_input_transition -min [expr {1*$period/100}] [remove_from_collection [all_inputs] [get_ports clk]]
set_input_transition -max [expr {10*$period/100}] [remove_from_collection [all_inputs] [get_ports clk]]

compile_ultra -no_autoungroup -gate_clock

optimize_registers

compile_ultra -incremental -gate_clock
optimize_netlist -area
compile_ultra -incremental -gate_clock
optimize_netlist -area
compile_ultra -incremental -gate_clock
optimize_netlist -area

change_names -rule verilog

analyze_datapath

write_file -format ddc -hierarchy -out micro.ddc
write_file -format verilog -hierarchy -out micro.v
write_sdc micro.sdc

report_area
report_clock
report_clock_gating
report_clock_gating_check
report_qor
report_power
report_constraint
report_resources -hier
report_timing

exit
