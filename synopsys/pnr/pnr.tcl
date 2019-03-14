source ../scripts/setup.tcl -echo

import_designs ../inputs/top_mapped_final.ddc -format ddc
link
check_timing

start_gui

source ../scripts/floorplan.tcl
check_physical_constraints

save_mw_cel -as top_post_floorplan
close_mw_cel
open_mw_cel top_post_floorplan

place_opt
report_timing

save_mw_cel -as top_post_place
close_mw_cel
open_mw_cel top_post_place

remove_clock_uncertainty [all_clocks]
clock_opt -fix_hold_all_clocks
derive_pg_connection -power_net {vdd!} -ground_net {gnd!} -verbose
preroute_standard_cells -nets {vdd! gnd!} -connect horizontal
report_timing
report_clock_tree

save_mw_cel -as top_post_cts
close_mw_cel
open_mw_cel top_post_cts

route_opt

save_mw_cel -as top_post_route
close_mw_cel
open_mw_cel top_post_route

insert_stdcell_filler -cell_with_metal "FILL8 FILL4 FILL2 FILL"  -connect_to_power "vdd!" -connect_to_ground "gnd!"
derive_pg_connection -power_net {vdd!} -ground_net {gnd!} -verbose
preroute_standard_cells -nets {vdd! gnd!} -connect horizontal

set_route_zrt_detail_options -default_gate_size 1.8 -default_port_external_gate_size 1.8
define_antenna_rule MW_top_LIB -mode 1 -diode_mode 0 -metal_ratio 1000 -cut_ratio 0
route_zrt_detail -incremental true
verify_zrt_route -antenna true

insert_metal_filler  -fill_poly -bounding_box { { 260 260 } { 3460 2626 } }  -out self  -timing_driven  -to_metal 3
set_parameter -name wellFillerAlignWithCell -value 1 -module apl
insert_well_filler -layer NWELL -ignore_PRboundary

save_mw_cel -as top_post_finish
close_mw_cel
open_mw_cel top_post_finish

verify_lvs
verify_pg_net

exit

