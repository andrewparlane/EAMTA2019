#.1
source scripts/setup.tcl -echo
# Waive some repeatetive messages
set_message_info -id PSYN-040 -limit 1
set_message_info -id PSYN-256 -limit 1
set_message_info -id PSYN-088 -limit 1
#return

#.2
 import_designs inputs/top_mapped_final.ddc -format ddc 
 check_timing
 source scripts/floorplan.tcl -echo
 check_physical_constraints
 save_mw_cel -as top_postFloorplan
#.3
 place_opt
 report_timing > reports/top.timing.placement.rpt
 report_congestion -grc_based -routing_stage global
 save_mw_cel -as top_postPlaceOPT
return
 remove_clock_uncertainty [all_clocks]
 clock_opt -fix_hold_all_clocks

 derive_pg_connection -power_net {vdd!} -ground_net {gnd!} -verbose
 preroute_standard_cells -nets {vdd! gnd!} -connect horizontal
 report_timing > reports/top.timing.cts.rpt
 report_clock_tree
 save_mw_cel -as top_postCTS
 route_opt
 save_mw_cel -as top_postRouteOpt

#.4a
 insert_stdcell_filler -cell_with_metal "FILL8 FILL4 FILL2 FILL" -connect_to_power "vdd!" -connect_to_ground "gnd!"
 derive_pg_connection -power_net {vdd!} -ground_net {gnd!} -verbose 
 preroute_standard_cells -nets {vdd! gnd!} -connect horizontal 
return
#.4b
 set_route_zrt_detail_options -default_gate_size 1.8 -default_port_external_gate_size 1.8
 define_antenna_rule my_mw_lib -mode 1 -diode_mode 0 -metal_ratio 1000 -cut_ratio 0
 route_zrt_detail -incremental true 
 verify_zrt_route -antenna true 
 insert_metal_filler -fill_poly -bounding_box { { 260 260 } {3460 2626} } -out self -timing_driven -to_metal 3
 set_parameter -name wellFillerAlignWithCell -value 1 -module apl
 insert_well_filler -layer NWELL -ignore_PRboundary 
 save_mw_cel -as TOP_postChipFinish
 verify_lvs
return
 verify_pg_net
 save_mw_cel -as top_final
 close_mw_lib
# exit

#icc_shell -f scripts/setup.tcl | tee icc.log
#grep ^Error icc.log
#start_gui

