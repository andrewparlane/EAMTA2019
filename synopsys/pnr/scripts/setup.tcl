set MW_REF_PATH "../MW"
set search_path ". ../ ../MW/ ../../libs/ ../scripts/ ../inputs/"

set target_library "c5n_utah_std_v5_t27.db icg4utah.db UofU_Digital_v1_2.db"
set synthetic_library "dw_foundation.sldb"
set link_library "* $target_library $synthetic_library c5n_utah_std_v5_t27.db io.db"
set mw_reference_library "../MW/UTAH_V1P1 ../MW/ICG ../MW/IO"
set TECH_FILE "../MW/UTAH.tf"; #Milkiway Tech
set MAP_FILE "${MW_REF_PATH}/ami500hxkx_3m.map"; #Map File
set TLUPLUS_MAX_FILE "${MW_REF_PATH}/ami500.tluplus"
set TLUPLUS_MIN_FILE "${MW_REF_PATH}/ami500.tluplus"

set mw_design_library MW_TOP_LIB 

close_mw_lib

# Only create new Milkyway design library if it doesn't already exist
sh rm -rf $mw_design_library
if {![file isdirectory $mw_design_library ]} {
  set  mw_use_layer_enhancement true
#  extend_mw_layers
  create_mw_lib  -technology $TECH_FILE \
                -mw_reference_library $mw_reference_library \
                $mw_design_library
} else {
  # If Milkyway design library already exists, ensure that it is consistent with specified Milky
  set_mw_lib_reference $mw_design_library -mw_reference_library $mw_reference_library
}

open_mw_lib     $mw_design_library

check_library

set_tlu_plus_files -max_tluplus $TLUPLUS_MAX_FILE \
                   -min_tluplus $TLUPLUS_MIN_FILE \
                   -tech2itf_map $MAP_FILE

check_tlu_plus_files

