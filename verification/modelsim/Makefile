# ----------------------------------------------------------------------------------
# Requirements:
#	QuestaSim - We use vcom and vsim.
#	Each test bench must contain an instantiation called dut.
#	Each test bench must call std.env.stop when it's finished testing.
# ----------------------------------------------------------------------------------

ifndef WLF
# set the default target to be all. Otherwise it's the first target it finds
.DEFAULT_GOAL := all
else
.DEFAULT_GOAL := view
endif

# Directories
SIM_DIR			= .
WAVES_DIR		= $(SIM_DIR)/waves
SRC_DIR			= ../../rtl
PKG_DIR			= $(SRC_DIR)/pkg
TB_SRC_DIR		= ../../tb

# src files
SRCS	= $(wildcard $(PKG_DIR)/*.sv) \
		  $(wildcard $(SRC_DIR)/*.sv)

# tb files
TB_SRCS	= $(wildcard $(TB_SRC_DIR)/*.sv)

# Extra arguments beyond the defaults in helper.mk
VCOM_FLAGS		=
VSIM_FLAGS		=

# Now include our helper.mk which has all the rules we need
# to create our library, compile and simulate our test benches
include helper.mk

# default rule is to create the library, and compile all the components
all: srcs tb_srcs

.PHONY: common all

# tests
adder1: all
	@$(call VSIM_CMD, OneBitAdder_tb)

mux4: all
	@$(call VSIM_CMD, Mux4_tb)

regbank: all
	@$(call VSIM_CMD, RegisterBank_tb)

alu: all
	@$(call VSIM_CMD, ALU_tb)

control: all
	@$(call VSIM_CMD, Control_tb)

.PHONY: adder1 mux4 regbank alu control

# delete the library and all compiled files
clean: helper_clean
	$(call DEL_VLIB_CMD, $(COMMON_LIB))

distclean: clean
	make -C $(COMMON_DIR) clean

.PHONY: clean distclean
