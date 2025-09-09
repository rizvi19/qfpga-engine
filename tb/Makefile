# Makefile for Cocotb Simulation

# Top-level Verilog file
TOPLEVEL = quantum_engine_top
# Language of the top-level file
TOPLEVEL_LANG = verilog

# All Verilog source files
VERILOG_SOURCES = ../rtl/fixed_point_pkg.sv \
                  ../rtl/engine_pkg.sv \
                  ../rtl/complex_math_unit.sv \
                  ../rtl/gate_applicator.sv \
                  ../rtl/bram_wrapper.sv \
                  ../rtl/instruction_sequencer.sv \
                  ../rtl/quantum_engine_top.sv

# Python test module
MODULE = test_quantum_engine

# Include the standard Cocotb make rules
include $(shell cocotb-config --makefiles)/Makefile.sim