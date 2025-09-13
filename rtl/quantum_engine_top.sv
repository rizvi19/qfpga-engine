// ---------------------------------------------------------------------------
// Filename: quantum_engine_top.sv
// Author: Your Name
// Date: 08-SEP-2025
//
// Description:
// Top-level quantum engine wrapper. Added N_QUBITS parameter and cycle counter
// exposure for performance / profiling.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

import fixed_point_pkg::*;
import engine_pkg::*;

module quantum_engine_top #(
    parameter int ADDR_WIDTH = 10, // State vector address width (supports up to 2^ADDR_WIDTH amplitudes)
    parameter int N_QUBITS   = 4   // Logical number of qubits used by programs
) (
    // -- System Interface --
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic [1:0] program_select,
    output logic done,
    output logic [31:0] cycle_count // Total cycles consumed (latched at DONE)
);

    // --- Internal Wires ---
    logic                      bram_rd_en_wire;
    logic [ADDR_WIDTH-1:0]     bram_rd_addr_wire;
    complex_t                  bram_rd_data_wire;
    logic                      bram_wr_en_wire;
    logic [ADDR_WIDTH-1:0]     bram_wr_addr_wire;
    complex_t                  bram_wr_data_wire;

    complex_t                  gate_u_00_wire;
    complex_t                  gate_u_01_wire;
    complex_t                  gate_u_10_wire;
    complex_t                  gate_u_11_wire;
    complex_t                  gate_state_in_1_wire;
    complex_t                  gate_state_in_2_wire;
    complex_t                  gate_state_out_1_wire;
    complex_t                  gate_state_out_2_wire;
    logic [31:0]               seq_cycle_counter_wire;

    // 1. BRAM instance
    bram_wrapper #(.ADDR_WIDTH(ADDR_WIDTH)) state_vector_bram (
        .clk(clk),
        .wr_en(bram_wr_en_wire),
        .wr_addr(bram_wr_addr_wire),
        .wr_data(bram_wr_data_wire),
        .rd_en(bram_rd_en_wire),
        .rd_addr(bram_rd_addr_wire),
        .rd_data(bram_rd_data_wire)
    );

    // 2. Gate applicator
    gate_applicator u_gate_applicator (
        .clk(clk), .rst_n(rst_n),
        .state_in_1(gate_state_in_1_wire), .state_in_2(gate_state_in_2_wire),
        .u_00(gate_u_00_wire), .u_01(gate_u_01_wire), .u_10(gate_u_10_wire), .u_11(gate_u_11_wire),
        .state_out_1(gate_state_out_1_wire), .state_out_2(gate_state_out_2_wire)
    );

    // 3. Instruction sequencer
    instruction_sequencer #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .N_QUBITS(N_QUBITS)
    ) u_sequencer (
        .clk(clk), .rst_n(rst_n),
        .start_computation(start), .program_select(program_select), .computation_done(done),
        .bram_rd_en(bram_rd_en_wire), .bram_rd_addr(bram_rd_addr_wire), .bram_rd_data(bram_rd_data_wire),
        .bram_wr_en(bram_wr_en_wire), .bram_wr_addr(bram_wr_addr_wire), .bram_wr_data(bram_wr_data_wire),
        .gate_u_00(gate_u_00_wire), .gate_u_01(gate_u_01_wire), .gate_u_10(gate_u_10_wire), .gate_u_11(gate_u_11_wire),
        .gate_state_in_1(gate_state_in_1_wire), .gate_state_in_2(gate_state_in_2_wire),
        .gate_state_out_1(gate_state_out_1_wire), .gate_state_out_2(gate_state_out_2_wire),
        .cycle_counter(seq_cycle_counter_wire)
    );

    assign cycle_count = seq_cycle_counter_wire;

endmodule