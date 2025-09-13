// ---------------------------------------------------------------------------
// Filename: bram_wrapper.sv
// Author: Shahriar Rizvi
// Date: 08-SEP-2025
//
// Description:
// A simple wrapper for a synchronous-read, synchronous-write BRAM.
// This module will store our quantum state vector. The data is read from the
// address specified on the previous clock cycle.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

import fixed_point_pkg::*;

module bram_wrapper #(
    // Parameterize the BRAM size
    parameter int ADDR_WIDTH = 10, // For 2^10 = 1024 elements (up to 10 qubits)
    parameter int NUM_WORDS  = 2**ADDR_WIDTH
) (
    // -- System Signals --
    input  logic clk,

    // -- Write Port --
    input  logic                   wr_en,      // Write enable
    input  logic [ADDR_WIDTH-1:0]  wr_addr,
    input  complex_t               wr_data,

    // -- Read Port --
    input  logic                   rd_en,      // Read enable
    input  logic [ADDR_WIDTH-1:0]  rd_addr,
    output complex_t               rd_data
);

    // This is the actual memory array.
    // We declare it as a 'reg' type, which is Verilog's term for a variable
    // that holds its value, like a register or memory.
    (* ram_style = "block" *) // Synthesis attribute to infer BRAM
    reg complex_t memory [NUM_WORDS-1:0];

    // Write Logic
    always_ff @(posedge clk) begin
        if (wr_en) begin
            memory[wr_addr] <= wr_data;
        end
    end

    // Read Logic
    // Using a temporary register for the read address ensures a synchronous read.
    // The data will appear on 'rd_data' one clock cycle after 'rd_addr' is set.
    reg [ADDR_WIDTH-1:0] rd_addr_reg;

    always_ff @(posedge clk) begin
        if (rd_en) begin
            rd_addr_reg <= rd_addr;
        end
    end

    assign rd_data = memory[rd_addr_reg];

endmodule