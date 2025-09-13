// ---------------------------------------------------------------------------
// Filename: gate_applicator.sv
// Author: Shahriar Rizvi
// Date: 08-SEP-2025
//
// Description:
// 2x2 unitary applicator using four complex multipliers (Q1.15 format).
// Latency: complex_math_unit (3) + this adder stage (1) = 4 cycles.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

import fixed_point_pkg::*;

module gate_applicator (
    // -- System Signals --
    input  logic clk,
    input  logic rst_n, // Active-low reset

    // -- Data Inputs --
    // State vector elements
    input  complex_t state_in_1,
    input  complex_t state_in_2,

    // 2x2 Unitary Matrix for the gate
    input  complex_t u_00,
    input  complex_t u_01,
    input  complex_t u_10,
    input  complex_t u_11,

    // -- Data Outputs --
    // Resulting state vector elements
    output complex_t state_out_1,
    output complex_t state_out_2
);

    // --- Internal Wires and Registers ---

    // Wires to hold the results of the four complex multiplications
    complex_t mul_result_00_1; // U_00 * state_in_1
    complex_t mul_result_01_2; // U_01 * state_in_2
    complex_t mul_result_10_1; // U_10 * state_in_1
    complex_t mul_result_11_2; // U_11 * state_in_2

    // Registers to hold the final sum after the additions
    complex_t sum_result_1;
    complex_t sum_result_2;

    // --- Instantiate Four Complex Multipliers ---
    // We run all four multiplications in parallel.

    complex_math_unit mul_00_1 (
        .clk(clk), .rst_n(rst_n), .operand_a(u_00), .operand_b(state_in_1), .result(mul_result_00_1)
    );
    complex_math_unit mul_01_2 (
        .clk(clk), .rst_n(rst_n), .operand_a(u_01), .operand_b(state_in_2), .result(mul_result_01_2)
    );
    complex_math_unit mul_10_1 (
        .clk(clk), .rst_n(rst_n), .operand_a(u_10), .operand_b(state_in_1), .result(mul_result_10_1)
    );
    complex_math_unit mul_11_2 (
        .clk(clk), .rst_n(rst_n), .operand_a(u_11), .operand_b(state_in_2), .result(mul_result_11_2)
    );


    // --- Final Adder Stage ---
    // The complex_math_unit has a 3-cycle latency. The results of the
    // multiplications will be available 3 clock cycles after the inputs are provided.
    // We then perform the additions. We add one more register stage for the final
    // output, making the total latency of this module longer, but allowing it to
    // accept new data every clock cycle.

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_result_1 <= '0;
            sum_result_2 <= '0;
        end else begin
            // Perform the two complex additions
            sum_result_1.re <= mul_result_00_1.re + mul_result_01_2.re;
            sum_result_1.im <= mul_result_00_1.im + mul_result_01_2.im;

            sum_result_2.re <= mul_result_10_1.re + mul_result_11_2.re;
            sum_result_2.im <= mul_result_10_1.im + mul_result_11_2.im;
        end
    end

    // Assign the final registered results to the output ports
    assign state_out_1 = sum_result_1;
    assign state_out_2 = sum_result_2;

endmodule