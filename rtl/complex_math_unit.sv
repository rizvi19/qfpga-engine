// ---------------------------------------------------------------------------
// Filename: complex_math_unit.sv
// Author: Shahriar Rizvi
// Date: 08-SEP-2025
//
// Description:
// A 3-stage pipelined unit for multiplying two complex numbers.
// Updated for Q1.15 fixed-point (result shift by 15).
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

import fixed_point_pkg::*;

module complex_math_unit (
    // -- System Signals --
    input  logic clk,
    input  logic rst_n, // Active-low reset

    // -- Data Inputs --
    input  complex_t operand_a,
    input  complex_t operand_b,

    // -- Data Output --
    output complex_t result
);

    // --- Internal Signals for Pipeline Stages ---

    // Stage 1 Registers (to hold inputs)
    complex_t s1_operand_a;
    complex_t s1_operand_b;

    // Stage 2 Registers (to hold multiplication results)
    // Note: When multiplying two 16-bit numbers, the result is 32 bits.
    logic signed [31:0] s2_ac; // ar * br
    logic signed [31:0] s2_bd; // ai * bi
    logic signed [31:0] s2_ad; // ar * bi
    logic signed [31:0] s2_bc; // ai * br

    // Stage 3 Registers (to hold final results before output)
    complex_t s3_result;

    // --- Pipeline Logic ---

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset logic
            s1_operand_a <= '0;
            s1_operand_b <= '0;
            s2_ac <= '0; s2_bd <= '0; s2_ad <= '0; s2_bc <= '0;
            s3_result <= '0;
        end else begin
            // -- Stage 1: Register the inputs --
            // This isolates the multiplier from the external logic.
            s1_operand_a <= operand_a;
            s1_operand_b <= operand_b;

            // -- Stage 2: Perform the four multiplications in parallel --
            // This is the most computationally intensive stage.
            // Q1.15 * Q1.15 => Q2.30
            s2_ac <= s1_operand_a.re * s1_operand_b.re;
            s2_bd <= s1_operand_a.im * s1_operand_b.im;
            s2_ad <= s1_operand_a.re * s1_operand_b.im;
            s2_bc <= s1_operand_a.im * s1_operand_b.re;

            // -- Stage 3: Perform final add/sub and rescale --
            // The multiplication result is 32 bits. We need to scale it
            // back down to our 16-bit fixed-point format (Q1.15).
            // Since we multiplied two Q1.15 numbers, the result is a
            // Q2.30 number. We need to select the correct 16 bits.
            // This is equivalent to a right-shift by 15.
            s3_result.re <= (s2_ac - s2_bd) >>> 15;
            s3_result.im <= (s2_ad + s2_bc) >>> 15;
        end
    end

    // Assign the final registered output to the module's output port
    assign result = s3_result;

endmodule