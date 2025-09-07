// ---------------------------------------------------------------------------
// Filename: complex_math_unit.sv
// Author: Shahriar Rizvi
// Date: 08-SEP-2025
//
// Description:
// A 3-stage pipelined unit for multiplying two complex numbers.
// Operation: result = operand_a * operand_b
//
//   Let operand_a = ar + j*ai
//   Let operand_b = br + j*bi
//
//   result_re = ar*br - ai*bi
//   result_im = ar*bi + ai*br
//
// Pipeline Stages:
//   - Stage 1: Register inputs.
//   - Stage 2: Perform the four multiplications in parallel.
//   - Stage 3: Perform additions/subtractions and register the final output.
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
            s2_ac <= '0;
            s2_bd <= '0;
            s2_ad <= '0;
            s2_bc <= '0;
            s3_result <= '0;
        end else begin
            // -- Stage 1: Register the inputs --
            // This isolates the multiplier from the external logic.
            s1_operand_a <= operand_a;
            s1_operand_b <= operand_b;

            // -- Stage 2: Perform the four multiplications in parallel --
            // This is the most computationally intensive stage.
            s2_ac <= s1_operand_a.re * s1_operand_b.re;
            s2_bd <= s1_operand_a.im * s1_operand_b.im;
            s2_ad <= s1_operand_a.re * s1_operand_b.im;
            s2_bc <= s1_operand_a.im * s1_operand_b.re;

            // -- Stage 3: Perform final add/sub and rescale --
            // The multiplication result is 32 bits. We need to scale it
            // back down to our 16-bit fixed-point format (Q3.13).
            // Since we multiplied two Q3.13 numbers, the result is a
            // Q6.26 number. We need to select the correct 16 bits.
            // This is equivalent to a right-shift by 13.
            s3_result.re <= (s2_ac - s2_bd) >>> 13;
            s3_result.im <= (s2_ad + s2_bc) >>> 13;
        end
    end

    // Assign the final registered output to the module's output port
    assign result = s3_result;

endmodule