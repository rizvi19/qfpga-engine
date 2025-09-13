// ---------------------------------------------------------------------------
// Filename: instruction_sequencer.sv
// NOTE: Completed diagonal gates (Z,PHASE,CZ,CPHASE), added CNOT & SWAP pair
// iteration, populated algorithm ROMs (QFT4, Grover2, QPE3), unified pair gate
// handling, PC advance for diagonal completion.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

import fixed_point_pkg::*;
import engine_pkg::*;

module instruction_sequencer #(
    parameter int ADDR_WIDTH = 10,
    parameter int N_QUBITS   = 4
) (
    // -- System Signals --
    input  logic clk,
    input  logic rst_n,
    input  logic start_computation,
    input  logic [1:0] program_select,
    output logic computation_done,

    // -- BRAM Interface --
    output logic                      bram_rd_en,
    output logic [ADDR_WIDTH-1:0]     bram_rd_addr,
    input  complex_t                  bram_rd_data,
    output logic                      bram_wr_en,
    output logic [ADDR_WIDTH-1:0]     bram_wr_addr,
    output complex_t                  bram_wr_data,

    // -- Gate Applicator Interface --
    output complex_t                  gate_u_00,
    output complex_t                  gate_u_01,
    output complex_t                  gate_u_10,
    output complex_t                  gate_u_11,
    output complex_t                  gate_state_in_1,
    output complex_t                  gate_state_in_2,
    input  complex_t                  gate_state_out_1,
    input  complex_t                  gate_state_out_2,

    // -- Performance / profiling --
    output logic [31:0]               cycle_counter
);

    // --- FSM State Definition ---
    typedef enum logic [4:0] {
        S_IDLE, S_DECODE,
        // Pair fetch path
        S_FETCH_1_ADDR, S_FETCH_1_WAIT, S_FETCH_2_ADDR, S_FETCH_2_WAIT,
        S_EXECUTE, S_GATE_WAIT_1, S_GATE_WAIT_2, S_GATE_WAIT_3, S_GATE_WAIT_4, S_WRITE_1, S_WRITE_2,
        // Diagonal path (single amplitude)
        S_DIAG_FETCH_ADDR, S_DIAG_FETCH_WAIT, S_DIAG_APPLY, S_DIAG_WRITE,
        S_DONE
    } state_t;

    state_t current_state, next_state;

    localparam int MAX_PROG_LEN = 32;

    // Instruction ROM (4 programs * MAX_PROG_LEN)
    reg [15:0] instruction_rom [0:4*MAX_PROG_LEN-1];
    initial begin
        integer i; for (i=0;i<4*MAX_PROG_LEN;i++) instruction_rom[i] = 16'h0000;
        // ---------------- Program 0: QFT4 ----------------
        // Sequence: H q0; CPHASE (1) 1->0; CPHASE (2) 2->0; CPHASE (3) 3->0;
        //           H q1; CPHASE (1) 2->1; CPHASE (2) 3->1;
        //           H q2; CPHASE (1) 3->2;
        //           H q3; SWAP 0,3; SWAP 1,2
        instruction_rom[0]  = {OP_H,      4'd0,4'd0,4'd0}; // H q0
        instruction_rom[1]  = {OP_CPHASE, 4'd1,4'd0,4'd1}; // cp(π/2) 1->0
        instruction_rom[2]  = {OP_CPHASE, 4'd2,4'd0,4'd2}; // cp(π/4) 2->0
        instruction_rom[3]  = {OP_CPHASE, 4'd3,4'd0,4'd3}; // cp(π/8) 3->0
        instruction_rom[4]  = {OP_H,      4'd1,4'd1,4'd0}; // H q1
        instruction_rom[5]  = {OP_CPHASE, 4'd2,4'd1,4'd1}; // cp(π/2) 2->1
        instruction_rom[6]  = {OP_CPHASE, 4'd3,4'd1,4'd2}; // cp(π/4) 3->1
        instruction_rom[7]  = {OP_H,      4'd2,4'd2,4'd0}; // H q2
        instruction_rom[8]  = {OP_CPHASE, 4'd3,4'd2,4'd1}; // cp(π/2) 3->2
        instruction_rom[9]  = {OP_H,      4'd3,4'd3,4'd0}; // H q3
        instruction_rom[10] = {OP_SWAP,   4'd0,4'd3,4'd0}; // SWAP 0,3
        instruction_rom[11] = {OP_SWAP,   4'd1,4'd2,4'd0}; // SWAP 1,2
        instruction_rom[12] = {OP_NOP,    4'd0,4'd0,4'd0};
        // ---------------- Program 1: Grover 2-qubit (one iteration) ----------------
        // H all; Oracle (CZ 1->0); Diffusion: H all; X all; CZ 1->0; X all; H all
        instruction_rom[32] = {OP_H,    4'd0,4'd0,4'd0}; // H q0
        instruction_rom[33] = {OP_H,    4'd1,4'd1,4'd0}; // H q1
        instruction_rom[34] = {OP_CZ,   4'd1,4'd0,4'd0}; // Oracle CZ
        instruction_rom[35] = {OP_H,    4'd0,4'd0,4'd0}; // H q0
        instruction_rom[36] = {OP_H,    4'd1,4'd1,4'd0}; // H q1
        instruction_rom[37] = {OP_X,    4'd0,4'd0,4'd0}; // X q0
        instruction_rom[38] = {OP_X,    4'd1,4'd1,4'd0}; // X q1
        instruction_rom[39] = {OP_CZ,   4'd1,4'd0,4'd0}; // CZ
        instruction_rom[40] = {OP_X,    4'd0,4'd0,4'd0}; // X q0
        instruction_rom[41] = {OP_X,    4'd1,4'd1,4'd0}; // X q1
        instruction_rom[42] = {OP_H,    4'd0,4'd0,4'd0}; // H q0
        instruction_rom[43] = {OP_H,    4'd1,4'd1,4'd0}; // H q1
        instruction_rom[44] = {OP_NOP,  4'd0,4'd0,4'd0};
        // ---------------- Program 2: QPE3 (placeholder simplified) ----------------
        // Counting qubits: 0,1; Eigen qubit: 2 (assumed |1>). Apply H on 0,1; CZ 1->2; CZ 0->2; H 1; H 0; SWAP 0,1
        instruction_rom[64] = {OP_H,    4'd0,4'd0,4'd0}; // H q0
        instruction_rom[65] = {OP_H,    4'd1,4'd1,4'd0}; // H q1
        instruction_rom[66] = {OP_CZ,   4'd1,4'd2,4'd0}; // CZ 1->2
        instruction_rom[67] = {OP_CZ,   4'd0,4'd2,4'd0}; // CZ 0->2
        instruction_rom[68] = {OP_H,    4'd1,4'd1,4'd0}; // H q1 (inverse QFT partial)
        instruction_rom[69] = {OP_H,    4'd0,4'd0,4'd0}; // H q0
        instruction_rom[70] = {OP_SWAP, 4'd0,4'd1,4'd0}; // SWAP 0,1 (bit reversal)
        instruction_rom[71] = {OP_NOP,  4'd0,4'd0,4'd0};
        // ---------------- Program 3: CNOT test (control 0 target 1) ----------------
        instruction_rom[96] = {OP_CNOT, 4'd0,4'd1,4'd0};
        instruction_rom[97] = {OP_NOP,  4'd0,4'd0,4'd0};
    end

    // Program base decode
    logic [7:0] program_base;
    always_comb begin
        case(program_select)
            2'd0: program_base = 0;
            2'd1: program_base = 32;
            2'd2: program_base = 64;
            2'd3: program_base = 96;
            default: program_base = 0;
        endcase
    end

    // PC & fields
    reg [7:0] pc; wire [15:0] instr_bits = instruction_rom[program_base + pc];
    wire opcode_t opcode = instr_bits[15:12];
    wire [3:0] instr_src = instr_bits[11:8]; // primary qubit (or control)
    wire [3:0] instr_dst = instr_bits[7:4];  // secondary qubit (target)
    wire [3:0] instr_imm = instr_bits[3:0];  // phase exponent k

    // Classification
    wire is_single_qubit_pair = (opcode == OP_H) || (opcode == OP_X);
    wire is_cnot_pair         = (opcode == OP_CNOT);
    wire is_swap_pair         = (opcode == OP_SWAP);
    wire is_pair_gate         = is_single_qubit_pair || is_cnot_pair || is_swap_pair;
    wire is_diagonal_single   = (opcode == OP_Z) || (opcode == OP_PHASE);
    wire is_diagonal_two      = (opcode == OP_CZ) || (opcode == OP_CPHASE);
    wire is_diagonal          = is_diagonal_single || is_diagonal_two;

    // Iteration state
    reg [ADDR_WIDTH-1:0] addr1, addr2;
    reg [ADDR_WIDTH-1:0] pair_index;
    reg [ADDR_WIDTH-1:0] pair_count_target;
    reg advance_pc;

    reg [ADDR_WIDTH-1:0] diag_index;
    reg [ADDR_WIDTH-1:0] diag_count_target;
    reg diag_done_flag;

    // State registers
    reg complex_t reg_state_1, reg_state_2;
    reg complex_t reg_diag, diag_new;

    // Cycle counter
    reg [31:0] cycle_ctr_q; assign cycle_counter = cycle_ctr_q;

    // --- Helper: build base for single qubit pair ---
    function automatic [ADDR_WIDTH-1:0] build_pair_single(input [ADDR_WIDTH-1:0] n, input int q);
        if (q == 0) build_pair_single = (n << 1);
        else build_pair_single = ({n >> q, 1'b0} << q) | (n & ((1<<q)-1));
    endfunction

    // --- Helper: build base for controlled pair (control=1,target=0) ---
    function automatic [ADDR_WIDTH-1:0] build_pair_cnot(input [ADDR_WIDTH-1:0] n, input int control, input int target);
        int free_idx = 0; build_pair_cnot = '0;
        for (int q=0; q<N_QUBITS; q++) begin
            if (q == control) build_pair_cnot[q] = 1'b1;
            else if (q == target) build_pair_cnot[q] = 1'b0;
            else begin build_pair_cnot[q] = (n >> free_idx) & 1'b1; free_idx++; end
        end
    endfunction

    // --- Helper: build base for swap pair (qa=0,qb=1) ---
    function automatic [ADDR_WIDTH-1:0] build_pair_swap(input [ADDR_WIDTH-1:0] n, input int qa, input int qb);
        int free_idx = 0; build_pair_swap = '0;
        for (int q=0; q<N_QUBITS; q++) begin
            if (q == qa) build_pair_swap[q] = 1'b0;
            else if (q == qb) build_pair_swap[q] = 1'b1;
            else begin build_pair_swap[q] = (n >> free_idx) & 1'b1; free_idx++; end
        end
    endfunction

    // --- State Register ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) current_state <= S_IDLE; else current_state <= next_state;
    end

    // --- Datapath Registers / Control ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 0; pair_index <= 0; pair_count_target <= 0; advance_pc <= 0;
            addr1 <= 0; addr2 <= 1; reg_state_1 <= '0; reg_state_2 <= '0;
            diag_index <= 0; diag_count_target <= 0; diag_done_flag <= 0; reg_diag <= '0; diag_new <= '0;
            cycle_ctr_q <= 0;
        end else begin
            // Cycle counter
            if (current_state != S_IDLE && current_state != S_DONE) cycle_ctr_q <= cycle_ctr_q + 1;
            if (current_state == S_IDLE && next_state == S_DECODE) cycle_ctr_q <= 0;

            // PC increment logic
            if (advance_pc) pc <= pc + 1;
            else if (current_state == S_IDLE && next_state == S_DECODE) pc <= 0;

            // Initialize iteration at decode
            if (current_state == S_DECODE && next_state != S_DONE) begin
                advance_pc <= 0;
                if (is_pair_gate) begin
                    pair_index <= 0;
                    if (is_single_qubit_pair) pair_count_target <= (1 << (N_QUBITS-1));
                    else if (is_cnot_pair || is_swap_pair) pair_count_target <= (1 << (N_QUBITS-2));
                    else pair_count_target <= 0;
                end else if (is_diagonal) begin
                    diag_index <= 0; diag_count_target <= (1 << N_QUBITS); diag_done_flag <= 0;
                end
            end

            // Capture fetched states
            if (current_state == S_FETCH_1_WAIT) reg_state_1 <= bram_rd_data;
            if (current_state == S_FETCH_2_WAIT) reg_state_2 <= bram_rd_data;
            if (current_state == S_DIAG_FETCH_WAIT) reg_diag <= bram_rd_data;

            // Pair iteration advance
            if (current_state == S_WRITE_2 && next_state == S_DECODE && is_pair_gate) begin
                if (pair_index + 1 < pair_count_target) begin
                    pair_index <= pair_index + 1;
                end else begin
                    advance_pc <= 1; // instruction complete
                end
            end

            // Diagonal iteration advance
            if (current_state == S_DIAG_WRITE) begin
                if (diag_index + 1 < diag_count_target) begin
                    diag_index <= diag_index + 1;
                end else begin
                    diag_done_flag <= 1; advance_pc <= 1; // instruction complete
                end
            end
        end
    end

    // --- Pair Address Generation ---
    logic [ADDR_WIDTH-1:0] gen_addr1, gen_addr2;
    always_comb begin
        gen_addr1 = addr1; gen_addr2 = addr2;
        if (is_pair_gate) begin
            if (is_single_qubit_pair) begin
                logic [ADDR_WIDTH-1:0] base = build_pair_single(pair_index, instr_src);
                gen_addr1 = base;
                gen_addr2 = base | (1 << instr_src);
            end else if (is_cnot_pair) begin
                logic [ADDR_WIDTH-1:0] base = build_pair_cnot(pair_index, instr_src, instr_dst);
                gen_addr1 = base; // control=1 target=0
                gen_addr2 = base | (1 << instr_dst); // control=1 target=1
            end else if (is_swap_pair) begin
                logic [ADDR_WIDTH-1:0] base = build_pair_swap(pair_index, instr_src, instr_dst);
                gen_addr1 = base; // qa=0 qb=1
                // swap pair toggles both bits (qa->1, qb->0)
                gen_addr2 = (base | (1<<instr_src)) & ~(1<<instr_dst);
            end
        end
    end

    // Latch new pair addresses at start & iteration
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr1 <= 0; addr2 <= 1;
        end else begin
            if (current_state == S_DECODE && next_state == S_FETCH_1_ADDR && is_pair_gate) begin
                addr1 <= gen_addr1; addr2 <= gen_addr2;
            end
            if (current_state == S_WRITE_2 && next_state == S_DECODE && is_pair_gate) begin
                addr1 <= gen_addr1; addr2 <= gen_addr2;
            end
        end
    end

    // --- Diagonal gate computation ---
    always_comb begin
        diag_new = reg_diag;
        if (is_diagonal) begin
            logic cond_single = is_diagonal_single && ((diag_index >> instr_src) & 1);
            logic cond_two    = is_diagonal_two && ((diag_index >> instr_src) & 1) && ((diag_index >> instr_dst) & 1);
            if (opcode == OP_Z && cond_single) begin
                diag_new.re = -reg_diag.re; diag_new.im = -reg_diag.im;
            end else if (opcode == OP_PHASE && cond_single) begin
                fixed_point_t cosv = phase_cos_lut[instr_imm];
                fixed_point_t sinv = phase_sin_lut[instr_imm];
                logic signed [31:0] t_re = reg_diag.re * cosv - reg_diag.im * sinv;
                logic signed [31:0] t_im = reg_diag.re * sinv + reg_diag.im * cosv;
                diag_new.re = t_re >>> 15; diag_new.im = t_im >>> 15;
            end else if (opcode == OP_CZ && cond_two) begin
                diag_new.re = -reg_diag.re; diag_new.im = -reg_diag.im;
            end else if (opcode == OP_CPHASE && cond_two) begin
                fixed_point_t cosv2 = phase_cos_lut[instr_imm];
                fixed_point_t sinv2 = phase_sin_lut[instr_imm];
                logic signed [31:0] t2_re = reg_diag.re * cosv2 - reg_diag.im * sinv2;
                logic signed [31:0] t2_im = reg_diag.re * sinv2 + reg_diag.im * cosv2;
                diag_new.re = t2_re >>> 15; diag_new.im = t2_im >>> 15;
            end
        end
    end

    // --- FSM Comb Logic ---
    always_comb begin
        next_state = current_state; computation_done = 1'b0;
        // Default outputs
        bram_rd_en = 0; bram_wr_en = 0; bram_rd_addr = '0; bram_wr_addr = '0; bram_wr_data = '0;
        gate_u_00='0; gate_u_01='0; gate_u_10='0; gate_u_11='0; gate_state_in_1='0; gate_state_in_2='0;

        unique case(current_state)
            S_IDLE: if(start_computation) next_state = S_DECODE;
            S_DECODE: begin
                if (opcode == OP_NOP) next_state = S_DONE;
                else if (is_diagonal) next_state = S_DIAG_FETCH_ADDR;
                else if (is_pair_gate) next_state = S_FETCH_1_ADDR;
                else next_state = S_DONE; // safety
            end
            // Pair path
            S_FETCH_1_ADDR: begin bram_rd_en=1; bram_rd_addr=addr1; next_state = S_FETCH_1_WAIT; end
            S_FETCH_1_WAIT: next_state = S_FETCH_2_ADDR;
            S_FETCH_2_ADDR: begin bram_rd_en=1; bram_rd_addr=addr2; next_state = S_FETCH_2_WAIT; end
            S_FETCH_2_WAIT: next_state = S_EXECUTE;
            S_EXECUTE: begin
                gate_state_in_1 = reg_state_1; gate_state_in_2 = reg_state_2;
                if (opcode == OP_H) begin
                    gate_u_00 = '{re:FP_INV_SQRT2, im:FP_ZERO}; gate_u_01 = '{re:FP_INV_SQRT2, im:FP_ZERO};
                    gate_u_10 = '{re:FP_INV_SQRT2, im:FP_ZERO}; gate_u_11 = '{re:-FP_INV_SQRT2, im:FP_ZERO};
                end else if (opcode == OP_X || opcode == OP_CNOT || opcode == OP_SWAP) begin
                    gate_u_00 = '{re:FP_ZERO, im:FP_ZERO}; gate_u_01 = '{re:FP_ONE, im:FP_ZERO};
                    gate_u_10 = '{re:FP_ONE, im:FP_ZERO}; gate_u_11 = '{re:FP_ZERO, im:FP_ZERO};
                end else begin
                    gate_u_00 = '{re:FP_ONE, im:FP_ZERO}; gate_u_11 = '{re:FP_ONE, im:FP_ZERO};
                end
                next_state = S_GATE_WAIT_1;
            end
            S_GATE_WAIT_1, S_GATE_WAIT_2, S_GATE_WAIT_3, S_GATE_WAIT_4: begin
                gate_state_in_1 = reg_state_1; gate_state_in_2 = reg_state_2;
                if (opcode == OP_H) begin
                    gate_u_00 = '{re:FP_INV_SQRT2, im:FP_ZERO}; gate_u_01 = '{re:FP_INV_SQRT2, im:FP_ZERO};
                    gate_u_10 = '{re:FP_INV_SQRT2, im:FP_ZERO}; gate_u_11 = '{re:-FP_INV_SQRT2, im:FP_ZERO};
                end else if (opcode == OP_X || opcode == OP_CNOT || opcode == OP_SWAP) begin
                    gate_u_00 = '{re:FP_ZERO, im:FP_ZERO}; gate_u_01 = '{re:FP_ONE, im:FP_ZERO};
                    gate_u_10 = '{re:FP_ONE, im:FP_ZERO}; gate_u_11 = '{re:FP_ZERO, im:FP_ZERO};
                end else begin
                    gate_u_00 = '{re:FP_ONE, im:FP_ZERO}; gate_u_11 = '{re:FP_ONE, im:FP_ZERO};
                end
                if (current_state == S_GATE_WAIT_4) next_state = S_WRITE_1; else next_state = S_GATE_WAIT_1 + (current_state - S_GATE_WAIT_1) + 1;
            end
            S_WRITE_1: begin bram_wr_en=1; bram_wr_addr=addr1; bram_wr_data = gate_state_out_1; next_state = S_WRITE_2; end
            S_WRITE_2: begin bram_wr_en=1; bram_wr_addr=addr2; bram_wr_data = gate_state_out_2; next_state = S_DECODE; end
            // Diagonal path
            S_DIAG_FETCH_ADDR: begin bram_rd_en=1; bram_rd_addr=diag_index; next_state = S_DIAG_FETCH_WAIT; end
            S_DIAG_FETCH_WAIT: next_state = S_DIAG_APPLY;
            S_DIAG_APPLY: next_state = S_DIAG_WRITE;
            S_DIAG_WRITE: begin bram_wr_en=1; bram_wr_addr=diag_index; bram_wr_data = diag_new; if (diag_done_flag) next_state = S_DECODE; else next_state = S_DIAG_FETCH_ADDR; end
            S_DONE: begin computation_done = 1'b1; if(!start_computation) next_state = S_IDLE; end
            default: next_state = S_IDLE;
        endcase
    end

endmodule