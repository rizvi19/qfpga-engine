// ---------------------------------------------------------------------------
// Filename: engine_pkg.sv
// Author: Shahriar Rizvi
// Date: 08-SEP-2025
//
// Description:
// Instruction set / opcodes (expanded to 4 bits) for the quantum engine.
// Finalized opcode list for current milestone.
// ---------------------------------------------------------------------------
package engine_pkg;

    // --- Opcodes ---
    // 4-bit space allows up to 16 operations (currently using 9).
    typedef enum logic [3:0] {
        OP_NOP      = 4'h0, // No operation / program terminator
        OP_H        = 4'h1, // Hadamard (single-qubit) uses gate applicator
        OP_X        = 4'h2, // Pauli-X (single-qubit) uses gate applicator
        OP_Z        = 4'h3, // Pauli-Z (single-qubit, diagonal fast path)
        OP_PHASE    = 4'h4, // Phase (single-qubit, diagonal fast path) imm=k => e^{i*pi/2^k}
        OP_CNOT     = 4'h5, // Controlled-NOT (two-qubit, swap-like fast path)
        OP_CZ       = 4'h6, // Controlled-Z (two-qubit, diagonal fast path)
        OP_CPHASE   = 4'h7, // Controlled phase (two-qubit, diagonal fast path) imm=k
        OP_SWAP     = 4'h8  // SWAP (two-qubit, swap fast path)
    } opcode_t;

    // --- Instruction Format ---
    // typedef struct packed {
    //     opcode_t opcode;    // [15:12]
    //     logic [3:0] src;    // [11:8] source / control / first qubit index
    //     logic [3:0] dst;    // [7:4]  destination / target / second qubit index
    //     logic [3:0] imm;    // [3:0]  immediate (phase exponent k)
    // } instruction_t; // 16-bit instruction
    typedef struct packed {
        opcode_t opcode;    // The operation to perform
        logic [3:0] src;    // Source or control qubit
        logic [3:0] dst;    // Destination or target qubit
        logic [3:0] imm;    // Immediate parameter / rotation angle
    } instruction_t; // 16-bit instruction

endpackage