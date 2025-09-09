import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer, RisingEdge, ClockCycles
import numpy as np

# Helper function to convert our fixed-point format to a Python float
def fixed_to_float(fixed_val, fractional_bits=13):
    if not isinstance(fixed_val, int):
        fixed_val = fixed_val.integer
    if (fixed_val >> 15) == 1:
        fixed_val = fixed_val - (1 << 16)
    return float(fixed_val) / (2**fractional_bits)

# Helper function to convert float to fixed-point
def float_to_fixed(float_val, fractional_bits=13):
    return int(float_val * (2**fractional_bits)) & 0xFFFF

@cocotb.test()
async def test_gate_applicator_direct(dut):
    """Test the gate applicator directly by driving its inputs."""
    
    # 1. Start the clock
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # 2. Reset the DUT
    dut._log.info("Resetting DUT...")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 2)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
    dut._log.info("Reset complete.")

    # 3. Set up the initial state |0⟩ = [1, 0]
    state_in_1_re = float_to_fixed(1.0)  # Real part of first amplitude
    state_in_1_im = float_to_fixed(0.0)  # Imaginary part of first amplitude
    state_in_2_re = float_to_fixed(0.0)  # Real part of second amplitude
    state_in_2_im = float_to_fixed(0.0)  # Imaginary part of second amplitude
    
    # 4. Set up Hadamard gate matrix: H = (1/√2) * [[1, 1], [1, -1]]
    h_val = float_to_fixed(1.0/np.sqrt(2))  # ≈ 0.7071
    
    # Drive gate applicator inputs
    dut.u_gate_applicator.state_in_1.value = (state_in_1_re << 16) | state_in_1_im
    dut.u_gate_applicator.state_in_2.value = (state_in_2_re << 16) | state_in_2_im
    dut.u_gate_applicator.u_00.value = (h_val << 16) | 0     # H[0,0] = 1/√2
    dut.u_gate_applicator.u_01.value = (h_val << 16) | 0     # H[0,1] = 1/√2
    dut.u_gate_applicator.u_10.value = (h_val << 16) | 0     # H[1,0] = 1/√2
    
    # For H[1,1] = -1/√2, we need to represent negative number in 2's complement
    h_neg_val = ((~h_val) + 1) & 0xFFFF  # Two's complement of h_val
    dut.u_gate_applicator.u_11.value = (h_neg_val << 16) | 0  # H[1,1] = -1/√2
    
    dut._log.info(f"h_val = 0x{h_val:04x} ({fixed_to_float(h_val):.6f})")
    dut._log.info(f"h_neg_val = 0x{h_neg_val:04x} ({fixed_to_float(h_neg_val):.6f})")
    
    # 5. Wait for the pipeline to complete (4 cycles)
    await ClockCycles(dut.clk, 6)
    
    # 6. Read the outputs
    state_out_1_raw = dut.u_gate_applicator.state_out_1.value.integer
    state_out_2_raw = dut.u_gate_applicator.state_out_2.value.integer
    
    # Extract real and imaginary parts
    out_1_re = (state_out_1_raw >> 16) & 0xFFFF
    out_1_im = state_out_1_raw & 0xFFFF
    out_2_re = (state_out_2_raw >> 16) & 0xFFFF
    out_2_im = state_out_2_raw & 0xFFFF
    
    # Convert to floating point
    out_1_re_float = fixed_to_float(out_1_re)
    out_1_im_float = fixed_to_float(out_1_im)
    out_2_re_float = fixed_to_float(out_2_re)
    out_2_im_float = fixed_to_float(out_2_im)
    
    hw_result = np.array([out_1_re_float + 1j*out_1_im_float, out_2_re_float + 1j*out_2_im_float])
    golden_state = np.array([1/np.sqrt(2) + 0.j, 1/np.sqrt(2) + 0.j])
    
    dut._log.info(f"Hardware result: {hw_result}")
    dut._log.info(f"Golden state: {golden_state}")
    
    # 7. Check the result
    assert np.allclose(hw_result, golden_state, atol=1e-3), "Gate applicator result does not match expected!"
    dut._log.info("✅ Gate Applicator Test Passed!")
