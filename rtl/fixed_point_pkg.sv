// In: rtl/fixed_point_pkg.sv

package fixed_point_pkg;

    // Q1.15 fixed-point type: 1 sign + 15 fractional bits.
    typedef logic signed [15:0] fixed_point_t;

    // Complex number type.
    typedef struct packed {
        fixed_point_t re;
        fixed_point_t im;
    } complex_t;

    // Scaling constants (saturated where needed)
    localparam fixed_point_t FP_ONE       = 16'sd32767;  // ~+1.0
    localparam fixed_point_t FP_NEG_ONE   = -16'sd32767; // ~-1.0
    localparam fixed_point_t FP_ZERO      = 16'sd0;
    localparam fixed_point_t FP_INV_SQRT2 = 16'sd23170;  // round(32767 / sqrt(2))

    // Phase LUT for e^{i * (pi / 2^k)}, k=0..15
    // angle_k = pi / 2^k
    // Values in Q1.15 (cos and sin components)
    localparam fixed_point_t phase_cos_lut [0:15] = '{
        // k:   0        1       2        3        4        5        6        7
        -16'sd32767, 16'sd0, 16'sd23170, 16'sd30274, 16'sd32138, 16'sd32610, 16'sd32729, 16'sd32757,
        // k:   8        9        10       11       12       13       14       15
         16'sd32763, 16'sd32766, 16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767
    };
    localparam fixed_point_t phase_sin_lut [0:15] = '{
        // k:   0       1        2        3       4       5       6       7
         16'sd0, 16'sd32767, 16'sd23170, 16'sd12540, 16'sd6383, 16'sd3212, 16'sd1608, 16'sd804,
        // k:   8       9        10       11      12      13      14      15
         16'sd402, 16'sd201, 16'sd100, 16'sd50, 16'sd25, 16'sd13, 16'sd6, 16'sd3
    };

endpackage