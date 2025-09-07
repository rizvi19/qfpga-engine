// In: rtl/fixed_point_pkg.sv

package fixed_point_pkg;

    // Define a 16-bit fixed-point type.
    // Q3.13 means 1 sign bit, 2 integer bits, 13 fractional bits.
    // This gives us a range of [-4, 3.999...]
    typedef logic signed [15:0] fixed_point_t;

    // Define a complex number using our fixed-point type.
    typedef struct packed {
        fixed_point_t re; // Real part
        fixed_point_t im; // Imaginary part
    } complex_t;

endpackage