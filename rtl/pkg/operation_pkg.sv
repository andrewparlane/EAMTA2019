`timescale 10ns/1ns

package operation_pkg;

    typedef enum logic [3:0]
    {
        Operation_ADD = 0,
        Operation_SUB,
        Operation_MUL,
        Operation_DIV,

        NUM_OPERATIONS
    } Operation;

endpackage