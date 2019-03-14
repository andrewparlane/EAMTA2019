`timescale 10ns/1ns

import operation_pkg::*;

module ALU
#(
    parameter WIDTH = 8
)
(
    input signed [WIDTH-1:0]            in1,
    input signed [WIDTH-1:0]            in2,
    input Operation                     op,
    input                               nvalid_data,
    output logic signed [2*WIDTH-1:0]   out,
    output logic                        zero,
    output logic                        error
);

    logic unsigned [WIDTH-1:0] absIn1;
    logic unsigned [WIDTH-1:0] absIn2;

    always_comb begin
        absIn1 = in1[WIDTH-1] ? -in1 : in1;
        absIn2 = in2[WIDTH-1] ? -in2 : in2;

        if ((nvalid_data == 1) ||
            (Operation'(op) == Operation_DIV) && (in2 == 0)) begin
            out = '1;   // -1
            error = 1;
            zero = 0;
        end
        else begin
            // assume not an error
            error = 0;

            case (op)
                Operation_ADD:  begin
                    out = in1 + in2;
                    zero = (out == 0);
                end
                Operation_SUB:  begin
                    out = in1 - in2;
                    zero = (out == 0);
                end
                Operation_MUL:  begin
                    out = in1 * in2;
                    zero = (in1 == 0) || (in2 == 0);
                end
                Operation_DIV:  begin
                    out = in1 / in2;
                    zero = absIn1 < absIn2;
                end
                default:        begin
                    out = '1; // -1
                    error = 1;
                    zero = 0;
                end
            endcase
        end
    end

endmodule
