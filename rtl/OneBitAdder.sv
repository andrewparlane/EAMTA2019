`timescale 10ns/1ns

module OneBitAdder
(
    input a,
    input b,
    input cin,
    output logic sum,
    output logic cout
);

    assign sum = a ^ b ^ cin;
    assign cout = (a & b) | (a & cin) | (b & cin);

endmodule
