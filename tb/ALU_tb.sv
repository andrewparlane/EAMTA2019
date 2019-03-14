`timescale 10ns/1ns

import operation_pkg::*;

module ALU_tb;

    localparam WIDTH = 8;

    // min value of a WIDTH bit signed number is
    // MSB set, rest of the bits are 0s
    localparam logic signed [WIDTH-1:0] MIN = 2**(WIDTH-1);
    // max is the opposite
    localparam logic signed [WIDTH-1:0] MAX = (2**(WIDTH-1)) - 1;

    logic signed [WIDTH-1:0]    in1;
    logic signed [WIDTH-1:0]    in2;
    Operation                   op;
    logic                       nvalid_data;
    logic signed [2*WIDTH-1:0]  out;
    logic                       zero;
    logic                       error;

    ALU #(.WIDTH(WIDTH)) dut (.*);

    // fake clock for assertions
    logic clk;
    initial begin
        clk <= 1'b0;
        forever begin
            #1;
            clk = ~clk;
        end
    end

    // if error is set, out must be -1
    errorMeansOutIsMinus1:
    assert property
    (
        @(posedge clk)
        error |-> (out == -1)
    );

    // error cases
    nvalidError:
    assert property
    (
        @(posedge clk)
        nvalid_data |-> error
    );

    invalidOpError:
    assert property
    (
        @(posedge clk)
        (op >= NUM_OPERATIONS) |-> error
    );

    div0Error:
    assert property
    (
        @(posedge clk)
        ((Operation'(op) == Operation_DIV) &&
         (in2 == 0)) |-> error
    );

    // only should be an error if one of those error cases
    errorOnlyIfActualError:
    assert property
    (
        @(posedge clk)
        error |->   (nvalid_data ||
                     op >= NUM_OPERATIONS ||
                     ((Operation'(op) == Operation_DIV) && (in2 == 0)))
    );

    // zero set if out == 0
    zeroTest:
    assert property
    (
        @(posedge clk)
        (out == 0) == zero
    );

    initial begin
        nvalid_data = 0;

        // test ADDs
        op = Operation_ADD;
        for (int a = MIN; a != MAX+1; a++) begin
            in1 = a;
            for (int b = MIN; b != MAX+1; b++) begin
                in2 = b;
                #10;
                addAssert: assert (out == a + b);
            end
        end

        // test SUBs
        op = Operation_SUB;
        for (int a = MIN; a != MAX+1; a++) begin
            in1 = a;
            for (int b = MIN; b != MAX+1; b++) begin
                in2 = b;
                #10;
                subAssert: assert (out == (a - b));
            end
        end

        // test MULs
        op = Operation_MUL;
        for (int a = MIN; a != MAX+1; a++) begin
            in1 = a;
            for (int b = MIN; b != MAX+1; b++) begin
                in2 = b;
                #10;
                mulAssert: assert (out == (a * b));
            end
        end

        // test DIVs
        op = Operation_DIV;
        for (int a = MIN; a != MAX+1; a++) begin
            in1 = a;
            for (int b = MIN; b != MAX+1; b++) begin
                in2 = b;
                #10;
                if (b != 0) begin
                    mulAssert: assert (out == (a / b));
                end
            end
        end

        // test some random ops with everything random
        // but OP valid
        for (int i = 0; i < 1000; i++) begin
            nvalid_data = $random();
            in1 = $random();
            in2 = $random();
            op = Operation'($random() % NUM_OPERATIONS);
            #10;
        end

        // test some random ops with everything random
        // with op maybe being invalid
        for (int i = 0; i < 1000; i++) begin
            nvalid_data = $random();
            in1 = $random();
            in2 = $random();
            op = Operation'($random());
            #10;
        end

        $stop;
    end

endmodule
