`timescale 10ns/1ns

import operation_pkg::*;

module Top
#(
    parameter WIDTH=8
)
(
    input                       clk,
    input                       rst,
    input [5:0]                 cmdin,
    input [WIDTH-1:0]           din_1,
    input [WIDTH-1:0]           din_2,
    input [WIDTH-1:0]           din_3,
    output logic [WIDTH-1:0]    dout_low,
    output logic [WIDTH-1:0]    dout_high,
    output logic                zero,
    output logic                error
);

    logic [1:0]         muxA_sel;
    logic [1:0]         muxB_sel;
    logic [WIDTH-1:0]   muxA_out;
    logic [WIDTH-1:0]   muxB_out;

    logic               muxReg_we;

    logic [WIDTH-1:0]   alu_in1;
    logic [WIDTH-1:0]   alu_in2;

    logic               cmdInReg_we;
    logic [5:0]         cmdinRegOut;

    Operation           alu_op;
    logic               alu_nvalid_data;
    logic [2*WIDTH-1:0] alu_out;
    logic               alu_zero;
    logic               alu_error;
    logic               aluReg_we;

    // MUX A and it's register
    Mux4 #(.WIDTH(WIDTH)) muxA
    (
        .din1   (din_1),
        .din2   (din_2),
        .din3   (din_3),
        .din4   (dout_low),
        .select (muxA_sel),
        .dout   (muxA_out)
    );

    RegisterBank #(.WIDTH(WIDTH)) muxAReg
    (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (muxReg_we),
        .in     (muxA_out),
        .out    (alu_in1)
    );

    // MUX B and it's register
    Mux4 #(.WIDTH(WIDTH)) muxB
    (
        .din1   (din_1),
        .din2   (din_2),
        .din3   (din_3),
        .din4   (dout_low),
        .select (muxB_sel),
        .dout   (muxB_out)
    );

    RegisterBank #(.WIDTH(WIDTH)) muxBReg
    (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (muxReg_we),
        .in     (muxB_out),
        .out    (alu_in2)
    );

    // CMDIN's register
    RegisterBank #(.WIDTH(6)) cmdInReg
    (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (cmdInReg_we),
        .in     (cmdin),
        .out    (cmdinRegOut)
    );

    // ALU and it's registers
    ALU #(.WIDTH(WIDTH)) alu
    (
        .in1            (alu_in1),
        .in2            (alu_in2),
        .op             (alu_op),
        .nvalid_data    (alu_nvalid_data),
        .out            (alu_out),
        .zero           (alu_zero),
        .error          (alu_error)
    );

    RegisterBank #(.WIDTH(2*WIDTH)) aluReg
    (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (aluReg_we),
        .in     (alu_out),
        .out    ({dout_high, dout_low})
    );

    RegisterBank #(.WIDTH(2)) aluFlagsReg
    (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (aluReg_we),
        .in     ({alu_zero, alu_error}),
        .out    ({zero, error})
    );

    // Control
    Control ctrl
    (
        .clk            (clk),
        .rst            (rst),
        .cmd_in         (cmdinRegOut),
        .p_error        (error),
        .aluin_reg_en   (muxReg_we),
        .datain_reg_en  (cmdInReg_we),
        .aluout_reg_en  (aluReg_we),
        .nvalid_data    (alu_nvalid_data),
        .in_select_a    (muxA_sel),
        .in_select_b    (muxB_sel),
        .opcode         (alu_op)
    );


endmodule
