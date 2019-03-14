`timescale 10ns/1ns

import operation_pkg::*;

module Control_tb;

    logic       clk;
    logic       rst;
    logic [5:0] cmd_in;
    logic       p_error;
    logic       aluin_reg_en;
    logic       datain_reg_en;
    logic       aluout_reg_en;
    logic       nvalid_data;
    logic [1:0] in_select_a;
    logic [1:0] in_select_b;
    Operation   opcode;

    Control dut(.*);

    // --------------------------------------------------------------
    // Generate the clock
    // --------------------------------------------------------------
    localparam CLOCK_FREQUENCY_MHZ = 100;
    localparam CLOCK_PERIOD_NS = 1000 / CLOCK_FREQUENCY_MHZ;

    initial begin
        clk <= 1'b0;
        forever begin
            #(CLOCK_PERIOD_NS/2);
            clk = ~clk;
        end
    end

    // --------------------------------------------------------------
    // Test stimulus
    // --------------------------------------------------------------
    logic [5:0] r;
    initial begin
        cmd_in = 0;
        p_error = 0;

        // reset
        rst = 1;
        @(posedge clk);
        @(posedge clk);
        rst = 0;
        @(posedge clk);

        for (int i = 0; i < 1000000; i++) begin
            cmd_in = $random();
            p_error = $random();
            r = $random(); // random number between 0 and 63
            if (r == 0) begin
                rst = 1;
            end
            else begin
                rst = 0;
            end
            @(posedge clk);
        end

        $stop;
    end

    muxSelA:
    assert property
    (
        @(posedge clk)
        (in_select_a == cmd_in[5:4])
    );

    muxSelB:
    assert property
    (
        @(posedge clk)
        (in_select_b == cmd_in[3:2])
    );

    opDecode:
    assert property
    (
        @(posedge clk)
        (opcode == Operation'({2'b00, cmd_in[1:0]}))
    );

    dataValidChec:
    assert property
    (
        @(posedge clk)
        ((p_error && (in_select_a == 2'b11 ||
                      in_select_b == 2'b11))
            == nvalid_data)
    );

    dataInOneTick:
    assert property
    (
        @(posedge clk)
        (datain_reg_en && !rst) |=> $fell(datain_reg_en)
    );

    aluInOneTick:
    assert property
    (
        @(posedge clk)
        aluin_reg_en |=> $fell(aluin_reg_en)
    );

    aluoutOneTick:
    assert property
    (
        @(posedge clk)
        aluout_reg_en |=> $fell(aluout_reg_en)
    );

    dataInToAluIn:
    assert property
    (
        @(posedge clk)
        ($rose(datain_reg_en) && !rst) |=> $rose(aluin_reg_en)
    );

    AluInToAluOut:
    assert property
    (
        @(posedge clk)
        (aluin_reg_en && !rst) |=> $rose(aluout_reg_en)
    );

    AluOutToDataIn:
    assert property
    (
        @(posedge clk)
        (aluout_reg_en && !rst) |=> datain_reg_en
    );

    resetCheck:
    assert property
    (
        @(posedge clk)
        ($fell(rst)) |=> $rose(datain_reg_en)
    );

    onlyOneRegEn1:
    assert property
    (
        @(posedge clk)
        (datain_reg_en |-> (!aluin_reg_en && !aluout_reg_en))
    );

    onlyOneRegEn2:
    assert property
    (
        @(posedge clk)
        (aluin_reg_en  |-> (!datain_reg_en && !aluout_reg_en))
    );

    onlyOneRegEn3:
    assert property
    (
        @(posedge clk)
        (aluout_reg_en |-> (!aluin_reg_en && !datain_reg_en))
    );

endmodule
