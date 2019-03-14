`timescale 10ns/1ns

module Mux4_tb;

    localparam WIDTH = 8;

    logic [WIDTH-1:0]   din1;
    logic [WIDTH-1:0]   din2;
    logic [WIDTH-1:0]   din3;
    logic [WIDTH-1:0]   din4;
    logic [1:0]         select;
    logic [WIDTH-1:0]   dout;

    Mux4 #(.WIDTH(WIDTH)) dut(.*);

    logic [WIDTH-1:0]   in [4];

    assign in[0] = din1;
    assign in[1] = din2;
    assign in[2] = din3;
    assign in[3] = din4;

    initial begin

        for (int s = 0; s < 4; s++) begin
            select = s;
            // run 100 tests on random vars per select
            for (int t = 0; t < 100; t++) begin
                din1 = $random();
                din2 = $random();
                din3 = $random();
                din4 = $random();

                #1;

                assert (dout == in[s]);

            end
        end

    end

endmodule
