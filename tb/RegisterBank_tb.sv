`timescale 10ns/1ns

module RegisterBank_tb;

    localparam WIDTH = 8;

    logic               clk;
    logic               rst;
    logic               wr_en;
    logic [WIDTH-1:0]   in;
    logic [WIDTH-1:0]   out;

    RegisterBank #(.WIDTH(WIDTH)) dut (.*);

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

    initial begin
        rst = 0;
        wr_en = 0;
        in = 0;

        // check it resets OK
        rst = 1;
        @(posedge clk);
        #1;
        assert(out == 0);

        // check it stays constant
        rst = 0;
        @(posedge clk);
        #1;
        assert(out == 0);

        // check it doesn't change if wr_en isn't set
        in = 8'hA5;
        @(posedge clk);
        #1;
        assert(out == 0);

        // check it does change if wr_en is set
        wr_en = 1;
        @(posedge clk);
        #1;
        assert(out == 8'hA5);

        // check we can reset it again
        // and that rst is syncronous
        rst = 1;
        #1
        assert(out == 8'hA5);   // hasn't reset yet
        @(posedge clk);
        #1;
        assert(out == 0);       // now it has

        $stop;
    end

endmodule
