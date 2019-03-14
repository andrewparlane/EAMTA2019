`timescale 10ns/1ns

module OneBitAdder_tb;

    logic a;
    logic b;
    logic cin;
    logic sum;
    logic cout;

    OneBitAdder dut(.*);

    initial begin
        for (int _a = 0; _a <= 1; _a++) begin
            a = _a;
            for (int _b = 0; _b <= 1; _b++) begin
                b = _b;
                for (int _c = 0; _c <= 1; _c++) begin
                    cin = _c;
                    #1;
                    assert({cout, sum} == (a + b + cin));
                end
            end
        end
    end

endmodule
