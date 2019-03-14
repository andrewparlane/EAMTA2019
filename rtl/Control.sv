`timescale 10ns/1ns

import operation_pkg::*;

module Control
(
    input               clk,
    input               rst,
    input [5:0]         cmd_in,
    input               p_error,
    output logic        aluin_reg_en,
    output logic        datain_reg_en,
    output logic        aluout_reg_en,
    output logic        nvalid_data,
    output logic [1:0]  in_select_a,
    output logic [1:0]  in_select_b,
    output Operation    opcode
);

    // cmd_in decode
    assign opcode = Operation'(cmd_in[1:0]);
    assign in_select_a = cmd_in[5:4];
    assign in_select_b = cmd_in[3:2];

    // nvalid_data is high if the last result was an error
    // and we are passing that data back into the ALU
    // (in_select_a or in_select_b is 3)
    assign nvalid_data = p_error &&
                         ((in_select_a == 2'd3) ||
                          (in_select_b == 2'd3));

    enum logic [1:0]
    {
        State_CMD_IN = 0,
        State_ALU_IN,
        State_RESULTS
    } state;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= State_CMD_IN;
            aluin_reg_en <= 0;
            datain_reg_en <= 0;
            aluout_reg_en <= 0;
        end
        else begin
            // all these should only be asserted for one tick
            aluin_reg_en <= 0;
            datain_reg_en <= 0;
            aluout_reg_en <= 0;

            case (state)
                State_CMD_IN:   begin
                    datain_reg_en <= 1;
                    state <= State_ALU_IN;
                end
                State_ALU_IN:   begin
                    aluin_reg_en <= 1;
                    state <= State_RESULTS;
                end
                State_RESULTS:  begin
                    aluout_reg_en <= 1;
                    state <= State_CMD_IN;
                end
            endcase
        end
    end

endmodule
