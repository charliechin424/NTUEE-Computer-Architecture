module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         2 : 0]      i_inst,  // instruction

    output [2*DATA_W - 1 : 0]   o_data,  // output value
    output                      o_done   // output valid signal
);
// Do not Modify the above part !!!

// Parameters

    //FSM based on operation cycles
    parameter S_IDLE           = 2'd0;
    parameter S_ONE_CYCLE_OP   = 2'd1;
    parameter S_MULTI_CYCLE_OP = 2'd2;

    //operation modes
    parameter ADD  = 3'd0;
    parameter SUB  = 3'd1;
    parameter AND  = 3'd2;
    parameter OR   = 3'd3;
    parameter SLT  = 3'd4;
    parameter SRA  = 3'd5;
    parameter MUL  = 3'd6;
    parameter DIV  = 3'd7;

// Wires & Regs
    // Todo
    reg done, done_nxt;
    reg [2*DATA_W - 1 : 0] shift_register, shift_register_nxt;
    reg [4:0] counter , counter_nxt;
    reg [32:0] alu_out;
    reg div_control;

    // state
    reg  [1: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!

    // load input
    reg  [DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [2: 0] inst, inst_nxt;

// Wire Assignments
    // Todo
    assign o_done = done;
    assign o_data = shift_register;

// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            inst_nxt      = i_inst;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            inst_nxt      = inst;
        end
    end

    // Todo: FSM
    always @(*) begin
        case(state)

            S_IDLE: begin
                if (i_valid) begin
                    if (i_inst <= 3'd5) begin
                        state_nxt = S_ONE_CYCLE_OP;
                    end else begin
                        state_nxt = S_MULTI_CYCLE_OP;
                    end
                end else begin
                    state_nxt = S_IDLE;
                end
            end

            S_ONE_CYCLE_OP: begin
                state_nxt = S_IDLE;
            end

            S_MULTI_CYCLE_OP: begin
                if (counter == 5'd31) begin
                    state_nxt = S_IDLE;
                end else begin
                    state_nxt = state;
                end
            end

            default : state_nxt = state;
        endcase
    end

    // Todo: Counter
    always @(*) begin
        if (state == S_MULTI_CYCLE_OP) begin
            counter_nxt = counter + 1'b1;
        end else begin
            counter_nxt = 5'b0;
        end
    end

    // Todo: ALU output
    always @(*) begin
        div_control = 1'b0;
        alu_out = 33'd0;
        case(state)

            S_ONE_CYCLE_OP: begin
                div_control = 1'b0;
                if (inst == ADD) begin
                    alu_out = $signed(shift_register[31:0]) + $signed(operand_b);
                    if ($signed(shift_register[31:0]) > 0 && $signed(operand_b) > 0 && $signed(alu_out[31:0]) < 0) begin
                        alu_out[31:0] = {1'b0, {31{1'b1}}};
                    end else if ($signed(shift_register[31:0]) < 0 && $signed(operand_b) < 0 && $signed(alu_out[31:0]) > 0) begin
                        alu_out[31:0] = {1'b1, {31{1'b0}}};
                    end else begin
                        alu_out[31:0] = $signed(shift_register[31:0]) + $signed(operand_b);
                    end
                end 

                else if (inst == SUB) begin
                    alu_out = $signed(shift_register[31:0]) - $signed(operand_b);
                    if ($signed(shift_register[31:0]) > 0 && $signed(operand_b) < 0 && $signed(alu_out[31:0]) < 0) begin
                        alu_out[31:0] = {1'b0, {31{1'b1}}};
                    end else if ($signed(shift_register[31:0]) < 0 && $signed(operand_b) > 0 && $signed(alu_out[31:0]) > 0) begin
                        alu_out[31:0] = {1'b1, {31{1'b0}}};
                    end else begin
                        alu_out[31:0] = $signed(shift_register[31:0]) - $signed(operand_b);
                    end 
                end 

                else if (inst == AND) begin
                    alu_out[31:0] = shift_register[31:0] & operand_b;
                end 

                else if (inst == OR) begin
                    alu_out[31:0] = shift_register[31:0] | operand_b;
                end 

                else if (inst == SLT) begin
                    if ($signed(shift_register[31:0]) < $signed(operand_b)) begin
                        alu_out = 33'd1;
                    end else begin
                        alu_out = 33'd0;
                    end
                end

                else begin
                    alu_out[31:0] = $signed(shift_register[31:0]) >>> operand_b;
                end
            end

            S_MULTI_CYCLE_OP: begin
                if (inst == MUL) begin
                    div_control = 1'b0;
                    if (shift_register[0] == 1'b1) begin
                        alu_out = shift_register[63:32] + operand_b;
                    end else begin
                        alu_out = shift_register[63:32];
                    end
                end

                else begin
                    div_control = shift_register[63:32] >= operand_b;
                    if (div_control) begin
                        alu_out = shift_register[63:32] - operand_b;
                    end else begin
                        alu_out = shift_register[63:32];
                    end
                end
            end

            default: begin
                div_control = 1'b0;
                alu_out = 33'd0;
            end

        endcase
    end
    
    // Todo: output valid signal
    always @(*) begin
        case(state) 
        
            S_ONE_CYCLE_OP: begin
                done_nxt = 1'b1;
            end

            S_MULTI_CYCLE_OP: begin
                if (counter == 31) begin
                    done_nxt = 1'b1;
                end else begin
                    done_nxt = 1'b0;
                end
            end

            default: done_nxt = 1'b0;
        endcase
    end


    // shift_register 
    always @(*) begin
        shift_register_nxt = shift_register;
        case(state)

            S_IDLE: begin
                if (i_valid) begin
                    if (i_inst == DIV) begin
                        shift_register_nxt[32:1] = i_A;
                    end else begin
                        shift_register_nxt[31:0] = i_A;
                    end
                end else begin
                    shift_register_nxt = 64'd0;
                end
            end

            S_ONE_CYCLE_OP: begin
                shift_register_nxt = {{32{1'b0}}, alu_out[31:0]}; 
            end

            S_MULTI_CYCLE_OP: begin
                if (inst == MUL) begin
                    shift_register_nxt = {alu_out, shift_register[31:1]};
                end else begin
                    if (div_control) begin
                        shift_register_nxt[32:0] = {shift_register[31:0], 1'b1};
                    end else begin
                        shift_register_nxt[32:0] = {shift_register[31:0], 1'b0};
                    end

                    if (counter == 31) begin
                        shift_register_nxt[63:32] = alu_out;
                    end else begin
                        shift_register_nxt[63:33] = alu_out[30:0];
                    end
                end
            end

            default: begin
                shift_register_nxt = shift_register;
            end

        endcase
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 32'd0;
            operand_b   <= 32'd0;
            inst        <= 3'd0;
            done        <= 1'b0;
            counter     <= 5'd0;
            shift_register <= 64'd0;
        end else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
            done        <= done_nxt;
            counter     <= counter_nxt;
            shift_register <= shift_register_nxt;
        end
    end

endmodule