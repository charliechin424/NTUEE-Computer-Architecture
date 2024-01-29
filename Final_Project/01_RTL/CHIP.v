// `include "./CHIP.v"；
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//   
module CHIP #(                                                                                  //      1. assign o_DMEM_wdata = (DMEM_wen && DMEM_cen) ? Read_data2 : 32'bx;
    parameter BIT_W = 32                                                                        //         此寫法寫不進memory
)(                                                                                              //   
    // clock                                                                                    //   
        input               i_clk,                                                              //    
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration

    // State 
    parameter S_IDLE           = 2'd0;
    parameter S_WAIT_READ      = 2'd1;
    parameter S_WAIT_WRITE     = 2'd2;
    parameter S_MULTI_CYCLE_OP = 2'd3;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, PC_nxt;
        reg [1:0] state, state_nxt;
        reg [5:0] counter , counter_nxt;

        reg mul_valid;
        wire [BIT_W-1:0] PC_nxt_temp;

        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        reg mem_stall;

        // output buffer
        reg finish, finish_nxt;
        reg IMEM_cen, IMEM_cen_nxt;
        reg DMEM_wen, DMEM_wen_nxt;
        reg DMEM_cen, DMEM_cen_nxt;
        reg proc_finish, proc_finish_nxt;

        // Reg_file output wire 
        wire [BIT_W-1:0] Write_data, Read_data1, Read_data2;

        // Control output wire
        wire [2:0]  Branch;
        wire        Jump_JALR;
        wire        Jump_JAL;
        wire       	MemRead;
        wire [1:0]  MemtoReg;
        wire       	MemWrite;
        wire       	ALUSrc1;
        wire       	ALUSrc2;
        wire       	RegWrite;
        wire [2:0] 	ALUOp;

        // Imm_Gen output wire
        wire [31:0] imm;

        // ALU_control output wire
        wire [3:0] 	ALU_control;

        // MUX_ALUSrc output wire
        wire [31:0] ALU_data1;
        wire [31:0] ALU_data2;

        // ALU output wire
        wire [31:0] 	o_data;
        wire [1:0]      o_zero;


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr = o_IMEM_cen ? PC : 0;
    assign o_IMEM_cen = IMEM_cen;    
    assign o_DMEM_addr = DMEM_cen ? o_data : 0;                         
    assign o_DMEM_wdata = Read_data2;                                    
    assign o_DMEM_wen = DMEM_wen;
    assign o_DMEM_cen = DMEM_cen;
    assign o_finish = finish;
    assign o_proc_finish = proc_finish;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (RegWrite),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (Write_data),             
        .rdata1 (Read_data1),           
        .rdata2 (Read_data2)
    );
    
    Control control(
        .i_Opcode         (i_IMEM_data[6:0]),
        .i_funct3         (i_IMEM_data[14:12]),
        .o_Branch         (Branch),
        .o_Jump_JALR      (Jump_JALR),
        .o_Jump_JAL       (Jump_JAL),
        .o_MemRead        (MemRead),
        .o_MemtoReg       (MemtoReg),
        .o_MemWrite       (MemWrite),
        .o_ALUSrc1        (ALUSrc1),
        .o_ALUSrc2        (ALUSrc2),
        .o_RegWrite       (RegWrite),
        .o_ALUOp          (ALUOp)
    );
      
    Imm_Gen imm_gen(
        .i_inst (i_IMEM_data),
        .o_imm  (imm)
    );
    
    ALU_control alu_control(
        .i_ALUOp       (ALUOp),
        .i_funct3      (i_IMEM_data[14:12]),
        .i_funct7      (i_IMEM_data[31:25]),
        .o_ALU_control (ALU_control)
    );
    
    PC_Adder pc_adder(
        .i_one_hot_control 	({Branch, o_zero, Jump_JALR, Jump_JAL}),
        .i_PC              	(PC),
        .i_offset          	(imm),
        .rs1                (Read_data1),
        .o_PC_nxt_temp      (PC_nxt_temp)
    );

    MUX2 MUX2_ALUSrc1( //auipc or other_alu
        .i_ctrl 	(ALUSrc1),
        .i_0    	(Read_data1),
        .i_1    	(PC),
        .o_data 	(ALU_data1)
    );
    
    MUX2 MUX2_ALUSrc2(//imm or normal_alu
        .i_ctrl 	(ALUSrc2),
        .i_0    	(Read_data2),
        .i_1    	(imm),
        .o_data 	(ALU_data2)
    );
    
    MUX3 MUX3_MemtoReg( 
        .i_ctrl       	(MemtoReg),
        .i_MEM_data   	(i_DMEM_rdata),
        .i_ALU_result 	(o_data),
        .i_PC_plus_four (PC+4),
        .o_data       	(Write_data)
    );
    
    ALU u_ALU(
        .i_clk   	( i_clk    ),
        .i_rst_n 	( i_rst_n  ),
        .i_A     	( ALU_data1 ),
        .i_B     	( ALU_data2 ),
        .i_inst  	( ALU_control ),
        .i_valid    ( mul_valid ),
        .o_data  	( o_data   ),
        .o_zero  	( o_zero   )
    );
    
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(*) begin

        state_nxt = state;
        finish_nxt = finish;
        proc_finish_nxt = proc_finish;
        IMEM_cen_nxt = IMEM_cen;
        DMEM_cen_nxt = DMEM_cen;
        DMEM_wen_nxt = DMEM_wen;
        mul_valid = 0;
        PC_nxt = PC;

        case(state)
            S_IDLE: begin
                DMEM_wen_nxt = MemWrite;
                DMEM_cen_nxt = MemWrite || MemRead;
                PC_nxt = PC_nxt_temp;
                IMEM_cen_nxt = 0;
                
                if (i_IMEM_data[6:0] == 7'b0000011) begin
                    state_nxt = S_WAIT_READ;
                end else if (i_IMEM_data[6:0] == 7'b0100011) begin
                    state_nxt = S_WAIT_WRITE;
                end else if (ALU_control == 7) begin
                    state_nxt = S_MULTI_CYCLE_OP;
                    mul_valid = 1;
                end else if (i_IMEM_data[6:0] == 7'b1110011) begin
                    state_nxt = S_IDLE;
                end else begin
                    state_nxt = S_IDLE;
                    IMEM_cen_nxt = 1;
                end

                if (i_cache_finish && proc_finish) begin
                    finish_nxt = 1;
                end else begin
                    finish_nxt = 0;
                end

                if (i_IMEM_data[6:0] == 7'b1110011) begin
                    proc_finish_nxt = 1;
                end else begin
                    proc_finish_nxt = 0;
                end
            end

            S_WAIT_READ:begin
                DMEM_cen_nxt = 0;
                if ((!i_DMEM_stall && mem_stall)) begin //valid data
                    state_nxt = S_IDLE;
                    IMEM_cen_nxt = 1;
                end else begin //stall
                    state_nxt = state;
                    IMEM_cen_nxt = 0;
                end
            end

            S_WAIT_WRITE:begin
                DMEM_cen_nxt = 0;
                DMEM_wen_nxt = 0;
                if (!i_DMEM_stall && mem_stall) begin //valid data
                    state_nxt = S_IDLE;
                    IMEM_cen_nxt = 1;
                end else begin //stall
                    state_nxt = state;
                    IMEM_cen_nxt = 0;
                end
            end

            S_MULTI_CYCLE_OP: begin
                mul_valid = 0;
                if (counter == 32) begin //mul over
					state_nxt = S_IDLE;
                    IMEM_cen_nxt = 1;
				end else begin
                    state_nxt = state;
                    IMEM_cen_nxt = 0;
                end
            end
        endcase
    end

    // counter
	always @(*) begin
        if (state == S_MULTI_CYCLE_OP) begin
            counter_nxt = counter + 1'b1;
        end else begin
            counter_nxt = 5'b0;
        end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= S_IDLE;
            counter <= 0;
            finish <= 0;
            IMEM_cen <= 1;
            DMEM_wen <= 0;
            DMEM_cen <= 0;
            mem_stall <= 0;
            proc_finish <= 0;
        end else begin
            PC <= PC_nxt;
            state <= state_nxt;
            counter <= counter_nxt;
            finish <= finish_nxt;
            IMEM_cen <= IMEM_cen_nxt;
            DMEM_wen <= DMEM_wen_nxt;
            DMEM_cen <= DMEM_cen_nxt;
            mem_stall <= i_DMEM_stall;
            proc_finish <= proc_finish_nxt;
        end
    end
endmodule

module Reg_file (i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;      // register length is 32 bit
    parameter word_depth = 32;// total number of register is 32
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;    
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];      // current data in register 
    reg [BITS-1:0] mem_nxt [0:word_depth-1];  // next data in register 

    integer i;

    assign rdata1 = mem[rs1];   // data in rs1 register 
    assign rdata2 = mem[rs2];   // data in rs2 register 

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)  // refresh to next data in register
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i]; 
    end  

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;   // X0 stores constant 0
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;  // X2 stores stack pointer
                    32'd3: mem[i] <= 32'h10008000;  // X3 stores global pointer
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit(i_clk, i_rst_n, i_valid, i_A, i_B, o_data, o_ready);
    input         i_clk, i_rst_n, i_valid;
    input  [31:0] i_A, i_B;
    output [31:0] o_data;
    output        o_ready;

    parameter S_IDLE = 1'b0;
    parameter S_PROC = 1'b1;

    reg [1:0]  state, state_nxt;
    reg [4:0]  counter , counter_nxt;
    reg [32:0] alu_out;

    // Output buffer
    reg        ready, ready_nxt;
    reg [63:0] shift_register, shift_register_nxt;

    // Input buffer
    reg [31:0] operand_a, operand_a_nxt;
    reg [31:0] operand_b, operand_b_nxt;

    // Output assignment
    assign o_ready = ready;
    assign o_data = ready ? shift_register[31:0] : 0;

    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
        end
    end

    // Combinational always block
    always @(*) begin
        case(state)
            S_IDLE: begin
                if (i_valid) begin
                    state_nxt = S_PROC;
                end else begin
                    state_nxt = S_IDLE;
                end
            end

            S_PROC: begin
                if (counter == 5'd31) begin
                    state_nxt = S_IDLE;
                end else begin
                    state_nxt = state;
                end
            end

            default : state_nxt = state;
        endcase
    end

    // Counter
    always @(*) begin
        if (state == S_PROC) begin
            counter_nxt = counter + 1'b1;
        end else begin
            counter_nxt = 5'b0;
        end
    end

    // ALU output
    always @(*) begin
        if (shift_register[0]) begin
            alu_out = shift_register[63:32] + operand_b;
        end else begin
            alu_out = shift_register[63:32];
        end
    end

    // output ready signal
    always @(*) begin
        if (state == S_PROC) begin
            if (counter == 31) begin
                ready_nxt = 1'b1;
            end else begin
                ready_nxt = 1'b0;
            end
        end else begin
            ready_nxt = 1'b0;
        end
    end

    // shift_register 
    always @(*) begin
        shift_register_nxt = shift_register;
        case(state)
            S_IDLE: begin
                if (i_valid) begin
                    shift_register_nxt = {32'd0, i_A};
                end else begin
                    shift_register_nxt = 64'd0;
                end
            end

            S_PROC: begin
                shift_register_nxt = {alu_out, shift_register[31:1]};
            end

            default: begin
                shift_register_nxt = shift_register;
            end
        endcase
    end

    // Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 32'd0;
            operand_b   <= 32'd0;
            ready       <= 1'b0;
            counter     <= 5'd0;
            shift_register <= 64'd0;
        end else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            ready        <= ready_nxt;
            counter     <= counter_nxt;
            shift_register <= shift_register_nxt;
        end
    end
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 1; // change this value to 1 if the cache is implemented

    // Todo: BONUS

    parameter index_w = 4;              
    parameter tag_w = (32-4-index_w);

    // cache memory 
    reg valid_memory [0:(1 << index_w)-1];
    reg dirty_memory [0:(1 << index_w)-1];
    reg [tag_w-1:0] tag_memory [0:(1 << index_w)-1];
    reg [127:0] data_memory [0:(1 << index_w)-1];
    reg valid_memory_nxt [0:(1 << index_w)-1];
    reg dirty_memory_nxt [0:(1 << index_w)-1];
    reg [tag_w-1:0] tag_memory_nxt [0:(1 << index_w)-1];
    reg [127:0] data_memory_nxt [0:(1 << index_w)-1];

    // address content which is sended from CPU
    wire [3:0] byte_offset;
    wire [index_w-1:0] index;
    wire [tag_w-1:0] tag;
    reg [31:0] proc_addr, proc_addr_prev;
    reg proc_cen, proc_cen_nxt;
    reg proc_wen, proc_wen_nxt;
    reg [ADDR_W-1:0] out_mem_addr;

    reg mem_stall;

    integer i;

    assign byte_offset = proc_addr[3:0];           
    assign index = proc_addr[4+index_w-1:4];       
    assign tag = proc_addr[31:4+index_w];         

    reg [2:0] state, state_nxt;
    reg mem_cen, mem_cen_nxt;
    reg mem_wen, mem_wen_nxt;
    reg proc_rdata_prepare, proc_rdata_prepare_nxt;
    reg proc_wdata_prepare, proc_wdata_prepare_nxt;
    reg cache_finish, cache_finish_nxt;
    reg proc_stall;
    reg [31:0] final_write_back_addr, final_write_back_addr_nxt;
    reg [127:0] final_write_back_data, final_write_back_data_nxt;
    reg [index_w-1:0] final_index, final_index_nxt;
    reg cache_function, cache_function_nxt;  // 1: write  0: read

    wire hit;
    wire read_hit;
    wire read_miss;
    wire write_hit;
    wire write_miss;

    assign hit = (tag_memory[index] == tag && valid_memory[index]) ? 1'b1 : 1'b0;
    assign read_hit = i_proc_cen ? (hit & i_proc_cen & !i_proc_wen) : read_hit;
    assign read_miss = i_proc_cen ? (!hit & i_proc_cen & !i_proc_wen) : read_miss;
    assign write_hit = i_proc_cen ? (hit & i_proc_cen & i_proc_wen) : write_hit;
    assign write_miss = i_proc_cen ? (!hit & i_proc_cen & i_proc_wen) : write_miss;

    assign o_mem_cen = o_cache_available ? mem_cen : i_proc_cen;    
    assign o_mem_wen = o_cache_available ? mem_wen : i_proc_wen;
    assign o_mem_wdata = o_cache_available ? (i_proc_finish ? final_write_back_data : data_memory[index]) : i_proc_wdata;
    assign o_proc_rdata = o_cache_available ? ((hit && !i_mem_stall && proc_rdata_prepare) ? data_memory[index][((byte_offset >> 2))*32 +: 32] : 32'b0) : i_mem_rdata[0+:BIT_W];
    assign o_proc_stall = o_cache_available ? proc_stall : i_mem_stall;
    assign o_cache_finish = o_cache_available ? cache_finish : 1'b1;
    assign o_mem_addr = o_cache_available ? out_mem_addr : i_proc_addr;

    parameter S_IDLE = 0;
    parameter S_READ_MISS = 1;
    parameter S_WRITE_BACK = 2;
    parameter S_WRITE_ALLOCATE = 3;
    parameter S_WRITE_HIT = 4;

    always @(*) begin
        out_mem_addr = 32'b0;
        if (mem_cen && mem_wen) begin
            if (i_proc_finish) begin
                out_mem_addr = final_write_back_addr;
            end else begin
                out_mem_addr = {tag_memory[index], index, i_offset[3:0]};
            end
        end else if (mem_cen) begin
            if (state == S_READ_MISS) begin
                out_mem_addr = {tag, index, i_offset[3:0]};
            end else if (state == S_WRITE_ALLOCATE) begin
                out_mem_addr = {tag, index, i_offset[3:0]};
            end
        end else begin
            out_mem_addr = 32'b0;
        end
    end

    always @(*) begin
        proc_addr = proc_addr_prev;
        if (i_proc_cen && i_proc_wen) begin
            if (i_proc_addr[31:28] == 4'hb) begin
                proc_addr = i_proc_addr;
            end else begin
                proc_addr = (i_proc_addr - i_offset[3:0]);
            end
        end 
        else if (i_proc_cen) begin
            if (i_proc_addr[31:28] == 4'hb) begin
                proc_addr = i_proc_addr;
            end else begin
                proc_addr = (i_proc_addr - i_offset[3:0]);
            end
        end 
        else begin
            proc_addr = proc_addr_prev;
        end
    end

    always @(*) begin
        state_nxt = state;
        mem_cen_nxt = mem_cen;
        mem_wen_nxt = mem_wen;
        cache_function_nxt = i_proc_cen ? (i_proc_wen ? 1'b1 : 1'b0) : cache_function;
        cache_finish_nxt = cache_finish;
        for (i=0 ; i < (1 << index_w) ; i=i+1) begin
            valid_memory_nxt[i] = valid_memory[i];
            dirty_memory_nxt[i] = dirty_memory[i];
            tag_memory_nxt[i] = tag_memory[i];
            data_memory_nxt[i] = data_memory[i];
        end

        final_index_nxt = final_index;
        final_write_back_addr_nxt = final_write_back_addr;
        final_write_back_data_nxt = final_write_back_data;

        if (!i_rst_n) begin
            for (i=0 ; i < (1 << index_w) ; i=i+1) begin
                valid_memory_nxt[i] = 0;
                dirty_memory_nxt[i] = 0;
                tag_memory_nxt[i] = 0;
                data_memory_nxt[i] = 0;
                /*final_index = 0;
                final_write_back_addr = 0;
                final_write_back_data = 0;*/
            end
            final_index_nxt = 0;
            final_write_back_addr_nxt = 0;
            final_write_back_data_nxt = 0;
        end

        case(state)

            S_IDLE: begin
                if (i_proc_cen) begin
                    if (read_miss) begin
                        mem_cen_nxt = 1;
                        if (dirty_memory[index]) begin
                            mem_wen_nxt = 1;
                            state_nxt = S_WRITE_BACK;
                        end else begin
                            state_nxt = S_READ_MISS;
                        end
                    end else if (write_hit) begin
                        state_nxt = S_WRITE_HIT;
                    end else if (write_miss) begin
                        mem_cen_nxt = 1;
                        if (dirty_memory[index]) begin
                            mem_wen_nxt = 1;
                            state_nxt = S_WRITE_BACK;
                        end else begin
                            state_nxt = S_WRITE_ALLOCATE;
                        end
                    end else if (read_hit) begin
                        state_nxt = S_IDLE;
                    end
                end else if (i_proc_finish) begin
                    
                    for (i=0 ; i < (1 << index_w) ; i=i+1) begin
                        if (dirty_memory[i]) begin
                            mem_wen_nxt = 1;
                            mem_cen_nxt = 1;
                            final_index_nxt = i;
                            final_write_back_addr_nxt = {tag_memory[i][tag_w-1:0], i[index_w-1:0], i_offset[3:0]};
                            final_write_back_data_nxt = data_memory[i];
                            state_nxt = S_WRITE_BACK;
                        end
                    end
                    
                    if (!mem_wen_nxt) begin
                        cache_finish_nxt = 1;
                    end else begin
                        cache_finish_nxt = 0;
                    end
                end else begin
                    state_nxt = S_IDLE;
                end
            end

            S_READ_MISS: begin
                mem_cen_nxt = 0;
                if (!i_mem_stall && mem_stall) begin
                    valid_memory_nxt[index] = 1;
                    dirty_memory_nxt[index] = 0;
                    tag_memory_nxt[index] = tag;
                    data_memory_nxt[index] = i_mem_rdata;
                    state_nxt = S_IDLE;
                end else begin
                    state_nxt = S_READ_MISS;
                end
            end

            S_WRITE_ALLOCATE: begin
                mem_cen_nxt = 0;
                if (!i_mem_stall && mem_stall) begin
                    valid_memory_nxt[index] = 1;
                    dirty_memory_nxt[index] = 1;
                    tag_memory_nxt[index] = tag;
                    data_memory_nxt[index] = i_mem_rdata;
                    data_memory_nxt[index][((byte_offset >> 2))*32 +: 32] = i_proc_wdata;
                    state_nxt = S_IDLE;
                end else begin
                    state_nxt = S_WRITE_ALLOCATE;
                end
            end

            S_WRITE_BACK: begin
                mem_cen_nxt = 0;
                mem_wen_nxt = 0;
                if (!i_mem_stall && mem_stall) begin
                    if(i_proc_finish)begin
                        dirty_memory_nxt[final_index] = 0;
                    end
                    else begin
                        dirty_memory_nxt[index] = 0;
                    end
                    if (read_miss && !i_proc_finish) begin
                        mem_cen_nxt = 1;
                        state_nxt = S_READ_MISS;
                    end else if (write_miss && !i_proc_finish) begin
                        mem_cen_nxt = 1;
                        state_nxt = S_WRITE_ALLOCATE;
                    end else begin
                        state_nxt = S_IDLE;
                    end
                end else begin
                    state_nxt = S_WRITE_BACK;
                end
            end

            S_WRITE_HIT: begin
                data_memory_nxt[index][((byte_offset >> 2))*32 +: 32] = i_proc_wdata;
                tag_memory_nxt[index] = tag;
                dirty_memory_nxt[index] = 1;
                state_nxt = S_IDLE;
            end
        endcase
    end

    always @(*) begin
        if (hit && !i_mem_stall && proc_rdata_prepare) begin
            proc_rdata_prepare_nxt = 0;
        end else if (i_proc_cen)begin
            proc_rdata_prepare_nxt = 1;
        end else begin
            proc_rdata_prepare_nxt = proc_rdata_prepare;
        end
    end

    always @(*) begin
        if (hit && !i_mem_stall && proc_wdata_prepare) begin
            proc_wdata_prepare_nxt = 0;
        end else if (i_proc_cen)begin
            proc_wdata_prepare_nxt = 1;
        end else begin
            proc_wdata_prepare_nxt = proc_wdata_prepare;
        end
    end

    always @(*) begin
        if (!i_rst_n) begin
            proc_stall = 0;
        end 
        else if (i_proc_cen && !proc_stall) begin
            proc_stall = 1;
        end 
        else if (proc_stall) begin
            if (!cache_function) begin
                if (hit && !i_mem_stall && proc_rdata_prepare) begin
                    proc_stall = 0;
                end else begin
                    proc_stall = 1;
                end
            end else begin
                if (hit && !i_mem_stall && proc_wdata_prepare) begin
                    proc_stall = 0;
                end else begin
                    proc_stall = 1;
                end
            end
        end
        else 
            proc_stall = 0;
    end

    always@(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state   <= S_IDLE;
            mem_cen <= 1'b0;
            mem_wen <= 1'b0;
            proc_rdata_prepare <= 0;
            proc_wdata_prepare <= 0;
            cache_finish <= 0;
            cache_function <= 0;
            mem_stall <= 0;
            proc_addr_prev <= 0;
            final_index <= 0;
            final_write_back_addr <= 0;
            final_write_back_data <= 0;
            for (i=0 ; i < (1 << index_w) ; i=i+1) begin
                valid_memory[i] <= 0;
                dirty_memory[i] <= 0;
                tag_memory[i] <= 0;
                data_memory[i] <= 0;
            end
        end else begin
            state   <= state_nxt;
            mem_cen <= mem_cen_nxt;
            mem_wen <= mem_wen_nxt;
            proc_rdata_prepare <= proc_rdata_prepare_nxt;
            proc_wdata_prepare <= proc_wdata_prepare_nxt;
            cache_finish <= cache_finish_nxt;
            cache_function <= cache_function_nxt;
            mem_stall <= i_mem_stall;
            proc_addr_prev <= proc_addr;
            final_index <= final_index_nxt;
            final_write_back_addr <= final_write_back_addr_nxt;
            final_write_back_data <= final_write_back_data_nxt;
            for (i=0 ; i < (1 << index_w) ; i=i+1) begin
                valid_memory[i] <= valid_memory_nxt[i];
                dirty_memory[i] <= dirty_memory_nxt[i];
                tag_memory[i] <= tag_memory_nxt[i];
                data_memory[i] <= data_memory_nxt[i];
            end
        end
    end
endmodule

module Control (i_Opcode, i_funct3, o_Branch, o_MemRead, o_MemtoReg, o_ALUOp, o_MemWrite, o_ALUSrc1, o_ALUSrc2, o_RegWrite, o_Jump_JALR, o_Jump_JAL);
    input [6:0] i_Opcode;
    input [2:0] i_funct3;
    output o_MemRead, o_MemWrite, o_ALUSrc1, o_ALUSrc2, o_RegWrite, o_Jump_JALR, o_Jump_JAL;
    output [2:0] o_Branch;
    output [1:0] o_MemtoReg; 
    output reg [2:0] o_ALUOp;

    assign o_MemRead = (i_Opcode == 7'b0000011) ? 1'b1 : 1'b0;//lw
    assign o_MemtoReg[0] = (i_Opcode == 7'b0000011) ? 1'b1 : 1'b0;//lw
    assign o_MemtoReg[1] = (i_Opcode == 7'b1101111 || i_Opcode == 7'b1100111) ? 1'b1 : 1'b0;//jal || jalr
    assign o_MemWrite = (i_Opcode == 7'b0100011) ? 1'b1 : 1'b0;//sw
    assign o_ALUSrc1 = (i_Opcode == 7'b0010111) ? 1'b1 : 1'b0; //auipc
    assign o_ALUSrc2 = (i_Opcode == 7'b0000011 || i_Opcode == 7'b0010011 || i_Opcode == 7'b1100111 || i_Opcode == 7'b0010111 || i_Opcode == 7'b1101111 || i_Opcode == 7'b0100011) ? 1'b1 : 1'b0; 
    // lw || Itype || jalr ||auipc || jal || sw
    assign o_RegWrite = (i_Opcode != 7'b0100011 && i_Opcode != 7'b1100011) ? 1'b1 : 1'b0;// !sw & !branch
    assign o_Branch = (i_Opcode == 7'b1100011) ? {1'b1, i_funct3[2],i_funct3[0]} : 3'd0;//branch
    assign o_Jump_JALR = (i_Opcode == 7'b1100111) ? 1'b1 : 1'b0;//jalr
    assign o_Jump_JAL = (i_Opcode == 7'b1101111) ? 1'b1 : 1'b0;//jal

    always @(*) begin
        case(i_Opcode)

            7'b0110011 : begin  // R-type : ADD, SUB, MUL, AND, XOR
                o_ALUOp = 3'b000;
            end

            7'b0010011 : begin  // I-type : ADDI, SLTI, SLLI, SRAI
                o_ALUOp = 3'b001;
            end 

            7'b1100111 : begin  // I-type : JALR
                o_ALUOp = 3'b010;
            end 

            7'b0000011 : begin  // I-type : LW
                o_ALUOp = 3'b010;
            end 

            7'b0100011 : begin  // S-type : SW
                o_ALUOp = 3'b010;
            end

            7'b1100011 : begin  // SB-type : BEQ, BNE, BLT, BGE
                o_ALUOp = 3'b011;
            end

            7'b0010111 : begin  // U-type : AUIPC 
                o_ALUOp = 3'b100;
            end

            7'b1101111 : begin  // UJ-type : JAL  
                o_ALUOp = 3'b100;
            end

            default : begin   
                o_ALUOp = 3'b111;
            end

        endcase
    end
endmodule

module ALU_control (i_ALUOp, i_funct3, i_funct7, o_ALU_control);
    input [2:0] i_ALUOp;
    input [2:0] i_funct3;
    input [6:0] i_funct7;
    output reg [3:0] o_ALU_control;

    always @(*) begin
        case(i_ALUOp)

            3'b000 : begin   // R-type 
                case(i_funct3)
                    3'b000 : begin// +-
                        case(i_funct7)
                            7'b0000000 : begin
                                o_ALU_control = 4'd0; // ALU: add
                            end

                            7'b0100000 : begin
                                o_ALU_control = 4'd1; // ALU: subtract
                            end

                            7'b0000001 : begin
                                o_ALU_control = 4'd7; // ALU: mul
                            end

                            default : begin
                                o_ALU_control = 4'b1111;
                            end
                        endcase
                    end

                    3'b111 : begin
                        o_ALU_control = 4'd2;  // ALU: AND
                    end

                    3'b110 : begin
                        o_ALU_control = 4'd3;  // ALU: XOR
                    end  

                    default : begin
                        o_ALU_control = 4'b1111;
                    end
                endcase
            end

            3'b001 : begin    // I-type
                case(i_funct3) 
                    3'b000 : begin  // ADDI
                        o_ALU_control = 4'd0;  // ALU: add 
                    end

                    3'b010 : begin  // SLTI
                        o_ALU_control = 4'd4;  // ALU: slt
                    end

                    3'b001 : begin  // SLLI
                        o_ALU_control = 4'd6;  // ALU: sll
                    end

                    3'b101 : begin  // SRAI
                        o_ALU_control = 4'd5;  // ALU: sra
                    end

                    default : begin
                        o_ALU_control = 4'b1111;
                    end
                endcase
            end

            3'b010 : begin   // I-type: LW, JALR, ECALL  S-type: SW -> ALU: add
                o_ALU_control = 4'd0;
            end

            3'b011 : begin   // SB-type -> ALU: subtract 
                o_ALU_control = 4'd1;
            end

            3'b100 : begin   // U-type: AUIPC  UJ-type: JAL -> ALU: add
                o_ALU_control = 4'd0;
            end

            default : begin
                o_ALU_control = 4'b1111;
            end

        endcase
    end
endmodule

module MUX2 (i_ctrl, i_0, i_1, o_data);
    input i_ctrl;
    input [31:0] i_0, i_1;
    output [31:0] o_data;

    assign o_data = i_ctrl ? i_1 : i_0;
endmodule

module MUX3 (i_ctrl, i_MEM_data, i_ALU_result, i_PC_plus_four, o_data);
    input [1:0] i_ctrl;//memtoreg
    input [31:0] i_MEM_data, i_ALU_result, i_PC_plus_four;
    output reg [31:0] o_data;//writedata

    always @(*) begin
        case(i_ctrl)  // i_ctrl[0]: LW  i_ctrl[1]: JAL JALR
            2'b00: begin//not lw or jal or jalr
                o_data = i_ALU_result;//alu output
            end

            2'b01: begin//lw
                o_data = i_MEM_data;
            end

            2'b10: begin//jal or jalr
                o_data = i_PC_plus_four;
            end

            default : begin
                o_data = i_ALU_result;
            end
        endcase
    end
endmodule

module Imm_Gen (i_inst, o_imm);
    input [31:0] i_inst;
    output reg [31:0] o_imm;

    always @(*) begin
        case(i_inst[6:0])
            7'b0100011 : begin  // S-type sw
                o_imm = {{20{i_inst[31]}}, i_inst[31:25], i_inst[11:7]};
            end

            7'b0010111 : begin  // U-type auipc
                o_imm = {i_inst[31:12], {12{1'b0}}};
            end

            7'b1101111 : begin  // UJ-type jal
                o_imm = {{11{i_inst[31]}},i_inst[31], i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0};
            end

            7'b1100011 : begin  // SB-type branch
                o_imm = {{19{i_inst[31]}}, i_inst[31], i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0};
            end

            7'b0000011 : begin  // I-type lw
                o_imm = {{20{i_inst[31]}}, i_inst[31:20]};
            end

            7'b0010011 : begin  // I-type 
                if(!i_inst[14] && !i_inst[12])
                    o_imm = {{20{i_inst[31]}}, i_inst[31:20]};  // ADDI, SLTI
                else
                    o_imm = {{27'd0}, i_inst[24:20]};           // SLLI, SRAI
            end

            7'b1100111 : begin  // I-type jalr
                o_imm = {{20{i_inst[31]}}, i_inst[31:20]};
            end

            default : begin     // R-type
                o_imm = 32'b0;
            end
        endcase
    end
endmodule

module ALU (i_clk, i_rst_n, i_A, i_B, i_inst, i_valid, o_data, o_zero);
    input           i_clk;   // clock
    input           i_rst_n; // reset

    input  [31:0]   i_A;     // input operand A
    input  [31:0]   i_B;     // input operand B
    input  [3:0]    i_inst;  // instruction
    input           i_valid;

    output [31:0]   o_data;  // output value
    output reg[1:0] o_zero;  // 0: zero 1: pos 2: neg
    
    
    wire [31:0] 	o_data_multiple;
    wire        	o_ready;

    reg [31:0] 	    o_data_single;
    
    MULDIV_unit u_MULDIV_unit(
        .i_clk   	( i_clk    ),
        .i_rst_n 	( i_rst_n  ),
        .i_valid 	( i_valid  ),
        .i_A     	( i_A      ),
        .i_B     	( i_B      ),
        .o_data  	( o_data_multiple ),
        .o_ready 	( o_ready  )
    );
    
    //operation modes
    parameter ADD  = 4'd0;
    parameter SUB  = 4'd1;
    parameter AND  = 4'd2;
    parameter XOR  = 4'd3;
    parameter SLT  = 4'd4;
    parameter SRA  = 4'd5;
    parameter SLL  = 4'd6;
    parameter MUL  = 4'd7;

    assign o_data = o_ready ? o_data_multiple : o_data_single;

    always@(*)begin
        o_zero = 2'd1;
        if(o_data_single == 32'd0) begin //a == b
            o_zero = 2'd0;
        end else if(o_data_single[31] == 1'd1)begin //a < b
            o_zero = 2'd2;
        end else begin //a > b
            o_zero = 2'd1;
        end
    end

    always @(*) begin
        case(i_inst)
            ADD: begin
                o_data_single = $signed(i_A) + $signed(i_B);
            end

            SUB: begin
                o_data_single = $signed(i_A) - $signed(i_B);
            end

            AND: begin
                o_data_single = i_A & i_B;
            end

            XOR: begin
                o_data_single = i_A ^ i_B;
            end

            SLT: begin
                o_data_single = ($signed(i_A) < $signed(i_B)) ? 32'b1 : 32'b0;
            end

            SRA: begin
                o_data_single = $signed(i_A) >>> i_B;
            end

            SLL: begin
                o_data_single = i_A << i_B;
            end

            default: begin
                o_data_single = i_A;
            end
        endcase
    end
endmodule

module PC_Adder (i_one_hot_control, i_PC, i_offset, rs1, o_PC_nxt_temp);
    input [6:0] i_one_hot_control;  //{Branch, o_zero, Jump_JALR, Jump_JAL}
    input [31:0] i_PC, i_offset, rs1;
    output reg [31:0] o_PC_nxt_temp;

    wire jal, jalr;
    wire [1:0] zero;
    wire branch;

    assign jal = i_one_hot_control[0];
    assign jalr = i_one_hot_control[1];
    assign zero = i_one_hot_control[3:2];
    assign branch = i_one_hot_control[6];

    always@(*)begin
        if (jal) begin
            o_PC_nxt_temp = i_PC + i_offset;
        end else if (jalr) begin
            o_PC_nxt_temp = rs1 + i_offset;
        end else if (branch) begin

            case(i_one_hot_control[5:4])
                2'd0: o_PC_nxt_temp = (zero == 2'd0) ? i_PC + i_offset : i_PC + 32'd4; // beq & a==b
                2'd1: o_PC_nxt_temp = (zero != 2'd0) ? i_PC + i_offset : i_PC + 32'd4; // bne & a!=b
                2'd2: o_PC_nxt_temp = (zero == 2'd2) ? i_PC + i_offset : i_PC + 32'd4; // blt & a<b
                2'd3: o_PC_nxt_temp = (zero != 2'd2) ? i_PC + i_offset : i_PC + 32'd4; // bge & !(a<b)
                default: o_PC_nxt_temp = i_PC + 32'd4;
            endcase

        end else begin
            o_PC_nxt_temp = i_PC + 32'd4;
        end
    end
endmodule