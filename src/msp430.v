// MSP430 FPGA Soft Processor
//  Author: Michael Kohn
//   Email: mike@mikekohn.net
//     Web: https://www.mikekohn.net/
//   Board: iceFUN iCE40 HX8K
// License: MIT
//
// Copyright 2024 by Michael Kohn

module msp430
(
  output [7:0] leds,
  output [3:0] column,
  input raw_clk,
  output eeprom_cs,
  output eeprom_clk,
  output eeprom_di,
  input  eeprom_do,
  output speaker_p,
  output speaker_m,
  output ioport_0,
  output ioport_1,
  output ioport_2,
  output ioport_3,
  input  button_reset,
  input  button_halt,
  input  button_program_select,
  input  button_0,
  output spi_clk,
  output spi_mosi,
  input  spi_miso
);

// iceFUN 8x4 LEDs used for debugging.
reg [7:0] leds_value;
reg [3:0] column_value;

assign leds = leds_value;
assign column = column_value;

// Memory bus (ROM, RAM, peripherals).
reg [15:0] mem_address = 0;
reg [15:0] mem_write = 0;
reg [1:0] mem_write_mask = 0;
wire [15:0] mem_read;
//wire mem_data_ready;
reg mem_bus_enable = 0;
reg mem_write_enable = 0;

//wire [7:0] mem_debug;

// Clock.
reg [21:0] count = 0;
reg [5:0] state = 0;
//reg [5:0] next_state = 0;
reg [19:0] clock_div;
reg [14:0] delay_loop;
wire clk;
assign clk = clock_div[1];

// Registers.
parameter PC = 0;
parameter SP = 1;
parameter SR = 2;
parameter CG = 3;

reg [15:0] registers [15:0];
wire [15:0] pc;
wire [15:0] sp;
wire [15:0] sr;

assign pc = { registers[PC][15:1], 1'b0 };
assign sp = registers[SP];
assign sr = registers[SR];

// Flags.
parameter FLAG_C = 0;
parameter FLAG_N = 1;
parameter FLAG_Z = 2;
parameter FLAG_V = 8;

wire flag_c;
wire flag_n;
wire flag_z;
wire flag_v;

assign flag_c = sr[FLAG_C];
assign flag_n = sr[FLAG_N];
assign flag_z = sr[FLAG_Z];
assign flag_v = sr[FLAG_V];

// Instruction
reg [15:0] instruction;

wire [1:0] as;
wire ad;
//wire [1:0] writeback_mode;

reg [2:0] mode;

assign as = instruction[5:4];
assign ad = instruction[7];

wire [3:0] rs;
wire [3:0] rd;

assign rd = instruction[3:0];
assign rs = instruction[11:8];

wire signed [10:0] branch_offset;
assign branch_offset = { instruction[9:0], 1'b0 };
wire signed [15:0] branch_address;
assign branch_address = $signed(pc) + branch_offset;

wire [2:0] alu_op_1;
wire [3:0] alu_op_2;
wire [3:0] jmp_op;
assign alu_op_1 = instruction[9:7];
assign alu_op_2 = instruction[15:12];
assign jmp_op = instruction[12:10];

wire is_single_op;
wire is_jmp;;
assign is_single_op = instruction[15:13] == 3'b000;
assign is_jmp       = instruction[15:13] == 3'b001;

wire [4:0] execute_state;
assign execute_state = is_single_op ? STATE_ALU_SINGLE : STATE_ALU_TWO_0;

wire bw;
assign bw = instruction[6];

reg [3:0] reg_src;
//reg [3:0] reg_dst;

reg [15:0] source;
reg [15:0] temp;
reg [16:0] result;

// Load / Store.
//assign memory_size = instruction[14:12];
reg [15:0] ea;

// Lower 6 its of the instruction.
wire [5:0] opcode;
assign opcode = instruction[5:0];

// Eeprom.
reg [10:0] eeprom_count;
wire [7:0] eeprom_data_out;
reg  [7:0] eeprom_holding [3:0];
reg [10:0] eeprom_address;
reg [15:0] eeprom_mem_address;
reg eeprom_strobe = 0;
wire eeprom_ready;
reg reti_state;

// Debug.
//reg [7:0] debug_0 = 0;
//reg [7:0] debug_1 = 0;
//reg [7:0] debug_2 = 0;
//reg [7:0] debug_3 = 0;

parameter STATE_RESET         = 0;
parameter STATE_DELAY_LOOP    = 1;
parameter STATE_FETCH_RESET_0 = 2;
parameter STATE_FETCH_RESET_1 = 3;
parameter STATE_FETCH_OP_0    = 4;
parameter STATE_FETCH_OP_1    = 5;
parameter STATE_START_DECODE  = 6;
parameter STATE_DECODE_MODE_0 = 7;

parameter STATE_FETCH_EA_0    = 8;
parameter STATE_FETCH_EA_1    = 9;
parameter STATE_FETCH_DATA_0  = 10;
parameter STATE_FETCH_DATA_1  = 11;

parameter STATE_ALU_SINGLE    = 12;
parameter STATE_WB_SINGLE_0   = 13;

parameter STATE_ALU_TWO_0     = 14;
parameter STATE_ALU_TWO_1     = 15;
parameter STATE_ALU_TWO_1_EA_0 = 16;
parameter STATE_ALU_TWO_1_EA_1 = 17;
parameter STATE_ALU_TWO_2     = 18;
parameter STATE_WB_TWO_0      = 19;
//parameter STATE_WB_TWO_1      = 20;

parameter STATE_PUSH_0        = 20;
parameter STATE_PUSH_1        = 21;
parameter STATE_RETI_0        = 22;
parameter STATE_RETI_1        = 23;

parameter STATE_DATA_STORE_0  = 24;
parameter STATE_DATA_STORE_1  = 25;

parameter STATE_EEPROM_START  = 57;
parameter STATE_EEPROM_READ   = 58;
parameter STATE_EEPROM_WAIT   = 59;
parameter STATE_EEPROM_WRITE  = 60;
parameter STATE_EEPROM_DONE   = 61;

parameter STATE_ERROR         = 62;
parameter STATE_HALTED        = 63;

parameter MODE_REG          = 0;
parameter MODE_ABSOLUTE     = 1;
parameter MODE_SYMBOLIC     = 2;
parameter MODE_REG_INDEXED  = 3;
parameter MODE_REG_INDIRECT = 4;
parameter MODE_IMMEDIATE    = 5;

parameter OP_RRC  = 3'b000;
parameter OP_SWPB = 3'b001;
parameter OP_RRA  = 3'b010;
parameter OP_SXT  = 3'b011;
parameter OP_PUSH = 3'b100;
parameter OP_CALL = 3'b101;
parameter OP_RETI = 3'b110;

parameter OP_JNZ = 3'b000;
parameter OP_JZ  = 3'b001;
parameter OP_JNC = 3'b010;
parameter OP_JC  = 3'b011;
parameter OP_JN  = 3'b100;
parameter OP_JGE = 3'b101;
parameter OP_JL  = 3'b110;
parameter OP_JMP = 3'b111;

parameter OP_MOV  = 4'b0100;
parameter OP_ADD  = 4'b0101;
parameter OP_ADDC = 4'b0110;
parameter OP_SUBC = 4'b0111;
parameter OP_SUB  = 4'b1000;
parameter OP_CMP  = 4'b1001;
parameter OP_DADD = 4'b1010;
parameter OP_BIT  = 4'b1011;
parameter OP_BIC  = 4'b1100;
parameter OP_BIS  = 4'b1101;
parameter OP_XOR  = 4'b1110;
parameter OP_AND  = 4'b1111;

// This block is simply a clock divider for the raw_clk.
always @(posedge raw_clk) begin
  count <= count + 1;
  clock_div <= clock_div + 1;
end

// Debug: This block simply drives the 8x4 LEDs.
always @(posedge raw_clk) begin
  case (count[9:7])
    3'b000: begin column_value <= 4'b0111; leds_value <= ~registers[4][7:0]; end
    3'b010: begin column_value <= 4'b1011; leds_value <= ~registers[4][15:8]; end
    //3'b000: begin column_value <= 4'b0111; leds_value <= ~registers[PC][7:0]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~registers[PC][15:8]; end
    //3'b000: begin column_value <= 4'b0111; leds_value <= ~result[7:0]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~result[15:8]; end
    //3'b000: begin column_value <= 4'b0111; leds_value <= ~ea[7:0]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~ea[15:8]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~pc[15:8]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~instruction[7:0]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~instruction[15:8]; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~alu_op_1; end
    //3'b010: begin column_value <= 4'b1011; leds_value <= ~instruction[9:7]; end
    3'b100: begin column_value <= 4'b1101; leds_value <= ~pc[7:0]; end
    3'b110: begin column_value <= 4'b1110; leds_value <= ~state; end
    default: begin column_value <= 4'b1111; leds_value <= 8'hff; end
  endcase
end

// This block is the main CPU instruction execute state machine.
always @(posedge clk) begin
  if (!button_reset)
    state <= STATE_RESET;
  else if (!button_halt)
    state <= STATE_HALTED;
  else
    case (state)
      STATE_RESET:
        begin
          mem_address <= 0;
          mem_write_enable <= 0;
          mem_write <= 0;
          delay_loop <= 12000;
          //eeprom_strobe <= 0;
          state <= STATE_DELAY_LOOP;
          reti_state <= 0;
        end
      STATE_DELAY_LOOP:
        begin
          // This is probably not needed. The chip starts up fine without it.
          if (delay_loop == 0) begin

            // If button is not pushed, start rom.v code otherwise use EEPROM.
            if (button_program_select) begin
              //registers[PC] <= 16'h4000;
              state <= STATE_FETCH_RESET_0;
            end else begin
              state <= STATE_EEPROM_START;
            end
          end else begin
            delay_loop <= delay_loop - 1;
          end
        end
      STATE_FETCH_RESET_0:
        begin
          mem_bus_enable <= 1;
          mem_write_enable <= 0;
          mem_address <= 16'hfffe;;
          state <= STATE_FETCH_RESET_1;
        end
      STATE_FETCH_RESET_1:
        begin
          mem_bus_enable <= 0;
          registers[PC] <= { mem_read[15:1], 1'b0 };
          state <= STATE_FETCH_OP_0;
        end
      STATE_FETCH_OP_0:
        begin
          mem_bus_enable <= 1;
          mem_address <= pc;
          registers[PC] <= registers[PC] + 2;
          state <= STATE_FETCH_OP_1;
        end
      STATE_FETCH_OP_1:
        begin
          mem_bus_enable <= 0;
          instruction <= mem_read;
          state <= STATE_START_DECODE;
        end
      STATE_START_DECODE:
        begin
          if (is_single_op) begin
            reg_src <= rd;

            if (alu_op_1 == OP_RETI)
              state <= STATE_RETI_0;
              //state <= STATE_ERROR;
            else
              state <= STATE_DECODE_MODE_0;
          end else if (is_jmp) begin
            case (jmp_op)

              OP_JNZ: if (!flag_z) registers[PC] <= branch_address;
              OP_JZ:  if (flag_z)  registers[PC] <= branch_address;
              OP_JNC: if (!flag_c) registers[PC] <= branch_address;
              OP_JC:  if (flag_c)  registers[PC] <= branch_address;
              OP_JN:  if (flag_n)  registers[PC] <= branch_address;
              OP_JGE: if (!(flag_n ^ flag_v)) registers[PC] <= branch_address;
              OP_JL:  if (  flag_n ^ flag_v)  registers[PC] <= branch_address;
              OP_JMP: registers[PC] <= branch_address;
            endcase

            state <= STATE_FETCH_OP_0;
          end else begin
            // Two operand arithmetic.
            reg_src <= rs;
            state <= STATE_DECODE_MODE_0;
          end
        end
      STATE_DECODE_MODE_0:
        begin
          if (reg_src == CG) begin
            case (as)
              0: source <= 0;
              1: source <= 1;
              2: source <= 2;
              3: source <= -1;
            endcase

            state <= execute_state;
          end else if (reg_src == SR) begin
            if (as == 2'b01) begin
              mode <= MODE_ABSOLUTE;
              state <= STATE_FETCH_EA_0;
            end else begin
              source <= as == 2 ? 4 : 8;
              state <= execute_state;
            end
          end else begin
            case (as)
              0:
                begin
                  mode <= MODE_REG;
                  source <= registers[reg_src];
                  state <= execute_state;
                end
              1:
                begin
                  mode <= MODE_REG_INDEXED;
                  state <= STATE_FETCH_EA_0;
                end
              2:
                begin
                  ea <= registers[reg_src];
                  state <= STATE_FETCH_DATA_0;
                end
              3:
                if (reg_src == PC) begin
                  mode <= MODE_IMMEDIATE;
                  state <= STATE_FETCH_EA_0;
                end else begin
                  ea <= registers[reg_src];

                  if (bw == 0)
                    registers[reg_src] <= registers[reg_src] + 2;
                  else
                    registers[reg_src] <= registers[reg_src] + 1;

                  state <= STATE_FETCH_DATA_0;
                end
            endcase
          end
        end
      STATE_FETCH_EA_0:
        begin
          mem_bus_enable <= 1;
          mem_address <= registers[PC];
          state <= STATE_FETCH_EA_1;
        end
      STATE_FETCH_EA_1:
        begin
          mem_bus_enable <= 0;
          registers[PC] <= registers[PC] + 2;

          case (mode)
            MODE_ABSOLUTE:
              ea <= mem_read;
            MODE_REG_INDEXED:
              ea <= $signed(registers[reg_src]) + $signed(mem_read);
            MODE_IMMEDIATE:
              if (bw)
                if (ea[0] == 0)
                  source <= mem_read[7:0];
                else
                  source <= mem_read[15:8];
              else
                source <= mem_read;
          endcase

          if (mode == MODE_IMMEDIATE)
            state <= execute_state;
          else
            state <= STATE_FETCH_DATA_0;
        end
      STATE_FETCH_DATA_0:
        begin
          mem_bus_enable <= 1;
          mem_address <= ea;
          state <= STATE_FETCH_DATA_1;
        end
      STATE_FETCH_DATA_1:
        begin
            mem_bus_enable <= 0;

            if (bw == 1) begin
              case (ea[0])
                0: source <= { 8'h00, mem_read[7:0]  };
                1: source <= { 8'h00, mem_read[15:8] };
              endcase
            end else begin
              source <= mem_read;
            end

            state <= execute_state;
        end
      STATE_ALU_SINGLE:
        begin
          case (alu_op_1)
            OP_RRC:
              begin
                if (bw == 0) begin
                  result[16] <= source[0];
                  result[15:0] <= { flag_c, source[15:1] };
                end else begin
                  result[16:9] <= 0;
                  result[8] <= source[0];
                  result[7:0] <= { flag_c, source[7:1] };
                end

                registers[SR][FLAG_C] <= source[0];
              end
            OP_SWPB:
              begin
                result[16] <= 0;
                result[15:8] <= source[7:0];
                result[7:0]  <= source[15:8];
              end
            OP_RRA:
              begin
                if (bw == 0) begin
                  result[16] <= source[0];
                  result[15:0] <= $signed(source) >> 1;
                end else begin
                  result[16:9] <= 0;
                  result[8] <= source[0];
                  result[7:0] <= $signed(source[7:0]) >> 1;
                end

                registers[SR][FLAG_C] <= source[0];
              end
            OP_SXT:
              begin
                // sxt.
                result <= $signed(source[7:0]);
              end
            OP_PUSH:
              begin
                // push.
                result <= source;
              end
            OP_CALL:
              begin
                // call
                registers[PC] <= { source[15:1], 1'b0 };
                result <= registers[PC];
              end
          endcase

          if (alu_op_1 == OP_PUSH || alu_op_1 == OP_CALL)
            state <= STATE_PUSH_0;
          else
            state <= STATE_WB_SINGLE_0;
        end
      STATE_WB_SINGLE_0:
        begin
          registers[SR][FLAG_V] <= 0;

          if (bw == 0) begin
            registers[SR][FLAG_N] <= result[15] == 1;
            registers[SR][FLAG_Z] <= result[15:0] == 0;
          end else begin
            registers[SR][FLAG_N] <= result[7] == 1;
            registers[SR][FLAG_Z] <= result[7:0] == 0;
          end

          if (as == 0) begin
            registers[rd] <= result;
            state <= STATE_FETCH_OP_0;
          end else begin
            state <= STATE_DATA_STORE_0;
          end
        end
      STATE_ALU_TWO_0:
        begin
          if (ad == 0) begin
            temp <= registers[rd];
            state <= STATE_ALU_TWO_2;
          end else begin
            mem_address <= pc;
            mem_bus_enable <= 1;
            state <= STATE_ALU_TWO_1;
          end
        end
      STATE_ALU_TWO_1:
        begin
          mem_bus_enable <= 0;
          registers[PC] <= registers[PC] + 2;

          if (rd == SR)
            ea <= mem_read;
          else
            ea <= $signed(registers[rd]) + $signed(mem_read);

          state <= STATE_ALU_TWO_1_EA_0;
        end
      STATE_ALU_TWO_1_EA_0:
        begin
          mem_address <= ea;
          mem_bus_enable <= 1;
          state <= STATE_ALU_TWO_1_EA_1;
        end
      STATE_ALU_TWO_1_EA_1:
        begin
          mem_bus_enable <= 0;
          temp <= mem_read;
          state <= STATE_ALU_TWO_2;
        end
      STATE_ALU_TWO_2:
        begin
/*
          if (ad == 1) begin
            mem_bus_enable <= 0;
            // FIXME: not in this cycle.
            // FIXME: shouldn't the bus already be disabled?
            temp <= mem_read;
          end
*/

          case (alu_op_2)
            OP_MOV:
              begin
                result <= source;
              end
            OP_ADD:
              begin
                result <= temp + source;
              end
            OP_ADDC:
              begin
                result <= temp + source + flag_c;
              end
            OP_SUBC:
              begin
                result <= { 1'b1, temp } - source - 1 + flag_c;
              end
            OP_SUB:
              begin
                result <= { 1'b1, temp } - source;
              end
            OP_CMP:
              begin
                result <= { 1'b1, temp } - source;
              end
            OP_DADD:
              begin
                result[3:0]   <= temp[3:0]   + source[3:0]   + flag_c;
                result[7:4]   <= temp[7:4]   + source[7:4]   + flag_c;
                result[11:8]  <= temp[11:8]  + source[11:8]  + flag_c;
                result[15:12] <= temp[15:12] + source[15:12] + flag_c;
              end
            OP_BIT:
              begin
                result <= temp & source;
              end
            OP_BIC:
              begin
                result <= temp & ~source;
              end
            OP_BIS:
              begin
                result <= temp | source;
              end
            OP_XOR:
              begin
                result <= temp ^ source;
              end
            OP_AND:
              begin
                result <= temp & source;
              end
          endcase

          state <= STATE_WB_TWO_0;
        end
      STATE_WB_TWO_0:
        begin
          if (!(alu_op_2 == OP_MOV ||
                alu_op_2 == OP_BIC ||
                alu_op_2 == OP_BIS ||
                alu_op_2 == OP_DADD)) begin
            if (bw == 0) begin
              registers[SR][FLAG_N] <= result[15] == 1;
              registers[SR][FLAG_Z] <= result[15:0] == 0;
            end else begin
              registers[SR][FLAG_N] <= result[7] == 1;
              registers[SR][FLAG_Z] <= result[7:0] == 0;
            end
          end

          if (alu_op_2 == OP_ADD ||
              alu_op_2 == OP_ADDC) begin
            if (bw == 0) begin
              registers[SR][FLAG_C] <= result[16];
              registers[SR][FLAG_V] <=
                source[15] == result[15] && temp[15] != source[15];
            end else begin
              registers[SR][FLAG_C] <= result[8];
              registers[SR][FLAG_V] <=
                source[7] == result[7] && temp[7] != source[7];
            end
          end else if (alu_op_2 == OP_SUB ||
                       alu_op_2 == OP_SUBC ||
                       alu_op_2 == OP_CMP) begin
            if (bw == 0) begin
              registers[SR][FLAG_C] <= result[16];
              registers[SR][FLAG_V] <=
                source[15] != result[15] && temp[15] != source[15];
            end else begin
              registers[SR][FLAG_C] <= result[8];
              registers[SR][FLAG_V] <=
                source[7] != result[7] && temp[7] != source[7];
            end
          end else begin
            registers[SR][FLAG_V] <= 0;
          end

          if (alu_op_2 == OP_CMP || alu_op_2 == OP_BIT)
            state <= STATE_FETCH_OP_0;
          else
            if (ad == 0) begin
              registers[rd] <= result;
              state <= STATE_FETCH_OP_0;
            end else begin
              //mem_address <= pc;
              //mem_bus_enable <= 1;
              //state <= STATE_WB_TWO_1;
              state <= STATE_DATA_STORE_0;
            end
        end
      STATE_PUSH_0:
        begin
          mem_bus_enable <= 1;
          mem_write_enable <= 1;
          mem_write <= result[15:0];
          mem_write_mask <= 2'b00;
          mem_address <= sp - 2;
          state <= STATE_PUSH_1;
        end
      STATE_PUSH_1:
        begin
          registers[SP] <= mem_address;
          mem_bus_enable <= 0;
          mem_write_enable <= 0;
          state <= STATE_FETCH_OP_0;
        end
      STATE_RETI_0:
        begin
          mem_bus_enable <= 1;
          mem_address <= registers[SP];
          registers[SP] <= registers[SP] + 2;
          state <= STATE_RETI_1;
        end
      STATE_RETI_1:
        begin
          mem_bus_enable <= 0;
          reti_state <= ^1;

          if (reti_state == 0) begin
            registers[SR] <= mem_read[15:0];
            state <= STATE_RETI_0;
          end else begin
            registers[PC] <= { mem_read[15:1], 1'b0 };
            state <= STATE_FETCH_OP_0;
          end
        end
      STATE_DATA_STORE_0:
        begin
          if (bw == 1)  begin
            case (ea[0])
              0:
                begin
                  mem_write <= { 8'h00, result[7:0] };
                  mem_write_mask <= 2'b10;
                end
              1:
                begin
                  mem_write <= { result[7:0], 8'h00 };
                  mem_write_mask <= 2'b01;
                end
            endcase
          end else begin
            mem_write <= result;
            mem_write_mask <= 2'b00;
          end

          mem_address <= ea;
          mem_write_enable <= 1;
          mem_bus_enable <= 1;
          state <= STATE_DATA_STORE_1;
        end
      STATE_DATA_STORE_1:
        begin
          mem_bus_enable <= 0;
          mem_write_enable <= 0;
          state <= STATE_FETCH_OP_0;
        end
      STATE_EEPROM_START:
        begin
          // Initialize values for reading from SPI-like EEPROM.
          if (eeprom_ready) begin
            //eeprom_mem_address <= pc;
            eeprom_mem_address <= 16'hc000;
            eeprom_count <= 0;
            state <= STATE_EEPROM_READ;
          end
        end
      STATE_EEPROM_READ:
        begin
          // Set the next EEPROM address to read from and strobe.
          mem_bus_enable <= 0;
          eeprom_address <= eeprom_count;
          eeprom_strobe <= 1;
          state <= STATE_EEPROM_WAIT;
        end
      STATE_EEPROM_WAIT:
        begin
          // Wait until 8 bits are clocked in.
          eeprom_strobe <= 0;

          if (eeprom_ready) begin

            if (eeprom_count[1:0] == 3) begin
              mem_address <= eeprom_mem_address;
              mem_write_mask <= 4'b0000;
              // After reading 4 bytes, store the 32 bit value to RAM.
              mem_write <= {
                eeprom_data_out,
                eeprom_holding[2],
                eeprom_holding[1],
                eeprom_holding[0]
              };

              state <= STATE_EEPROM_WRITE;
            end else begin
              // Read 3 bytes into a holding register.
              eeprom_holding[eeprom_count[1:0]] <= eeprom_data_out;
              state <= STATE_EEPROM_READ;
            end

            eeprom_count <= eeprom_count + 1;
          end
        end
      STATE_EEPROM_WRITE:
        begin
          // Write value read from EEPROM into memory.
          mem_bus_enable <= 1;
          mem_write_enable <= 1;
          eeprom_mem_address <= eeprom_mem_address + 4;

          state <= STATE_EEPROM_DONE;
        end
      STATE_EEPROM_DONE:
        begin
          // Finish writing and read next byte if needed.
          mem_bus_enable <= 0;
          mem_write_enable <= 0;

          if (eeprom_count == 0) begin
            // Read in 2048 bytes.
            state <= STATE_FETCH_OP_0;
          end else
            state <= STATE_EEPROM_READ;
        end
      STATE_ERROR:
        begin
          state <= STATE_ERROR;
        end
      STATE_HALTED:
        begin
          state <= STATE_HALTED;
        end
    endcase
end

memory_bus memory_bus_0(
  .address      (mem_address),
  .data_in      (mem_write),
  .write_mask   (mem_write_mask),
  .data_out     (mem_read),
  //.debug        (mem_debug),
  //.data_ready   (mem_data_ready),
  .bus_enable   (mem_bus_enable),
  .write_enable (mem_write_enable),
  .clk          (clk),
  .raw_clk      (raw_clk),
  .speaker_p    (speaker_p),
  .speaker_m    (speaker_m),
  .ioport_0     (ioport_0),
  .ioport_1     (ioport_1),
  .ioport_2     (ioport_2),
  .ioport_3     (ioport_3),
  .button_0     (button_0),
  .reset        (~button_reset),
  .spi_clk      (spi_clk),
  .spi_mosi     (spi_mosi),
  .spi_miso     (spi_miso)
);

eeprom eeprom_0
(
  .address    (eeprom_address),
  .strobe     (eeprom_strobe),
  .raw_clk    (raw_clk),
  .eeprom_cs  (eeprom_cs),
  .eeprom_clk (eeprom_clk),
  .eeprom_di  (eeprom_di),
  .eeprom_do  (eeprom_do),
  .ready      (eeprom_ready),
  .data_out   (eeprom_data_out)
);

endmodule

