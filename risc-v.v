module fetch (
  input zero, rst, clk, branch, jump,
  input [31:0] sigext,
  output [31:0] inst
  );

  wire [31:0] pc, pc_4, new_pc, new_pc_old;

  assign pc_4 = 4 + pc; // pc+4  Adder
  assign new_pc_old = (branch & zero) ? pc_4 + sigext : pc_4; // new PC Mux
  assign new_pc = jump ? sigext : new_pc_old;

  PC program_counter(new_pc, clk, rst, pc);

  reg [31:0] inst_mem [0:31];

  assign inst = inst_mem[pc[31:2]];

  initial begin
    // Exemplos
    inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  ok
    // ---- ss
    // imm1   |rs2  |rs1  |fu3|im2  |opc
    // 0000000|00001|00101|000|01000|0100100
    inst_mem[3] <= 32'b00000000000100101000010000100100; // ss  x5, x1, 8  [novo comando]
    // ----
    // ---- lwi
    // imm1   |rs2  |rs1  |im3|rd   |opc
    // 0000000|00010|00010|000|00001|0000100
    inst_mem[4] <= 32'b00000000001000010000000010000100; // lwi x1, x2, x2 [novo comando]
    // ----
    // ---- swap
    // imm1   |rs2  |rs1  |im3|rd   |opc
    // 0000000|01010|00010|000|00000|0100101
    inst_mem[5] <= 32'b00000000101000010000000000100101; // swap x2, x10 [novo comando]
    // ----
    // ---- BLT
    // [12:5] |rs2  |rs1  |fu3|[4:0]|opc
    // 0000000|00010|01010|100|10110|1100100
    inst_mem[6] <= 32'b00000000001001010100101101100100; // BLT x10, x2, 22 [novo comando]
    // ----
    // ---- BGE
    // [12:5] |rs2  |rs1  |fu3|[4:0]|opc
    // 0000000|00010|01010|101|10110|1100100
    inst_mem[7] <= 32'b00000000001001010101101101100100; // BGE x10, x2, 22 [novo comando]
    // ----
    // ####################################
    // Shift the contents of register rs1 left by a number of bits specified in the immediate field, storing the result in rd
    // immm       |rs1  |fc3|rd   |opc
    // 00000000010|00101|001|00010|0010100
    // imm = 2; rs1 = 5; rd = 2
    inst_mem[8] <= 32'b0000000001000101001000100010100; // slli x2, x5, 2 - R[rd] = R[rs1] << imm
    // ####################################
    // ----
    // ---- ORI
    // immm       |rs1  |fc3|rd   |opc
    // 00000001010|00101|110|00010|0100101
    inst_mem[9] <= 32'b00000000001001010101101101100100; // ORI x2, x5, 10 [novo comando]
    // ----
    // Load with increment
    // imm                 |rd   |opc
    // 00000000000000001001|10000|0110111
    inst_mem[10] <= 32'b00000000000000001001100000110111; // lui x20, 9
    // ----
    // ####################################
    // JUMP
    // i|[10:1]    |i|[19:12] |rd   |opc
    // 0|0001100100|0|00000000|00000|1101111
    // imm = 100; rd = 0
    inst_mem[11] <= 32'b00001100100000000000000001101111; // jump 100 [jump $address]
    // ####################################
    //inst_mem[1] <= 32'h00202223; // sw x2, 8(x0) ok
    //inst_mem[1] <= 32'h0050a423; // sw x5, 8(x1) ok
    //inst_mem[2] <= 32'h0000a003; // lw x1, x0(0) ok
    //inst_mem[1] <= 32'hfff00113; // addi x2,x0,-1 ok
    //inst_mem[2] <= 32'h00318133; // add x2, x3, x3 ok
    //inst_mem[3] <= 32'h40328133; // sub x2, x5, x3 ok
  end

endmodule

module PC (input [31:0] pc_in, input clk, rst, output reg [31:0] pc_out);

  always @(posedge clk) begin
    pc_out <= pc_in;
    if (~rst)
      pc_out <= 0;
  end

endmodule

module decode (
  input [31:0] inst, writedata,
  input clk,
  output [31:0] data1, data2, ImmGen,
  output alusrc, aluSrcA, memread, memwrite, memtoreg, branch, jump, adressSrc, writeDataSrc,
  output [1:0] aluop,
  output [9:0] funct
);

  wire branch, memread, memtoreg, MemWrite, alusrc, regwrite;
  wire aluSrcA, adressSrc, writeDataSrc, regWrite2, writeregSrc;
  wire [1:0] aluop;
  wire [4:0] writereg, rs1, rs2, rd;
  wire [6:0] opcode;
  wire [9:0] funct;
  wire [31:0] ImmGen;
  assign opcode = inst[6:0];
  assign rs1    = inst[19:15];
  assign rs2    = inst[24:20];
  assign rd     = inst[11:7];
  assign funct = {inst[31:25],inst[14:12]};

  ControlUnit control (
    opcode,
    inst,
    alusrc,
    aluSrcA,
    adressSrc,
    writeDataSrc,
    memtoreg,
    regwrite,
    regWrite2,
    writeregSrc,
    memread,
    memwrite,
    branch,
    jump,
    aluop,
    ImmGen
  );

  assign writereg = writeregSrc ? rs1 : rd;

  Register_Bank Registers (
    clk, regwrite, regWrite2,
    rs1, rs2, writereg,
    writedata,
    data1, data2
  );

endmodule

module ControlUnit (
  input [6:0] opcode,
  input [31:0] inst,
  output reg alusrc,
  aluSrcA,
  adressSrc,
  writeDataSrc,
  memtoreg,
  regwrite,
  regWrite2,
  writeregSrc,
  memread,
  memwrite,
  branch,
  jump,
  output reg [1:0] aluop,
  output reg [31:0] ImmGen
);

  always @(opcode) begin
    alusrc   <= 0;
    memtoreg <= 0;
    regwrite <= 0;
    memread  <= 0;
    memwrite <= 0;
    branch   <= 0;
    aluop    <= 0;
    writeregSrc <= 0;
    regWrite2 <= 0;
    aluSrcA  <= 0;
    adressSrc <= 0;
    writeDataSrc <= 0;
    ImmGen   <= 0;
    jump     <= 0;
    case(opcode)
      7'b0110011: begin // R type == 51
        regwrite <= 1;
        aluop    <= 2;
      end
      7'b1100011: begin // beq == 99
        branch   <= 1;
        aluop    <= 1;
        ImmGen   <= {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],1'b0};
      end
      7'b0010011: begin // addi == 19
        alusrc   <= 1;
        regwrite <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
      7'b0000011: begin // lw == 3
        alusrc   <= 1;
        memtoreg <= 1;
        regwrite <= 1;
        memread  <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
      7'b0000100: begin // lwi == 4
        alusrc   <= 0; // adress é calculado de rs1 + rs2 quando alusrc = 0
        memtoreg <= 1;
        regwrite <= 1;
        memread  <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
      7'b0100011: begin // sw == 35
        alusrc   <= 1;
        memwrite <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:25],inst[11:7]};
      end
      7'b0100100: begin // ss == 36
        alusrc       <= 1;
        aluSrcA      <= 1;
        memwrite     <= 1;
        adressSrc    <= 1;
        writeDataSrc <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:25],inst[11:7]};
      end
      7'b0100101: begin // ORI == 37
        alusrc <= 1;
        regwrite <= 1;
        aluop <= 2;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
      7'b0010100: begin // #################################### slli
        alusrc <= 1;
        regwrite <=  1;
        aluop <= 2;
        ImmGen <= {{20{inst[31]}},inst[31:20]}; // repete 20x o msb e concatena a ele os 11 bits do imm
      end // ####################################
      7'b0100101: begin // swap == 37
        regwrite <= 1;
        regWrite2 <= 1;
        writeregSrc <= 1;
      end
      7'b1100100: begin // BGE, BLT == 100
        branch   <= 1;
        aluop    <= 2;
        ImmGen   <= {{20{inst[31]}},inst[31:25],inst[11:7]};
      end
      7'b0110111: begin // lui == 55
      	regwrite <= 1;
        ImmGen   <= {inst[31:12],12'b0};
      end
      7'b1101111: begin // JUMP == 111
        jump     <= 1;//           | [20]   | [19:12]   |[11]    | [10:1]
        ImmGen   <= {{12{inst[31]}},inst[31],inst[19:12],inst[11],inst[30:21]};
      end
    endcase
  end

endmodule

module Register_Bank (
  input clk, regwrite, regWrite2,
  input [4:0] read_reg1, read_reg2, writereg,
  input [31:0] writedata,
  output [31:0] read_data1, read_data2
);

  integer i;
  wire isSwap;
  reg [31:0] memory [0:31]; // 32 registers de 32 bits cada
  reg [31:0] writeReg1Data;
  reg [31:0] writeReg2Data;
  reg [31:0] read_data1_init;
  reg [31:0] read_data2_init;
  // fill the memory
  initial begin
    for (i = 0; i <= 31; i++)
      memory[i] <= i;
  end

  assign isSwap = (regwrite && regWrite2);
  assign writeReg1Data = isSwap ? memory[read_reg2] : writedata;
  assign writeReg2Data = isSwap ? memory[read_reg1] : memory[read_reg2];

  assign read_data1_init = (regwrite && read_reg1==writereg) ? writedata : memory[read_reg1];
  assign read_data2_init = (regwrite && read_reg2==writereg) ? writedata : memory[read_reg2];
  assign read_data1 = isSwap ? writeReg1Data : read_data1_init;
  assign read_data2 = isSwap ? writeReg2Data : read_data2_init;


  always @(posedge clk) begin
    if (regwrite) begin
      memory[writereg] <= writeReg1Data;
    end
    if (regWrite2) begin
      memory[read_reg2] <= writeReg2Data;
    end
  end

endmodule

module execute (
  input [31:0] in1, in2, ImmGen,
  input alusrc, aluSrcA,
  input [1:0] aluop,
  input [9:0] funct,
  output zero,
  output [31:0] aluout
);

  wire [31:0] alu_A;
  wire [31:0] alu_B;
  wire [3:0] aluctrl;

  assign alu_B = (alusrc) ? ImmGen : in2 ;
  assign alu_A = aluSrcA ? in2 : in1;

  //Unidade Lógico Aritimética
  ALU alu (aluctrl, alu_A, alu_B, aluout, zero);

  alucontrol alucontrol (aluop, funct, aluctrl);

endmodule

module alucontrol (input [1:0] aluop, input [9:0] funct, output reg [3:0] alucontrol);

  wire [7:0] funct7;
  wire [2:0] funct3;

  assign funct3 = funct[2:0];
  assign funct7 = funct[9:3];

  always @(aluop) begin
    case (aluop)
      0: alucontrol <= 4'd2; // ADD to SW and LW
      1: alucontrol <= 4'd6; // SUB to branch
      default: begin
        case (funct3)
          0: alucontrol <= (funct7 == 0) ? /*ADD*/ 4'd2 : /*SUB*/ 4'd6;
          1: alucontrol <= 4'd5; // ####################################
          2: alucontrol <= 4'd7; // SLT
          4: alucontrol <= 4'd13; // BLT
          5: alucontrol <= 4'd14; // BGE
          6: alucontrol <= 4'd1; // OR
          //39: alucontrol <= 4'd12; // NOR
          7: alucontrol <= 4'd0; // AND
          default: alucontrol <= 4'd15; // Nop
        endcase
      end
    endcase
  end
endmodule

module ALU (input [3:0] alucontrol, input [31:0] A, B, output reg [31:0] aluout, output zero);

  assign zero = (aluout == 0); // Zero recebe um valor lógico caso aluout seja igual a zero.

  always @(alucontrol, A, B) begin
      case (alucontrol)
        0: aluout <= A & B; // AND
        1: aluout <= A | B; // OR
        2: aluout <= A + B; // ADD
        5: aluout <= A << B; // SLLI ####################################
        6: aluout <= A - B; // SUB
        //7: aluout <= A < B ? 32'd1:32'd0; //SLT
        //12: aluout <= ~(A | B); // NOR
        // aqui temos q colocar a operação contrária
        // pois queremos que a condição seja satisfeita
        // quanto zero = 1
        13: aluout <= A >= B; // zero if A < B
        14: aluout <= A < B; // zero if A >= B (BGE)
      default: aluout <= 0; //default 0, Nada acontece;
    endcase
  end
endmodule

module memory (
  input [31:0] data1, data2, aluout,
  input memread, memwrite, clk, adressSrc, writeDataSrc,
  output [31:0] readdata
);

  wire [31:0] address = adressSrc ? data1 : aluout; // Adress MUX
  wire [31:0] writeData = writeDataSrc ? aluout : data2; // Write data mux

  integer i;
  reg [31:0] memory [0:127];

  // fill the memory
  initial begin
    for (i = 0; i <= 127; i++)
      memory[i] <= i;
  end

  assign readdata = (memread) ? memory[address[31:2]] : 0;

  always @(posedge clk) begin
    if (memwrite)
      memory[address[31:2]] <= writeData;
  end
endmodule

module writeback (input [31:0] aluout, readdata, input memtoreg, output reg [31:0] write_data);
  always @(memtoreg) begin
    write_data <= (memtoreg) ? readdata : aluout;
  end
endmodule

// TOP -------------------------------------------
module mips (input clk, rst, output [31:0] writedata);

  wire [31:0] inst, sigext, data1, data2, aluout, readdata;
  wire zero, memread, memwrite, memtoreg, branch, alusrc;
  wire aluSrcA, adressSrc, writeDataSrc, jump;
  wire [9:0] funct;
  wire [1:0] aluop;

  // FETCH STAGE
  fetch fetch (zero, rst, clk, branch, jump, sigext, inst);

  // DECODE STAGE
  decode decode (
    inst,
    writedata,
    clk,
    data1,
    data2,
    sigext,
    alusrc,
    aluSrcA,
    memread,
    memwrite,
    memtoreg,
    branch,
    jump,
    adressSrc,
    writeDataSrc,
    aluop,
    funct,
  );

  // EXECUTE STAGE
  execute execute (
    data1, data2, sigext,
    alusrc, aluSrcA,
    aluop,
    funct,
    zero,
    aluout
  );

  // MEMORY STAGE
  memory memory (
    data1, data2, aluout,
    memread, memwrite, clk, adressSrc, writeDataSrc,
    readdata
  );

  // WRITEBACK STAGE
  writeback writeback (aluout, readdata, memtoreg, writedata);

endmodule