module riscvpipeline (
    input        clk,
    input        reset,
    output [31:0] PC,
    input  [31:0] Instr,
    output [31:0] Address,
    output [31:0] WriteData,
    output        MemWrite,
    input  [31:0] ReadData,
    output [3:0] WriteMask
);

   /* The 10 "recognizers" for the 10 codeops */
   function isALUreg; input [31:0] I; isALUreg=(I[6:0]==7'b0110011); endfunction
   function isALUimm; input [31:0] I; isALUimm=(I[6:0]==7'b0010011); endfunction
   function isBranch; input [31:0] I; isBranch=(I[6:0]==7'b1100011); endfunction
   function isJALR;   input [31:0] I; isJALR  =(I[6:0]==7'b1100111); endfunction
   function isJAL;    input [31:0] I; isJAL   =(I[6:0]==7'b1101111); endfunction
   function isAUIPC;  input [31:0] I; isAUIPC =(I[6:0]==7'b0010111); endfunction
   function isLUI;    input [31:0] I; isLUI   =(I[6:0]==7'b0110111); endfunction
   function isLoad;   input [31:0] I; isLoad  =(I[6:0]==7'b0000011); endfunction
   function isStore;  input [31:0] I; isStore =(I[6:0]==7'b0100011); endfunction
   function isSYSTEM; input [31:0] I; isSYSTEM=(I[6:0]==7'b1110011); endfunction

   /* Register indices */
   function [4:0] rs1Id; input [31:0] I; rs1Id = I[19:15];      endfunction
   function [4:0] rs2Id; input [31:0] I; rs2Id = I[24:20];      endfunction
   function [4:0] shamt; input [31:0] I; shamt = I[24:20];      endfunction
   function [4:0] rdId;  input [31:0] I; rdId  = I[11:7];       endfunction
   function [1:0] csrId; input [31:0] I; csrId = {I[27],I[21]}; endfunction

   /* funct3 and funct7 */
   function [2:0] funct3; input [31:0] I; funct3 = I[14:12]; endfunction
   function [6:0] funct7; input [31:0] I; funct7 = I[31:25]; endfunction

   /* EBREAK and CSRRS instruction "recognizers" */
   function isEBREAK; input [31:0] I; isEBREAK = (isSYSTEM(I) && funct3(I) == 3'b000); endfunction

   /* The 5 immediate formats */
   function [31:0] Uimm; input [31:0] I; Uimm={I[31:12],{12{1'b0}}}; endfunction
   function [31:0] Iimm; input [31:0] I; Iimm={{21{I[31]}},I[30:20]}; endfunction
   function [31:0] Simm; input [31:0] I; Simm={{21{I[31]}},I[30:25],I[11:7]}; endfunction
   function [31:0] Bimm; input [31:0] I; Bimm = {{20{I[31]}},I[7],I[30:25],I[11:8],1'b0}; endfunction
   function [31:0] Jimm; input [31:0] I; Jimm = {{12{I[31]}},I[19:12],I[20],I[30:21],1'b0}; endfunction

   /* Read/Write tests */
   function writesRd; input [31:0] I; writesRd = !isStore(I) && !isBranch(I); endfunction
   function readsRs1; input [31:0] I; readsRs1 = !(isJAL(I) || isAUIPC(I) || isLUI(I)); endfunction
   function readsRs2; input [31:0] I; readsRs2 = isALUreg(I) || isBranch(I) || isStore(I); endfunction

/**********************  F: Instruction fetch *********************************/
   localparam NOP = 32'b0000000_00000_00000_000_00000_0110011;

   reg [31:0] F_PC;
   reg [31:0] FD_PC;
   reg [31:0] FD_instr;
   reg        FD_nop;

   assign PC = F_PC;

   /** These two signals come from the Execute stage **/
   wire [31:0] jumpOrBranchAddress;
   wire        jumpOrBranch;

   /* Hazard control signals (computed later, usados aqui) */
   wire stall;  // bolha após lw
   // flush é simplesmente jumpOrBranch (desvio tomado)

/*** IF stage com suporte a stall e flush ***/
   always @(posedge clk) begin
      if (reset) begin
         F_PC    <= 32'b0;
         FD_PC   <= 32'b0;
         FD_instr<= NOP;
         FD_nop  <= 1'b1;
      end else begin
         // Se não estiver em stall, o PC e o registrador FD avançam
         if (!stall) begin
            FD_instr <= Instr;
            FD_PC    <= F_PC;

            // PC sequencial
            F_PC <= F_PC + 4;
            // Redireciona em salto/desvio
            if (jumpOrBranch)
               F_PC <= jumpOrBranchAddress;
         end

         // FD_nop controla se a próxima instrução decodificada vira NOP
         if (jumpOrBranch) begin
            // flush da instrução que está em F/D
            FD_nop <= 1'b1;
         end else if (!stall) begin
            FD_nop <= 1'b0;
         end
      end
   end

/************************ D: Instruction decode *******************************/
   reg [31:0] DE_PC;
   reg [31:0] DE_instr;
   reg [31:0] DE_rs1;
   reg [31:0] DE_rs2;

   /* These three signals come from the Writeback stage */
   wire        writeBackEn;
   wire [31:0] writeBackData;
   wire [4:0]  wbRdId;

   reg [31:0] RegisterBank [0:31];

   /* Índices de registradores na instrução em F/D */
   wire [4:0] D_rs1Id = rs1Id(FD_instr);
   wire [4:0] D_rs2Id = rs2Id(FD_instr);

   /* Leitura básica do banco de registradores */
   wire [31:0] D_rs1_val_raw = D_rs1Id ? RegisterBank[D_rs1Id] : 32'b0;
   wire [31:0] D_rs2_val_raw = D_rs2Id ? RegisterBank[D_rs2Id] : 32'b0;

   /* Bypass de writeback para leitura/escrita no mesmo ciclo */
   wire [31:0] D_rs1_val =
      (writeBackEn && wbRdId != 0 && wbRdId == D_rs1Id) ? writeBackData : D_rs1_val_raw;

   wire [31:0] D_rs2_val =
      (writeBackEn && wbRdId != 0 && wbRdId == D_rs2Id) ? writeBackData : D_rs2_val_raw;

   /* Sinais auxiliares para hazard de lw */
   wire [4:0] E_rdId   = rdId(DE_instr);
   wire       E_isLoad = isLoad(DE_instr);
   wire       D_readsRs1 = readsRs1(FD_instr);
   wire       D_readsRs2 = readsRs2(FD_instr);

   /* Hazard de load-use: lw em E, próxima instrução em D usa o rd */
   wire loadUseHazard =
      E_isLoad &&
      (E_rdId != 0) &&
      ( (D_readsRs1 && (D_rs1Id == E_rdId)) ||
        (D_readsRs2 && (D_rs2Id == E_rdId)) );

   assign stall = loadUseHazard; // uma bolha após lw quando necessário
   wire flushD = jumpOrBranch;   // flush na D->E quando desvio é tomado

   always @(posedge clk) begin
      if (reset) begin
         DE_PC    <= 32'b0;
         DE_instr <= NOP;
         DE_rs1   <= 32'b0;
         DE_rs2   <= 32'b0;
      end else if (flushD) begin
         // Descarta a instrução que iria para E após um desvio tomado
         DE_PC    <= 32'b0;
         DE_instr <= NOP;
         DE_rs1   <= 32'b0;
         DE_rs2   <= 32'b0;
      end else if (stall) begin
         // Insere bolha em E (nop). A instrução em F/D fica "parada".
         DE_instr <= NOP;
         DE_rs1   <= 32'b0;
         DE_rs2   <= 32'b0;
         // DE_PC pode ser mantido ou zerado, não é relevante para NOP
      end else begin
         DE_PC    <= FD_PC;
         DE_instr <= FD_nop ? NOP : FD_instr;
         DE_rs1   <= D_rs1_val;
         DE_rs2   <= D_rs2_val;
      end

      // Escrita no banco de registradores (writeback)
      if (writeBackEn)
         RegisterBank[wbRdId] <= writeBackData;
   end

/************************ E: Execute *****************************************/
   reg [31:0] EM_PC;
   reg [31:0] EM_instr;
   reg [31:0] EM_rs2;
   reg [31:0] EM_Eresult;
   reg [31:0] EM_addr;

   /* Índices na instrução em E */
   wire [4:0] E_rs1Id = rs1Id(DE_instr);
   wire [4:0] E_rs2Id = rs2Id(DE_instr);

   /* Informações das instruções em M e W para encaminhamento */
   wire [4:0] M_rdId     = rdId(EM_instr);
   wire       M_writesRd = writesRd(EM_instr);
   wire       M_isLoad   = isLoad(EM_instr);

   wire [4:0] W_rdId     = rdId(MW_instr);
   wire       W_writesRd = writesRd(MW_instr);

   /* Resultado que sai da etapa M (ALU/JAL/LUI/AUIPC etc.) */
   wire [31:0] M_result = EM_Eresult;

   /* Resultado que sai da etapa W (já decide entre ALU e LOAD) */
   wire [31:0] W_result =
      isLoad(MW_instr) ? MW_Mdata : MW_Eresult;

   /* Encaminhamento para rs1 em E */
   wire forwardA_fromM =
      M_writesRd && !M_isLoad && (M_rdId != 0) && (M_rdId == E_rs1Id);

   wire forwardA_fromW =
      W_writesRd && (W_rdId != 0) && (W_rdId == E_rs1Id);

   wire [31:0] E_rs1_val =
      forwardA_fromM ? M_result :
      forwardA_fromW ? W_result :
      DE_rs1;

   /* Encaminhamento para rs2 em E */
   wire forwardB_fromM =
      M_writesRd && !M_isLoad && (M_rdId != 0) && (M_rdId == E_rs2Id);

   wire forwardB_fromW =
      W_writesRd && (W_rdId != 0) && (W_rdId == E_rs2Id);

   wire [31:0] E_rs2_val =
      forwardB_fromM ? M_result :
      forwardB_fromW ? W_result :
      DE_rs2;

   /* Entradas da ALU (já com encaminhamento aplicado) */
   wire [31:0] E_aluIn1 = E_rs1_val;
   wire [31:0] E_aluSrc2Reg = E_rs2_val;

   wire [31:0] E_aluIn2 =
      (isALUreg(DE_instr) | isBranch(DE_instr)) ? E_aluSrc2Reg : Iimm(DE_instr);

   wire [4:0]  E_shamt  =
      isALUreg(DE_instr) ? E_aluSrc2Reg[4:0] : shamt(DE_instr);

   wire E_minus       = DE_instr[30] & isALUreg(DE_instr);
   wire E_arith_shift = DE_instr[30];

   // Alias para o $monitor do testbench
   wire [31:0] aluIn1 = E_aluIn1;
   wire [31:0] aluIn2 = E_aluIn2;

   // The adder is used by both arithmetic instructions and JALR.
   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;

   // Use a single 33 bits subtract to do subtraction and all comparisons
   // (trick borrowed from swapforth/J1)
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   // Flip a 32 bit word. Used by the shifter (a single shifter for
   // left and right shifts, saves silicium !)
   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7],
                x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15],
                x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
                x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   wire [31:0] E_shifter_in =
      funct3(DE_instr) == 3'b001 ? flip32(E_aluIn1) : E_aluIn1;

   wire [31:0] E_shifter =
      $signed({E_arith_shift & E_aluIn1[31], E_shifter_in}) >>> E_aluIn2[4:0];

   wire [31:0] E_leftshift = flip32(E_shifter);

   reg [31:0] E_aluOut;
   always @(*) begin
      case(funct3(DE_instr))
         3'b000: E_aluOut = E_minus ? E_aluMinus[31:0] : E_aluPlus;
         3'b001: E_aluOut = E_leftshift;
         3'b010: E_aluOut = {31'b0, E_LT};
         3'b011: E_aluOut = {31'b0, E_LTU};
         3'b100: E_aluOut = E_aluIn1 ^ E_aluIn2;
         3'b101: E_aluOut = E_shifter;
         3'b110: E_aluOut = E_aluIn1 | E_aluIn2;
         3'b111: E_aluOut = E_aluIn1 & E_aluIn2;
      endcase
   end

   /*********** Branch, JAL, JALR ***********************************/
   reg E_takeBranch;
   always @(*) begin
      case (funct3(DE_instr))
         3'b000: E_takeBranch = E_EQ;
         3'b001: E_takeBranch = !E_EQ;
         3'b100: E_takeBranch = E_LT;
         3'b101: E_takeBranch = !E_LT;
         3'b110: E_takeBranch = E_LTU;
         3'b111: E_takeBranch = !E_LTU;
         default: E_takeBranch = 1'b0;
      endcase
   end

   wire E_JumpOrBranch = (
         isJAL(DE_instr)  ||
         isJALR(DE_instr) ||
         (isBranch(DE_instr) && E_takeBranch)
   );

   wire [31:0] E_JumpOrBranchAddr =
      isBranch(DE_instr) ? DE_PC + Bimm(DE_instr) :
      isJAL(DE_instr)    ? DE_PC + Jimm(DE_instr) :
      /* JALR */           {E_aluPlus[31:1],1'b0} ;

   wire [31:0] E_result =
      (isJAL(DE_instr) | isJALR(DE_instr)) ? DE_PC+4                :
      isLUI(DE_instr)                      ? Uimm(DE_instr)         :
      isAUIPC(DE_instr)                    ? DE_PC + Uimm(DE_instr) :
                                            E_aluOut               ;

   always @(posedge clk) begin
      EM_PC      <= DE_PC;
      EM_instr   <= DE_instr;
      EM_rs2     <= E_rs2_val; // dado de store com encaminhamento
      EM_Eresult <= E_result;
      EM_addr    <= isStore(DE_instr) ?
                      (E_rs1_val + Simm(DE_instr)) :
                      (E_rs1_val + Iimm(DE_instr)) ;
   end

/************************ M: Memory *******************************************/
   reg [31:0] MW_PC;
   reg [31:0] MW_instr;
   reg [31:0] MW_Eresult;
   reg [31:0] MW_addr;
   reg [31:0] MW_Mdata;
   reg [31:0] MW_IOresult;
   reg [31:0] MW_CSRresult;

   wire [2:0] M_funct3 = funct3(EM_instr);
   wire       M_isB    = (M_funct3[1:0] == 2'b00);
   wire       M_isH    = (M_funct3[1:0] == 2'b01);

   /*************** STORE **************************/
   wire [31:0] M_STORE_data = EM_rs2;

   assign Address  = EM_addr;
   assign MemWrite = isStore(EM_instr);
   assign WriteData = EM_rs2;

   // -----------------------------------------------------------------
   // WriteMask para SB / SH / SW
   // funct3 (store):
   //   3'b000 -> SB
   //   3'b001 -> SH
   //   3'b010 -> SW
   // WriteMask[0] = byte mais baixo
   // WriteMask[3] = byte mais alto
   // -----------------------------------------------------------------
   reg [3:0] M_WriteMask;
   wire [1:0] M_byteOffset = EM_addr[1:0];

   always @(*) begin
      if (!isStore(EM_instr)) begin
         M_WriteMask = 4'b0000;
      end else begin
         case (M_funct3)
           3'b000: begin  // SB
              case (M_byteOffset)
                2'b00: M_WriteMask = 4'b0001;
                2'b01: M_WriteMask = 4'b0010;
                2'b10: M_WriteMask = 4'b0100;
                2'b11: M_WriteMask = 4'b1000;
              endcase
           end
           3'b001: begin  // SH
              case (M_byteOffset[1])
                1'b0: M_WriteMask = 4'b0011; // bytes 0 e 1
                1'b1: M_WriteMask = 4'b1100; // bytes 2 e 3
              endcase
           end
           3'b010: begin  // SW
              M_WriteMask = 4'b1111;
           end
           default: begin
              M_WriteMask = 4'b0000;
           end
         endcase
      end
   end

   assign WriteMask = M_WriteMask;

   always @(posedge clk) begin
      MW_PC        <= EM_PC;
      MW_instr     <= EM_instr;
      MW_Eresult   <= EM_Eresult;
      MW_Mdata     <= ReadData;
      MW_addr      <= EM_addr;
   end

/************************ W: WriteBack ****************************************/

   wire [2:0] W_funct3 = funct3(MW_instr);
   wire       W_isB    = (W_funct3[1:0] == 2'b00);
   wire       W_isH    = (W_funct3[1:0] == 2'b01);
   wire       W_sext   = !W_funct3[2];
   wire       W_isIO   = MW_addr[22];

   // ---------------------- LOAD (LB, LH, LW, LBU, LHU) ---------------------
   wire [1:0] W_byteOffset   = MW_addr[1:0];
   wire [31:0] W_shiftedLoad = MW_Mdata >> (W_byteOffset * 8);
   wire [7:0]  W_loadByte    = W_shiftedLoad[7:0];
   wire [15:0] W_loadHalf    = W_shiftedLoad[15:0];

   reg [31:0] W_loadData;
   always @(*) begin
      case (W_funct3)
         3'b000: // LB  (byte, com extensão de sinal)
            W_loadData = {{24{W_loadByte[7]}}, W_loadByte};
         3'b001: // LH  (halfword, com extensão de sinal)
            W_loadData = {{16{W_loadHalf[15]}}, W_loadHalf};
         3'b010: // LW
            W_loadData = MW_Mdata;
         3'b100: // LBU (byte, zero-extend)
            W_loadData = {24'b0, W_loadByte};
         3'b101: // LHU (halfword, zero-extend)
            W_loadData = {16'b0, W_loadHalf};
         default:
            W_loadData = MW_Mdata;
      endcase
   end

   assign writeBackData = isLoad(MW_instr) ? W_loadData : MW_Eresult;
   assign writeBackEn   = writesRd(MW_instr) && rdId(MW_instr) != 0;
   assign wbRdId        = rdId(MW_instr);

   assign jumpOrBranchAddress = E_JumpOrBranchAddr;
   assign jumpOrBranch        = E_JumpOrBranch;

/******************************************************************************/

   wire halt;
   assign halt = !reset & isEBREAK(MW_instr);

   always @(posedge clk) begin
      if (halt) begin
         $writememh("regs.out", RegisterBank);
         $finish();
      end
   end
endmodule
