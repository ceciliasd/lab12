module memmulti (
  input  logic        clk, we,
  input  logic [31:0] a, wd,
  output logic [31:0] rd,
  input               mem_rstrb,
  input  logic [3:0]  mem_wmask,
);

  // MUDANÇA MÁGICA: "Packed Array" ([3:0][7:0])
  // Isso define explicitamente 4 bytes por palavra.
  // O Quartus adora isso para inferir RAM M10K com Byte Enable.
  (* ramstyle = "M10K" *) logic [3:0][7:0] RAM [0:255];

  // Inicialização: Funciona porque o SystemVerilog sabe carregar
  // um hex de 32 bits para dentro de um array empacotado [3:0][7:0].
  initial begin
    $readmemh("riscv.hex", RAM);
  end

endmodule
