module top(
    input CLOCK_50,
    input [9:0] SW,
    input [3:0] KEY,
    output reg [9:0] LEDR,
    output [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0
);

    wire clk, reset;
    wire [31:0] pc, instr;
    wire [31:0] writedata, addr;
    wire [31:0] readdata, MEM_readdata, IO_readdata;
    wire memwrite;
    wire [3:0] writemask;

    integer counter;
    always @(posedge CLOCK_50)
        counter <= counter + 1;
    assign clk = counter[21];
    assign reset = ~KEY[3];

    // CPU
    riscvpipeline cpu(
        clk, reset,
        pc, instr,
        addr, writedata,
        memwrite,
        readdata,
        writemask
    );

    // ===============================
// ADDRESS DECODE
// ===============================
wire isIO;
wire isRAM;

assign isIO  = addr[8];
assign isRAM = ~isIO;


// ===============================
// DATA MEMORY
// ===============================
mem #("data.hex") data_mem (
    .clk(clk),
    .we(memwrite & isRAM),
    .a(addr),
    .wd(writedata),
    .rd(MEM_readdata),
    .wm(writemask)
);

//////////////////
// instr mem
////////////////

mem #("text.hex") instr_mem (
    .a(pc),
    .rd(instr)
);


    // ===============================
    // MEMORY MAPPED I/O
    // ===============================

    localparam IO_LEDS_bit = 2;
    localparam IO_HEX_bit  = 3;
    localparam IO_KEY_bit  = 4;
    localparam IO_SW_bit   = 5;

    reg [23:0] hex_digits;

    dec7seg hex0(hex_digits[ 3: 0], HEX0);
    dec7seg hex1(hex_digits[ 7: 4], HEX1);
    dec7seg hex2(hex_digits[11: 8], HEX2);
    dec7seg hex3(hex_digits[15:12], HEX3);
    dec7seg hex4(hex_digits[19:16], HEX4);
    dec7seg hex5(hex_digits[23:20], HEX5);

    always @(posedge clk)
        if (memwrite & isIO) begin
            if (addr[IO_LEDS_bit])
                LEDR <= writedata[9:0];
            if (addr[IO_HEX_bit])
                hex_digits <= writedata[23:0];
        end

    assign IO_readdata =
          addr[IO_KEY_bit] ? {28'b0, KEY} :
          addr[IO_SW_bit ] ? {22'b0, SW } :
                             32'b0;

    // ===============================
    // READ DATA MUX
    // ===============================
    assign readdata = isIO ? IO_readdata : MEM_readdata;

endmodule
