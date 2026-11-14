`timescale 1ns/1ps
//============================================================================
// axi_dut.sv : AXI4-Lite slave with internal memory (all RTL in one file)
//   modules: axi4lite_mem, axi4lite_write_resp, axi4lite_read_resp, axi4lite_slave
//============================================================================

//============================================================================
// axi4lite_mem.sv : Internal memory (256 bytes = 64 words of 32 bits)
//   - Synchronous write with byte strobes
//   - Combinational read
//============================================================================
module axi4lite_mem #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 8
)(
  input  logic                    ACLK,
  // Memory write interface
  input  logic                    wr_en,
  input  logic [ADDR_WIDTH-1:0]   wr_addr,
  input  logic [DATA_WIDTH-1:0]   wr_data,
  input  logic [DATA_WIDTH/8-1:0] wr_strb,
  // Memory read interface
  input  logic [ADDR_WIDTH-1:0]   rd_addr,
  output logic [DATA_WIDTH-1:0]   rd_data
);

  localparam DEPTH = 2**(ADDR_WIDTH-2);   // number of 32-bit words

  logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  integer b;

  // Memory starts at zero so that the scoreboard model (also zero) matches
  initial begin
    for (i = 0; i < DEPTH; i = i + 1)
      mem[i] = {DATA_WIDTH{1'b0}};
  end

  // Write: only byte lanes selected by WSTRB are updated
  always @(posedge ACLK) begin
    if (wr_en) begin
      for (b = 0; b < DATA_WIDTH/8; b = b + 1) begin
        if (wr_strb[b])
          mem[wr_addr[ADDR_WIDTH-1:2]][8*b +: 8] <= wr_data[8*b +: 8];
      end
    end
  end

  // Read
  assign rd_data = mem[rd_addr[ADDR_WIDTH-1:2]];

endmodule

//============================================================================
// axi4lite_write_resp.sv : Write channel responder (AW, W, B channels)
//   - Accepts AW and W independently (any order)
//   - When both are received -> one memory write, then BVALID/BRESP=OKAY
//   - BVALID held until BREADY
//============================================================================
module axi4lite_write_resp #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 8
)(
  input  logic                    ACLK,
  input  logic                    ARESETn,
  // Write address channel
  input  logic [ADDR_WIDTH-1:0]   AWADDR,
  input  logic                    AWVALID,
  output logic                    AWREADY,
  // Write data channel
  input  logic [DATA_WIDTH-1:0]   WDATA,
  input  logic [DATA_WIDTH/8-1:0] WSTRB,
  input  logic                    WVALID,
  output logic                    WREADY,
  // Write response channel
  output logic [1:0]              BRESP,
  output logic                    BVALID,
  input  logic                    BREADY,
  // Memory write interface
  output logic                    mem_wr_en,
  output logic [ADDR_WIDTH-1:0]   mem_wr_addr,
  output logic [DATA_WIDTH-1:0]   mem_wr_data,
  output logic [DATA_WIDTH/8-1:0] mem_wr_strb
);

  logic                    aw_got   = 1'b0;
  logic                    w_got    = 1'b0;
  logic                    bvalid_r = 1'b0;
  logic [ADDR_WIDTH-1:0]   addr_q;
  logic [DATA_WIDTH-1:0]   data_q;
  logic [DATA_WIDTH/8-1:0] strb_q;

  assign AWREADY = !aw_got && !bvalid_r;
  assign WREADY  = !w_got  && !bvalid_r;

  assign mem_wr_en   = aw_got && w_got && !bvalid_r;
  assign mem_wr_addr = addr_q;
  assign mem_wr_data = data_q;
  assign mem_wr_strb = strb_q;

  assign BVALID = bvalid_r;
  assign BRESP  = 2'b00;            // OKAY

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      aw_got   <= 1'b0;
      w_got    <= 1'b0;
      bvalid_r <= 1'b0;
      addr_q   <= {ADDR_WIDTH{1'b0}};
      data_q   <= {DATA_WIDTH{1'b0}};
      strb_q   <= {(DATA_WIDTH/8){1'b0}};
    end else begin
      // accept write address
      if (AWVALID && AWREADY) begin
        aw_got <= 1'b1;
        addr_q <= AWADDR;
      end
      // accept write data
      if (WVALID && WREADY) begin
        w_got  <= 1'b1;
        data_q <= WDATA;
        strb_q <= WSTRB;
      end
      // both received: memory is written at this clock edge, send response
      if (mem_wr_en) begin
        aw_got   <= 1'b0;
        w_got    <= 1'b0;
        bvalid_r <= 1'b1;
      end
      // response accepted
      if (bvalid_r && BREADY)
        bvalid_r <= 1'b0;
    end
  end

endmodule

//============================================================================
// axi4lite_read_resp.sv : Read channel responder (AR, R channels)
//   - Accepts ARADDR, reads memory, returns RDATA with RRESP=OKAY
//   - RVALID held until RREADY
//============================================================================
module axi4lite_read_resp #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 8
)(
  input  logic                  ACLK,
  input  logic                  ARESETn,
  // Read address channel
  input  logic [ADDR_WIDTH-1:0] ARADDR,
  input  logic                  ARVALID,
  output logic                  ARREADY,
  // Read data channel
  output logic [DATA_WIDTH-1:0] RDATA,
  output logic [1:0]            RRESP,
  output logic                  RVALID,
  input  logic                  RREADY,
  // Memory read interface
  output logic [ADDR_WIDTH-1:0] mem_rd_addr,
  input  logic [DATA_WIDTH-1:0] mem_rd_data
);

  logic                  rvalid_r = 1'b0;
  logic [DATA_WIDTH-1:0] rdata_r;

  assign ARREADY     = !rvalid_r;
  assign mem_rd_addr = ARADDR;
  assign RVALID      = rvalid_r;
  assign RDATA       = rdata_r;
  assign RRESP       = 2'b00;       // OKAY

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      rvalid_r <= 1'b0;
      rdata_r  <= {DATA_WIDTH{1'b0}};
    end else begin
      if (ARVALID && ARREADY) begin
        rvalid_r <= 1'b1;
        rdata_r  <= mem_rd_data;
      end else if (rvalid_r && RREADY) begin
        rvalid_r <= 1'b0;
      end
    end
  end

endmodule

//============================================================================
// axi4lite_slave.sv : Top of the AXI4-Lite peripheral (DUT)
//   write responder + read responder + internal memory
//============================================================================
module axi4lite_slave #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 8
)(
  input  logic                    ACLK,
  input  logic                    ARESETn,
  // Write address channel
  input  logic [ADDR_WIDTH-1:0]   AWADDR,
  input  logic [2:0]              AWPROT,
  input  logic                    AWVALID,
  output logic                    AWREADY,
  // Write data channel
  input  logic [DATA_WIDTH-1:0]   WDATA,
  input  logic [DATA_WIDTH/8-1:0] WSTRB,
  input  logic                    WVALID,
  output logic                    WREADY,
  // Write response channel
  output logic [1:0]              BRESP,
  output logic                    BVALID,
  input  logic                    BREADY,
  // Read address channel
  input  logic [ADDR_WIDTH-1:0]   ARADDR,
  input  logic [2:0]              ARPROT,
  input  logic                    ARVALID,
  output logic                    ARREADY,
  // Read data channel
  output logic [DATA_WIDTH-1:0]   RDATA,
  output logic [1:0]              RRESP,
  output logic                    RVALID,
  input  logic                    RREADY
);

  logic                    mem_wr_en;
  logic [ADDR_WIDTH-1:0]   mem_wr_addr;
  logic [DATA_WIDTH-1:0]   mem_wr_data;
  logic [DATA_WIDTH/8-1:0] mem_wr_strb;
  logic [ADDR_WIDTH-1:0]   mem_rd_addr;
  logic [DATA_WIDTH-1:0]   mem_rd_data;

  axi4lite_write_resp #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_wr (
    .ACLK(ACLK), .ARESETn(ARESETn),
    .AWADDR(AWADDR), .AWVALID(AWVALID), .AWREADY(AWREADY),
    .WDATA(WDATA), .WSTRB(WSTRB), .WVALID(WVALID), .WREADY(WREADY),
    .BRESP(BRESP), .BVALID(BVALID), .BREADY(BREADY),
    .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr),
    .mem_wr_data(mem_wr_data), .mem_wr_strb(mem_wr_strb)
  );

  axi4lite_read_resp #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_rd (
    .ACLK(ACLK), .ARESETn(ARESETn),
    .ARADDR(ARADDR), .ARVALID(ARVALID), .ARREADY(ARREADY),
    .RDATA(RDATA), .RRESP(RRESP), .RVALID(RVALID), .RREADY(RREADY),
    .mem_rd_addr(mem_rd_addr), .mem_rd_data(mem_rd_data)
  );

  axi4lite_mem #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_mem (
    .ACLK(ACLK),
    .wr_en(mem_wr_en), .wr_addr(mem_wr_addr),
    .wr_data(mem_wr_data), .wr_strb(mem_wr_strb),
    .rd_addr(mem_rd_addr), .rd_data(mem_rd_data)
  );

endmodule