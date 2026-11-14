`timescale 1ns/1ps
//============================================================================
// axi_tb.sv : interface + SVA + bind + UVM environment + tests + top module
//   compile AFTER axi_dut.sv
//============================================================================

//============================================================================
// axi_if.sv : AXI4-Lite interface with clocking blocks for driver / monitor
//============================================================================
interface axi_if #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 8
)(
  input logic ACLK,
  input logic ARESETn
);

  // Write address channel
  logic [ADDR_WIDTH-1:0]   AWADDR;
  logic [2:0]              AWPROT;
  logic                    AWVALID;
  logic                    AWREADY;
  // Write data channel
  logic [DATA_WIDTH-1:0]   WDATA;
  logic [DATA_WIDTH/8-1:0] WSTRB;
  logic                    WVALID;
  logic                    WREADY;
  // Write response channel
  logic [1:0]              BRESP;
  logic                    BVALID;
  logic                    BREADY;
  // Read address channel
  logic [ADDR_WIDTH-1:0]   ARADDR;
  logic [2:0]              ARPROT;
  logic                    ARVALID;
  logic                    ARREADY;
  // Read data channel
  logic [DATA_WIDTH-1:0]   RDATA;
  logic [1:0]              RRESP;
  logic                    RVALID;
  logic                    RREADY;

  // Driver (AXI master) clocking block
  clocking drv_cb @(posedge ACLK);
    default input #1step output #1;
    output AWADDR, AWPROT, AWVALID;
    output WDATA, WSTRB, WVALID;
    output BREADY;
    output ARADDR, ARPROT, ARVALID;
    output RREADY;
    input  AWREADY, WREADY, BRESP, BVALID, ARREADY, RDATA, RRESP, RVALID;
  endclocking

  // Monitor clocking block (all inputs)
  clocking mon_cb @(posedge ACLK);
    default input #1step;
    input AWADDR, AWVALID, AWREADY;
    input WDATA, WSTRB, WVALID, WREADY;
    input BRESP, BVALID, BREADY;
    input ARADDR, ARVALID, ARREADY;
    input RDATA, RRESP, RVALID, RREADY;
  endclocking

endinterface

//============================================================================
// axi_sva.sv : SystemVerilog Assertions for the AXI4-Lite slave
//   Bound to the DUT in axi_bind.sv (ports are connected by name)
//============================================================================
module axi_sva #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 8
)(
  input logic                    ACLK,
  input logic                    ARESETn,
  input logic [ADDR_WIDTH-1:0]   AWADDR,
  input logic [2:0]              AWPROT,
  input logic                    AWVALID,
  input logic                    AWREADY,
  input logic [DATA_WIDTH-1:0]   WDATA,
  input logic [DATA_WIDTH/8-1:0] WSTRB,
  input logic                    WVALID,
  input logic                    WREADY,
  input logic [1:0]              BRESP,
  input logic                    BVALID,
  input logic                    BREADY,
  input logic [ADDR_WIDTH-1:0]   ARADDR,
  input logic [2:0]              ARPROT,
  input logic                    ARVALID,
  input logic                    ARREADY,
  input logic [DATA_WIDTH-1:0]   RDATA,
  input logic [1:0]              RRESP,
  input logic                    RVALID,
  input logic                    RREADY
);

  // ---------------- handshake counters (used for ordering checks) -----------
  integer aw_cnt = 0, w_cnt = 0, b_cnt = 0, ar_cnt = 0, r_cnt = 0;

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      aw_cnt <= 0; w_cnt <= 0; b_cnt <= 0; ar_cnt <= 0; r_cnt <= 0;
    end else begin
      if (AWVALID && AWREADY) aw_cnt <= aw_cnt + 1;
      if (WVALID  && WREADY ) w_cnt  <= w_cnt  + 1;
      if (BVALID  && BREADY ) b_cnt  <= b_cnt  + 1;
      if (ARVALID && ARREADY) ar_cnt <= ar_cnt + 1;
      if (RVALID  && RREADY ) r_cnt  <= r_cnt  + 1;
    end
  end

  // ---------------- A1 : reset behaviour -----------------------------------
  A1_reset_valids_low : assert property (
    @(posedge ACLK) !ARESETn |-> (!BVALID && !RVALID)
  ) else $error("A1: BVALID/RVALID not low during reset");

  // ---------------- A2..A6 : VALID must stay high until READY --------------
  A2_awvalid_stable : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    (AWVALID && !AWREADY) |=> (AWVALID && $stable(AWADDR))
  ) else $error("A2: AWVALID dropped or AWADDR changed before AWREADY");

  A3_wvalid_stable : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    (WVALID && !WREADY) |=> (WVALID && $stable(WDATA) && $stable(WSTRB))
  ) else $error("A3: WVALID dropped or WDATA/WSTRB changed before WREADY");

  A4_arvalid_stable : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    (ARVALID && !ARREADY) |=> (ARVALID && $stable(ARADDR))
  ) else $error("A4: ARVALID dropped or ARADDR changed before ARREADY");

  A5_bvalid_stable : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    (BVALID && !BREADY) |=> (BVALID && $stable(BRESP))
  ) else $error("A5: BVALID dropped or BRESP changed before BREADY");

  A6_rvalid_stable : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    (RVALID && !RREADY) |=> (RVALID && $stable(RDATA) && $stable(RRESP))
  ) else $error("A6: RVALID dropped or RDATA/RRESP changed before RREADY");

  // ---------------- A7, A8 : response only after the request ---------------
  A7_bvalid_after_aw_w : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    BVALID |-> (aw_cnt > b_cnt) && (w_cnt > b_cnt)
  ) else $error("A7: BVALID without a completed AW and W handshake");

  A8_rvalid_after_ar : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    RVALID |-> (ar_cnt > r_cnt)
  ) else $error("A8: RVALID without a completed AR handshake");

  // ---------------- A9, A10 : response code is OKAY -------------------------
  A9_bresp_okay : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    BVALID |-> (BRESP == 2'b00)
  ) else $error("A9: BRESP is not OKAY");

  A10_rresp_okay : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    RVALID |-> (RRESP == 2'b00)
  ) else $error("A10: RRESP is not OKAY");

  // ---------------- A11 : no X/Z on slave control outputs -------------------
  A11_no_x : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    !$isunknown({AWREADY, WREADY, BVALID, ARREADY, RVALID})
  ) else $error("A11: X/Z on slave control outputs");

  // ---------------- A12 : read latency is bounded ---------------------------
  A12_read_latency : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    (ARVALID && ARREADY) |-> ##[1:4] RVALID
  ) else $error("A12: RVALID did not follow AR handshake within 4 cycles");

  // ---------------- A13 : write latency is bounded --------------------------
  A13_write_latency : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    ((aw_cnt > b_cnt) && (w_cnt > b_cnt)) |-> ##[0:3] BVALID
  ) else $error("A13: BVALID did not follow AW+W handshakes within 3 cycles");

  // ---------------- A14 : slave does not accept new AR while R pending ------
  A14_no_ar_while_r_pending : assert property (
    @(posedge ACLK) disable iff (!ARESETn)
    RVALID |-> !ARREADY
  ) else $error("A14: ARREADY high while a read response is pending");

  // ---------------- cover properties ----------------------------------------
  C1_aw_handshake : cover property (@(posedge ACLK) disable iff (!ARESETn) AWVALID && AWREADY);
  C2_w_handshake  : cover property (@(posedge ACLK) disable iff (!ARESETn) WVALID  && WREADY);
  C3_b_handshake  : cover property (@(posedge ACLK) disable iff (!ARESETn) BVALID  && BREADY);
  C4_ar_handshake : cover property (@(posedge ACLK) disable iff (!ARESETn) ARVALID && ARREADY);
  C5_r_handshake  : cover property (@(posedge ACLK) disable iff (!ARESETn) RVALID  && RREADY);
  C6_b_stall      : cover property (@(posedge ACLK) disable iff (!ARESETn) BVALID  && !BREADY);
  C7_r_stall      : cover property (@(posedge ACLK) disable iff (!ARESETn) RVALID  && !RREADY);

endmodule

//============================================================================
// axi_bind.sv : binds the assertion module into the DUT
//   (compile AFTER axi4lite_slave.sv and axi_sva.sv)
//============================================================================

//============================================================================
// UVM package: item, sequences, driver, monitor, agent, scoreboard,
//              coverage, env, tests
//============================================================================
package axi_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

//============================================================================
// axi_item.svh : AXI4-Lite transaction (one read or one write)
//============================================================================
class axi_item extends uvm_sequence_item;

  rand bit        write;            // 1 = write, 0 = read
  rand bit [7:0]  addr;
  rand bit [31:0] data;             // write data, or read data returned
  rand bit [3:0]  strb;             // write strobes
  rand int unsigned aw_delay;       // idle cycles before each handshake
  rand int unsigned w_delay;
  rand int unsigned b_delay;
  rand int unsigned ar_delay;
  rand int unsigned r_delay;
  bit [1:0]       resp;             // BRESP or RRESP

  constraint c_align { addr[1:0] == 2'b00; }

  constraint c_delay {
    aw_delay dist { [0:1] :/ 60, [2:6] :/ 40 };
    w_delay  dist { [0:1] :/ 60, [2:6] :/ 40 };
    b_delay  dist { [0:1] :/ 60, [2:6] :/ 40 };
    ar_delay dist { [0:1] :/ 60, [2:6] :/ 40 };
    r_delay  dist { [0:1] :/ 60, [2:6] :/ 40 };
  }

  `uvm_object_utils(axi_item)

  function new(string name = "axi_item");
    super.new(name);
  endfunction

  function string convert2string();
    return $sformatf("%s addr=0x%02h data=0x%08h strb=%04b resp=%0d",
                     write ? "WRITE" : "READ ", addr, data, strb, resp);
  endfunction

endclass

//============================================================================
// axi_sequences.svh : base sequence + directed / write-read / random sequences
//============================================================================

// ---------------------------------------------------------------- base
class axi_base_seq extends uvm_sequence #(axi_item);
  `uvm_object_utils(axi_base_seq)

  function new(string name = "axi_base_seq");
    super.new(name);
  endfunction

  // one write with given fields
  task do_write(bit [7:0] a, bit [31:0] d, bit [3:0] s);
    axi_item t;
    t = axi_item::type_id::create("t");
    start_item(t);
    if (!t.randomize() with { write == 1; addr == a; data == d; strb == s; })
      `uvm_error("SEQ", "randomize failed (write)")
    finish_item(t);
  endtask

  // one read of given address
  task do_read(bit [7:0] a);
    axi_item t;
    t = axi_item::type_id::create("t");
    start_item(t);
    if (!t.randomize() with { write == 0; addr == a; })
      `uvm_error("SEQ", "randomize failed (read)")
    finish_item(t);
  endtask

  // one fully random transaction
  task do_random();
    axi_item t;
    t = axi_item::type_id::create("t");
    start_item(t);
    if (!t.randomize() with { write dist { 1 := 50, 0 := 50 }; })
      `uvm_error("SEQ", "randomize failed (random)")
    finish_item(t);
  endtask
endclass

// ---------------------------------------------------------------- directed
class axi_directed_seq extends axi_base_seq;
  `uvm_object_utils(axi_directed_seq)

  function new(string name = "axi_directed_seq");
    super.new(name);
  endfunction

  task body();
    `uvm_info("SEQ", "Directed sequence started", UVM_LOW)

    // first and last word
    do_write(8'h00, 32'hDEADBEEF, 4'hF);
    do_read (8'h00);
    do_write(8'hFC, 32'hCAFEF00D, 4'hF);
    do_read (8'hFC);

    // all-zeros / all-ones / alternating patterns
    do_write(8'h10, 32'h00000000, 4'hF);  do_read(8'h10);
    do_write(8'h14, 32'hFFFFFFFF, 4'hF);  do_read(8'h14);
    do_write(8'h18, 32'hAAAAAAAA, 4'hF);  do_read(8'h18);
    do_write(8'h1C, 32'h55555555, 4'hF);  do_read(8'h1C);

    // byte-strobe tests on one word
    do_write(8'h20, 32'h00000000, 4'hF);
    do_write(8'h20, 32'h11223344, 4'b0001); do_read(8'h20);
    do_write(8'h20, 32'h55667788, 4'b0010); do_read(8'h20);
    do_write(8'h20, 32'h99AABBCC, 4'b0100); do_read(8'h20);
    do_write(8'h20, 32'hDDEEFF00, 4'b1000); do_read(8'h20);
    do_write(8'h20, 32'h12345678, 4'b0000); do_read(8'h20);   // no byte written
    do_write(8'h20, 32'hA1B2C3D4, 4'b0110); do_read(8'h20);

    // read of never-written location (memory resets to 0)
    do_read(8'h80);

    // overwrite
    do_write(8'h30, 32'h1111_1111, 4'hF);
    do_write(8'h30, 32'h2222_2222, 4'hF);
    do_read (8'h30);

    `uvm_info("SEQ", "Directed sequence finished", UVM_LOW)
  endtask
endclass

// ---------------------------------------------------------------- write then read back
class axi_write_read_seq extends axi_base_seq;
  `uvm_object_utils(axi_write_read_seq)

  int unsigned num = 50;

  function new(string name = "axi_write_read_seq");
    super.new(name);
  endfunction

  task body();
    axi_item t;
    `uvm_info("SEQ", $sformatf("Write/read-back sequence: %0d iterations", num), UVM_LOW)
    repeat (num) begin
      t = axi_item::type_id::create("t");
      start_item(t);
      if (!t.randomize() with { write == 1; })
        `uvm_error("SEQ", "randomize failed")
      finish_item(t);
      do_read(t.addr);
    end
  endtask
endclass

// ---------------------------------------------------------------- random
class axi_random_seq extends axi_base_seq;
  `uvm_object_utils(axi_random_seq)

  int unsigned num = 300;

  function new(string name = "axi_random_seq");
    super.new(name);
  endfunction

  task body();
    `uvm_info("SEQ", $sformatf("Random sequence: %0d transactions", num), UVM_LOW)
    repeat (num) do_random();
  endtask
endclass

//============================================================================
// axi_stall_seq : creates back-pressure so that A2, A3, A4 are really checked
//   - write: 2nd AW/W is sent while BVALID is pending (BREADY low)
//            -> AWREADY/WREADY low while AWVALID/WVALID high   (A2, A3)
//   - read : 2nd AR is sent while RVALID is pending (RREADY low)
//            -> ARREADY low while ARVALID high                  (A4)
//============================================================================
class axi_stall_seq extends axi_base_seq;
  `uvm_object_utils(axi_stall_seq)

  virtual axi_if vif;

  function new(string name = "axi_stall_seq");
    super.new(name);
  endfunction

  task send_aw(bit [7:0] a);
    vif.drv_cb.AWADDR  <= a;
    vif.drv_cb.AWPROT  <= 3'b000;
    vif.drv_cb.AWVALID <= 1'b1;
    @(vif.drv_cb);
    while (!vif.drv_cb.AWREADY) @(vif.drv_cb);
    vif.drv_cb.AWVALID <= 1'b0;
  endtask

  task send_w(bit [31:0] d, bit [3:0] s);
    vif.drv_cb.WDATA  <= d;
    vif.drv_cb.WSTRB  <= s;
    vif.drv_cb.WVALID <= 1'b1;
    @(vif.drv_cb);
    while (!vif.drv_cb.WREADY) @(vif.drv_cb);
    vif.drv_cb.WVALID <= 1'b0;
  endtask

  task send_ar(bit [7:0] a);
    vif.drv_cb.ARADDR  <= a;
    vif.drv_cb.ARPROT  <= 3'b000;
    vif.drv_cb.ARVALID <= 1'b1;
    @(vif.drv_cb);
    while (!vif.drv_cb.ARREADY) @(vif.drv_cb);
    vif.drv_cb.ARVALID <= 1'b0;
  endtask

  // two back-to-back writes, response of the first one is delayed
  task stall_write(bit [7:0] a1, bit [31:0] d1, bit [7:0] a2, bit [31:0] d2);
    @(vif.drv_cb);
    fork
      send_aw(a1);
      send_w(d1, 4'hF);
    join                                   // write 1 accepted, BREADY still low
    @(vif.drv_cb);
    fork
      send_aw(a2);                         // waits: AWREADY is low
      send_w(d2, 4'hF);                    // waits: WREADY is low
      begin
        repeat (4) @(vif.drv_cb);
        vif.drv_cb.BREADY <= 1'b1;         // now accept response of write 1
      end
    join
    @(vif.drv_cb);
    while (!vif.drv_cb.BVALID) @(vif.drv_cb);   // response of write 2
    vif.drv_cb.BREADY <= 1'b0;
  endtask

  // two back-to-back reads, response of the first one is delayed
  task stall_read(bit [7:0] a1, bit [7:0] a2);
    @(vif.drv_cb);
    send_ar(a1);                           // RREADY still low
    @(vif.drv_cb);
    fork
      send_ar(a2);                         // waits: ARREADY is low
      begin
        repeat (4) @(vif.drv_cb);
        vif.drv_cb.RREADY <= 1'b1;         // now accept response of read 1
      end
    join
    @(vif.drv_cb);
    while (!vif.drv_cb.RVALID) @(vif.drv_cb);   // response of read 2
    vif.drv_cb.RREADY <= 1'b0;
  endtask

  task body();
    if (!uvm_config_db#(virtual axi_if)::get(m_sequencer, "", "vif", vif))
      `uvm_fatal("NOVIF", "vif not found in axi_stall_seq")
    `uvm_info("SEQ", "Stall sequence started", UVM_LOW)

    stall_write(8'h40, 32'hA5A5A5A5, 8'h44, 32'h5A5A5A5A);
    stall_read (8'h40, 8'h44);

    stall_write(8'h50, 32'h12345678, 8'h54, 32'h9ABCDEF0);
    stall_read (8'h50, 8'h54);

    stall_write(8'hF8, 32'hCAFEBABE, 8'hFC, 32'hDEADC0DE);
    stall_read (8'hF8, 8'hFC);

    `uvm_info("SEQ", "Stall sequence finished", UVM_LOW)
  endtask
endclass

//============================================================================
// axi_driver.svh : AXI4-Lite master driver (one transaction at a time)
//============================================================================
class axi_driver extends uvm_driver #(axi_item);
  `uvm_component_utils(axi_driver)

  virtual axi_if vif;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual axi_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface 'vif' not set for driver")
  endfunction

  task run_phase(uvm_phase phase);
    init_signals();
    @(posedge vif.ARESETn);          // wait for reset release
    @(vif.drv_cb);                   // align to a clock edge
    forever begin
      seq_item_port.get_next_item(req);
      if (req.write) drive_write(req);
      else           drive_read(req);
      seq_item_port.item_done();
    end
  endtask

  task init_signals();
    vif.AWADDR  <= '0;  vif.AWPROT <= '0;  vif.AWVALID <= 1'b0;
    vif.WDATA   <= '0;  vif.WSTRB  <= '0;  vif.WVALID  <= 1'b0;
    vif.BREADY  <= 1'b0;
    vif.ARADDR  <= '0;  vif.ARPROT <= '0;  vif.ARVALID <= 1'b0;
    vif.RREADY  <= 1'b0;
  endtask

  // ------------------------------------------------------------- write
  task drive_write(axi_item t);
    fork
      // write address channel
      begin
        repeat (t.aw_delay) @(vif.drv_cb);
        vif.drv_cb.AWADDR  <= t.addr;
        vif.drv_cb.AWPROT  <= 3'b000;
        vif.drv_cb.AWVALID <= 1'b1;
        @(vif.drv_cb);
        while (!vif.drv_cb.AWREADY) @(vif.drv_cb);
        vif.drv_cb.AWVALID <= 1'b0;
      end
      // write data channel
      begin
        repeat (t.w_delay) @(vif.drv_cb);
        vif.drv_cb.WDATA  <= t.data;
        vif.drv_cb.WSTRB  <= t.strb;
        vif.drv_cb.WVALID <= 1'b1;
        @(vif.drv_cb);
        while (!vif.drv_cb.WREADY) @(vif.drv_cb);
        vif.drv_cb.WVALID <= 1'b0;
      end
    join
    // write response channel
    repeat (t.b_delay) @(vif.drv_cb);
    vif.drv_cb.BREADY <= 1'b1;
    @(vif.drv_cb);
    while (!vif.drv_cb.BVALID) @(vif.drv_cb);
    t.resp = vif.drv_cb.BRESP;
    vif.drv_cb.BREADY <= 1'b0;
  endtask

  // ------------------------------------------------------------- read
  task drive_read(axi_item t);
    // read address channel
    repeat (t.ar_delay) @(vif.drv_cb);
    vif.drv_cb.ARADDR  <= t.addr;
    vif.drv_cb.ARPROT  <= 3'b000;
    vif.drv_cb.ARVALID <= 1'b1;
    @(vif.drv_cb);
    while (!vif.drv_cb.ARREADY) @(vif.drv_cb);
    vif.drv_cb.ARVALID <= 1'b0;
    // read data channel
    repeat (t.r_delay) @(vif.drv_cb);
    vif.drv_cb.RREADY <= 1'b1;
    @(vif.drv_cb);
    while (!vif.drv_cb.RVALID) @(vif.drv_cb);
    t.data = vif.drv_cb.RDATA;
    t.resp = vif.drv_cb.RRESP;
    vif.drv_cb.RREADY <= 1'b0;
  endtask

endclass

//============================================================================
// axi_monitor.svh : passive monitor; rebuilds write/read transactions
//                   from channel handshakes and publishes them
//============================================================================
class axi_monitor extends uvm_monitor;
  `uvm_component_utils(axi_monitor)

  virtual axi_if vif;
  uvm_analysis_port #(axi_item) ap;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual axi_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface 'vif' not set for monitor")
  endfunction

  task run_phase(uvm_phase phase);
    fork
      collect_writes();
      collect_reads();
    join
  endtask

  // ------------------------------------------------------------- writes
  task collect_writes();
    bit [7:0]  a;
    bit [31:0] d;
    bit [3:0]  s;
    axi_item   t;
    forever begin
      @(vif.mon_cb);
      if (vif.ARESETn === 1'b1) begin
        if (vif.mon_cb.AWVALID && vif.mon_cb.AWREADY) a = vif.mon_cb.AWADDR;
        if (vif.mon_cb.WVALID  && vif.mon_cb.WREADY) begin
          d = vif.mon_cb.WDATA;
          s = vif.mon_cb.WSTRB;
        end
        if (vif.mon_cb.BVALID && vif.mon_cb.BREADY) begin
          t = axi_item::type_id::create("mon_wr");
          t.write = 1'b1;
          t.addr  = a;
          t.data  = d;
          t.strb  = s;
          t.resp  = vif.mon_cb.BRESP;
          `uvm_info("MON", {"Observed ", t.convert2string()}, UVM_HIGH)
          ap.write(t);
        end
      end
    end
  endtask

  // ------------------------------------------------------------- reads
  task collect_reads();
    bit [7:0] a;
    axi_item  t;
    forever begin
      @(vif.mon_cb);
      if (vif.ARESETn === 1'b1) begin
        if (vif.mon_cb.ARVALID && vif.mon_cb.ARREADY) a = vif.mon_cb.ARADDR;
        if (vif.mon_cb.RVALID && vif.mon_cb.RREADY) begin
          t = axi_item::type_id::create("mon_rd");
          t.write = 1'b0;
          t.addr  = a;
          t.data  = vif.mon_cb.RDATA;
          t.strb  = 4'h0;
          t.resp  = vif.mon_cb.RRESP;
          `uvm_info("MON", {"Observed ", t.convert2string()}, UVM_HIGH)
          ap.write(t);
        end
      end
    end
  endtask

endclass

//============================================================================
// axi_agent.svh : sequencer + driver + monitor
//============================================================================
typedef uvm_sequencer #(axi_item) axi_sequencer;

class axi_agent extends uvm_agent;
  `uvm_component_utils(axi_agent)

  axi_sequencer sqr;
  axi_driver    drv;
  axi_monitor   mon;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    mon = axi_monitor::type_id::create("mon", this);
    if (get_is_active() == UVM_ACTIVE) begin
      sqr = axi_sequencer::type_id::create("sqr", this);
      drv = axi_driver::type_id::create("drv", this);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (get_is_active() == UVM_ACTIVE)
      drv.seq_item_port.connect(sqr.seq_item_export);
  endfunction

endclass

//============================================================================
// axi_scoreboard.svh : reference memory model; checks every read and response
//============================================================================
class axi_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(axi_scoreboard)

  uvm_analysis_imp #(axi_item, axi_scoreboard) imp;

  bit [31:0] ref_mem [64];     // 256 bytes, initialised to 0 (same as the DUT)

  int unsigned wr_cnt;
  int unsigned rd_cnt;
  int unsigned pass_cnt;
  int unsigned err_cnt;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    imp = new("imp", this);
  endfunction

  function void write(axi_item t);
    int unsigned idx;
    idx = t.addr[7:2];

    if (t.resp !== 2'b00) begin
      err_cnt++;
      `uvm_error("SCB", $sformatf("Unexpected response %0d for %s", t.resp, t.convert2string()))
    end

    if (t.write) begin
      wr_cnt++;
      for (int b = 0; b < 4; b++)
        if (t.strb[b]) ref_mem[idx][8*b +: 8] = t.data[8*b +: 8];
      `uvm_info("SCB", $sformatf("WRITE addr=0x%02h strb=%04b data=0x%08h -> model=0x%08h",
                                 t.addr, t.strb, t.data, ref_mem[idx]), UVM_MEDIUM)
    end else begin
      rd_cnt++;
      if (t.data === ref_mem[idx]) begin
        pass_cnt++;
        `uvm_info("SCB", $sformatf("READ  addr=0x%02h data=0x%08h MATCH", t.addr, t.data), UVM_MEDIUM)
      end else begin
        err_cnt++;
        `uvm_error("SCB", $sformatf("READ  addr=0x%02h MISMATCH got=0x%08h expected=0x%08h",
                                    t.addr, t.data, ref_mem[idx]))
      end
    end
  endfunction

  function void report_phase(uvm_phase phase);
    super.report_phase(phase);
    `uvm_info("SCB", "================ SCOREBOARD SUMMARY ================", UVM_NONE)
    `uvm_info("SCB", $sformatf("Writes=%0d  Reads=%0d  Read matches=%0d  Errors=%0d",
                               wr_cnt, rd_cnt, pass_cnt, err_cnt), UVM_NONE)
    if (err_cnt == 0 && rd_cnt > 0)
      `uvm_info("SCB", "*** TEST PASSED ***", UVM_NONE)
    else
      `uvm_error("SCB", "*** TEST FAILED ***")
  endfunction

endclass

//============================================================================
// axi_coverage.svh : functional coverage collector (UVM subscriber)
//============================================================================
class axi_coverage extends uvm_subscriber #(axi_item);
  `uvm_component_utils(axi_coverage)

  bit        write_f;
  bit [5:0]  word_f;
  bit [3:0]  strb_f;
  bit [31:0] data_f;
  bit [1:0]  resp_f;

  covergroup cg;
    option.per_instance = 1;

    cp_type : coverpoint write_f {
      bins read_op  = {0};
      bins write_op = {1};
    }

    cp_addr : coverpoint word_f {
      bins first_word = {0};
      bins low        = {[1:15]};
      bins mid        = {[16:47]};
      bins high       = {[48:62]};
      bins last_word  = {63};
    }

    cp_strb : coverpoint strb_f iff (write_f) {
      bins single[] = {4'b0001, 4'b0010, 4'b0100, 4'b1000};
      bins pairs[]  = {4'b0011, 4'b0101, 4'b0110, 4'b1001, 4'b1010, 4'b1100};
      bins triples[]= {4'b0111, 4'b1011, 4'b1101, 4'b1110};
      bins full     = {4'b1111};
      bins none     = {4'b0000};
    }

    cp_data : coverpoint data_f iff (write_f) {
      bins zeros    = {32'h00000000};
      bins ones     = {32'hFFFFFFFF};
      bins alt_aa   = {32'hAAAAAAAA};
      bins alt_55   = {32'h55555555};
      bins others   = default;
    }

    cp_resp : coverpoint resp_f {
      bins okay = {2'b00};
    }

    x_type_addr : cross cp_type, cp_addr;
    x_addr_strb : cross cp_addr, cp_strb;
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg = new();
  endfunction

  function void write(axi_item t);
    write_f = t.write;
    word_f  = t.addr[7:2];
    strb_f  = t.strb;
    data_f  = t.data;
    resp_f  = t.resp;
    cg.sample();
  endfunction

  function void report_phase(uvm_phase phase);
    super.report_phase(phase);
    `uvm_info("COV", $sformatf("Functional coverage (cg) = %0.2f %%", cg.get_inst_coverage()), UVM_NONE)
  endfunction

endclass

//============================================================================
// axi_env.svh : agent + scoreboard + coverage collector
//============================================================================
class axi_env extends uvm_env;
  `uvm_component_utils(axi_env)

  axi_agent      agent;
  axi_scoreboard scb;
  axi_coverage   cov;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent = axi_agent::type_id::create("agent", this);
    scb   = axi_scoreboard::type_id::create("scb", this);
    cov   = axi_coverage::type_id::create("cov", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    agent.mon.ap.connect(scb.imp);
    agent.mon.ap.connect(cov.analysis_export);
  endfunction

endclass

//============================================================================
// axi_tests.svh : base test, directed test, random test
//   select with  +UVM_TESTNAME=axi_directed_test | axi_random_test
//============================================================================
class axi_base_test extends uvm_test;
  `uvm_component_utils(axi_base_test)

  axi_env env;

  function new(string name = "axi_base_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = axi_env::type_id::create("env", this);
  endfunction

  function void end_of_elaboration_phase(uvm_phase phase);
    super.end_of_elaboration_phase(phase);
    uvm_top.print_topology();
  endfunction

endclass

// ---------------------------------------------------------------- directed
class axi_directed_test extends axi_base_test;
  `uvm_component_utils(axi_directed_test)

  function new(string name = "axi_directed_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    axi_directed_seq seq;
    phase.raise_objection(this);
    seq = axi_directed_seq::type_id::create("seq");
    seq.start(env.agent.sqr);
    #200ns;
    phase.drop_objection(this);
  endtask

endclass

// ---------------------------------------------------------------- random
class axi_random_test extends axi_base_test;
  `uvm_component_utils(axi_random_test)

  function new(string name = "axi_random_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    axi_directed_seq   dseq;
    axi_write_read_seq wrseq;
    axi_random_seq     rseq;
    phase.raise_objection(this);

    dseq  = axi_directed_seq::type_id::create("dseq");
    dseq.start(env.agent.sqr);

    wrseq = axi_write_read_seq::type_id::create("wrseq");
    wrseq.num = 60;
    wrseq.start(env.agent.sqr);

    rseq  = axi_random_seq::type_id::create("rseq");
    rseq.num = 400;
    rseq.start(env.agent.sqr);

    #200ns;
    phase.drop_objection(this);
  endtask

endclass

// ---------------------------------------------------------------- full test
//   directed + write/read-back + random + stall  (use this one for final results)
class axi_full_test extends axi_base_test;
  `uvm_component_utils(axi_full_test)

  function new(string name = "axi_full_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    axi_directed_seq   dseq;
    axi_write_read_seq wrseq;
    axi_random_seq     rseq;
    axi_stall_seq      sseq;
    phase.raise_objection(this);

    dseq = axi_directed_seq::type_id::create("dseq");
    dseq.start(env.agent.sqr);

    sseq = axi_stall_seq::type_id::create("sseq");
    sseq.start(env.agent.sqr);

    wrseq = axi_write_read_seq::type_id::create("wrseq");
    wrseq.num = 60;
    wrseq.start(env.agent.sqr);

    rseq = axi_random_seq::type_id::create("rseq");
    rseq.num = 400;
    rseq.start(env.agent.sqr);

    #200ns;
    phase.drop_objection(this);
  endtask

endclass

endpackage

//============================================================================
// tb_top.sv : top testbench - clock, reset, interface, DUT, run_test()
//============================================================================

module tb_top;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import axi_pkg::*;

  localparam DATA_WIDTH = 32;
  localparam ADDR_WIDTH = 8;

  logic ACLK    = 1'b0;
  logic ARESETn = 1'b0;

  // 100 MHz clock
  always #5 ACLK = ~ACLK;

  // reset: low for 5 clocks
  initial begin
    ARESETn = 1'b0;
    repeat (5) @(posedge ACLK);
    #2 ARESETn = 1'b1;
  end

  axi_if #(DATA_WIDTH, ADDR_WIDTH) axi_vif (.ACLK(ACLK), .ARESETn(ARESETn));

  // Attach the assertion module to the DUT (bind placed inside a module so
  // that Questa elaborates it without needing -cuname)
  bind axi4lite_slave axi_sva #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_axi_sva (.*);

  axi4lite_slave #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) dut (
    .ACLK    (ACLK),
    .ARESETn (ARESETn),
    .AWADDR  (axi_vif.AWADDR),
    .AWPROT  (axi_vif.AWPROT),
    .AWVALID (axi_vif.AWVALID),
    .AWREADY (axi_vif.AWREADY),
    .WDATA   (axi_vif.WDATA),
    .WSTRB   (axi_vif.WSTRB),
    .WVALID  (axi_vif.WVALID),
    .WREADY  (axi_vif.WREADY),
    .BRESP   (axi_vif.BRESP),
    .BVALID  (axi_vif.BVALID),
    .BREADY  (axi_vif.BREADY),
    .ARADDR  (axi_vif.ARADDR),
    .ARPROT  (axi_vif.ARPROT),
    .ARVALID (axi_vif.ARVALID),
    .ARREADY (axi_vif.ARREADY),
    .RDATA   (axi_vif.RDATA),
    .RRESP   (axi_vif.RRESP),
    .RVALID  (axi_vif.RVALID),
    .RREADY  (axi_vif.RREADY)
  );

  initial begin
    uvm_config_db#(virtual axi_if)::set(null, "*", "vif", axi_vif);
    run_test();
  end

endmodule