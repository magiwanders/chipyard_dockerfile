`default_nettype none

module serv_top
  #(parameter WITH_CSR = 1,
    parameter PRE_REGISTER = 1,
    parameter RESET_STRATEGY = "MINI",
    parameter RESET_PC = 32'd0,
    parameter [0:0] MDU = 1'b0,
    parameter [0:0] COMPRESSED=0,
    parameter [0:0] ALIGN = COMPRESSED)
   (
   input wire 		      clk,
   input wire 		      i_rst,
   input wire 		      i_timer_irq,























   //RF Interface
   output wire 		      o_rf_rreq,
   output wire 		      o_rf_wreq,
   input wire 		      i_rf_ready,
   output wire [4+WITH_CSR:0] o_wreg0,
   output wire [4+WITH_CSR:0] o_wreg1,
   output wire 		      o_wen0,
   output wire 		      o_wen1,
   output wire 		      o_wdata0,
   output wire 		      o_wdata1,
   output wire [4+WITH_CSR:0] o_rreg0,
   output wire [4+WITH_CSR:0] o_rreg1,
   input wire 		      i_rdata0,
   input wire 		      i_rdata1,

   output wire [31:0] 	      o_ibus_adr,
   output wire 		      o_ibus_cyc,
   input wire [31:0] 	      i_ibus_rdt,
   input wire 		      i_ibus_ack,
   output wire [31:0] 	      o_dbus_adr,
   output wire [31:0] 	      o_dbus_dat,
   output wire [3:0] 	      o_dbus_sel,
   output wire 		      o_dbus_we ,
   output wire 		      o_dbus_cyc,
   input wire [31:0] 	      i_dbus_rdt,
   input wire 		      i_dbus_ack,
   //Extension
   output wire [ 2:0] o_ext_funct3,
   input  wire        i_ext_ready,
   input wire  [31:0] i_ext_rd,
   output wire [31:0] o_ext_rs1,
   output wire [31:0] o_ext_rs2,
   //MDU
   output wire        o_mdu_valid);

   wire [4:0]    rd_addr;
   wire [4:0]    rs1_addr;
   wire [4:0]    rs2_addr;

   wire [3:0] 	 immdec_ctrl;
   wire [3:0] 	immdec_en;

   wire          sh_right;
   wire 	 bne_or_bge;
   wire 	 cond_branch;
   wire 	 two_stage_op;
   wire 	 e_op;
   wire 	 ebreak;
   wire 	 branch_op;
   wire 	 shift_op;
   wire 	 slt_or_branch;
   wire 	 rd_op;
   wire   mdu_op;

   wire 	 rd_alu_en;
   wire 	 rd_csr_en;
   wire 	 rd_mem_en;
   wire          ctrl_rd;
   wire          alu_rd;
   wire          mem_rd;
   wire          csr_rd;
   wire 	 mtval_pc;

   wire          ctrl_pc_en;
   wire          jump;
   wire          jal_or_jalr;
   wire          utype;
   wire 	 mret;
   wire          imm;
   wire 	 trap;
   wire 	 pc_rel;
   wire          iscomp;

   wire          init;
   wire          cnt_en;
   wire 	 cnt0to3;
   wire 	 cnt12to31;
   wire          cnt0;
   wire          cnt1;
   wire          cnt2;
   wire          cnt3;
   wire          cnt7;

   wire 	 cnt_done;

   wire 	 bufreg_en;
   wire          bufreg_sh_signed;
   wire 	 bufreg_rs1_en;
   wire 	 bufreg_imm_en;
   wire 	 bufreg_clr_lsb;
   wire 	 bufreg_q;
   wire 	 bufreg2_q;
   wire [31:0] dbus_rdt;
   wire        dbus_ack;

   wire          alu_sub;
   wire [1:0] 	 alu_bool_op;
   wire          alu_cmp_eq;
   wire          alu_cmp_sig;
   wire          alu_cmp;
   wire [2:0]    alu_rd_sel;

   wire          rs1;
   wire          rs2;
   wire          rd_en;

   wire          op_b;
   wire          op_b_sel;

   wire          mem_signed;
   wire          mem_word;
   wire          mem_half;
   wire [1:0] 	 mem_bytecnt;
   wire 	 sh_done;
   wire 	 sh_done_r;
   wire 	 byte_valid;

   wire 	 mem_misalign;

   wire 	 bad_pc;

   wire 	 csr_mstatus_en;
   wire 	 csr_mie_en;
   wire 	 csr_mcause_en;
   wire [1:0]	 csr_source;
   wire 	 csr_imm;
   wire 	 csr_d_sel;
   wire 	 csr_en;
   wire [1:0] 	 csr_addr;
   wire 	 csr_pc;
   wire 	 csr_imm_en;
   wire 	 csr_in;
   wire 	 rf_csr_out;
   wire 	 dbus_en;

   wire 	 new_irq;

   wire [1:0]   lsb;

   wire [31:0] i_wb_rdt;

   wire [31:0] wb_ibus_adr;
   wire        wb_ibus_cyc;
   wire [31:0] wb_ibus_rdt;
   wire        wb_ibus_ack;

   generate
      if (ALIGN) begin
         serv_aligner  align
           (
            .clk(clk),
            .rst(i_rst),
            // serv_rf_top
            .i_ibus_adr(wb_ibus_adr),
            .i_ibus_cyc(wb_ibus_cyc),
            .o_ibus_rdt(wb_ibus_rdt),
            .o_ibus_ack(wb_ibus_ack),
            // servant_arbiter
            .o_wb_ibus_adr(o_ibus_adr),
            .o_wb_ibus_cyc(o_ibus_cyc),
            .i_wb_ibus_rdt(i_ibus_rdt),
            .i_wb_ibus_ack(i_ibus_ack));
      end else begin
         assign  o_ibus_adr  = wb_ibus_adr;
         assign  o_ibus_cyc  = wb_ibus_cyc;
         assign  wb_ibus_rdt = i_ibus_rdt;
         assign  wb_ibus_ack = i_ibus_ack;
        end
   endgenerate

   generate 
      if (COMPRESSED) begin
         serv_compdec compdec
           (
            .i_clk(clk),
            .i_instr(wb_ibus_rdt),
            .i_ack(wb_ibus_ack),
            .o_instr(i_wb_rdt),
            .o_iscomp(iscomp));
      end else begin
         assign i_wb_rdt =  wb_ibus_rdt;
         assign iscomp   =  1'b0;
      end
   endgenerate

   serv_state
     #(.RESET_STRATEGY (RESET_STRATEGY),
       .WITH_CSR (WITH_CSR[0:0]),
       .MDU(MDU),
       .ALIGN(ALIGN))
   state
     (
      .i_clk (clk),
      .i_rst          (i_rst),
      //State
      .i_new_irq      (new_irq),
      .i_alu_cmp      (alu_cmp),
      .o_init         (init),
      .o_cnt_en       (cnt_en),
      .o_cnt0to3      (cnt0to3),
      .o_cnt12to31    (cnt12to31),
      .o_cnt0         (cnt0),
      .o_cnt1         (cnt1),
      .o_cnt2         (cnt2),
      .o_cnt3         (cnt3),
      .o_cnt7         (cnt7),
      .o_cnt_done     (cnt_done),
      .o_bufreg_en    (bufreg_en),
      .o_ctrl_pc_en   (ctrl_pc_en),
      .o_ctrl_jump    (jump),
      .o_ctrl_trap    (trap),
      .i_ctrl_misalign(lsb[1]),
      .i_sh_done      (sh_done),
      .i_sh_done_r    (sh_done_r),
      .o_mem_bytecnt  (mem_bytecnt),
      .i_mem_misalign (mem_misalign),
      //Control
      .i_bne_or_bge   (bne_or_bge),
      .i_cond_branch  (cond_branch),
      .i_dbus_en      (dbus_en),
      .i_two_stage_op (two_stage_op),
      .i_branch_op    (branch_op),
      .i_shift_op     (shift_op),
      .i_sh_right     (sh_right),
      .i_slt_or_branch (slt_or_branch),
      .i_e_op         (e_op),
      .i_rd_op        (rd_op),
      //MDU
      .i_mdu_op       (mdu_op),
      .o_mdu_valid    (o_mdu_valid),
      //Extension
      .i_mdu_ready    (i_ext_ready),
      //External
      .o_dbus_cyc     (o_dbus_cyc),
      .i_dbus_ack     (i_dbus_ack),
      .o_ibus_cyc     (wb_ibus_cyc),
      .i_ibus_ack     (wb_ibus_ack),
      //RF Interface
      .o_rf_rreq      (o_rf_rreq),
      .o_rf_wreq      (o_rf_wreq),
      .i_rf_ready     (i_rf_ready),
      .o_rf_rd_en     (rd_en));

   serv_decode
     #(.PRE_REGISTER (PRE_REGISTER),
       .MDU(MDU))
   decode
     (
      .clk (clk),
      //Input
      .i_wb_rdt           (i_wb_rdt[31:2]),
      .i_wb_en            (wb_ibus_ack),
      //To state
      .o_bne_or_bge       (bne_or_bge),
      .o_cond_branch      (cond_branch),
      .o_dbus_en          (dbus_en),
      .o_e_op             (e_op),
      .o_ebreak           (ebreak),
      .o_branch_op        (branch_op),
      .o_shift_op         (shift_op),
      .o_slt_or_branch    (slt_or_branch),
      .o_rd_op            (rd_op),
      .o_sh_right         (sh_right),
      .o_mdu_op           (mdu_op),
      .o_two_stage_op     (two_stage_op),
      //Extension
      .o_ext_funct3       (o_ext_funct3),

      //To bufreg
      .o_bufreg_rs1_en    (bufreg_rs1_en),
      .o_bufreg_imm_en    (bufreg_imm_en),
      .o_bufreg_clr_lsb   (bufreg_clr_lsb),
      .o_bufreg_sh_signed (bufreg_sh_signed),
      //To bufreg2
      .o_op_b_source      (op_b_sel),
      //To ctrl
      .o_ctrl_jal_or_jalr (jal_or_jalr),
      .o_ctrl_utype       (utype),
      .o_ctrl_pc_rel      (pc_rel),
      .o_ctrl_mret        (mret),
      //To alu
      .o_alu_sub          (alu_sub),
      .o_alu_bool_op      (alu_bool_op),
      .o_alu_cmp_eq       (alu_cmp_eq),
      .o_alu_cmp_sig      (alu_cmp_sig),
      .o_alu_rd_sel       (alu_rd_sel),
      //To mem IF
      .o_mem_cmd          (o_dbus_we),
      .o_mem_signed       (mem_signed),
      .o_mem_word         (mem_word),
      .o_mem_half         (mem_half),
      //To CSR
      .o_csr_en           (csr_en),
      .o_csr_addr         (csr_addr),
      .o_csr_mstatus_en   (csr_mstatus_en),
      .o_csr_mie_en       (csr_mie_en),
      .o_csr_mcause_en    (csr_mcause_en),
      .o_csr_source       (csr_source),
      .o_csr_d_sel        (csr_d_sel),
      .o_csr_imm_en       (csr_imm_en),
      .o_mtval_pc         (mtval_pc      ),
      //To top
      .o_immdec_ctrl      (immdec_ctrl),
      .o_immdec_en        (immdec_en),
      //To RF IF
      .o_rd_mem_en        (rd_mem_en),
      .o_rd_csr_en        (rd_csr_en),
      .o_rd_alu_en        (rd_alu_en));

   serv_immdec immdec
     (
      .i_clk        (clk),
      //State
      .i_cnt_en     (cnt_en),
      .i_cnt_done   (cnt_done),
      //Control
      .i_immdec_en        (immdec_en),
      .i_csr_imm_en (csr_imm_en),
      .i_ctrl       (immdec_ctrl),
      .o_rd_addr    (rd_addr),
      .o_rs1_addr   (rs1_addr),
      .o_rs2_addr   (rs2_addr),
      //Data
      .o_csr_imm    (csr_imm),
      .o_imm        (imm),
      //External
      .i_wb_en      (wb_ibus_ack),
      .i_wb_rdt     (i_wb_rdt[31:7]));

   serv_bufreg
      #(.MDU(MDU))
   bufreg
     (
      .i_clk    (clk),
      //State
      .i_cnt0   (cnt0),
      .i_cnt1   (cnt1),
      .i_en     (bufreg_en),
      .i_init   (init),
      .i_mdu_op (mdu_op),
      .o_lsb    (lsb),
      //Control
      .i_sh_signed (bufreg_sh_signed),
      .i_rs1_en    (bufreg_rs1_en),
      .i_imm_en    (bufreg_imm_en),
      .i_clr_lsb   (bufreg_clr_lsb),
      //Data
      .i_rs1    (rs1),
      .i_imm    (imm),
      .o_q      (bufreg_q),
      //External
      .o_dbus_adr (o_dbus_adr),
      .o_ext_rs1  (o_ext_rs1));

   serv_bufreg2 bufreg2
     (
      .i_clk        (clk),
      //State
      .i_en         (cnt_en),
      .i_init       (init),
      .i_cnt_done   (cnt_done),
      .i_lsb        (lsb),
      .i_byte_valid (byte_valid),
      .o_sh_done    (sh_done),
      .o_sh_done_r  (sh_done_r),
      //Control
      .i_op_b_sel   (op_b_sel),
      .i_shift_op   (shift_op),
      //Data
      .i_rs2        (rs2),
      .i_imm        (imm),
      .o_op_b       (op_b),
      .o_q          (bufreg2_q),
      //External
      .o_dat        (o_dbus_dat),
      .i_load       (dbus_ack),
      .i_dat        (dbus_rdt));

   serv_ctrl
     #(.RESET_PC (RESET_PC),
       .RESET_STRATEGY (RESET_STRATEGY),
       .WITH_CSR (WITH_CSR))
   ctrl
     (
      .clk        (clk),
      .i_rst      (i_rst),
      //State
      .i_pc_en    (ctrl_pc_en),
      .i_cnt12to31 (cnt12to31),
      .i_cnt0     (cnt0),
      .i_cnt1     (cnt1),
      .i_cnt2     (cnt2),
      //Control
      .i_jump     (jump),
      .i_jal_or_jalr (jal_or_jalr),
      .i_utype    (utype),
      .i_pc_rel   (pc_rel),
      .i_trap     (trap | mret),
      .i_iscomp    (iscomp),
      //Data
      .i_imm      (imm),
      .i_buf      (bufreg_q),
      .i_csr_pc   (csr_pc),
      .o_rd       (ctrl_rd),
      .o_bad_pc   (bad_pc),
      //External
      .o_ibus_adr (wb_ibus_adr));

   serv_alu alu
     (
      .clk        (clk),
      //State
      .i_en       (cnt_en),
      .i_cnt0     (cnt0),
      .o_cmp      (alu_cmp),
      //Control
      .i_sub      (alu_sub),
      .i_bool_op  (alu_bool_op),
      .i_cmp_eq   (alu_cmp_eq),
      .i_cmp_sig  (alu_cmp_sig),
      .i_rd_sel   (alu_rd_sel),
      //Data
      .i_rs1      (rs1),
      .i_op_b     (op_b),
      .i_buf      (bufreg_q),
      .o_rd       (alu_rd));

   serv_rf_if
     #(.WITH_CSR (WITH_CSR))
   rf_if
     (//RF interface
      .i_cnt_en    (cnt_en),
      .o_wreg0     (o_wreg0),
      .o_wreg1     (o_wreg1),
      .o_wen0      (o_wen0),
      .o_wen1      (o_wen1),
      .o_wdata0    (o_wdata0),
      .o_wdata1    (o_wdata1),
      .o_rreg0     (o_rreg0),
      .o_rreg1     (o_rreg1),
      .i_rdata0    (i_rdata0),
      .i_rdata1    (i_rdata1),

      //Trap interface
      .i_trap      (trap),
      .i_mret      (mret),
      .i_mepc      (wb_ibus_adr[0]),
      .i_mtval_pc  (mtval_pc),
      .i_bufreg_q  (bufreg_q),
      .i_bad_pc    (bad_pc),
      .o_csr_pc    (csr_pc),
      //CSR write port
      .i_csr_en    (csr_en),
      .i_csr_addr  (csr_addr),
      .i_csr       (csr_in),
      //RD write port
      .i_rd_wen    (rd_en),
      .i_rd_waddr  (rd_addr),
      .i_ctrl_rd   (ctrl_rd),
      .i_alu_rd    (alu_rd),
      .i_rd_alu_en (rd_alu_en),
      .i_csr_rd    (csr_rd),
      .i_rd_csr_en (rd_csr_en),
      .i_mem_rd    (mem_rd),
      .i_rd_mem_en (rd_mem_en),

      //RS1 read port
      .i_rs1_raddr (rs1_addr),
      .o_rs1       (rs1),
      //RS2 read port
      .i_rs2_raddr (rs2_addr),
      .o_rs2       (rs2),

      //CSR read port
      .o_csr       (rf_csr_out));

   serv_mem_if
     #(.WITH_CSR (WITH_CSR[0:0]))
   mem_if
     (
      .i_clk        (clk),
      //State
      .i_bytecnt    (mem_bytecnt),
      .i_lsb        (lsb),
      .o_byte_valid (byte_valid),
      .o_misalign   (mem_misalign),
      //Control
      .i_mdu_op     (mdu_op),
      .i_signed     (mem_signed),
      .i_word       (mem_word),
      .i_half       (mem_half),
      //Data
      .i_bufreg2_q  (bufreg2_q),
      .o_rd         (mem_rd),
      //External interface
      .o_wb_sel     (o_dbus_sel));

   generate
      if (|WITH_CSR) begin
	 serv_csr
	   #(.RESET_STRATEGY (RESET_STRATEGY))
	 csr
	   (
	    .i_clk        (clk),
	    .i_rst        (i_rst),
	    //State
	    .i_init       (init),
	    .i_en         (cnt_en),
	    .i_cnt0to3    (cnt0to3),
	    .i_cnt3       (cnt3),
	    .i_cnt7       (cnt7),
	    .i_cnt_done   (cnt_done),
	    .i_mem_op     (!mtval_pc),
	    .i_mtip       (i_timer_irq),
	    .i_trap       (trap),
	    .o_new_irq    (new_irq),
	    //Control
	    .i_e_op       (e_op),
	    .i_ebreak     (ebreak),
	    .i_mem_cmd    (o_dbus_we),
	    .i_mstatus_en (csr_mstatus_en),
	    .i_mie_en     (csr_mie_en    ),
	    .i_mcause_en  (csr_mcause_en ),
	    .i_csr_source (csr_source),
	    .i_mret       (mret),
	    .i_csr_d_sel  (csr_d_sel),
	    //Data
	    .i_rf_csr_out (rf_csr_out),
	    .o_csr_in     (csr_in),
	    .i_csr_imm    (csr_imm),
	    .i_rs1        (rs1),
	    .o_q          (csr_rd));
      end else begin
	 assign csr_in = 1'b0;
	 assign csr_rd = 1'b0;
	 assign new_irq = 1'b0;
      end
   endgenerate







































































generate
  if (MDU) begin
    assign dbus_rdt = i_ext_ready ? i_ext_rd:i_dbus_rdt;
    assign dbus_ack = i_dbus_ack | i_ext_ready;
  end else begin
    assign dbus_rdt = i_dbus_rdt;
    assign dbus_ack = i_dbus_ack;
  end
  assign o_ext_rs2 = o_dbus_dat;
endgenerate

endmodule
`default_nettype wire

`default_nettype none
module serv_alu
  (
   input wire 	    clk,
   //State
   input wire 	    i_en,
   input wire 	    i_cnt0,
   output wire 	    o_cmp,
   //Control
   input wire 	    i_sub,
   input wire [1:0] i_bool_op,
   input wire 	    i_cmp_eq,
   input wire 	    i_cmp_sig,
   input wire [2:0] i_rd_sel,
   //Data
   input wire 	    i_rs1,
   input wire 	    i_op_b,
   input wire 	    i_buf,
   output wire 	    o_rd);

   wire        result_add;

   reg 	       cmp_r;

   wire        add_cy;
   reg 	       add_cy_r;

   //Sign-extended operands
   wire rs1_sx  = i_rs1 & i_cmp_sig;
   wire op_b_sx = i_op_b  & i_cmp_sig;

   wire add_b = i_op_b^i_sub;

   assign {add_cy,result_add}   = i_rs1+add_b+add_cy_r;

   wire result_lt = rs1_sx + ~op_b_sx + add_cy;

   wire result_eq = !result_add & (cmp_r | i_cnt0);

   assign o_cmp = i_cmp_eq ? result_eq : result_lt;

   /*
    The result_bool expression implements the following operations between
    i_rs1 and i_op_b depending on the value of i_bool_op

    00 xor
    01 0
    10 or
    11 and

    i_bool_op will be 01 during shift operations, so by outputting zero under
    this condition we can safely or result_bool with i_buf
    */
   wire result_bool = ((i_rs1 ^ i_op_b) & ~ i_bool_op[0]) | (i_bool_op[1] & i_op_b & i_rs1);

   assign o_rd = i_buf |
                 (i_rd_sel[0] & result_add) |
                 (i_rd_sel[1] & cmp_r & i_cnt0) |
                 (i_rd_sel[2] & result_bool);

   always @(posedge clk) begin
      add_cy_r <= i_en ? add_cy : i_sub;

      if (i_en)
	cmp_r <= o_cmp;
   end

endmodule

module serv_bufreg #(
      parameter [0:0] MDU = 0
)(
   input wire 	      i_clk,
   //State
   input wire 	      i_cnt0,
   input wire 	      i_cnt1,
   input wire 	      i_en,
   input wire 	      i_init,
   input wire           i_mdu_op,
   output wire [1:0]    o_lsb,
   //Control
   input wire 	      i_rs1_en,
   input wire 	      i_imm_en,
   input wire 	      i_clr_lsb,
   input wire 	      i_sh_signed, 
   //Data
   input wire 	      i_rs1,
   input wire 	      i_imm,
   output wire 	      o_q,
   //External
   output wire [31:0] o_dbus_adr,
   //Extension
   output wire [31:0] o_ext_rs1);

   wire 	      c, q;
   reg 		      c_r;
   reg [31:2] 	      data;
   reg [1:0]            lsb;

   wire 	      clr_lsb = i_cnt0 & i_clr_lsb;

   assign {c,q} = {1'b0,(i_rs1 & i_rs1_en)} + {1'b0,(i_imm & i_imm_en & !clr_lsb)} + c_r;

   always @(posedge i_clk) begin
      //Make sure carry is cleared before loading new data
      c_r <= c & i_en;

      if (i_en)
	data <= {i_init ? q : (data[31] & i_sh_signed), data[31:3]};

      if (i_init ? (i_cnt0 | i_cnt1) : i_en)
	lsb <= {i_init ? q : data[2],lsb[1]};
   end

   assign o_q = lsb[0] & i_en;
   assign o_dbus_adr = {data, 2'b00};
   assign o_ext_rs1  = {o_dbus_adr[31:2],lsb};
   assign o_lsb = (MDU & i_mdu_op) ? 2'b00 : lsb;

endmodule

module serv_bufreg2
  (
   input wire 	      i_clk,
   //State
   input wire 	      i_en,
   input wire 	      i_init,
   input wire 	      i_cnt_done,
   input wire [1:0]   i_lsb,
   input wire 	      i_byte_valid,
   output wire 	      o_sh_done,
   output wire 	      o_sh_done_r,
   //Control
   input wire 	      i_op_b_sel,
   input wire 	      i_shift_op,
   //Data
   input wire 	      i_rs2,
   input wire 	      i_imm,
   output wire 	      o_op_b,
   output wire 	      o_q,
   //External
   output wire [31:0] o_dat,
   input wire 	      i_load,
   input wire [31:0]  i_dat);

   reg [31:0] 	 dat;

   assign o_op_b = i_op_b_sel ? i_rs2 : i_imm;

   wire 	 dat_en = i_shift_op | (i_en & i_byte_valid);

   /* The dat register has three different use cases for store, load and
    shift operations.
    store : Data to be written is shifted to the correct position in dat during
            init by dat_en and is presented on the data bus as o_wb_dat
    load  : Data from the bus gets latched into dat during i_wb_ack and is then
            shifted out at the appropriate time to end up in the correct
            position in rd
    shift : Data is shifted in during init. After that, the six LSB are used as
            a downcounter (with bit 5 initially set to 0) that triggers
            o_sh_done and o_sh_done_r when they wrap around to indicate that
            the requested number of shifts have been performed
    */
   wire [5:0] dat_shamt = (i_shift_op & !i_init) ?
	      //Down counter mode
	      dat[5:0]-1 :
	      //Shift reg mode with optional clearing of bit 5
	      {dat[6] & !(i_shift_op & i_cnt_done),dat[5:1]};

   assign o_sh_done = dat_shamt[5];
   assign o_sh_done_r = dat[5];

   assign o_q =
	       ((i_lsb == 2'd3) & dat[24]) |
	       ((i_lsb == 2'd2) & dat[16]) |
	       ((i_lsb == 2'd1) & dat[8]) |
	       ((i_lsb == 2'd0) & dat[0]);

   assign o_dat = dat;

   always @(posedge i_clk) begin
      if (dat_en | i_load)
	dat <= i_load ? i_dat : {o_op_b, dat[31:7], dat_shamt};
   end

endmodule

`default_nettype none
module serv_csr
  #(parameter RESET_STRATEGY = "MINI")
  (
   input wire 	    i_clk,
   input wire 	    i_rst,
   //State
   input wire 	    i_init,
   input wire 	    i_en,
   input wire 	    i_cnt0to3,
   input wire 	    i_cnt3,
   input wire 	    i_cnt7,
   input wire 	    i_cnt_done,
   input wire 	    i_mem_op,
   input wire 	    i_mtip,
   input wire 	    i_trap,
   output reg 	    o_new_irq,
   //Control
   input wire 	    i_e_op,
   input wire 	    i_ebreak,
   input wire 	    i_mem_cmd,
   input wire 	    i_mstatus_en,
   input wire 	    i_mie_en,
   input wire 	    i_mcause_en,
   input wire [1:0] i_csr_source,
   input wire 	    i_mret,
   input wire 	    i_csr_d_sel,
   //Data
   input wire 	    i_rf_csr_out,
   output wire 	    o_csr_in,
   input wire 	    i_csr_imm,
   input wire 	    i_rs1,
   output wire 	    o_q);

   localparam [1:0]
     CSR_SOURCE_CSR = 2'b00,
     CSR_SOURCE_EXT = 2'b01,
     CSR_SOURCE_SET = 2'b10,
     CSR_SOURCE_CLR = 2'b11;

   reg 		    mstatus_mie;
   reg 		    mstatus_mpie;
   reg 		    mie_mtie;

   reg 		mcause31;
   reg [3:0] 	mcause3_0;
   wire 	mcause;

   wire 	csr_in;
   wire 	csr_out;

   reg 		timer_irq_r;

   wire 	d = i_csr_d_sel ? i_csr_imm : i_rs1;

   assign csr_in = (i_csr_source == CSR_SOURCE_EXT) ? d :
		   (i_csr_source == CSR_SOURCE_SET) ? csr_out | d :
		   (i_csr_source == CSR_SOURCE_CLR) ? csr_out & ~d :
		   (i_csr_source == CSR_SOURCE_CSR) ? csr_out :
		   1'bx;

   assign csr_out = (i_mstatus_en & mstatus_mie & i_cnt3) |
		    i_rf_csr_out |
		    (i_mcause_en & i_en & mcause);

   assign o_q = csr_out;

   wire 	timer_irq = i_mtip & mstatus_mie & mie_mtie;

   assign mcause = i_cnt0to3 ? mcause3_0[0] : //[3:0]
		   i_cnt_done ? mcause31 //[31]
		   : 1'b0;

   assign o_csr_in = csr_in;

   always @(posedge i_clk) begin
      if (!i_init & i_cnt_done) begin
	 timer_irq_r <= timer_irq;
	 o_new_irq   <= timer_irq & !timer_irq_r;
      end

      if (i_mie_en & i_cnt7)
	mie_mtie <= csr_in;

      /*
       The mie bit in mstatus gets updated under three conditions

       When a trap is taken, the bit is cleared
       During an mret instruction, the bit is restored from mpie
       During a mstatus CSR access instruction it's assigned when
        bit 3 gets updated

       These conditions are all mutually exclusibe
       */
      if ((i_trap & i_cnt_done) | i_mstatus_en & i_cnt3 | i_mret)
	mstatus_mie <= !i_trap & (i_mret ?  mstatus_mpie : csr_in);

      /*
       Note: To save resources mstatus_mpie (mstatus bit 7) is not
       readable or writable from sw
       */
      if (i_trap & i_cnt_done)
	mstatus_mpie <= mstatus_mie;

      /*
       The four lowest bits in mcause hold the exception code

       These bits get updated under three conditions

       During an mcause CSR access function, they are assigned when
       bits 0 to 3 gets updated

       During an external interrupt the exception code is set to
       7, since SERV only support timer interrupts

       During an exception, the exception code is assigned to indicate
       if it was caused by an ebreak instruction (3),
       ecall instruction (11), misaligned load (4), misaligned store (6)
       or misaligned jump (0)

       The expressions below are derived from the following truth table
       irq  => 0111 (timer=7)
       e_op => x011 (ebreak=3, ecall=11)
       mem  => 01x0 (store=6, load=4)
       ctrl => 0000 (jump=0)
       */
      if (i_mcause_en & i_en & i_cnt0to3 | (i_trap & i_cnt_done)) begin
	 mcause3_0[3] <= (i_e_op & !i_ebreak) | (!i_trap & csr_in);
	 mcause3_0[2] <= o_new_irq | i_mem_op | (!i_trap & mcause3_0[3]);
	 mcause3_0[1] <= o_new_irq | i_e_op | (i_mem_op & i_mem_cmd) | (!i_trap & mcause3_0[2]);
	 mcause3_0[0] <= o_new_irq | i_e_op | (!i_trap & mcause3_0[1]);
      end
      if (i_mcause_en & i_cnt_done | i_trap)
	mcause31 <= i_trap ? o_new_irq : csr_in;
      if (i_rst)
	if (RESET_STRATEGY != "NONE") begin
	   o_new_irq <= 1'b0;
	   mie_mtie <= 1'b0;
	end
   end

endmodule

`default_nettype none
module serv_ctrl
  #(parameter RESET_STRATEGY = "MINI",
    parameter RESET_PC = 32'd0,
    parameter WITH_CSR = 1)
  (
   input wire 	     clk,
   input wire 	     i_rst,
   //State
   input wire 	     i_pc_en,
   input wire 	     i_cnt12to31,
   input wire 	     i_cnt0,
   input wire        i_cnt1,
   input wire 	     i_cnt2,
   //Control
   input wire 	     i_jump,
   input wire 	     i_jal_or_jalr,
   input wire 	     i_utype,
   input wire 	     i_pc_rel,
   input wire 	     i_trap,
   input wire        i_iscomp,
   //Data
   input wire 	     i_imm,
   input wire 	     i_buf,
   input wire 	     i_csr_pc,
   output wire 	     o_rd,
   output wire 	     o_bad_pc,
   //External
   output reg [31:0] o_ibus_adr);

   wire       pc_plus_4;
   wire       pc_plus_4_cy;
   reg 	      pc_plus_4_cy_r;
   wire       pc_plus_offset;
   wire       pc_plus_offset_cy;
   reg 	      pc_plus_offset_cy_r;
   wire       pc_plus_offset_aligned;
   wire       plus_4;

   wire       pc = o_ibus_adr[0];

   wire       new_pc;

   wire       offset_a;
   wire       offset_b;

  /*  If i_iscomp=1: increment pc by 2 else increment pc by 4  */

   assign plus_4        = i_iscomp ? i_cnt1 : i_cnt2;

   assign o_bad_pc = pc_plus_offset_aligned;

   assign {pc_plus_4_cy,pc_plus_4} = pc+plus_4+pc_plus_4_cy_r;

   generate
      if (|WITH_CSR)
	assign new_pc = i_trap ? (i_csr_pc & !i_cnt0) : i_jump ? pc_plus_offset_aligned : pc_plus_4;
      else
	assign new_pc = i_jump ? pc_plus_offset_aligned : pc_plus_4;
   endgenerate
   assign o_rd  = (i_utype & pc_plus_offset_aligned) | (pc_plus_4 & i_jal_or_jalr);

   assign offset_a = i_pc_rel & pc;
   assign offset_b = i_utype ? (i_imm & i_cnt12to31): i_buf;
   assign {pc_plus_offset_cy,pc_plus_offset} = offset_a+offset_b+pc_plus_offset_cy_r;

   assign pc_plus_offset_aligned = pc_plus_offset & !i_cnt0;

   initial if (RESET_STRATEGY == "NONE") o_ibus_adr = RESET_PC;

   always @(posedge clk) begin
      pc_plus_4_cy_r <= i_pc_en & pc_plus_4_cy;
      pc_plus_offset_cy_r <= i_pc_en & pc_plus_offset_cy;

      if (RESET_STRATEGY == "NONE") begin
	 if (i_pc_en)
	   o_ibus_adr <= {new_pc, o_ibus_adr[31:1]};
      end else begin
	 if (i_pc_en | i_rst)
	   o_ibus_adr <= i_rst ? RESET_PC : {new_pc, o_ibus_adr[31:1]};
      end
   end
endmodule

`default_nettype none
module serv_decode
  #(parameter [0:0] PRE_REGISTER = 1,
    parameter [0:0] MDU = 0)
  (
   input wire        clk,
   //Input
   input wire [31:2] i_wb_rdt,
   input wire        i_wb_en,
   //To state
   output reg       o_sh_right,
   output reg       o_bne_or_bge,
   output reg       o_cond_branch,
   output reg       o_e_op,
   output reg       o_ebreak,
   output reg       o_branch_op,
   output reg       o_shift_op,
   output reg       o_slt_or_branch,
   output reg       o_rd_op,
   output reg       o_two_stage_op,
   output reg       o_dbus_en,
   //MDU
   output reg       o_mdu_op,
   //Extension
   output reg [2:0] o_ext_funct3,
   //To bufreg
   output reg       o_bufreg_rs1_en,
   output reg       o_bufreg_imm_en,
   output reg       o_bufreg_clr_lsb,
   output reg       o_bufreg_sh_signed,
   //To ctrl
   output reg       o_ctrl_jal_or_jalr,
   output reg       o_ctrl_utype,
   output reg       o_ctrl_pc_rel,
   output reg       o_ctrl_mret,
   //To alu
   output reg       o_alu_sub,
   output reg [1:0] o_alu_bool_op,
   output reg       o_alu_cmp_eq,
   output reg       o_alu_cmp_sig,
   output reg [2:0] o_alu_rd_sel,
   //To mem IF
   output reg       o_mem_signed,
   output reg       o_mem_word,
   output reg       o_mem_half,
   output reg       o_mem_cmd,
   //To CSR
   output reg       o_csr_en,
   output reg [1:0] o_csr_addr,
   output reg       o_csr_mstatus_en,
   output reg       o_csr_mie_en,
   output reg       o_csr_mcause_en,
   output reg [1:0] o_csr_source,
   output reg       o_csr_d_sel,
   output reg       o_csr_imm_en,
   output reg       o_mtval_pc,
   //To top
   output reg [3:0] o_immdec_ctrl,
   output reg [3:0] o_immdec_en,
   output reg       o_op_b_source,
   //To RF IF
   output reg       o_rd_mem_en,
   output reg       o_rd_csr_en,
   output reg       o_rd_alu_en);

   reg [4:0] opcode;
   reg [2:0] funct3;
   reg        op20;
   reg        op21;
   reg        op22;
   reg        op26;

   reg       imm25;
   reg       imm30;

   wire co_mdu_op     = MDU & (opcode == 5'b01100) & imm25;

   wire co_two_stage_op =
	~opcode[2] | (funct3[0] & ~funct3[1] & ~opcode[0] & ~opcode[4]) |
	(funct3[1] & ~funct3[2] & ~opcode[0] & ~opcode[4]) | co_mdu_op;
   wire co_shift_op = (opcode[2] & ~funct3[1]) & !co_mdu_op;
   wire co_slt_or_branch = (opcode[4] | (funct3[1] & opcode[2]) | (imm30 & opcode[2] & opcode[3] & ~funct3[2])) & !co_mdu_op;
   wire co_branch_op = opcode[4];
   wire co_dbus_en    = ~opcode[2] & ~opcode[4];
   wire co_mtval_pc   = opcode[4];   
   wire co_mem_word   = funct3[1];
   wire co_rd_alu_en  = !opcode[0] & opcode[2] & !opcode[4] & !co_mdu_op;
   wire co_rd_mem_en  = (!opcode[2] & !opcode[0]) | co_mdu_op;
   wire [2:0] co_ext_funct3 = funct3;

   //jal,branch =     imm
   //jalr       = rs1+imm
   //mem        = rs1+imm
   //shift      = rs1
   wire co_bufreg_rs1_en = !opcode[4] | (!opcode[1] & opcode[0]);
   wire co_bufreg_imm_en = !opcode[2];

   //Clear LSB of immediate for BRANCH and JAL ops
   //True for BRANCH and JAL
   //False for JALR/LOAD/STORE/OP/OPIMM?
   wire co_bufreg_clr_lsb = opcode[4] & ((opcode[1:0] == 2'b00) | (opcode[1:0] == 2'b11));

   //Conditional branch
   //True for BRANCH
   //False for JAL/JALR
   wire co_cond_branch = !opcode[0];

   wire co_ctrl_utype       = !opcode[4] & opcode[2] & opcode[0];
   wire co_ctrl_jal_or_jalr = opcode[4] & opcode[0];

   //PC-relative operations
   //True for jal, b* auipc, ebreak
   //False for jalr, lui
   wire co_ctrl_pc_rel = (opcode[2:0] == 3'b000)  |
                          (opcode[1:0] == 2'b11)  |
                          (opcode[4] & opcode[2]) & op20|
                          (opcode[4:3] == 2'b00);
   //Write to RD
   //True for OP-IMM, AUIPC, OP, LUI, SYSTEM, JALR, JAL, LOAD
   //False for STORE, BRANCH, MISC-MEM
   wire co_rd_op = (opcode[2] |
                     (!opcode[2] & opcode[4] & opcode[0]) |
                     (!opcode[2] & !opcode[3] & !opcode[0]));

   //
   //funct3
   //

   wire co_sh_right   = funct3[2];
   wire co_bne_or_bge = funct3[0];

   //Matches system ops except eceall/ebreak/mret
   wire csr_op = opcode[4] & opcode[2] & (|funct3);


   //op20
   wire co_ebreak = op20;


   //opcode & funct3 & op21

   wire co_ctrl_mret = opcode[4] & opcode[2] & op21 & !(|funct3);
   //Matches system opcodes except CSR accesses (funct3 == 0)
   //and mret (!op21)
   wire co_e_op = opcode[4] & opcode[2] & !op21 & !(|funct3);

   //opcode & funct3 & imm30

   wire co_bufreg_sh_signed = imm30;

   /*
    True for sub, b*, slt*
    False for add*
    op    opcode f3  i30
    b*    11000  xxx x   t
    addi  00100  000 x   f
    slt*  0x100  01x x   t
    add   01100  000 0   f
    sub   01100  000 1   t
    */
   wire co_alu_sub = funct3[1] | funct3[0] | (opcode[3] & imm30) | opcode[4];

   /*
    Bits 26, 22, 21 and 20 are enough to uniquely identify the eight supported CSR regs
    mtvec, mscratch, mepc and mtval are stored externally (normally in the RF) and are
    treated differently from mstatus, mie and mcause which are stored in serv_csr.

    The former get a 2-bit address as seen below while the latter get a
    one-hot enable signal each.

    Hex|2 222|Reg     |csr
    adr|6 210|name    |addr
    ---|-----|--------|----
    300|0_000|mstatus | xx
    304|0_100|mie     | xx
    305|0_101|mtvec   | 01
    340|1_000|mscratch| 00
    341|1_001|mepc    | 10
    342|1_010|mcause  | xx
    343|1_011|mtval   | 11

    */

   //true  for mtvec,mscratch,mepc and mtval
   //false for mstatus, mie, mcause
   wire csr_valid = op20 | (op26 & !op21);

   wire co_rd_csr_en = csr_op;

   wire co_csr_en         = csr_op & csr_valid;
   wire co_csr_mstatus_en = csr_op & !op26 & !op22;
   wire co_csr_mie_en     = csr_op & !op26 &  op22 & !op20;
   wire co_csr_mcause_en  = csr_op         &  op21 & !op20;

   wire [1:0] co_csr_source = funct3[1:0];
   wire co_csr_d_sel = funct3[2];
   wire co_csr_imm_en = opcode[4] & opcode[2] & funct3[2];
   wire [1:0] co_csr_addr = {op26 & op20, !op26 | op21};

   wire co_alu_cmp_eq = funct3[2:1] == 2'b00;

   wire co_alu_cmp_sig = ~((funct3[0] & funct3[1]) | (funct3[1] & funct3[2]));

   wire co_mem_cmd  = opcode[3];
   wire co_mem_signed = ~funct3[2];
   wire co_mem_half   = funct3[0];

   wire [1:0] co_alu_bool_op = funct3[1:0];

   wire [3:0] co_immdec_ctrl;
   //True for S (STORE) or B (BRANCH) type instructions
   //False for J type instructions
   assign co_immdec_ctrl[0] = opcode[3:0] == 4'b1000;
   //True for OP-IMM, LOAD, STORE, JALR  (I S)
   //False for LUI, AUIPC, JAL           (U J)
   assign co_immdec_ctrl[1] = (opcode[1:0] == 2'b00) | (opcode[2:1] == 2'b00);
   assign co_immdec_ctrl[2] = opcode[4] & !opcode[0];
   assign co_immdec_ctrl[3] = opcode[4];

   wire [3:0] co_immdec_en;
   assign co_immdec_en[3] = opcode[4] | opcode[3] | opcode[2] | !opcode[0];                 //B I J S U
   assign co_immdec_en[2] = (opcode[4] & opcode[2]) | !opcode[3] | opcode[0];               //  I J   U
   assign co_immdec_en[1] = (opcode[2:1] == 2'b01) | (opcode[2] & opcode[0]) | co_csr_imm_en;//    J   U
   assign co_immdec_en[0] = ~co_rd_op;                                                       //B     S

   wire [2:0] co_alu_rd_sel;
   assign co_alu_rd_sel[0] = (funct3 == 3'b000); // Add/sub
   assign co_alu_rd_sel[1] = (funct3[2:1] == 2'b01); //SLT*
   assign co_alu_rd_sel[2] = funct3[2]; //Bool

   //0 (OP_B_SOURCE_IMM) when OPIMM
   //1 (OP_B_SOURCE_RS2) when BRANCH or OP
   wire co_op_b_source = opcode[3];

   generate
      if (PRE_REGISTER) begin

         always @(posedge clk) begin
            if (i_wb_en) begin
               funct3 <= i_wb_rdt[14:12];
               imm30  <= i_wb_rdt[30];
               imm25  <= i_wb_rdt[25];
               opcode <= i_wb_rdt[6:2];
               op20   <= i_wb_rdt[20];
               op21   <= i_wb_rdt[21];
               op22   <= i_wb_rdt[22];
               op26   <= i_wb_rdt[26];
            end
         end

         always @(*) begin
            o_sh_right         = co_sh_right;
            o_bne_or_bge       = co_bne_or_bge;
            o_cond_branch      = co_cond_branch;
            o_dbus_en          = co_dbus_en;
            o_mtval_pc         = co_mtval_pc;
	    o_two_stage_op     = co_two_stage_op;
            o_e_op             = co_e_op;
            o_ebreak           = co_ebreak;
            o_branch_op        = co_branch_op;
            o_shift_op         = co_shift_op;
            o_slt_or_branch    = co_slt_or_branch;
            o_rd_op            = co_rd_op;
            o_mdu_op           = co_mdu_op;
            o_ext_funct3       = co_ext_funct3;
            o_bufreg_rs1_en    = co_bufreg_rs1_en;
            o_bufreg_imm_en    = co_bufreg_imm_en;
            o_bufreg_clr_lsb   = co_bufreg_clr_lsb;
            o_bufreg_sh_signed = co_bufreg_sh_signed;
            o_ctrl_jal_or_jalr = co_ctrl_jal_or_jalr;
            o_ctrl_utype       = co_ctrl_utype;
            o_ctrl_pc_rel      = co_ctrl_pc_rel;
            o_ctrl_mret        = co_ctrl_mret;
            o_alu_sub          = co_alu_sub;
            o_alu_bool_op      = co_alu_bool_op;
            o_alu_cmp_eq       = co_alu_cmp_eq;
            o_alu_cmp_sig      = co_alu_cmp_sig;
            o_alu_rd_sel       = co_alu_rd_sel;
            o_mem_signed       = co_mem_signed;
            o_mem_word         = co_mem_word;
            o_mem_half         = co_mem_half;
            o_mem_cmd          = co_mem_cmd;
            o_csr_en           = co_csr_en;
            o_csr_addr         = co_csr_addr;
            o_csr_mstatus_en   = co_csr_mstatus_en;
            o_csr_mie_en       = co_csr_mie_en;
            o_csr_mcause_en    = co_csr_mcause_en;
            o_csr_source       = co_csr_source;
            o_csr_d_sel        = co_csr_d_sel;
            o_csr_imm_en       = co_csr_imm_en;
            o_immdec_ctrl      = co_immdec_ctrl;
            o_immdec_en        = co_immdec_en;
            o_op_b_source      = co_op_b_source;
            o_rd_csr_en        = co_rd_csr_en;
            o_rd_alu_en        = co_rd_alu_en;
            o_rd_mem_en        = co_rd_mem_en;
         end

      end else begin

         always @(*) begin
            funct3  = i_wb_rdt[14:12];
            imm30   = i_wb_rdt[30];
            imm25   = i_wb_rdt[25];
            opcode  = i_wb_rdt[6:2];
            op20    = i_wb_rdt[20];
            op21    = i_wb_rdt[21];
            op22    = i_wb_rdt[22];
            op26    = i_wb_rdt[26];
         end

         always @(posedge clk) begin
            if (i_wb_en) begin
               o_sh_right         <= co_sh_right;
               o_bne_or_bge       <= co_bne_or_bge;
               o_cond_branch      <= co_cond_branch;
               o_e_op             <= co_e_op;
               o_ebreak           <= co_ebreak;
               o_two_stage_op     <= co_two_stage_op;
               o_dbus_en          <= co_dbus_en;
               o_mtval_pc         <= co_mtval_pc;
               o_branch_op        <= co_branch_op;
               o_shift_op         <= co_shift_op;
               o_slt_or_branch    <= co_slt_or_branch;
               o_rd_op            <= co_rd_op;
               o_mdu_op           <= co_mdu_op;
               o_ext_funct3       <= co_ext_funct3;
               o_bufreg_rs1_en    <= co_bufreg_rs1_en;
               o_bufreg_imm_en    <= co_bufreg_imm_en;
               o_bufreg_clr_lsb   <= co_bufreg_clr_lsb;
               o_bufreg_sh_signed <= co_bufreg_sh_signed;
               o_ctrl_jal_or_jalr <= co_ctrl_jal_or_jalr;
               o_ctrl_utype       <= co_ctrl_utype;
               o_ctrl_pc_rel      <= co_ctrl_pc_rel;
               o_ctrl_mret        <= co_ctrl_mret;
               o_alu_sub          <= co_alu_sub;
               o_alu_bool_op      <= co_alu_bool_op;
               o_alu_cmp_eq       <= co_alu_cmp_eq;
               o_alu_cmp_sig      <= co_alu_cmp_sig;
               o_alu_rd_sel       <= co_alu_rd_sel;
               o_mem_signed       <= co_mem_signed;
               o_mem_word         <= co_mem_word;
               o_mem_half         <= co_mem_half;
               o_mem_cmd          <= co_mem_cmd;
               o_csr_en           <= co_csr_en;
               o_csr_addr         <= co_csr_addr;
               o_csr_mstatus_en   <= co_csr_mstatus_en;
               o_csr_mie_en       <= co_csr_mie_en;
               o_csr_mcause_en    <= co_csr_mcause_en;
               o_csr_source       <= co_csr_source;
               o_csr_d_sel        <= co_csr_d_sel;
               o_csr_imm_en       <= co_csr_imm_en;
               o_immdec_ctrl      <= co_immdec_ctrl;
               o_immdec_en        <= co_immdec_en;
               o_op_b_source      <= co_op_b_source;
               o_rd_csr_en        <= co_rd_csr_en;
               o_rd_alu_en        <= co_rd_alu_en;
               o_rd_mem_en        <= co_rd_mem_en;
            end
         end

      end
   endgenerate

endmodule

`default_nettype none
module serv_immdec
  #(parameter SHARED_RFADDR_IMM_REGS = 1)
  (
   input wire 	     i_clk,
   //State
   input wire 	     i_cnt_en,
   input wire 	     i_cnt_done,
   //Control
   input wire [3:0]  i_immdec_en,
   input wire 	     i_csr_imm_en,
   input wire [3:0]  i_ctrl,
   output wire [4:0] o_rd_addr,
   output wire [4:0] o_rs1_addr,
   output wire [4:0] o_rs2_addr,
   //Data
   output wire 	     o_csr_imm,
   output wire 	     o_imm,
   //External
   input wire 	     i_wb_en,
   input wire [31:7] i_wb_rdt);

   reg 		     imm31;

   reg [8:0]  imm19_12_20;
   reg 	      imm7;
   reg [5:0]  imm30_25;
   reg [4:0]  imm24_20;
   reg [4:0]  imm11_7;

   assign o_csr_imm = imm19_12_20[4];

   wire       signbit = imm31 & !i_csr_imm_en;

   generate
      if (SHARED_RFADDR_IMM_REGS) begin
	 assign o_rs1_addr = imm19_12_20[8:4];
	 assign o_rs2_addr = imm24_20;
	 assign o_rd_addr  = imm11_7;

	 always @(posedge i_clk) begin
	    if (i_wb_en) begin
	       /* CSR immediates are always zero-extended, hence clear the signbit */
	       imm31     <= i_wb_rdt[31];
	    end
	    if (i_wb_en | (i_cnt_en & i_immdec_en[1]))
	      imm19_12_20 <= i_wb_en ? {i_wb_rdt[19:12],i_wb_rdt[20]} : {i_ctrl[3] ? signbit : imm24_20[0], imm19_12_20[8:1]};
	    if (i_wb_en | (i_cnt_en))
	      imm7        <= i_wb_en ? i_wb_rdt[7]                    : signbit;

	    if (i_wb_en | (i_cnt_en & i_immdec_en[3]))
	      imm30_25    <= i_wb_en ? i_wb_rdt[30:25] : {i_ctrl[2] ? imm7 : i_ctrl[1] ? signbit : imm19_12_20[0], imm30_25[5:1]};

	    if (i_wb_en | (i_cnt_en & i_immdec_en[2]))
	      imm24_20    <= i_wb_en ? i_wb_rdt[24:20] : {imm30_25[0], imm24_20[4:1]};

	    if (i_wb_en | (i_cnt_en & i_immdec_en[0]))
	      imm11_7     <= i_wb_en ? i_wb_rdt[11:7] : {imm30_25[0], imm11_7[4:1]};
	 end
      end else begin
	 reg [4:0]  rd_addr;
	 reg [4:0]  rs1_addr;
	 reg [4:0]  rs2_addr;

	 assign o_rd_addr  = rd_addr;
	 assign o_rs1_addr = rs1_addr;
	 assign o_rs2_addr = rs2_addr;
	 always @(posedge i_clk) begin
	    if (i_wb_en) begin
	       /* CSR immediates are always zero-extended, hence clear the signbit */
	       imm31       <= i_wb_rdt[31];
	       imm19_12_20 <= {i_wb_rdt[19:12],i_wb_rdt[20]};
	       imm7        <= i_wb_rdt[7];
	       imm30_25    <= i_wb_rdt[30:25];
	       imm24_20    <= i_wb_rdt[24:20];
	       imm11_7     <= i_wb_rdt[11:7];

               rd_addr  <= i_wb_rdt[11:7];
               rs1_addr <= i_wb_rdt[19:15];
               rs2_addr <= i_wb_rdt[24:20];
	    end
	    if (i_cnt_en) begin
	       imm19_12_20 <= {i_ctrl[3] ? signbit : imm24_20[0], imm19_12_20[8:1]};
	       imm7        <= signbit;
	       imm30_25    <= {i_ctrl[2] ? imm7 : i_ctrl[1] ? signbit : imm19_12_20[0], imm30_25[5:1]};
	       imm24_20    <= {imm30_25[0], imm24_20[4:1]};
	       imm11_7     <= {imm30_25[0], imm11_7[4:1]};
	    end
	 end
      end
   endgenerate

	 assign o_imm = i_cnt_done ? signbit : i_ctrl[0] ? imm11_7[0] : imm24_20[0];
	 
endmodule

`default_nettype none
module serv_mem_if
  #(parameter [0:0] WITH_CSR = 1)
  (
   input wire 	     i_clk,
   //State
   input wire [1:0]  i_bytecnt,
   input wire [1:0]  i_lsb,
   output wire 	     o_byte_valid,
   output wire 	     o_misalign,
   //Control
   input wire 	     i_signed,
   input wire 	     i_word,
   input wire 	     i_half,
   //MDU
   input wire 	     i_mdu_op,
   //Data
   input wire 	     i_bufreg2_q,
   output wire 	     o_rd,
   //External interface
   output wire [3:0] o_wb_sel);

   reg           signbit;

   /*
    Before a store operation, the data to be written needs to be shifted into
    place. Depending on the address alignment, we need to shift different
    amounts. One formula for calculating this is to say that we shift when
    i_lsb + i_bytecnt < 4. Unfortunately, the synthesis tools don't seem to be
    clever enough so the hideous expression below is used to achieve the same
    thing in a more optimal way.
    */
   assign o_byte_valid
     = (!i_lsb[0] & !i_lsb[1])         |
       (!i_bytecnt[0] & !i_bytecnt[1]) |
       (!i_bytecnt[1] & !i_lsb[1])     |
       (!i_bytecnt[1] & !i_lsb[0])     |
       (!i_bytecnt[0] & !i_lsb[1]);

   wire dat_valid =
	i_mdu_op |
	i_word |
	(i_bytecnt == 2'b00) |
	(i_half & !i_bytecnt[1]);

   assign o_rd = dat_valid ? i_bufreg2_q : signbit & i_signed;

   assign o_wb_sel[3] = (i_lsb == 2'b11) | i_word | (i_half & i_lsb[1]);
   assign o_wb_sel[2] = (i_lsb == 2'b10) | i_word;
   assign o_wb_sel[1] = (i_lsb == 2'b01) | i_word | (i_half & !i_lsb[1]);
   assign o_wb_sel[0] = (i_lsb == 2'b00);

   always @(posedge i_clk) begin
      if (dat_valid)
        signbit <= i_bufreg2_q;
   end

   /*
    mem_misalign is checked after the init stage to decide whether to do a data
    bus transaction or go to the trap state. It is only guaranteed to be correct
    at this time
    */
   assign o_misalign = WITH_CSR & ((i_lsb[0] & (i_word | i_half)) | (i_lsb[1] & i_word));

endmodule

`default_nettype none
module serv_rf_if
  #(parameter WITH_CSR = 1)
  (//RF Interface
   input wire 		      i_cnt_en,
   output wire [4+WITH_CSR:0] o_wreg0,
   output wire [4+WITH_CSR:0] o_wreg1,
   output wire 		      o_wen0,
   output wire 		      o_wen1,
   output wire 		      o_wdata0,
   output wire 		      o_wdata1,
   output wire [4+WITH_CSR:0] o_rreg0,
   output wire [4+WITH_CSR:0] o_rreg1,
   input wire 		      i_rdata0,
   input wire 		      i_rdata1,

   //Trap interface
   input wire 		      i_trap,
   input wire 		      i_mret,
   input wire 		      i_mepc,
   input wire 		      i_mtval_pc,
   input wire 		      i_bufreg_q,
   input wire 		      i_bad_pc,
   output wire 		      o_csr_pc,
   //CSR interface
   input wire 		      i_csr_en,
   input wire [1:0] 	      i_csr_addr,
   input wire 		      i_csr,
   output wire 		      o_csr,
   //RD write port
   input wire 		      i_rd_wen,
   input wire [4:0] 	      i_rd_waddr,
   input wire 		      i_ctrl_rd,
   input wire 		      i_alu_rd,
   input wire 		      i_rd_alu_en,
   input wire 		      i_csr_rd,
   input wire 		      i_rd_csr_en,
   input wire 		      i_mem_rd,
   input wire 		      i_rd_mem_en,

   //RS1 read port
   input wire [4:0] 	      i_rs1_raddr,
   output wire 		      o_rs1,
   //RS2 read port
   input wire [4:0] 	      i_rs2_raddr,
   output wire 		      o_rs2);


   /*
    ********** Write side ***********
    */

   wire 	     rd_wen = i_rd_wen & (|i_rd_waddr);

   generate
   if (|WITH_CSR) begin
   wire 	     rd = (i_ctrl_rd ) |
			  (i_alu_rd & i_rd_alu_en) |
			  (i_csr_rd & i_rd_csr_en) |
			  (i_mem_rd & i_rd_mem_en);

   wire 	     mtval = i_mtval_pc ? i_bad_pc : i_bufreg_q;

   assign 	     o_wdata0 = i_trap ? mtval  : rd;
   assign	     o_wdata1 = i_trap ? i_mepc : i_csr;

   /* Port 0 handles writes to mtval during traps and rd otherwise
    * Port 1 handles writes to mepc during traps and csr accesses otherwise
    *
    * GPR registers are mapped to address 0-31 (bits 0xxxxx).
    * Following that are four CSR registers
    * mscratch 100000
    * mtvec    100001
    * mepc     100010
    * mtval    100011
    */

   assign o_wreg0 = i_trap ? {6'b100011} : {1'b0,i_rd_waddr};
   assign o_wreg1 = i_trap ? {6'b100010} : {4'b1000,i_csr_addr};

   assign       o_wen0 = i_cnt_en & (i_trap | rd_wen);
   assign       o_wen1 = i_cnt_en & (i_trap | i_csr_en);

   /*
    ********** Read side ***********
    */

   //0 : RS1
   //1 : RS2 / CSR

   assign o_rreg0 = {1'b0, i_rs1_raddr};

   /*
    The address of the second read port (o_rreg1) can get assigned from four
    different sources

    Normal operations : i_rs2_raddr
    CSR access        : i_csr_addr
    trap              : MTVEC
    mret              : MEPC

    Address 0-31 in the RF are assigned to the GPRs. After that follows the four
    CSRs on addresses 32-35

    32 MSCRATCH
    33 MTVEC
    34 MEPC
    35 MTVAL

    The expression below is an optimized version of this logic
    */
   wire sel_rs2 = !(i_trap | i_mret | i_csr_en);
   assign o_rreg1 = {~sel_rs2,
		     i_rs2_raddr[4:2] & {3{sel_rs2}},
		     {1'b0,i_trap} | {i_mret,1'b0} | ({2{i_csr_en}} & i_csr_addr) | ({2{sel_rs2}} & i_rs2_raddr[1:0])};

   assign o_rs1 = i_rdata0;
   assign o_rs2 = i_rdata1;
   assign o_csr = i_rdata1 & i_csr_en;
   assign o_csr_pc = i_rdata1;

   end else begin
      wire 	     rd = (i_ctrl_rd ) |
			  (i_alu_rd & i_rd_alu_en) |
			  (i_mem_rd & i_rd_mem_en);

      assign 	     o_wdata0 = rd;
      assign	     o_wdata1 = 1'b0;

      assign o_wreg0 = i_rd_waddr;
      assign o_wreg1 = 5'd0;

      assign       o_wen0 = i_cnt_en & rd_wen;
      assign       o_wen1 = 1'b0;

   /*
    ********** Read side ***********
    */

      assign o_rreg0 = i_rs1_raddr;
      assign o_rreg1 = i_rs2_raddr;

      assign o_rs1 = i_rdata0;
      assign o_rs2 = i_rdata1;
      assign o_csr = 1'b0;
      assign o_csr_pc = 1'b0;
   end // else: !if(WITH_CSR)
   endgenerate
endmodule

module serv_state
  #(parameter RESET_STRATEGY = "MINI",
    parameter [0:0] WITH_CSR = 1,
    parameter [0:0] ALIGN =0,
    parameter [0:0] MDU = 0)
  (
   input wire 	     i_clk,
   input wire 	     i_rst,
   //State
   input wire 	     i_new_irq,
   input wire 	     i_alu_cmp,
   output wire 	     o_init,
   output wire 	     o_cnt_en,
   output wire 	     o_cnt0to3,
   output wire 	     o_cnt12to31,
   output wire 	     o_cnt0,
   output wire 	     o_cnt1,
   output wire 	     o_cnt2,
   output wire 	     o_cnt3,
   output wire 	     o_cnt7,
   output reg 	     o_cnt_done,
   output wire 	     o_bufreg_en,
   output wire 	     o_ctrl_pc_en,
   output reg 	     o_ctrl_jump,
   output wire 	     o_ctrl_trap,
   input wire 	     i_ctrl_misalign,
   input wire 	     i_sh_done,
   input wire 	     i_sh_done_r,
   output wire [1:0] o_mem_bytecnt,
   input wire 	     i_mem_misalign,
   //Control
   input wire 	     i_bne_or_bge,
   input wire 	     i_cond_branch,
   input wire 	     i_dbus_en,
   input wire 	     i_two_stage_op,
   input wire 	     i_branch_op,
   input wire 	     i_shift_op,
   input wire 	     i_sh_right,
   input wire 	     i_slt_or_branch,
   input wire 	     i_e_op,
   input wire 	     i_rd_op,
   //MDU
   input wire 	     i_mdu_op,
   output wire 	     o_mdu_valid,
   //Extension
   input wire 	     i_mdu_ready,
   //External
   output wire 	     o_dbus_cyc,
   input wire 	     i_dbus_ack,
   output wire 	     o_ibus_cyc,
   input wire 	     i_ibus_ack,
   //RF Interface
   output wire 	     o_rf_rreq,
   output wire 	     o_rf_wreq,
   input wire 	     i_rf_ready,
   output wire 	     o_rf_rd_en);

   reg 	stage_two_req;
   reg 	init_done;
   wire misalign_trap_sync;

   reg [4:2] o_cnt;
   reg [3:0] o_cnt_r;

   reg 	     ibus_cyc;
   //Update PC in RUN or TRAP states
   assign o_ctrl_pc_en  = o_cnt_en & !o_init;

   assign o_cnt_en = |o_cnt_r;

   assign o_mem_bytecnt = o_cnt[4:3];

   assign o_cnt0to3   = (o_cnt[4:2] == 3'd0);
   assign o_cnt12to31 = (o_cnt[4] | (o_cnt[3:2] == 2'b11));
   assign o_cnt0 = (o_cnt[4:2] == 3'd0) & o_cnt_r[0];
   assign o_cnt1 = (o_cnt[4:2] == 3'd0) & o_cnt_r[1];
   assign o_cnt2 = (o_cnt[4:2] == 3'd0) & o_cnt_r[2];
   assign o_cnt3 = (o_cnt[4:2] == 3'd0) & o_cnt_r[3];
   assign o_cnt7 = (o_cnt[4:2] == 3'd1) & o_cnt_r[3];

   //Take branch for jump or branch instructions (opcode == 1x0xx) if
   //a) It's an unconditional branch (opcode[0] == 1)
   //b) It's a conditional branch (opcode[0] == 0) of type beq,blt,bltu (funct3[0] == 0) and ALU compare is true
   //c) It's a conditional branch (opcode[0] == 0) of type bne,bge,bgeu (funct3[0] == 1) and ALU compare is false
   //Only valid during the last cycle of INIT, when the branch condition has
   //been calculated.
   wire      take_branch = i_branch_op & (!i_cond_branch | (i_alu_cmp^i_bne_or_bge));

   //valid signal for mdu
   assign o_mdu_valid = MDU & !o_cnt_en & init_done & i_mdu_op;

   //Prepare RF for writes when everything is ready to enter stage two
   // and the first stage didn't cause a misalign exception
   assign o_rf_wreq = !misalign_trap_sync & !o_cnt_en & init_done &
	   	      ((i_shift_op & (i_sh_done | !i_sh_right)) |
	   	       i_dbus_ack | (MDU & i_mdu_ready) |
	   	       i_slt_or_branch);

   assign o_dbus_cyc = !o_cnt_en & init_done & i_dbus_en & !i_mem_misalign;

   //Prepare RF for reads when a new instruction is fetched
   // or when stage one caused an exception (rreq implies a write request too)
   assign o_rf_rreq = i_ibus_ack | (stage_two_req & misalign_trap_sync);

   assign o_rf_rd_en = i_rd_op & !o_init;

   /*
    bufreg is used during mem. branch and shift operations

    mem : bufreg is used for dbus address. Shift in data during phase 1.
          Shift out during phase 2 if there was an misalignment exception.

    branch : Shift in during phase 1. Shift out during phase 2

    shift : Shift in during phase 1. Continue shifting between phases (except
            for the first cycle after init). Shift out during phase 2
    */
   assign o_bufreg_en = (o_cnt_en & (o_init | ((o_ctrl_trap | i_branch_op) & i_two_stage_op))) | (i_shift_op & !stage_two_req & (i_sh_right | i_sh_done_r) & init_done);

   assign o_ibus_cyc = ibus_cyc & !i_rst;

   assign o_init = i_two_stage_op & !i_new_irq & !init_done;

   always @(posedge i_clk) begin
      //ibus_cyc changes on three conditions.
      //1. i_rst is asserted. Together with the async gating above, o_ibus_cyc
      //   will be asserted as soon as the reset is released. This is how the
      //   first instruction is fetced
      //2. o_cnt_done and o_ctrl_pc_en are asserted. This means that SERV just
      //   finished updating the PC, is done with the current instruction and
      //   o_ibus_cyc gets asserted to fetch a new instruction
      //3. When i_ibus_ack, a new instruction is fetched and o_ibus_cyc gets
      //   deasserted to finish the transaction
      if (i_ibus_ack | o_cnt_done | i_rst)
	ibus_cyc <= o_ctrl_pc_en | i_rst;

      if (o_cnt_done) begin
	 init_done <= o_init & !init_done;
	 o_ctrl_jump <= o_init & take_branch;
      end
      o_cnt_done <= (o_cnt[4:2] == 3'b111) & o_cnt_r[2];

      //Need a strobe for the first cycle in the IDLE state after INIT
      stage_two_req <= o_cnt_done & o_init;

      /*
       Because SERV is 32-bit bit-serial we need a counter than can count 0-31
       to keep track of which bit we are currently processing. o_cnt and o_cnt_r
       are used together to create such a counter.
       The top three bits (o_cnt) are implemented as a normal counter, but
       instead of the two LSB, o_cnt_r is a 4-bit shift register which loops 0-3
       When o_cnt_r[3] is 1, o_cnt will be increased.

       The counting starts when the core is idle and the i_rf_ready signal
       comes in from the RF module by shifting in the i_rf_ready bit as LSB of
       the shift register. Counting is stopped by using o_cnt_done to block the
       bit that was supposed to be shifted into bit 0 of o_cnt_r.

       There are two benefit of doing the counter this way
       1. We only need to check four bits instead of five when we want to check
       if the counter is at a certain value. For 4-LUT architectures this means
       we only need one LUT instead of two for each comparison.
       2. We don't need a separate enable signal to turn on and off the counter
       between stages, which saves an extra FF and a unique control signal. We
       just need to check if o_cnt_r is not zero to see if the counter is
       currently running
       */
      o_cnt <= o_cnt + {2'd0,o_cnt_r[3]};
      o_cnt_r <= {o_cnt_r[2:0],(o_cnt_r[3] & !o_cnt_done) | (i_rf_ready & !o_cnt_en)};
      if (i_rst) begin
	 if (RESET_STRATEGY != "NONE") begin
	    o_cnt   <= 3'd0;
	    init_done <= 1'b0;
	    o_ctrl_jump <= 1'b0;
	    o_cnt_done <= 1'b0;
	    o_cnt_r <= 4'b0000;
	    stage_two_req <= 1'b0;
	 end
      end
   end

   assign o_ctrl_trap = WITH_CSR & (i_e_op | i_new_irq | misalign_trap_sync);

   generate
      if (WITH_CSR) begin
	 reg 	misalign_trap_sync_r;

	 //trap_pending is only guaranteed to have correct value during the
	 // last cycle of the init stage
	 wire trap_pending = WITH_CSR & ((take_branch & i_ctrl_misalign & !ALIGN) |
					 (i_dbus_en   & i_mem_misalign));

	 always @(posedge i_clk) begin
	    if (o_cnt_done)
	      misalign_trap_sync_r <= trap_pending & o_init;
	    if (i_rst)
	      if (RESET_STRATEGY != "NONE")
		misalign_trap_sync_r <= 1'b0;
	 end
	 assign misalign_trap_sync = misalign_trap_sync_r;
      end else
	assign misalign_trap_sync = 1'b0;
   endgenerate
endmodule

