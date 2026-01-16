`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/08/2026 06:38:19 PM
// Design Name: 
// Module Name: tb_alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////



interface alu_if(input logic clk
);
     logic [31:0] src1;
     logic [31:0] src2;
     logic [3:0]  op;
     logic [31:0] result;
     logic        zero;
     
     clocking ckb@(posedge clk);
        default input #1step output #2ns;
        input result, zero;
        output src1,src2;
        output op;
     endclocking
endinterface


class packet;
     rand bit [31:0] src1;
     rand bit [31:0] src2;
     rand bit [3:0]  op;
          bit [31:0] result;
          bit        zero;
          
     constraint op_limit { op inside  {[0:10]}; };
endclass


class generator;
     mailbox #(packet) gen2drv;  //Mail to send to driver
     int count;
     
     function new( mailbox #(packet) m, int c);
         this.gen2drv = m;
         this.count = c;
     endfunction
     task run();
        repeat(count) 
           begin
              packet pkt = new();
              assert(pkt.randomize());
              gen2drv.put(pkt);
              $display("Sending packet to DUT");
              $display("[GEN] src1=%0d src2=%0d op=%0d",
                     pkt.src1, pkt.src2, pkt.op);
           end
     endtask
endclass


class driver;
    mailbox #(packet) gen2drv;
    virtual alu_if vif;
    
    function new( mailbox #(packet) m , virtual alu_if v);
        this.gen2drv = m;
        this.vif = v;
    endfunction
     
    task run();
        packet pkt;
        forever 
        begin
            gen2drv.get(pkt);
            @(vif.ckb);
            vif.ckb.src1 <= pkt.src1;
            vif.ckb.src2 <= pkt.src2;
            vif.ckb.op   <= pkt.op;

            $display("[DRV] src1=%0d src2=%0d op=%0d",
                     pkt.src1, pkt.src2, pkt.op);
        end
    endtask
endclass


class monitor;
    mailbox #(packet) mon2scb;
    virtual alu_if vif;
    
    function new( mailbox #(packet) m, virtual alu_if v);
        this.mon2scb = m;
        this.vif = v;
    endfunction
    
    task run();
        packet pkt;
        forever
        begin
        @(vif.ckb);
        pkt = new();
        pkt.src1 = vif.src1;
        pkt.src2 = vif.src2;
        pkt.op   = vif.op;
        pkt.result = vif.result;
        pkt.zero  = vif.zero;

        mon2scb.put(pkt);

        $display("[MON] result=%0d zero=%0b",
                     pkt.result, pkt.zero);

        end
    endtask
endclass


class scoreboard;
     mailbox #(packet) mon2scb;

     function new(mailbox #(packet) m);
          this.mon2scb = m;
     endfunction
      

     task run();
          packet pkt;
          bit [31:0] exp_result;
          bit exp_zero;
          forever begin
          mon2scb.get(pkt);
          case(pkt.op) 
          
          4'd0: exp_result = pkt.src1 + pkt.src2; //add
          4'd1: exp_result = pkt.src1 - pkt.src2; //sub
          4'd2: exp_result = pkt.src1 & pkt.src2; //and
          4'd3: exp_result = pkt.src1 | pkt.src2; //or
          4'd4: exp_result = pkt.src1 ^ pkt.src2; //xor
          4'd5: exp_result = pkt.src1 << pkt.src2[4:0]; //sll (shift left logic)
          4'd6: exp_result = pkt.src1 >> pkt.src2[4:0]; //srl  (- right -)
          4'd7: exp_result = (pkt.src1 < pkt.src2 )? 32'b1: 32'b0 ; //uslt 
          4'd8: exp_result = ( $signed(pkt.src1) < $signed(pkt.src2) )? 32'b1: 32'b0 ; //slt
          4'd9: exp_result = $signed(pkt.src1) >>> pkt.src2[4:0] ; //arithmetic shift right 
          4'd10: exp_result = pkt.src2; //function for lui
          default: exp_result = 32'b0;
          endcase
          exp_zero = (exp_result == 0);
          if (pkt.result !== exp_result || pkt.zero !== exp_zero)
                $error("[SCB] MISMATCH exp=%0d got=%0d",
                       exp_result, pkt.result);
          else
                $display("[SCB] PASS");
          end
     endtask
endclass


class environment;
     mailbox #(packet) gen2drv;
     mailbox #(packet) mon2scb;

     generator gen;
     driver drv;
     monitor mon;
     scoreboard scb;
    
     virtual alu_if vif;
     
     function new(virtual alu_if vif);
           this.vif = vif;

           gen2drv = new();
           mon2scb = new();
     
           gen = new(gen2drv,20);
           drv = new(gen2drv, vif);
           mon = new(mon2scb,vif);
           scb = new(mon2scb);
     endfunction

     task run();
       fork
          gen.run();
          drv.run();
          mon.run();
          scb.run();
       join_none
     endtask
endclass


module tb_alu;
    logic clk;
    always #5 clk = ~clk;

    alu_if intf(clk);

    logic [31:0] src1;
    logic [31:0] src2;
    logic [3:0]  op;
    logic [31:0] result;
    logic        zero;

    alu dut (
        .s1(intf.src1),
        .s2(intf.src2),
        .op(intf.op),
        .result(intf.result),
        .zero(intf.zero)
    );

    environment env;

    initial begin
       clk = 0;
       env = new(intf);
       env.run();
       #500 $finish;
    end
    
endmodule