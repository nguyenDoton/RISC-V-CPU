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


module tb_alu;


    logic [31:0] src1;
    logic [31:0] src2;
    logic [3:0]  op;
    logic [31:0] result;
    logic        zero;

    alu dut (
        .s1(src1),
        .s2(src2),
        .op(op),
        .result(result),
        .zero(zero)
    );
endmodule


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
          
     constraint op_limit { op inside  {[0:9]}; };
endclass


class generator;
     packet pkt;
     mailbox #(packet) mail;
     int count;
     
     function new( mailbox #(packet) m, int count);
         this.mail = m;
         this.count = count;
     endfunction
     task create();
        repeat(count) 
           begin
              pkt = new();
              pkt.randomize();
              mail.put(pkt);
              $display("Sending packet to DUT");
           end
     endtask
endclass


class driver;
    mailbox #(packet) mail;
    packet pkt;
    
    task run();
        
    endtask
endclass