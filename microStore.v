//ROM For Load immediate Offset and Register Offset
module encoder(instruction, state);
   output reg [7:0]state;
   input wire [31:0] instruction;

   always@(instruction)
    begin
        //$display("INSTRUCCION: %b",instruction);
        if(instruction[27:25]==3'b001)
            begin
            //$display("STATE 5");
            state = 8'b00000101;
            end
        else if(instruction[27:25]==3'b101&&instruction[24]==0)
            begin
            //$display("STATE 10");
            state = 8'b00001010;
            end
        else if(instruction[27:25]==3'b101&&instruction[24]==1)
            begin
            //$display("STATE 11");
            state = 8'b00001011;
            end
        //$display("STATE: %b", state);
    end
endmodule
//ROM For Load immediate Offset and Register Offset
module ROM(MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, state, clk,S1,S0);
   output reg MBS = 1'b0,  MUXMDR = 0, MDREn = 0, MAREn = 0, IREn = 0, Inv=1'b0, N2=0, N1=1, N0=1, S1 = 1'b0, S0 = 1'b0,ShiftEn = 0,RFEn = 1,RW = 0, MemEn = 0, MOC = 1, MOV = 1;
   output reg [1:0]SignExtSel = 2'b00,MA = 2'b00, MB = 2'b10, MBSMRF = 2'b00,DataType = 2'b00;
   output reg [2:0] MC = 3'b001,SHF_S = 3'b000;
   output reg [5:0] CR = 6'b000001;
   output reg [4:0]OP = 5'b01101;
   input wire [7:0] state;
   input wire clk;

   always@(posedge clk)
    begin
        if(state==8'b00000000)
            begin
            $display("STATE 0");
            MA = 2'b00;
            MB = 2'b10;
            MC = 2'b01;
            MBS = 1'b0;
            MBSMRF = 1'b0;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 0;
            IREn = 0;
            SHF_S = 3'b000;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 1;
            OP = 5'b01101;
            DataType = 2'b00;
            RW = 0;
            MemEn = 0;
            MOC = 1;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 1;
            Inv = 0;
            S1 = 0; 
            S0 = 0;
            CR = 6'b000001;
            end
        else if(state==8'b0000001)
            begin
            $display("STATE 1");
            MA = 2'b10;
            MB = 2'b00;
            MC = 2'b00;
            MBS = 1'b0;
            MBSMRF = 1'b0;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 1;
            IREn = 0;
            SHF_S = 3'b000;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 1;
            OP = 5'b10001;
            DataType = 2'b00;
            RW = 0;
            MemEn = 0;
            MOC = 1;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 1;
            Inv = 0;
            S1 = 1; 
            S0 = 1;
            CR = 6'b000010;
            end
        else if(state==8'b0000010)
            begin
            $display("STATE 2");
            MA = 2'b10;
            MB = 2'b00;
            MC = 2'b10;
            MBS = 1'b0;
            MBSMRF = 1'b0;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 0;
            IREn = 0;
            SHF_S = 3'b000;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 1;
            OP = 5'b10001;
            DataType = 2'b00;
            RW = 0;
            MemEn = 1;
            MOC = 0;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 1;
            Inv = 0;
            S1 = 0; 
            S0 = 0;
            CR = 6'b000011;
            end
        else if(state==8'b00000011)
            begin
            $display("STATE 3");
            MA = 2'b00;
            MB = 2'b00;
            MC = 2'b00;
            MBS = 1'b0;
            MBSMRF = 1'b0;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 0;
            IREn = 1;
            SHF_S = 3'b000;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 01;
            OP = 5'b10001;
            DataType = 2'b00;
            RW = 1;
            MemEn = 1;
            MOC = 1;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 1;
            Inv = 0;
            S1 = 0; 
            S0 = 0;
            CR = 6'b000100;
            end
        else if(state==8'b00000100)
            begin
            $display("STATE 4");
            MA = 2'b00;
            MB = 2'b00;
            MC = 2'b00;
            MBS = 1'b0;
            MBSMRF = 1'b0;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 0;
            IREn = 1;
            SHF_S = 3'b000;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 01;
            OP = 5'b10001;
            DataType = 2'b00;
            RW = 0;
            MemEn = 0;
            MOC = 0;
            MOV = 0;
            N2 = 0;
            N1 = 0;
            N0 = 0;
            Inv = 0;
            S1 = 0; 
            S0 = 1;
            CR = 6'b000001;
            end
        else if(state==8'b00001010)
            begin
            $display("STATE 10");
            MA = 2'b01;
            MB = 2'b00;
            MC = 2'b01;
            MBS = 1'b0;
            MBSMRF = 2'b10;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 0;
            IREn = 0;
            SHF_S = 2'b00;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 1;
            OP = 5'b11111;
            RW = 0;
            MemEn = 0;
            MOC = 1;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 1;
            Inv = 0;
            S1 = 0; 
            S0 = 0;
            CR = 6'b000001;
            end
        else if(state==8'b00001011)
            begin
            $display("STATE 11");
            MA = 2'b10;
            MB = 2'b00;
            MC = 2'b11;
            MBS = 1'b0;
            MBSMRF = 2'b10;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 0;
            IREn = 0;
            SHF_S = 2'b00;
            ShiftEn = 0;
            SignExtSel = 2'b00;
            RFEn = 1;
            OP = 5'b11111;
            RW = 0;
            MemEn = 0;
            MOC = 1;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 0;
            Inv = 0;
            S1 = 1; 
            S0 = 0;
            CR = 6'b000001;
            end
         else if(state==8'b00000101)
            begin
            $display("STATE 5");
            MA = 2'b00;
            MB = 2'b01;
            MC = 2'b00;
            MBS = 1'b1;
            MBSMRF = 2'b10;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 1;
            IREn = 1;
            SHF_S = 3'b010;
            ShiftEn = 1;
            SignExtSel = 2'b10;
            RFEn = 1;
            OP = 5'b11111;
            RW = 1;
            MemEn = 0;
            MOC = 1;
            MOV = 1;
            N2 = 0;
            N1 = 1;
            N0 = 0;
            Inv = 0;
            S1 = 1; 
            S0 = 0;
            CR = 6'b000001;
            end
    end
endmodule

module multiplexer4x1(outR, inR0, inR1, inR2, inR3, select);
   input wire S1, S0;
   input [7:0] inR0, inR1, inR2, inR3;
   output reg [7:0] outR = 8'b00000000;
   input [1:0] select;
   always@(select or inR0 or inR1 or inR2 or inR3)
    begin
        case(select)
            2'b00 : outR = inR0;
            2'b01 : outR = inR1;
            2'b10 : outR = inR2;
            2'b11 : outR = inR3;
    endcase
     //$display("SELECT WAS: %b AND OUT WAS: %b",select, outR);
    end
endmodule

module adder(input [7:0] value,input [7:0] one,output  [7:0] result);
        wire [7:0]result2 = value + one;
		assign result = result2;
		always@(result)
		begin
		    //$display("RESULT: ",result);
		    end
endmodule

//Next State Address Selector
module nextStAddSel(M, Sts, N2, N1, N0);
   input wire Sts;
   input wire S1, S0;
   input wire N2, N1, N0;
   output reg [1:0] M;
   wire [2:0] nSelect;
   assign nSelect[0] = N0;
   assign nSelect[1] = N1;
   assign nSelect[2] = N2;
   always@(nSelect or N2 or N1 or N0)
    begin
        if(nSelect==000)
            M<=00;
        else if(nSelect==3'b001)
            M<=01;
        else if(nSelect==3'b010)
            M<=10;
        else if(nSelect==3'b011)
            M=11;
        else if(nSelect==3'b100 && ~Sts)
            M=00;
        else if(nSelect==3'b100 && Sts)
            M=10;
        else if(nSelect==3'b101 && ~Sts)
            M=11;
        else if(nSelect==3'b101 && Sts)
            M=10;
        else if(nSelect==3'b110 && ~Sts)
            M=11;
        else if(nSelect==3'b110 && Sts)
            M=00;
        else if(nSelect==3'b111)
            M=00;
        //$display("NEXT ADDRESS IS: %b",M);
    end
endmodule

module inverter(iBit, inv, oBit);
    input wire iBit, inv;
    output reg oBit;
    always@(iBit or inv)
    begin
        if(inv==1'b1)
            oBit=~iBit;
        else if(inv==1'b0)
            oBit=iBit;
    //$display("INVERTER IN: %b",iBit);
    //$display("INVERTER OUT: %b",oBit);
    end
endmodule

module conditionMux(outR, inR0, inR1, inR2, inR3, select);
   input wire S1, S0;
   input inR0, inR1, inR2, inR3;
   output reg outR;
   input [1:0] select;
   always@(select or inR0 or inR1 or inR2 or inR3)
    begin
        case(select)
            2'b00 : outR = inR0;
            2'b01 : outR = inR1;
            2'b10 : outR = inR2;
            2'b11 : outR = inR3;
    endcase
     //$display("CONDITIONSELECT WAS: %b",select);
     //$display("CONDITIONMUX OUT: %b",outR);
    end
endmodule



module main;
  wire [7:0] state;
  wire [7:0] stateEncoder;
  reg [31:0] instruction;
  wire [31:0] out;
  wire [7:0] outMux;
  wire MBS,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0, S1, S0, Sts,iBit, inv, cOutMux,invOut;
  wire [1:0]SignExtSel,MA,MB,MBSMRF;
  wire [2:0] MC;
  wire [5:0] CR;
  wire [4:0] OP;
  wire [7:0] outAdd;
  wire [1:0] M;
  reg clk, c0, c1;
  encoder enc(instruction, stateEncoder);
  conditionMux cMux(cOutMux,1'b0,1'b1,1'b0,1'b0,{S1,S0});
  inverter invert(cOutMux,Inv, invOut);
  nextStAddSel next(M, invOut, N2, N1, N0);
  multiplexer4x1 mux(outMux, stateEncoder, 0, CR, outAdd, M);
  adder add(outMux, 8'b00000001,outAdd);
  ROM rom(MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, outMux,clk,S1,S0);
  initial
  begin
    //   $display("INSTRUCTION = %b",instruction);
    //   $display("STATE  = %b",state);
    //   $display("OUTPUT MUX = %b",outAdd);
      $display("MA  |  MB  |  MC  |  MBS  |  MBSMRF  |  MUXMDR  |  MDREn  |  MAREn  |  IREn  |  SHF_S  |  ShiftEn  |  RFEn  |  RW  |  MemEn  |  MOC  |   Inv  |   N2  |   N1  |   N0  |  SignExtSel  |  CR      |  OP |   S1|S0");
      $monitor("%b     %b     %b     %b      %b          %b        %b        %b          %b        %b          %b           %b      %b        %b       %b       %b         %b       %b  %b              %b     %b      %b     %b     %b",MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP,S1,S0);
      //32 bit shift
      instruction = 32'b11100011110100010010000000000000;
       #5 clk = 1'b1;
       #15 clk = 1'b0;
       #20 clk = 1'b1;
       #30 clk = 1'b0;
       #35 clk = 1'b1;
       #40 clk = 1'b0;
       #45 clk = 1'b1;
       #50 clk = 1'b0;
       #55 clk = 1'b1;
       #60 clk = 1'b0;
       #55 clk = 1'b1;
       #60 clk = 1'b0;
       #65 clk = 1'b1;
       #70 clk = 1'b0;
       #75 clk = 1'b1;
       #80 clk = 1'b0;
       #85 clk = 1'b1;
       #90 clk = 1'b0;
       //Branch
       #93 instruction =32'b11101010000000000000000000000001;
       #95 clk = 1'b1;
       #100 clk = 1'b0;
       #105 clk = 1'b1;
       #110 clk = 1'b0;
       #115 clk = 1'b1;
       #120 clk = 1'b0;
       #125 clk = 1'b1;
       #130 clk = 1'b0;
       #135 clk = 1'b1;
       #140 clk = 1'b0;
       #145 clk = 1'b1;
       #150 clk = 1'b0;      
      end
 endmodule