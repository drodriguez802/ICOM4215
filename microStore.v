//ROM For Load immediate Offset and Register Offset
module encoder(instruction, state);
   output reg [7:0]state = 8'b00000000;
   input wire [31:0] instruction;

   always@(instruction)
    begin
        if(instruction[27:25]==3'b001)
            begin
            $display("STATE 5");
            state = 8'b00000101;
            end
        else if(instruction[27:25]==3'b101&&instruction[24]==0)
            begin
            $display("STATE 10");
            state = 8'b00001010;
            end
        else if(instruction[27:25]==3'b101&&instruction[24]==1)
            begin
            $display("STATE 11");
            state = 8'b00001011;
            end
        $display("STATE: %b", state);
    end
endmodule
//ROM For Load immediate Offset and Register Offset
module ROM(MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, state, clk);
   output reg MBS,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, MOV,DataType, Inv, N2, N1, N0, S1, S0;
   output reg [1:0]SignExtSel,MA,MB,MBSMRF;
   output reg [2:0] MC;
   output reg [5:0] CR;
   output reg [4:0]OP;
   input wire [7:0] state;
   input wire clk;

   always@(posedge clk)
    begin
    $display("INSIDE ROM STATE IS: %b",state);
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
            N2 = 1;
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
            CR = 6'b001010;
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
   output reg [7:0] outR;
   input [1:0] select;
   always@(select or inR0 or inR1 or inR2 or inR3)
    begin
        case(select)
            2'b00 : outR = inR0;
            2'b01 : outR = inR1;
            2'b10 : outR = inR2;
            2'b11 : outR = inR3;
    endcase
    // $display("SELECT WAS: %b",select);
    // $display("MUX OUT: %b",outR);
    end
endmodule

module adder(input [7:0] value,input [7:0] one,output  [7:0] result);
        wire [7:0]result2 = value + one;
		assign result = result2;
		always@(result)
		begin
		    $display("RESULT: ",result);
		    end
endmodule


module main;
  wire [7:0] state;
  wire [7:0] stateEncoder;
  reg [31:0] instruction;
  wire [31:0] out;
  wire [7:0] outMux;
  wire MBS,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0, S1, S0, Sts,iBit, inv;
  wire [1:0]SignExtSel,MA,MB,MBSMRF;
  wire [2:0] MC;
  wire [5:0] CR;
  wire [4:0] OP;
  wire [7:0] outAdd;
  reg [1:0] M;
  reg clk;
  encoder enc(instruction, stateEncoder);
  multiplexer4x1 mux(outMux, stateEncoder, 0, CR, outAdd, M);
  adder add(outMux, 8'b00000001,outAdd);
  ROM rom(MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, outMux,clk);
  initial
  begin
    //   $display("INSTRUCTION = %b",instruction);
    //   $display("STATE  = %b",state);
    //   $display("OUTPUT MUX = %b",outAdd);
      $display("MA  |  MB  |  MC  |  MBS  |  MBSMRF  |  MUXMDR  |  MDREn  |  MAREn  |  IREn  |  SHF_S  |  ShiftEn  |  RFEn  |  RW  |  MemEn  |  MOC  |   Inv  |   N2  |   N1  |   N0  |  SignExtSel  |  CR      |  OP ");
      $monitor("%b     %b     %b     %b      %b            %b        %b        %b          %b        %b          %b           %b      %b        %b       %b       %b         %b       %b      %b              %b     %b      %b",MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP);
      
       #5 clk = 1'b1;
       #10 M = 2'b00;
       #15 clk = 1'b0;
       #20 clk = 1'b1;
       #25 M = 2'b11;
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
    //   #5 state = 8'b00000001;
    //   #5 state = 8'b00000010;
    //   #5 state = 8'b00000011;
    //   #5 state = 8'b00000100;
      
      end
 endmodule