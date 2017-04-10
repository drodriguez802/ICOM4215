//ROM For Load immediate Offset and Register Offset
module encoder(instruction, state);
   output reg [7:0]state;
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
    end
endmodule
//ROM For Load immediate Offset and Register Offset
module ROM(MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, state);
   output reg MBS,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, MOV,DataType, Inv, N2, N1, N0, S1, S0;
   output reg [1:0]SignExtSel,MA,MB,MBSMRF;
   output reg [2:0] MC;
   output reg [5:0] CR;
   output reg [4:0]OP;
   input wire [7:0] state;

   always@(state)
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
    end
endmodule

module multiplexer4x1(outR, inR0, inR1, inR2, inR3, S1, S0);
   input wire S1, S0;
   input [7:0] inR0, inR1, inR2, inR3;
   output reg [7:0] outR;
   wire [1:0] select;
   assign select[0] = S0;
   assign select[1] = S1;
   always@(select or inR0 or inR1 or inR2 or inR3)
    begin
        case(select)
            2'b00 : outR = inR0;
            2'b01 : outR = inR1;
            2'b10 : outR = inR2;
            2'b11 : outR = inR3;
    endcase
    end
endmodule

module adder(value, add);
   input [7:0] value;
   output  wire [7:0] add;
   wire one = 1'b1;
   
   assign add = value+one;
   
endmodule

module incrementalRegister(out, in);
    input wire [7:0] in;
    output wire [7:0] out;
        assign out = in;
endmodule



    

module main;
  wire [7:0] state = 8'b00000000;
  wire [7:0] stateEncoder;
  reg [31:0] instruction;
  wire [31:0] out;
  wire [7:0] outMux;
  wire MBS,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0, S1, S0, Sts,iBit, inv;
  wire [1:0]SignExtSel,MA,MB,MBSMRF;
  wire [2:0] MC, M;
  wire [5:0] CR;
  wire [4:0] OP;
  wire [7:0] add;
   
  ROM rom(MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, state);
  encoder enc(instruction, stateEncoder);
  multiplexer4x1 mux(outMux, stateEncoder, 0, CR, add, M[1], M[0]);
  adder addTo(outMux, add);
  nextStAddSel addSelect(M, Sts, N2, N1, N0);
  inverter invert(iBit, inv, Sts);
  initial
  begin
      $display("INSTRUCTION = %b",instruction);
      $display("STATE ENCODER = %b",stateEncoder);
      $display("OUTPUT MUX = %b",outMux);
      $display("MA  |  MB  |  MC  |  MBS  |  MBSMRF  |  MUXMDR  |  MDREn  |  MAREn  |  IREn  |  SHF_S  |  ShiftEn  |  RFEn  |  RW  |  MemEn  |  MOC  |   Inv  |   N2  |   N1  |   N0  |  SignExtSel  |  CR      |  OP ");
      $monitor("%b     %b     %b     %b      %b            %b        %b        %b          %b        %b          %b           %b      %b        %b       %b       %b         %b       %b      %b              %b     %b      %b",MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP);
      
      #5 instruction = 32'b11110010000000000000000000000000;
      #5 instruction = 32'b11111010000000000000000000000000;
      
      end
 endmodule

