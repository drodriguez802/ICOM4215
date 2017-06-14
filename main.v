module arithmetic_logic_unit (output reg [31: 0] out, output reg zero, n, c, v, input [31: 0] A, B, input [5: 0] Op, input Cin);
//Setting the conditional codes to 0 initially.

//Setting interconexions
reg [31: 0] temp;
wire [31: 0]  zero_num;
assign zero_num=32'b00000000000000000000000000000000;

always@(A,B,Op,Cin)
begin
//$display("ALU: A: %b B: %b OP:%b",A,B,Op);
//Case for defining the operation to be executed
case(Op)
//AND
5'b00000:
begin
out = A & B;
c = Cin;
v = 1'b0;
end

//XOR
5'b00001:
begin
out = A ^ B;
c = Cin;
v = 1'b0;
end

//Subtract
5'b00010:
begin
c = Cin;
{c, out} = A - B;
c = ~c;
if(A[31]!=B[31])
if(B[31]==out[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//Reverse Subtract
5'b00011:
begin
c = Cin;
{c, out} = B - A;
c = ~c;
if(A[31]!=B[31])
if(A[31]==out[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//ADD
5'b00100:
begin
c = Cin;
{c, out} = A + B;
if(A[31]==B[31])
if(out[31]!=B[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//ADD w/ Carry
5'b00101:
begin
c = Cin;
{c, out} = A + B + Cin;
if(A[31]==B[31])
if(out[31]!=B[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//Subtract w/ Carry
5'b00110:
begin
c = Cin;
{c, out} = A - B - !Cin;
c = ~c;
if(A[31]!=B[31])
if(B[31]==out[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//Reverse subtract w/ Carry
5'b00111:
begin
c = Cin;
{c, out} = B - A - !Cin;
c = ~c;
if(A[31]!=B[31])
if(A[31]==out[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//Test
5'b01000:
begin
temp = A & B;
c = Cin;
v = 1'b0;
end

//Test Equivalence
5'b01001:
begin
temp = A ^ B;
c = Cin;
v = 1'b0;
end

//Compare
5'b01010:
begin
c = Cin;
{c, temp} = A - B;
c = ~c;
if(A[31]!=B[31])
if(B[31]==temp[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//Compare Negated
5'b01011:
begin
c = Cin;
{c, temp} = A + B;
if(A[31]==B[31])
if(temp[31]!=B[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//OR
5'b01100:
begin
out = A | B;
c = Cin;
v = 1'b0;
end

//Move
5'b01101:
begin
out = B;
c = Cin;
v = 1'b0;
end

//Bit Clear
5'b01110:
begin
out = A & ~B;
c = Cin;
v = 1'b0;
end

//Move Not
5'b01111:
begin
out = ~B;
c = Cin;
v = 1'b0;
end

//A
5'b10000:
begin
out = A;
c = Cin;
end

//A + 4
5'b10001:
begin
{c, out} = A + 32'b00000000000000000000000000000100;
if(A[31]==B[31])
if(out[31]!=B[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end

//A + B + 4
5'b10010:
begin
{c, out} = A + B + 32'b00000000000000000000000000000100;
if(A[31]==B[31])
if(out[31]!=B[31])
v=1'b1;
else v=1'b0;
else v=1'b0;
end
endcase

//Setting the condition codes:
//N
if((Op<5'b01000) || (Op>5'b01011)) n = out[31];
else n = temp[31];
//Z
zero = out && zero_num;
end
endmodule

module registerfile(portC, portA, portB, decS3, decS2, decS1, decS0, muxAS3, muxAS2, muxAS1, muxAS0, muxBS3, muxBS2, muxBS1, muxBS0, enable, clk);
  output wire [31:0] portA, portB;
  input wire decS3;
  input wire decS2;
  input wire decS1;
  input wire decS0;
  input wire muxAS3, muxAS2, muxAS1, muxAS0, muxBS3, muxBS2, muxBS1, muxBS0;
  input wire clk, enable;
  input wire [31:0] portC;
  wire [31:0]R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15;//Register Data
  wire R0e, R1e, R2e, R3e, R4e, R5e, R6e, R7e, R8e, R9e, R10e, R11e, R12e, R13e, R14e, R15e;//enable for register Rn
 
  decoder #1registerFileDecoder(R0e, R1e, R2e, R3e, R4e, R5e, R6e, R7e, R8e, R9e, R10e, R11e, R12e, R13e, R14e, R15e, decS3, decS2, decS1, decS0, enable, clk);
 
  register #1register0(R0, portC, R0e, clk);
  register #1register1(R1, portC, R1e, clk);
  register #1register2(R2, portC, R2e, clk);
  register #1register3(R3, portC, R3e, clk);
  register #1register4(R4, portC, R4e, clk);
  register #1register5(R5, portC, R5e, clk);
  register #1register6(R6, portC, R6e, clk);
  register #1register7(R7, portC, R7e, clk);
  register #1register8(R8, portC, R8e, clk);
  register #1register9(R9, portC, R9e, clk);
  register #1register10(R10, portC, R10e, clk);
  register #1register11(R11, portC, R11e, clk);
  register #1register12(R12, portC, R12e, clk);
  register #1register13(R13, portC, R13e, clk);
  register #1register14(R14, portC, R14e, clk);
  register #1register15(R15, portC, R15e, clk);
 
  mux16x1 #1muxA(portA, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15, muxAS3, muxAS2, muxAS1, muxAS0);
  mux16x1 #1muxB(portB, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15, muxBS3, muxBS2, muxBS1, muxBS0);
  endmodule


module decoder(R0e, R1e, R2e, R3e, R4e, R5e, R6e, R7e, R8e, R9e, R10e, R11e, R12e, R13e, R14e, R15e, S3, S2, S1, S0, enable, clk);
        input wire S3, S2, S1, S0, enable, clk;
        output wire R0e, R1e, R2e, R3e, R4e, R5e, R6e, R7e, R8e, R9e, R10e, R11e, R12e, R13e, R14e, R15e;
      
        assign R0e = enable&&~S0&&~S1&&~S2&&~S3&&1'b1;//0000 - R0
        assign R1e= enable&&S0&&~S1&&~S2&&~S3&&1'b1;//0001 - R1
        assign R2e= enable&&~S0&&S1&&~S2&&~S3&&1'b1;//0010 - R2
        assign R3e= enable&&S0&&S1&&~S2&&~S3&&1'b1;//0011 - R3
        assign R4e= enable&&~S0&&~S1&&S2&&~S3&&1'b1;//0100 - R4
        assign R5e= enable&&S0&&~S1&&S2&&~S3&&1'b1;//0101 - R5
        assign R6e= enable&&~S0&&S1&&S2&&~S3&&1'b1;//0110 - R6
        assign R7e= enable&&S0&&S1&&S2&&~S3&&1'b1;//0111 - R7
        assign R8e= enable&&~S0&&~S1&&~S2&&S3&&1'b1;//1000 - R8
        assign R9e= enable&&S0&&~S1&&~S2&&S3&&1'b1;//1001 - R9
        assign R10e= enable&&~S0&&S1&&~S2&&S3&&1'b1;//1010 - R10
        assign R11e= enable&&S0&&S1&&~S2&&S3&&1'b1;//1011 - R11
        assign R12e= enable&&~S0&&~S1&&S2&&S3&&1'b1;//1100 - R12
        assign R13e= enable&&S0&&~S1&&S2&&S3&&1'b1;//1101 - R13
        assign R14e= enable&&~S0&&S1&&S2&&S3&&1'b1;//1110 - R14
        assign R15e= enable&&S0&&S1&&S2&&S3&&1'b1;//1111 - R015
      
endmodule


module register(out, in, enable, clk);
   input wire [31:0] in;
   input wire enable, clk;
   output reg [31:0] out;
   always@(posedge clk)
    begin
        if(enable==1'b1)
            $display("INREG: %b",in);
            out = in;
    end
endmodule

module rfmultiplexer4x1(outR, inR0, inR1, inR2, inR3, S1, S0);
   input wire S1, S0;
   input [31:0] inR0, inR1, inR2, inR3;
   output reg [31:0] outR;
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

module rfMux(outR, inR0, inR1, inR2, inR3, S1, S0);
   input wire S1, S0;
   input [3:0] inR0, inR1, inR2, inR3;
   output reg [3:0] outR;
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

module mux16x1(outR, inR0, inR1, inR2, inR3, inR4, inR5, inR6, inR7,
inR8, inR9, inR10, inR11, inR12, inR13, inR14, inR15, S3, S2, S1, S0);
   input wire S3, S2, S1, S0;
   input [31:0] inR0, inR1, inR2, inR3, inR4, inR5, inR6, inR7,
inR8, inR9, inR10, inR11, inR12, inR13, inR14, inR15;
   output wire [31:0] outR;
   wire [31:0] zero, one, two, three;
   wire [3:0] select;
 
   rfmultiplexer4x1 muxZero(zero, inR0, inR1, inR2, inR3, S1, S0);
   rfmultiplexer4x1 muxOne(one, inR4, inR5, inR6, inR7, S1, S0);
   rfmultiplexer4x1 muxTwo(two, inR8, inR9, inR10, inR11, S1, S0);
   rfmultiplexer4x1 muxThree(three, inR12, inR13, inR14, inR15, S1, S0);
   rfmultiplexer4x1 muxFinal(outR, zero, one, two, three, S3, S2);
 
endmodule

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
//ROM For Load immediate Offset and Register Offset
module ROM(currentState,MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, state, clk,S1,S0,instruction);
   output reg MBS = 1'b0,  MUXMDR = 0, MDREn = 0, MAREn = 0, IREn = 0, Inv=1'b0, N2=0, N1=1, N0=1, S1 = 1'b0, S0 = 1'b0,ShiftEn = 0,RFEn = 1'b0,RW = 0, MemEn = 0, MOC = 1, MOV = 1;
   output reg [1:0]SignExtSel = 2'b00,MA = 2'b00, MB = 2'b10, MBSMRF = 2'b00,DataType = 2'b00;
   output reg [2:0] MC = 3'b001,SHF_S = 3'b000;
   output reg [5:0] CR = 6'b000001;
   output reg [4:0]OP = 5'b00000;
   output reg [7:0] currentState = 8'b00000000;
   input wire [7:0] state;
   input wire [31:0] instruction;
   input wire clk;

   always@(posedge clk)
    begin
        if(state==8'b00000000)
            begin
            currentState = 8'b00000000;
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
            RFEn = 1'b0;
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
            currentState = 8'b00000001;
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
            RFEn = 1'b0;
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
            currentState = 8'b00000010;
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
            RFEn = 1'b0;
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
            currentState = 8'b00000011;
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
            RFEn = 1'b0;
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
            currentState = 8'b00000100;
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
            RFEn = 1'b0;
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
            currentState = 8'b00001010;
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
            RFEn = 1'b0;
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
            currentState = 8'b00001011;
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
            RFEn = 1'b0;
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
            currentState = 8'b00000101;
            MA = 2'b00;
            MB = 2'b01;
            MC = 2'b00;
            MBS = 1'b1;
            MBSMRF = 2'b10;
            MUXMDR = 0;
            MDREn = 0;
            MAREn = 1;
            IREn = 1;
            SHF_S = 3'b01;
            ShiftEn = 1'b1;
            SignExtSel = 2'b10;
            RFEn = 1'b1;
            OP = {5'b0,instruction[24:21]};
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

module shifter(outR, inR0,select);
   input wire S1, S0;
   input [31:0] inR0;
   output reg [31:0] outR;
   input [1:0] select;
   always@(select or inR0)
    begin
   $display("INSHIFT: %b",inR0);
        case(select)
            2'b00 : outR = inR0;
            2'b01 : outR = inR0[7:0];
        
    endcase
    $display("OUTSHFT: %b",outR);
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
  wire MBS,MUXMDR,MDREn,MAREn,IREn,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0, S1, S0, Sts,iBit, inv, cOutMux,invOut;
  wire [1:0]SignExtSel,MA,MB,MBSMRF;
  wire [2:0] MC,SHF_S;
  wire [5:0] CR;
  wire [4:0] OP;
  wire [7:0] outAdd;
  wire [1:0] M;
  reg clk, c0, c1;
  //alu+rf
  wire [31:0] portA,portB;
  wire [7:0]currentState;

  //Cin
  reg Cin;
  //output
  //Condition codes
  wire zero, n, c, v;
  //alu+rfo
  wire [3:0] outMC,outMA;
  wire [31:0] shifterOut;
  encoder enc(instruction, stateEncoder);
  conditionMux cMux(cOutMux,1'b0,1'b1,1'b0,1'b0,{S1,S0});
  inverter invert(cOutMux,Inv, invOut);
  nextStAddSel next(M, invOut, N2, N1, N0);
  multiplexer4x1 mux(outMux, stateEncoder, 8'b00000000, CR, outAdd, M);
  adder add(outMux, 8'b00000001,outAdd);
  ROM rom(currentState,MA,MB,MC,MBS,MBSMRF,MUXMDR,MDREn,MAREn,IREn,SHF_S,ShiftEn,RFEn,RW,MemEn,MOC, Inv, N2, N1, N0,SignExtSel,CR,OP, outMux,clk,S1,S0,instruction);
  shifter sh(shifterOut,instruction,SHF_S);
  arithmetic_logic_unit alu(out, zero, n, c, v, portA, shifterOut, OP, Cin);
  rfMux mc(outMC, instruction[15:12], 4'b1111, instruction[19:16], 4'b1110, MC[1], MC[0]);
  rfMux ma(outMA, instruction[19:16],instruction[15:12], 4'b1111,4'b0000, MA[1], MA[0]);
  registerfile rf(out, portA, portB, outMC[3],outMC[2], outMC[1], outMC[0], outMA[3], outMA[2], outMA[1],outMA[0], instruction[3], instruction[2], instruction[1], instruction[0], RFEn, clk);
 
  initial
  begin
      $display("STATE   |   PORT A  |   PORTC   |   OP  |   SHF_S");
      $monitor("%d          %b      %b      %b      %b",currentState,outMA, outMC,OP,SHF_S);
      instruction = 32'b11100011110100010010000000000000;
       #1 clk = 1'b0;
       #5 clk = 1'b1;
       #30 clk = 1'b0;
       #35 clk = 1'b1;
       #70 clk = 1'b0;
       #75 clk = 1'b1;
       #90 clk = 1'b0;
       #95 clk = 1'b1;
       #100 clk = 1'b0;
       #105 clk = 1'b1;
       instruction = 32'b11100010000000010000000000000000;
       #110 clk = 1'b0;
       #115 clk = 1'b1;
       #120 clk = 1'b0;
       #125 clk = 1'b1;
       #130 clk = 1'b0;
       #135 clk = 1'b1;
       #140 clk = 1'b0;
       #145 clk = 1'b1;
       instruction = 32'b11100011100000000001000000101000;
       #150 clk = 1'b0;
       #120 clk = 1'b0;
       #125 clk = 1'b1;
       #130 clk = 1'b0;
       #135 clk = 1'b1;
       #140 clk = 1'b0;
       #145 clk = 1'b1;
       #150 clk = 1'b0;
      end
 endmodule