/****************************************************
Implementation of ALU- Arithmetic Logic Unit
Designed and Implemented by- Luis A. Sala Ortiz
*****************************************************/

//Initialization of ALU module
module arithmetic_logic_unit (output reg [31: 0] out, output reg zero, n, c, v, input [31: 0] A, B, input [4: 0] Op, input Cin);
//Setting the conditional codes to 0 initially.

//Setting interconexions
reg [31: 0] temp;
reg  signed [31: 0] pctemp;
wire [31: 0]  zero_num;
assign zero_num=32'b00000000000000000000000000000000;

always@(A,B,Op,Cin)
begin
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

//Signed A*4 + B (Add an offset to PC)
5'b10011:
begin
pctemp = B * 32'h4;
out = pctemp + A;
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

/*********************************************
                  ALU Tester
**********************************************/
module alu_tester;
  //Defines Operands an input variables
  reg [31: 0] A, B;
  reg [4: 0] Op;

  //Cin
  reg Cin;

  //Signed Variables
  reg signed [31: 0] temp;

  //output
  wire [31: 0] out;
  //Condition codes
  wire zero, n, c, v;

  //Instantiation of the Alu module
  arithmetic_logic_unit alu(out, zero, n, c, v, A, B, Op, Cin);

  //Defines the time of execution of the tester
  initial #500 $finish;

	initial begin
	//B = 40
	A = 32'h9C4;
	//Set A to temp = -1
	temp = 32'hFFFFFF6A;
	B = temp;
	//A + B
	//Should give 36
	Op = 5'b10011;
	$monitor("A = %d  B = %d  Out = %d  Time = %0d", A, B, out, $time);
	end
endmodule
