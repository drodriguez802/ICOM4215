module ram256x8 (output reg [31:0] DataOut, output reg MOC, input Enable, ReadWrite,
input [7:0] Address,input [31:0] DataIn, input [1:0]dSize);
// MFA = Memory Function Active ==> ENABLE
//MOC = Memory Operation Complete




reg [7:0]Memory[0:255]; //128 localizaciones de 32 bits always @ (Enable, ReadWrite)
reg [31:0]temp;
always @(*)
begin
//if(MOC)begin

if (Enable) begin
  if (ReadWrite)begin MOC = 1'b0;
  case(dSize)
  2'b00: begin
  MOC=1'b0;
  temp= {Memory[Address]};
  DataOut<=temp;
  MOC = 1'b1;
  end
  2'b01: begin  //halfword 16 bits
  MOC=1'b0;
  temp= {Memory[Address],Memory[Address+1]};
  DataOut<=temp;
  MOC= 1'b1;
  //MOC = 1'b1;
  end

  2'b10: begin //word 32 bits
  MOC=1'b0;
  temp= {Memory[Address],Memory[Address+1],Memory[Address+2],Memory[Address+3]};
  DataOut<=temp;
  MOC= 1'b1;
  end
  endcase
end//ReadWrite
else //write
begin
case(dSize)
2'b00: begin
MOC=1'b0;
Memory[Address] <= DataIn[7:0];
MOC=1'b1;
end

2'b01: begin
MOC=1'b0;
Memory[Address] <= DataIn[15:8];
Memory[Address+1] <= DataIn[7:0];
MOC=1'b1;

//Mem[Address+1] = DataIn[8:];

end

2'b10: begin //Escribir el word
MOC=1'b0;
Memory[Address] <= DataIn[31:24];
Memory[Address+1] <= DataIn[23:16];
Memory[Address+2] <= DataIn[15:8];
Memory[Address+3] <= DataIn[7:0];
//#3;
MOC=1'b1;
end

endcase

end//else
end//enable
else begin
DataOut = 32'bz;
MOC = 0;
end
end //begin del  always
endmodule


module RAM_Access;
integer fi,fo,code,e; reg[31:0]data; reg Enable, ReadWrite;
reg [31:0] DataIn; reg [7:0] Address; wire [31:0] DataOut;
reg [1:0] dSize;
wire moc;
//reg MOC;
//module ram256x8 (output reg [31:0] DataOut, output reg MOC, input Enable, ReadWrite,
//input [7:0] Address,input [31:0] DataIn, input [1:0]dSize);
ram256x8 ram1 (DataOut, moc, Enable,
ReadWrite, Address, DataIn, dSize);
integer data_file; // file handler
integer scan_file; // file handler
integer captured_data;
integer i=0;
reg[7:0] temp=0;
integer j=0;
`define NULL 0;





initial begin
  $display("Llenando RAM");
  i = 0;
  data_file = $fopen("input_file.txt","r");
  if (data_file == 0) begin
    $display("data_file handle was NULL");
    $finish;
  end
  while(!$feof(data_file)) begin
     scan_file = $fscanf(data_file, "%h\n", captured_data);

     if (!$feof(data_file)) begin
     	$display("%h", captured_data);
     ram1.Memory[i] = captured_data;
     	$display("%h", ram1.Memory[i]);
     	i = i+1;
       //use captured_data as you would any other wire or reg value;
     end//if
 end//while
 $display("Ha terminado de llenar el RAM");

//ram256x8 ram1 (DataOut, moc, Enable,
//ReadWrite, Address, DataIn, dSize);


 #3DataIn=32'hA00000AB;
 Address=0;
 dSize=2'b10;
 ReadWrite=0;
 #4Enable=1;
 #5ReadWrite=1;
$monitor("%h",DataOut);


 #3DataIn=32'hCAFECAFE;
 #2Address=4;
 dSize=2'b10;
 ReadWrite=0;
 #4Enable=1;
 #5ReadWrite=1;
$monitor("%h",DataOut);

#3DataIn=32'h0000CAFE;
#2Address=8;
dSize=2'b01;
ReadWrite=0;
#4Enable=1;
#5ReadWrite=1;
$monitor("%h",DataOut);

#3DataIn=32'h000000FE;
#2Address=10;
dSize=2'b00;
ReadWrite=0;
#4Enable=1;
#5ReadWrite=1;
$monitor("%h",DataOut);

#2 Address= 8;
dSize = 2'b10;
$monitor("%h",DataOut);



//Address=1;
//Address=2;
//Address=3;
// $display("%h", ram1.Memory[0]);
// $display("%h", ram1.Memory[1]);
// $display("%h", ram1.Memory[2]);
// $display("%h", ram1.Memory[3]);



end//initial



endmodule
