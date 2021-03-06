module data_conver(
input  data_in,
input  dot_disp,
output data_out
);
wire [7:0] dot_disp;
wire [31:0]data_in;
wire [63:0]data_out;
reg [7:0] seg0,seg1,seg2,seg3,seg4,seg5,seg6,seg7;
always@(*) begin
        case(data_in[3:0])
            4'd0  : seg0  <=  {1,7'b100_0000};    //显示数字0
            4'd1  : seg0  <=  {1,7'b111_1001};    //显示数字1
            4'd2  : seg0  <=  {1,7'b010_0100};    //显示数字2
            4'd3  : seg0  <=  {1,7'b011_0000};    //显示数字3
            4'd4  : seg0  <=  {1,7'b001_1001};    //显示数字4
            4'd5  : seg0  <=  {1,7'b001_0010};    //显示数字5
            4'd6  : seg0  <=  {1,7'b000_0010};    //显示数字6
            4'd7  : seg0  <=  {1,7'b111_1000};    //显示数字7
            4'd8  : seg0  <=  {1,7'b000_0000};    //显示数字8
            4'd9  : seg0  <=  {1,7'b001_0000};    //显示数字9
            4'd10 : seg0  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg0  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg0  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[7:4])
            4'd0  : seg1  <=  {0,7'b100_0000};    //显示数字0
            4'd1  : seg1  <=  {0,7'b111_1001};    //显示数字1
            4'd2  : seg1  <=  {0,7'b010_0100};    //显示数字2
            4'd3  : seg1  <=  {0,7'b011_0000};    //显示数字3
            4'd4  : seg1  <=  {0,7'b001_1001};    //显示数字4
            4'd5  : seg1  <=  {0,7'b001_0010};    //显示数字5
            4'd6  : seg1  <=  {0,7'b000_0010};    //显示数字6
            4'd7  : seg1  <=  {0,7'b111_1000};    //显示数字7
            4'd8  : seg1  <=  {0,7'b000_0000};    //显示数字8
            4'd9  : seg1  <=  {0,7'b001_0000};    //显示数字9
            4'd10 : seg1  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg1  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg1  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[11:8])
            4'd0  : seg2  <=  {1,7'b100_0000};    //显示数字0
            4'd1  : seg2  <=  {1,7'b111_1001};    //显示数字1
            4'd2  : seg2  <=  {1,7'b010_0100};    //显示数字2
            4'd3  : seg2  <=  {1,7'b011_0000};    //显示数字3
            4'd4  : seg2  <=  {1,7'b001_1001};    //显示数字4
            4'd5  : seg2  <=  {1,7'b001_0010};    //显示数字5
            4'd6  : seg2  <=  {1,7'b000_0010};    //显示数字6
            4'd7  : seg2  <=  {1,7'b111_1000};    //显示数字7
            4'd8  : seg2  <=  {1,7'b000_0000};    //显示数字8
            4'd9  : seg2  <=  {1,7'b001_0000};    //显示数字9
            4'd10 : seg2  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg2  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg2  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[15:12])
            4'd0  : seg3  <=  {1,7'b100_0000};    //显示数字0
            4'd1  : seg3  <=  {1,7'b111_1001};    //显示数字1
            4'd2  : seg3  <=  {1,7'b010_0100};    //显示数字2
            4'd3  : seg3  <=  {1,7'b011_0000};    //显示数字3
            4'd4  : seg3  <=  {1,7'b001_1001};    //显示数字4
            4'd5  : seg3  <=  {1,7'b001_0010};    //显示数字5
            4'd6  : seg3  <=  {1,7'b000_0010};    //显示数字6
            4'd7  : seg3  <=  {1,7'b111_1000};    //显示数字7
            4'd8  : seg3  <=  {1,7'b000_0000};    //显示数字8
            4'd9  : seg3  <=  {1,7'b001_0000};    //显示数字9
            4'd10 : seg3  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg3  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg3  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[19:16])
            4'd0  : seg4  <=  {1,7'b100_0000};    //显示数字0
            4'd1  : seg4  <=  {1,7'b111_1001};    //显示数字1
            4'd2  : seg4  <=  {1,7'b010_0100};    //显示数字2
            4'd3  : seg4  <=  {1,7'b011_0000};    //显示数字3
            4'd4  : seg4  <=  {1,7'b001_1001};    //显示数字4
            4'd5  : seg4  <=  {1,7'b001_0010};    //显示数字5
            4'd6  : seg4  <=  {1,7'b000_0010};    //显示数字6
            4'd7  : seg4  <=  {1,7'b111_1000};    //显示数字7
            4'd8  : seg4  <=  {1,7'b000_0000};    //显示数字8
            4'd9  : seg4  <=  {1,7'b001_0000};    //显示数字9
            4'd10 : seg4  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg4  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg4  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[23:20])
            4'd0  : seg5  <=  {0,7'b100_0000};    //显示数字0
            4'd1  : seg5  <=  {0,7'b111_1001};    //显示数字1
            4'd2  : seg5  <=  {0,7'b010_0100};    //显示数字2
            4'd3  : seg5  <=  {0,7'b011_0000};    //显示数字3
            4'd4  : seg5  <=  {0,7'b001_1001};    //显示数字4
            4'd5  : seg5  <=  {0,7'b001_0010};    //显示数字5
            4'd6  : seg5  <=  {0,7'b000_0010};    //显示数字6
            4'd7  : seg5  <=  {0,7'b111_1000};    //显示数字7
            4'd8  : seg5  <=  {0,7'b000_0000};    //显示数字8
            4'd9  : seg5  <=  {0,7'b001_0000};    //显示数字9
            4'd10 : seg5  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg5  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg5  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[27:24])
            4'd0  : seg6  <=  {1,7'b100_0000};    //显示数字0
            4'd1  : seg6  <=  {1,7'b111_1001};    //显示数字1
            4'd2  : seg6  <=  {1,7'b010_0100};    //显示数字2
            4'd3  : seg6  <=  {1,7'b011_0000};    //显示数字3
            4'd4  : seg6  <=  {1,7'b001_1001};    //显示数字4
            4'd5  : seg6  <=  {1,7'b001_0010};    //显示数字5
            4'd6  : seg6  <=  {1,7'b000_0010};    //显示数字6
            4'd7  : seg6  <=  {1,7'b111_1000};    //显示数字7
            4'd8  : seg6  <=  {1,7'b000_0000};    //显示数字8
            4'd9  : seg6  <=  {1,7'b001_0000};    //显示数字9
            4'd10 : seg6  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg6  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg6  <=  8'b1100_0000;
        endcase
end
always@(*) begin
        case(data_in[31:28])
            4'd0  : seg7  <=  {1,7'b100_0000};    //显示数字0
            4'd1  : seg7  <=  {1,7'b111_1001};    //显示数字1
            4'd2  : seg7  <=  {1,7'b010_0100};    //显示数字2
            4'd3  : seg7  <=  {1,7'b011_0000};    //显示数字3
            4'd4  : seg7  <=  {1,7'b001_1001};    //显示数字4
            4'd5  : seg7  <=  {1,7'b001_0010};    //显示数字5
            4'd6  : seg7  <=  {1,7'b000_0010};    //显示数字6
            4'd7  : seg7  <=  {1,7'b111_1000};    //显示数字7
            4'd8  : seg7  <=  {1,7'b000_0000};    //显示数字8
            4'd9  : seg7  <=  {1,7'b001_0000};    //显示数字9
            4'd10 : seg7  <=  8'b1011_1111          ;    //显示负号
            4'd11 : seg7  <=  8'b1111_1111          ;    //不显示任何字符
            default:seg7  <=  8'b1100_0000;
        endcase
end
assign data_out = {seg7,seg6,seg5,seg4,seg3,seg2,seg1,seg0};
endmodule