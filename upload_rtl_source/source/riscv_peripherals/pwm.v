 /*                                                                      
	Created by wangsijia 2021/10/14                                        
 */
 
`include "../riscv_core/defines.v"
 
module pwm
    #(
    	parameter CNT_1US_MAX = 6'd49   ,
    	parameter CNT_1MS_MAX = 10'd999 
    )
    (
	input wire clk,
	input wire rst,
	
	input wire[31:0]	data_duty_i,//pwm 占空比 
	input wire[31:0]	addr_i     ,
	input wire			we_i	   ,
	//温湿度传感器信号
    input wire[31:0]    tem_hum_i  ,//温湿度传感器数据
	output wire         hum_flag_o ,//LED信号测试输出
	//光电传感器
	input wire			light_sense,//光电传感
	
	
	output reg[31:0]    data_o,     //相关寄存器数据输出
	output reg       	pwm_out_1, 	//pwm reg_1输出
	output reg       	pwm_out_2, 	//pwm reg_2输出
	output reg       	pwm_out_3, 	//pwm reg_3输出
	output wire			led_out_io  //pwm wire输出
	);
	
	//寄存器地址
	localparam REG_DUTY_1 = 4'h0;
	localparam REG_DUTY_2 = 4'h4;
	localparam REG_DUTY_3 = 4'h8;
	localparam REG_HUM_FLAG = 4'hc;
	//寄存器定义		
	reg [5:0]   cnt_1us     ;
	reg [9:0]   cnt_1ms     ;
	reg [9:0]   duty_r_1    ; //占空比寄存器
	reg [9:0]   duty_r_2    ; 
	reg [9:0]   duty_r_3    ; 
	reg [9:0]	hum_flag	; //加湿器工作的标志寄存器
	
	//写寄存器
	always@(posedge clk)
	begin
		if(rst == `RstEnable)
		begin
			duty_r_1   <= 10'd0;
			duty_r_2   <= 10'd0;
			duty_r_3   <= 10'd0;
		end else begin
			if(we_i == `WriteEnable) begin
                case (addr_i[3:0])
                    REG_DUTY_1: begin
                        duty_r_1 <= data_duty_i;
                    end
					REG_DUTY_2: begin
                        duty_r_2 <= data_duty_i;
                    end
					REG_DUTY_3: begin
                        duty_r_3 <= data_duty_i;
                    end
                endcase
			end
		end
	end
	
	//***************************** Main Code ****************************//
	//cnt_1us:1us计数器
	always@(posedge clk)
		if(rst == 1'b0)
			cnt_1us <= 6'b0;
		else    if(cnt_1us == CNT_1US_MAX)
			cnt_1us <= 6'b0;
		else
			cnt_1us <= cnt_1us + 1'b1;
	
	//cnt_1ms:1ms计数器
	always@(posedge clk)
		if(rst == 1'b0)
			cnt_1ms <= 10'b0;
		else    if(cnt_1ms == CNT_1MS_MAX && cnt_1us == CNT_1US_MAX)
			cnt_1ms <= 10'b0;
		else    if(cnt_1us == CNT_1US_MAX)
			cnt_1ms <= cnt_1ms + 1'b1;
	
	//always@(posedge clk or negedge rst)
	//	if(rst == 1'b0)
	//		duty_r <= 10'd0;
	//	else 
	//		duty_r <= 10'd666;//duty;

	//pwm_out_1:输出信号连接到外部的led灯
	always@(posedge clk)
		if((rst == 1'b0) || (light_sense == 1'b0))
			pwm_out_1 <= 1'b0;
//		else    if( cnt_1ms <= duty_r_1 )
        else    if( cnt_1ms <= 'd450)
			pwm_out_1 <= 1'b1;
		else
			pwm_out_1 <= 1'b0;
	assign led_out_io = pwm_out_1;
	
	//pwm_out_2:输出信号连接到外部的led灯
	always@(posedge clk)
		if((rst == 1'b0) || (light_sense == 1'b0))
			pwm_out_2 <= 1'b0;
	//	else    if( cnt_1ms <= duty_r_2 )
        else    if( cnt_1ms <= 'd440 )
			pwm_out_2 <= 1'b1;
		else
			pwm_out_2 <= 1'b0;
			
	//pwm_out_3:输出信号连接到外部的led灯
	always@(posedge clk)
		if(rst == 1'b0)
			pwm_out_3 <= 1'b0;
		else    if( cnt_1ms <= duty_r_3 )
			pwm_out_3 <= 1'b1;
		else
			pwm_out_3<= 1'b0;
			
	// 读寄存器
    always @ (*) begin
        if (rst == `RstEnable) begin
            data_o = `ZeroWord;
        end else begin
            case (addr_i[3:0])
				//占空比寄存器输出
                REG_DUTY_1: begin
                    data_o = duty_r_1;
                end
                REG_DUTY_2: begin
                    data_o = duty_r_2;
                end
                REG_DUTY_3: begin
                    data_o = duty_r_3;
                end
                REG_HUM_FLAG: begin
                    data_o = hum_flag;
                end
                default: begin
                    data_o = `ZeroWord;
                end
            endcase
        end
    end
	
	//温湿度数据处理
	assign hum_flag_o = hum_flag[0]; 
    always@(posedge clk) begin
        if (rst == `RstEnable) begin
            hum_flag <= 10'd0;	
        end
		else if(tem_hum_i[31:16] < 16'd700)
			hum_flag <= 10'd1;
		else
			hum_flag <= 10'd0;
	end
	
endmodule






