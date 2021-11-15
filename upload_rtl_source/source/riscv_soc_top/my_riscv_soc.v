 /*                                                                      
	Created by Blue Liang
	The latest version modified by MB��WSJ��DWT / 2021.11.13
 */

`include "../riscv_core/defines.v"
// my_riscv_soc����ģ��
module my_riscv_soc(

    input wire clk,
    input wire rst,
    input wire stop,         //����ȫ�ֿ���

    output wire uart_tx_pin, // UART��������
    input wire uart_rx_pin,  // UART��������
    inout wire[9:0] gpio,    // GPIO����
	
    //PWM�ź�
	output wire pwm_out_1,	 	//pwm�����
	output wire pwm_out_2,	 	//pwm�����
	output wire pwm_out_3,	 	//pwm�����
	output wire led_out_io,     //pwm���������

	//��ʪ�ȴ����ź�
    inout         dht22,        //��ʪ�ȴ�������  
	output		  hum_flag_led,   //��ʪ��������־�Ĳ������
    output        hum_flag_o,
	//��紫��
	input  wire	  light_sense,
	//8λ������ź�
    output wire   SCLK,                //shcp
    output wire   RCLK,                //stcp
    output wire   DIO,
	//���ڷ�������
	output wire	  tx,
    output wire   tx_uart
    );
    assign tx_uart = tx;

    // master 0 interface
    wire[`MemAddrBus] m0_addr_i;
    wire[`MemBus] m0_data_i;
    wire[`MemBus] m0_data_o;
    wire m0_req_i;
    wire m0_we_i;

    // master 1 interface
    wire[`MemAddrBus] m1_addr_i;
    wire[`MemBus] m1_data_i;
    wire[`MemBus] m1_data_o;
    wire m1_req_i;
    wire m1_we_i;

    // master 2 interface
    wire[`MemAddrBus] m2_addr_i;
    wire[`MemBus] m2_data_i;
    wire[`MemBus] m2_data_o;
    wire m2_req_i;
    wire m2_we_i;

    // master 3 interface
    wire[`MemAddrBus] m3_addr_i;
    wire[`MemBus] m3_data_i;
    wire[`MemBus] m3_data_o;
    wire m3_req_i;
    wire m3_we_i;

    // slave 0 interface
    wire[`MemAddrBus] s0_addr_o;
    wire[`MemBus] s0_data_o;
    wire[`MemBus] s0_data_i;
    wire s0_we_o;

    // slave 1 interface
    wire[`MemAddrBus] s1_addr_o;
    wire[`MemBus] s1_data_o;
    wire[`MemBus] s1_data_i;
    wire s1_we_o;

    // slave 2 interface
    wire[`MemAddrBus] s2_addr_o;
    wire[`MemBus] s2_data_o;
    wire[`MemBus] s2_data_i;
    wire s2_we_o;

    // slave 3 interface
    wire[`MemAddrBus] s3_addr_o;
    wire[`MemBus] s3_data_o;
    wire[`MemBus] s3_data_i;
    wire s3_we_o;

    // slave 4 interface
    wire[`MemAddrBus] s4_addr_o;
    wire[`MemBus] s4_data_o;
    wire[`MemBus] s4_data_i;
    wire s4_we_o;

    // slave 5 interface
    wire[`MemAddrBus] s5_addr_o;
    wire[`MemBus] s5_data_o;
    wire[`MemBus] s5_data_i;
    wire s5_we_o;
	
	// slave pwm interface
    wire[`MemAddrBus] pwm_addr_o;
    wire[`MemBus] pwm_data_o;
    wire[`MemBus] pwm_data_i;
    wire pwm_we_o;
	
	
	

    // riscv_bus
    wire riscv_bus_hold_flag_o;

    // jtag
    wire jtag_halt_req_o;
    wire jtag_reset_req_o;
    wire[`RegAddrBus] jtag_reg_addr_o;
    wire[`RegBus] jtag_reg_data_o;
    wire jtag_reg_we_o;
    wire[`RegBus] jtag_reg_data_i;

    // tinyriscv
    wire[`INT_BUS] int_flag;

    // timer0
    wire timer0_int;

    // gpio
    wire[9:0] io_in;
    wire[31:0] gpio_ctrl;
    wire[31:0] gpio_data;

    assign int_flag = {7'h0, timer0_int};

    // �͵�ƽ����LED
    // �͵�ƽ��ʾ�Ѿ�haltסCPU
    assign halted_ind = ~jtag_halt_req_o;
    reg  over,succ;
    wire[`RegBus] regs_26;  // �����Ƿ�����ź�
    wire[`RegBus] regs_27;  // �����Ƿ�ɹ��ź�
    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            over <= 1'b1;
            succ <= 1'b1;
        end else begin
            over <= ~regs_26;    // when = 1, run over                
            succ <= ~regs_27;    // when = 1, run succ, otherwise fail
        end
    end


    //------------��������������ȫ��ʱ���ź�--------------------//
    //wire stop_flag;
    //wire clk;
    //reg  clk_lock;
    
    //key_filter key_filter_inst(
    //    .sys_clk     (clk_sys          ),   //ϵͳʱ��50Mhz
    //    .sys_rst_n   (rst              ),           //ȫ�ָ�λ
    //    .key_in      (stop             ),   //���������ź�
    //    .key_flag    (stop_flag        )    //key_flagΪ1ʱ��ʾ�������⵽����������
    //                                      
    //);
    
    //always@(posedge clk_sys)
    //    if(rst == 1'b0)
    //        clk_lock <= 1'b1;
    //    else if(stop_flag)
    //        clk_lock <= ~clk_lock;
    //    else
    //        clk_lock <= clk_lock;
    
    //assign clk =( 1'b1 )? clk_sys : 1'b0;

    top_dht22 top_dht22_inst(   
        .clk    (clk		),
        .rst    (rst		),
    	//��ʪ�ȴ����ź�
        .dht22  (dht22		),                
    	.tem_hum(tem_hum	),
    	//8λ������ź�       
        .SCLK   (SCLK		),   //shcp
        .RCLK   (RCLK		),   //stcp
        .DIO    (DIO		),
    	//���ڷ�������
    	.tx     (tx			),
    	//����
    	.led    (			)
     );

    // tinyriscv��������ģ������
    riscv_core riscv_core_inst(
        .clk				(clk			 ),
        .rst				(rst			 ),
        .rib_ex_addr_o		(m0_addr_i		 ),
        .rib_ex_data_i		(m0_data_o		 ),
        .rib_ex_data_o		(m0_data_i		 ),
        .rib_ex_req_o		(m0_req_i		 ),
        .rib_ex_we_o		(m0_we_i		 ),	
											  
        .rib_pc_addr_o		(m1_addr_i		 ),
        .rib_pc_data_i		(m1_data_o		 ),
											  
        .jtag_reg_addr_i	(jtag_reg_addr_o ),
        .jtag_reg_data_i	(jtag_reg_data_o ),
        .jtag_reg_we_i		(jtag_reg_we_o	 ),
        .jtag_reg_data_o	(jtag_reg_data_i ),
											  
        .rib_hold_flag_i	(riscv_bus_hold_flag_o ),
        .jtag_halt_flag_i	(jtag_halt_req_o ),
        .jtag_reset_flag_i	(jtag_reset_req_o),

        .int_i				(int_flag		 ),
		.regs_26			(regs_26		 ),
		.regs_27			(regs_27		 )
    );

    // romģ������
    rom rom_inst(
        .clk	(clk		),
        .rst	(rst		),
        .we_i	(s0_we_o	),
        .addr_i	(s0_addr_o	),
        .data_i	(s0_data_o	),
        .data_o	(s0_data_i	)
    );

    // ramģ������
    ram ram_inst(
        .clk	(clk		),
        .rst	(rst		),
        .we_i	(s1_we_o	),
        .addr_i	(s1_addr_o	),
        .data_i	(s1_data_o	),
        .data_o	(s1_data_i	)
    );

    // timerģ������
    timer timer_inst(
        .clk		(clk	 	),
        .rst		(rst		),
        .data_i		(s2_data_o	),
        .addr_i		(s2_addr_o	),
        .we_i		(s2_we_o	),
        .data_o		(s2_data_i	),
        .int_sig_o	(timer0_int	)
    );

    // io0 // 0: ���裬1�������2������
    assign gpio[0] = (gpio_ctrl[1:0] == 2'b01)? gpio_data[0]: 1'bz;
    assign io_in[0] = gpio[0];
    // io1 // 0: ���裬1�������2������
    assign gpio[1] = (gpio_ctrl[3:2] == 2'b01)? gpio_data[1]: 1'bz;
    assign io_in[1] = gpio[1];
    // io2 // 0: ���裬1�������2������
    assign gpio[2] = (gpio_ctrl[5:4] == 2'b01)? gpio_data[2]: 1'bz;
    assign io_in[2] = gpio[2];
    // io3 // 0: ���裬1�������2������
    assign gpio[3] = (gpio_ctrl[7:6] == 2'b01)? gpio_data[3]: 1'bz;
    assign io_in[3] = gpio[3];
    // io4 // 0: ���裬1�������2������
    assign gpio[4] = (gpio_ctrl[9:8] == 2'b01)? gpio_data[4]: 1'bz;
    assign io_in[4] = gpio[4];
    // io5 // 0: ���裬1�������2������
    assign gpio[5] = (gpio_ctrl[11:10] == 2'b01)? gpio_data[5]: 1'bz;
    assign io_in[5] = gpio[5];
    // io6 // 0: ���裬1�������2������
    assign gpio[6] = (gpio_ctrl[13:12] == 2'b01)? gpio_data[6]: 1'bz;
    assign io_in[6] = gpio[6];
    // io7 // 0: ���裬1�������2������
    assign gpio[7] = (gpio_ctrl[15:14] == 2'b01)? gpio_data[7]: 1'bz;
    assign io_in[7] = gpio[7];
    // io8 // 0: ���裬1�������2������
    assign gpio[8] = (gpio_ctrl[17:16] == 2'b01)? gpio_data[8]: 1'bz;
    assign io_in[8] = gpio[8];
    // io9 // 0: ���裬1�������2������
    assign gpio[9] = (gpio_ctrl[19:18] == 2'b01)? gpio_data[9]: 1'bz;
    assign io_in[9] = gpio[9];

    // gpioģ������
    gpio gpio_inst(
        .clk	 (clk		),			
        .rst	 (rst		),
				 
        .we_i	 (s4_we_o	),			
        .addr_i	 (s4_addr_o	),   //����ӻ���GPIO��ؼĴ�����ַ
        .data_i	 (s4_data_o	),   //���ģʽ��д��ӻ���GPIO�����ƽ
		                        
        .data_o	 (s4_data_i	),   //���ӻ�GPIO�Ĵ����������
		                        
        .io_pin_i(io_in		),   //����ģʽ�£�GPIO����������
        .reg_ctrl(gpio_ctrl	),   //���д���GPIO���ƼĴ���
        .reg_data(gpio_data	)    //����ģʽ�£�ģ���������д���GPIO�����ƽ
    );
	
	//pwm ģ������
	wire [31:0]	tem_hum; //��ʪ������
    wire   		hum_flag_o;
    assign 		hum_flag_led = hum_flag_o;
	pwm pwm_inst(
	    .clk			(clk		),
	    .rst			(rst		),
			
	    .data_duty_i	(pwm_data_o	),//pwm ���� �� ռ�ձ�
	    .addr_i 		(pwm_addr_o	),
	    .we_i			(pwm_we_o	),
	    //��ʪ��	
	    .tem_hum_i		(tem_hum	),
	    .hum_flag_o		(hum_flag_o	), //��ʪ��������־�Ĳ������
	    //��紫��	
	    .light_sense	(light_sense),
				
	    .data_o			(pwm_data_i	),
	    .pwm_out_1 		(pwm_out_1	),  //pwm reg���
	    .pwm_out_2 		(pwm_out_2	),  //pwm reg���
	    .pwm_out_3 		(pwm_out_3	),  //pwm reg���
	    .led_out_io		(led_out_io	)	//pwm wire���
	);

    // ribģ������
    riscv_bus riscv_bus_inst(
        .clk		(clk			),
        .rst		(rst			),
	
        // master 0 interface	
        .m0_addr_i	(m0_addr_i		),
        .m0_data_i	(m0_data_i		),
        .m0_data_o	(m0_data_o		),
        .m0_req_i	(m0_req_i		),
        .m0_we_i	(m0_we_i		),
	
        // master 1 interface	
        .m1_addr_i	(m1_addr_i		),
        .m1_data_i	(`ZeroWord		),
        .m1_data_o	(m1_data_o		),
        .m1_req_i	(`RIB_REQ		),
        .m1_we_i	(`WriteDisable	),

        // master 2 interface
        .m2_addr_i	(m2_addr_i		),
        .m2_data_i	(m2_data_i		),
        .m2_data_o	(m2_data_o		),
        .m2_req_i	(m2_req_i		),
        .m2_we_i	(m2_we_i		),

        // master 3 interface
        .m3_addr_i	(m3_addr_i		),
        .m3_data_i	(m3_data_i		),
        .m3_data_o	(m3_data_o		),
        .m3_req_i	(m3_req_i		),
        .m3_we_i	(m3_we_i		),

        // slave 0 interface
        .s0_addr_o	(s0_addr_o		),
        .s0_data_o	(s0_data_o		),
        .s0_data_i	(s0_data_i		),
        .s0_we_o	(s0_we_o		),

        // slave 1 interface
        .s1_addr_o	(s1_addr_o		),
        .s1_data_o	(s1_data_o		),
        .s1_data_i	(s1_data_i		),
        .s1_we_o	(s1_we_o		),

        // slave 2 interface
        .s2_addr_o	(s2_addr_o		),
        .s2_data_o	(s2_data_o		),
        .s2_data_i	(s2_data_i		),
        .s2_we_o	(s2_we_o		),

        // slave 3 interface
        .s3_addr_o	(s3_addr_o		),
        .s3_data_o	(s3_data_o		),
        .s3_data_i	(s3_data_i		),
        .s3_we_o	(s3_we_o		),

        // slave 4 interface
        .s4_addr_o	(s4_addr_o		),
        .s4_data_o	(s4_data_o		),
        .s4_data_i	(s4_data_i		),
        .s4_we_o	(s4_we_o		),
        
        // slave 5 interface
        .s5_addr_o	(s5_addr_o		),
        .s5_data_o	(s5_data_o		),
        .s5_data_i	(s5_data_i		),
        .s5_we_o	(s5_we_o		),
		
		// pwm interface
		.pwm_addr_o	(pwm_addr_o		), // ������ӻ��� �ӻ��Ĵ�����ַ
		.pwm_data_o	(pwm_data_o		), // ����д��ӻ�������
		.pwm_data_i	(pwm_data_i		), // �ӻ����͵�����������
		.pwm_we_o  	(pwm_we_o  		), // �������뵽�ӻ���д��־

        .hold_flag_o(riscv_bus_hold_flag_o)
    );

endmodule
