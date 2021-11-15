

`include "defines.v"

// 通用寄存器模块
module regs(

    input wire clk,
    input wire rst,

    // from ex
    input wire we_i,                      // 写寄存器标志
    input wire[`RegAddrBus] waddr_i,      // 写寄存器地址
    input wire[`RegBus] wdata_i,          // 写寄存器数据

    // from jtag
    input wire jtag_we_i,                 // 写寄存器标志
    input wire[`RegAddrBus] jtag_addr_i,  // 读、写寄存器地址
    input wire[`RegBus] jtag_data_i,      // 写寄存器数据

    // from id
    input wire[`RegAddrBus] raddr1_i,     // 读寄存器1地址

    // to id
    output reg[`RegBus] rdata1_o,         // 读寄存器1数据

    // from id
    input wire[`RegAddrBus] raddr2_i,     // 读寄存器2地址

    // to id
    output reg [`RegBus] rdata2_o,         // 读寄存器2数据

    // to jtag
    output reg [`RegBus] jtag_data_o,      // 读寄存器数据

    output wire [`RegBus] regs_26,
    output wire [`RegBus] regs_27
    );

    reg[`RegBus] regs_1[0:`RegNum - 1];
	reg[`RegBus] regs_2[0:`RegNum - 1];
	reg[`RegBus] regs_jtag[0:`RegNum - 1];


    assign regs_26 = regs_1[26];
    assign regs_27 = regs_1[27];

    // 写寄存器
    always @ (posedge clk) begin
        if (rst == `RstDisable) begin
            // 优先ex模块写操作
            if ((we_i == `WriteEnable) && (waddr_i != `ZeroReg)) begin
                regs_1[waddr_i]    <= wdata_i;
				regs_2[waddr_i]    <= wdata_i;
				regs_jtag[waddr_i] <= wdata_i;
            end else if ((jtag_we_i == `WriteEnable) && (jtag_addr_i != `ZeroReg)) begin
                regs_1[jtag_addr_i] <= jtag_data_i;
				regs_2[waddr_i]   <= jtag_data_i;
				regs_jtag[waddr_i]<= jtag_data_i;
            end
        end
    end

    // 读寄存器1
	//assign rdata1_o = (raddr1_i == `ZeroReg) ? `ZeroWord : ((raddr1_i == waddr_i && we_i == `WriteEnable) ? wdata_i : regs[raddr1_i]); 
    always @ (*) begin
        if (raddr1_i == `ZeroReg) begin
            rdata1_o = `ZeroWord;
        //如果读地址等于写地址，并且正在写操作，则直接返回写数据
        end else if (raddr1_i == waddr_i && we_i == `WriteEnable) begin
            rdata1_o = wdata_i;
        end else begin
            rdata1_o = regs_1[raddr1_i];
        end
    end

    // 读寄存器2
	//assign rdata2_o = (raddr2_i == `ZeroReg) ? `ZeroWord : ((raddr2_i == waddr_i && we_i == `WriteEnable) ? wdata_i : regs[raddr2_i]); 
    always @ (*) begin
        if (raddr2_i == `ZeroReg) begin
            rdata2_o = `ZeroWord;
        //如果读地址等于写地址，并且正在写操作，则直接返回写数据
        end else if (raddr2_i == waddr_i && we_i == `WriteEnable) begin
            rdata2_o = wdata_i;
        end else begin
            rdata2_o = regs_2[raddr2_i];
        end
    end

    // jtag读寄存器
	//assign rdata2_o = (jtag_addr_i == `ZeroReg) ? `ZeroWord : regs[jtag_addr_i]; 
    always @ (*) begin
        if (jtag_addr_i == `ZeroReg) begin
            jtag_data_o = `ZeroWord;
        end else begin
            jtag_data_o = regs_jtag[jtag_addr_i];
        end
  end

endmodule
