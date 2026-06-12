// ==========================================================================
// ds18b20_tb.v — DS18B20 温度传感器 + 数码管显示 仿真测试平台
//
// 仿真内容:
//   1. 50 MHz 时钟 + 复位
//   2. DS18B20 从机行为模型 (1-Wire Slave)
//      - 响应复位 + 存在脉冲
//      - 接收 Skip ROM (0xCC) / Convert T (0x44) / Read Scratchpad (0xBE)
//      - 返回温度数据 (模拟 25.0°C)
//   3. 数码管输出监控
//   4. 协议时序验证
//
// 运行: 在 ModelSim 中编译并仿真 ~2 秒 (覆盖一轮完整的温度读取周期)
// ==========================================================================

// synthesis translate_off
`timescale 1ns / 1ns

module ds18b20_tb;

// ==========================================================================
// 信号声明
// ==========================================================================
reg         clk;
reg         rst_n;
wire [5:0]  sel;
wire [7:0]  dig;
wire        dq;             // 1-Wire 总线 (DUT + 从机 + 上拉)

// DUT 侧 (在 ds18b20 模块内部有三态)
// 从机侧三态
reg         slave_out;
reg         slave_oe;       // 1 = 驱动, 0 = 高阻
wire        slave_dq_drv = slave_oe ? slave_out : 1'bz;
assign      dq = slave_dq_drv;   // 从机通过三态驱动 DQ

// 上拉电阻模拟 (弱上拉)
pullup pu(dq);

// ==========================================================================
// 从机模型内部信号
// ==========================================================================
reg  [7:0]  scratchpad [0:8];    // DS18B20 暂存器 (9 字节)
reg  [7:0]  slave_cmd;           // 接收到的命令
reg  [3:0]  slave_bit;           // 位计数
reg  [7:0]  slave_shift;         // 移位寄存器
integer     i, j;
reg         converting;          // 正在转换标志

// ==========================================================================
// DUT 实例化
// ==========================================================================
ds18b20 u_dut (
    .clk    (clk),
    .rst_n  (rst_n),
    .sel    (sel),
    .dig    (dig),
    .dq     (dq)
);

// ==========================================================================
// 50 MHz 时钟 (周期 20 ns)
// ==========================================================================
initial begin
    clk = 0;
    forever #10 clk = ~clk;
end

// ==========================================================================
// 温度数据预设: 模拟 25.0°C
//   raw_temp = 25.0 / 0.0625 = 400 = 0x0190
//   Byte 0 (LSB) = 0x90
//   Byte 1 (MSB) = 0x01
// ==========================================================================
initial begin
    scratchpad[0] = 8'h90;   // 温度 LSB
    scratchpad[1] = 8'h01;   // 温度 MSB
    scratchpad[2] = 8'h4B;   // TH 寄存器
    scratchpad[3] = 8'h46;   // TL 寄存器
    scratchpad[4] = 8'h7F;   // 配置寄存器 (12-bit 分辨率)
    scratchpad[5] = 8'hFF;   // 保留
    scratchpad[6] = 8'h00;   // 保留
    scratchpad[7] = 8'h10;   // 保留
    scratchpad[8] = 8'h3E;   // CRC (近似值)
end

// ==========================================================================
// DS18B20 从机行为模型
// ==========================================================================
initial begin
    slave_oe   = 1'b0;
    slave_out  = 1'b1;
    converting = 1'b0;

    // 等待复位释放
    wait(rst_n == 1'b1);
    #1000;  // 等待系统稳定

    $display("============================================================");
    $display("  DS18B20 Temperature Sensor Simulation");
    $display("  Simulated Temperature: 25.0 C (raw=0x0190)");
    $display("  Expected Display: 25.0");
    $display("============================================================");

    // ----------------------------------------------------------------
    // 主循环: 不断响应 DUT 的 1-Wire 操作
    // ----------------------------------------------------------------
    forever begin
        // ============================================================
        // 等待并响应复位脉冲
        // ============================================================
        wait_reset();

        // ============================================================
        // 接收功能命令 (Skip ROM 之后的实际命令)
        // ============================================================
        slave_cmd = receive_byte();

        $display("  [SLAVE] Received command: 0x%02h @ %0t ns", slave_cmd, $time);

        case (slave_cmd)
            // ---- Skip ROM (0xCC): 接收下一条命令 ----
            8'hCC: begin
                $display("  [SLAVE] -> Skip ROM");
                slave_cmd = receive_byte();
                $display("  [SLAVE] Received function command: 0x%02h @ %0t ns",
                         slave_cmd, $time);

                case (slave_cmd)
                    // ---- Convert T (0x44): 启动温度转换 ----
                    8'h44: begin
                        $display("  [SLAVE] -> Convert T: starting conversion...");
                        converting = 1'b1;
                        // 实际需要 750ms, 仿真中缩短到 100ms
                        #100_000_000;
                        converting = 1'b0;
                        $display("  [SLAVE] -> Conversion complete");
                    end

                    // ---- Read Scratchpad (0xBE): 返回暂存器数据 ----
                    8'hBE: begin
                        $display("  [SLAVE] -> Read Scratchpad: sending 2 bytes");
                        // DS18B20 返回 9 字节, DUT 只读前 2 字节 (温度)
                        send_byte(scratchpad[0]);  // 温度 LSB
                        send_byte(scratchpad[1]);  // 温度 MSB
                        $display("  [SLAVE] -> Temperature bytes sent: 0x%02x 0x%02x",
                                 scratchpad[0], scratchpad[1]);
                    end

                    default: begin
                        $display("  [SLAVE] -> Unknown function command: 0x%02x", slave_cmd);
                    end
                endcase
            end

            // ---- 其他 ROM 命令 ----
            8'h33: begin  // Read ROM
                $display("  [SLAVE] -> Read ROM (not fully implemented)");
                // 发送 64-bit ROM code placeholder
                for (j = 0; j < 8; j = j + 1)
                    send_byte(8'h00);
            end

            default: begin
                $display("  [SLAVE] -> Unknown ROM command: 0x%02x", slave_cmd);
            end
        endcase
    end
end

// ==========================================================================
// 从机任务: 等待并响应复位脉冲
// ==========================================================================
task wait_reset;
    reg [31:0] low_time;
    begin
        // 等待 DQ 下降沿
        @(negedge dq);
        low_time = 0;

        // 测量低电平持续时间
        while (dq === 1'b0) begin
            #1000;           // 每 1µs 检查一次
            low_time = low_time + 1;
            if (low_time > 1000) begin  // 超过 1ms, 超时
                $display("  [SLAVE] WARNING: DQ stuck low, timeout");
                @(posedge dq);
                low_time = 0;
            end
        end

        // 判断是否为复位脉冲 (低电平 > 480µs)
        if (low_time >= 400) begin
            $display("  [SLAVE] Reset detected (low=%0d us) @ %0t ns", low_time, $time);
            // 等待 30µs 后发送存在脉冲
            #30_000;
            // 拉低 120µs
            slave_oe  = 1'b1;
            slave_out = 1'b0;
            #120_000;
            slave_oe  = 1'b0;
            slave_out = 1'b1;
            #50_000;  // 等待存在检测窗口结束
            $display("  [SLAVE] Presence pulse sent");
        end else begin
            $display("  [SLAVE] Short pulse detected (low=%0d us), ignoring", low_time);
        end
    end
endtask

// ==========================================================================
// 从机任务: 接收一个字节 (LSB first)
// ==========================================================================
function [7:0] receive_byte;
    reg [7:0] result;
    integer   k;
    begin
        result = 8'h00;
        for (k = 0; k < 8; k = k + 1) begin
            @(negedge dq);      // 主机拉低, 时隙开始
            #15_000;            // 等待到采样点 (~15µs)
            result[k] = dq;     // 采样 (LSB first)
            @(posedge dq);      // 等待主机释放
            #5_000;             // 恢复时间
        end
        receive_byte = result;
    end
endfunction

// ==========================================================================
// 从机任务: 发送一个字节 (LSB first)
// ==========================================================================
task send_byte;
    input [7:0] data;
    integer k;
    begin
        for (k = 0; k < 8; k = k + 1) begin
            @(negedge dq);      // 主机拉低, 读时隙开始
            #5_000;             // 等待主机释放

            // 根据数据位驱动 DQ
            if (data[k] == 1'b0) begin
                // 发送 0: 拉低 DQ
                slave_oe  = 1'b1;
                slave_out = 1'b0;
                #45_000;        // 保持低电平到采样窗口之后
                slave_oe  = 1'b0;
            end else begin
                // 发送 1: 释放 DQ (上拉电阻保持高电平)
                slave_oe  = 1'b0;
                #45_000;        // 等待时隙结束
            end

            @(posedge dq);      // 等待时隙结束 (或主机释放)
            #5_000;             // 恢复时间
        end
    end
endtask

// ==========================================================================
// 激励生成
// ==========================================================================
initial begin
    // 上电复位
    rst_n = 0;
    #200 rst_n = 1;
    $display("  [TB] Reset released @ %0t ns", $time);

    // 等待第一轮温度读取完成 (~1.5 秒)
    #1_500_000_000;
    $display("============================================================");
    $display("  First reading cycle should be complete.");
    $display("  Check: DUT should read 25.0 C and display 25.0");
    $display("============================================================");

    // 再等一轮确认循环工作
    #1_500_000_000;
    $display("============================================================");
    $display("  Second reading cycle should be complete.");
    $display("============================================================");

    // 仿真运行 5 秒后结束
    #2_000_000_000;
    $display("============================================================");
    $display("  Simulation finished (5 seconds elapsed)");
    $display("============================================================");
    $finish;
end

// ==========================================================================
// 数码管输出监控
// ==========================================================================
// 检测 sel/dig 变化, 打印温度显示情况
reg        prev_dp;
reg [5:0]  prev_sel;
reg [7:0]  prev_dig;

always @(sel or dig) begin
    if (sel !== prev_sel || dig !== prev_dig) begin
        prev_sel <= sel;
        prev_dig <= dig;
        // 仅在选中且段码有效时打印
        if (sel != 6'b111111 && dig != 8'hFF) begin
            // 可选: 打印每次显示刷新 (非常频繁, 默认关闭)
            // $display("  [DISP] SEL=%b DIG=%b @ %0t", sel, dig, $time);
        end
    end
end

endmodule
// synthesis translate_on
