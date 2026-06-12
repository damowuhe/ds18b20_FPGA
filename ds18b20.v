// ==========================================================================
// ds18b20.v — DS18B20 数字温度传感器 + 6位共阳极数码管显示
//            温度精确到小数点后1位，显示在最右边3位数码管
//
// Board:   AWC_C4MB_V11
// FPGA:    EP4CE6F17C8N (Cyclone IV E)
// Clock:   50 MHz 板载晶振
// Display: CS3061BH/S 6-digit common-anode 7-segment display
// Sensor:  DS18B20 1-Wire 数字温度传感器
//          DQ → PIN_E6 (需外接 4.7kΩ 上拉电阻到 3.3V)
//          VDD → 3.3V (外部供电模式)
//
// 段选映射: DIG[7:0] = {DP, G, F, E, D, C, B, A}  (0=亮, 1=灭)
// 位选映射: SEL[5:0] → PNP S9012 → 共阳极  (0=选中, 1=关闭)
//
// 1-Wire 时序 (标准速度, 基于 1 µs 节拍):
//   复位: 主机拉低 500µs → 释放 70µs → 检测存在脉冲
//   写1: 拉低 5µs → 释放 65µs → 恢复 5µs
//   写0: 拉低 60µs → 释放 10µs → 恢复 5µs
//   读位: 拉低 5µs → 释放 → 15µs处采样 → 等待 → 恢复 5µs
//
// DS18B20 协议流程 (每轮约 1 秒, 自动循环):
//   复位+存在 → Skip ROM(0xCC) → Convert T(0x44) → 等待 750ms
//   → 复位+存在 → Skip ROM(0xCC) → Read Scratchpad(0xBE)
//   → 读温度LSB → 读温度MSB → 计算BCD → 更新显示 → 重新开始
// ==========================================================================

module ds18b20 (
    input  wire        clk,          // 50 MHz 系统时钟
    input  wire        rst_n,        // 低电平复位 (KEY1)
    output wire  [5:0] sel,          // 数码管位选 (低有效)
    output wire  [7:0] dig,          // 数码管段选 (低有效)
    inout  wire        dq            // DS18B20 1-Wire 数据线
);

// ==========================================================================
// 1. 参数 / 常量定义
// ==========================================================================

// ---- 1-Wire 时序 (单位: µs) ----
localparam [20:0] T_RST_LOW   = 21'd500;     // 复位低电平
localparam [20:0] T_RST_WAIT  = 21'd70;      // 复位释放后等待
localparam [20:0] T_PRESENCE  = 21'd240;     // 存在脉冲检测窗口
localparam [20:0] T_WR1_LOW   = 21'd5;       // 写1低电平
localparam [20:0] T_WR0_LOW   = 21'd60;      // 写0低电平
localparam [20:0] T_RD_LOW    = 21'd5;       // 读低电平
localparam [20:0] T_RD_SAMPLE = 21'd15;      // 读采样点 (距时隙起始)
localparam [20:0] T_SLOT      = 21'd70;      // 时隙总长
localparam [20:0] T_RECOV     = 21'd5;       // 位间恢复
localparam [20:0] T_750MS     = 21'd750_000; // 转换等待 750ms
localparam [20:0] T_INIT      = 21'd100;     // 上电稳定
localparam [20:0] T_RESTART   = 21'd2000;    // 循环间隔

// ---- DS18B20 命令 ----
localparam [7:0] CMD_SKIP_ROM  = 8'hCC;      // 0xCC = 11001100
localparam [7:0] CMD_CONVERT_T = 8'h44;      // 0x44 = 01000100
localparam [7:0] CMD_READ_SCR  = 8'hBE;      // 0xBE = 10111110

// ---- FSM 状态编码 (5位, 29个状态) ----
localparam [4:0]
    S_INIT          = 5'd0,      // 上电初始化
    // 第1次复位
    S_RST1_LOW      = 5'd1,      // 复位1: 拉低 DQ
    S_RST1_WAIT     = 5'd2,      // 复位1: 释放等待
    S_RST1_CHECK    = 5'd3,      // 复位1: 检测存在
    // 写 Skip ROM (0xCC)
    S_WR_CC_LOW     = 5'd4,      // 写 0xCC: 拉低
    S_WR_CC_REL     = 5'd5,      // 写 0xCC: 释放
    S_WR_CC_NEXT    = 5'd6,      // 写 0xCC: 恢复→下一位
    // 写 Convert T (0x44)
    S_WR_44_LOW     = 5'd7,      // 写 0x44: 拉低
    S_WR_44_REL     = 5'd8,      // 写 0x44: 释放
    S_WR_44_NEXT    = 5'd9,      // 写 0x44: 恢复→下一位
    // 等待转换
    S_WAIT_CONV     = 5'd10,     // 等待 750ms
    // 第2次复位
    S_RST2_LOW      = 5'd11,     // 复位2: 拉低
    S_RST2_WAIT     = 5'd12,     // 复位2: 释放等待
    S_RST2_CHECK    = 5'd13,     // 复位2: 检测存在
    // 写 Skip ROM (0xCC) 第2次
    S_WR_CC2_LOW    = 5'd14,     // 写 0xCC: 拉低
    S_WR_CC2_REL    = 5'd15,     // 写 0xCC: 释放
    S_WR_CC2_NEXT   = 5'd16,     // 写 0xCC: 恢复→下一位
    // 写 Read Scratchpad (0xBE)
    S_WR_BE_LOW     = 5'd17,     // 写 0xBE: 拉低
    S_WR_BE_REL     = 5'd18,     // 写 0xBE: 释放
    S_WR_BE_NEXT    = 5'd19,     // 写 0xBE: 恢复→下一位
    // 读温度 LSB
    S_RD_LSB_LOW    = 5'd20,     // 读 LSB: 拉低
    S_RD_LSB_SAMPLE = 5'd21,     // 读 LSB: 采样
    S_RD_LSB_WAIT   = 5'd22,     // 读 LSB: 等待时隙结束
    S_RD_LSB_NEXT   = 5'd23,     // 读 LSB: 恢复→下一位
    // 读温度 MSB
    S_RD_MSB_LOW    = 5'd24,     // 读 MSB: 拉低
    S_RD_MSB_SAMPLE = 5'd25,     // 读 MSB: 采样
    S_RD_MSB_WAIT   = 5'd26,     // 读 MSB: 等待时隙结束
    S_RD_MSB_NEXT   = 5'd27,     // 读 MSB: 恢复→下一位
    // 计算 & 显示
    S_DONE          = 5'd28;

// ==========================================================================
// 2. 内部信号
// ==========================================================================

// ---- 1 µs 节拍: 50 MHz / 50 = 1 MHz ----
reg  [5:0]  us_cnt;
wire        us_tick;

// ---- µs 定时器 (递减计数, 0 = 到达) ----
reg  [20:0] timer;
wire        timer_done;

// ---- 1-Wire I/O ----
reg         dq_out;
reg         dq_oe;          // 1=驱动输出, 0=高阻(输入+外部上拉)
wire        dq_in;

// ---- FSM ----
reg  [4:0]  state;
reg  [3:0]  bit_cnt;
reg  [7:0]  shift_reg;

// ---- 温度数据 ----
reg         sensor_ok;
reg  [15:0] raw_temp;
reg  [11:0] temp_tenths;

// ---- BCD 显示位 ----
reg  [3:0]  disp_hundreds;
reg  [3:0]  disp_tens;
reg  [3:0]  disp_ones;

// ---- 数码管扫描 ----
reg  [13:0] scan_cnt;
wire [2:0]  active_digit;
reg  [5:0]  sel_reg;
reg  [7:0]  dig_reg;

// ==========================================================================
// 3. 引脚连接
// ==========================================================================
assign dq    = dq_oe ? dq_out : 1'bz;
assign dq_in = dq;
assign sel   = sel_reg;
assign dig   = dig_reg;
assign timer_done = (timer == 21'd0);

// ==========================================================================
// 4. 1 µs 节拍生成 (50 MHz → 1 MHz tick)
// ==========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        us_cnt <= 6'd0;
    else if (us_cnt == 6'd49)
        us_cnt <= 6'd0;
    else
        us_cnt <= us_cnt + 6'd1;
end

assign us_tick = (us_cnt == 6'd49);

// ==========================================================================
// 5. DS18B20 协议状态机 (定时器 + FSM 合并为一个 always 块)
// ==========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_INIT;
        timer       <= T_INIT;
        bit_cnt     <= 4'd0;
        shift_reg   <= 8'd0;
        dq_out      <= 1'b1;
        dq_oe       <= 1'b0;      // 高阻 (= 释放, 上拉电阻拉高)
        sensor_ok   <= 1'b0;
        raw_temp    <= 16'd0;
        temp_tenths <= 12'd0;
        disp_hundreds <= 4'd0;
        disp_tens     <= 4'd0;
        disp_ones     <= 4'd0;
    end
    else if (us_tick) begin
        if (timer_done) begin
            // ============================================================
            // 定时到达 → 状态转移, 加载新状态的定时值
            // ============================================================
            case (state)

                // ----------------------------------------------------------
                // S_INIT: 上电稳定 → 开始第1次复位
                // ----------------------------------------------------------
                S_INIT: begin
                    state   <= S_RST1_LOW;
                    timer   <= T_RST_LOW;
                    dq_out  <= 1'b0;
                    dq_oe   <= 1'b1;     // 拉低 DQ, 启动复位
                end

                // ----------------------------------------------------------
                // 复位1: 拉低 500µs → 释放
                // ----------------------------------------------------------
                S_RST1_LOW: begin
                    state   <= S_RST1_WAIT;
                    timer   <= T_RST_WAIT;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;     // 释放 DQ
                end

                // ----------------------------------------------------------
                // 复位1: 等待 70µs 后检测存在脉冲
                // ----------------------------------------------------------
                S_RST1_WAIT: begin
                    state      <= S_RST1_CHECK;
                    timer      <= T_PRESENCE;
                    sensor_ok  <= (dq_in == 1'b0);  // 低电平 = 传感器存在
                end

                // ----------------------------------------------------------
                // 复位1: 存在脉冲窗口结束 → 写 Skip ROM
                // ----------------------------------------------------------
                S_RST1_CHECK: begin
                    state     <= S_WR_CC_LOW;
                    timer     <= T_WR0_LOW;   // 0xCC bit0 = 0 → WR0
                    shift_reg <= CMD_SKIP_ROM;
                    bit_cnt   <= 4'd0;
                    dq_out    <= 1'b0;
                    dq_oe     <= 1'b1;
                end

                // ==========================================================
                // 写 Skip ROM (0xCC = 11001100, LSB first)
                // ==========================================================
                S_WR_CC_LOW: begin
                    state   <= S_WR_CC_REL;
                    timer   <= T_SLOT;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;     // 释放
                end

                S_WR_CC_REL: begin
                    state   <= S_WR_CC_NEXT;
                    timer   <= T_RECOV;
                end

                S_WR_CC_NEXT: begin
                    if (bit_cnt < 4'd7) begin
                        // 发送下一位: shift_reg[1] → 新的 bit0
                        state     <= S_WR_CC_LOW;
                        timer     <= shift_reg[1] ? T_WR1_LOW : T_WR0_LOW;
                        bit_cnt   <= bit_cnt + 4'd1;
                        shift_reg <= shift_reg >> 1;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end else begin
                        // 8位发送完毕 → 写 Convert T
                        state     <= S_WR_44_LOW;
                        timer     <= T_WR0_LOW;  // 0x44 bit0 = 0
                        shift_reg <= CMD_CONVERT_T;
                        bit_cnt   <= 4'd0;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end
                end

                // ==========================================================
                // 写 Convert T (0x44 = 01000100, LSB first)
                // ==========================================================
                S_WR_44_LOW: begin
                    state   <= S_WR_44_REL;
                    timer   <= T_SLOT;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;
                end

                S_WR_44_REL: begin
                    state   <= S_WR_44_NEXT;
                    timer   <= T_RECOV;
                end

                S_WR_44_NEXT: begin
                    if (bit_cnt < 4'd7) begin
                        state     <= S_WR_44_LOW;
                        timer     <= shift_reg[1] ? T_WR1_LOW : T_WR0_LOW;
                        bit_cnt   <= bit_cnt + 4'd1;
                        shift_reg <= shift_reg >> 1;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end else begin
                        // 8位发送完毕 → 等待转换
                        state <= S_WAIT_CONV;
                        timer <= T_750MS;
                    end
                end

                // ----------------------------------------------------------
                // 等待温度转换 (750ms)
                // ----------------------------------------------------------
                S_WAIT_CONV: begin
                    state   <= S_RST2_LOW;
                    timer   <= T_RST_LOW;
                    dq_out  <= 1'b0;
                    dq_oe   <= 1'b1;     // 第2次复位开始
                end

                // ----------------------------------------------------------
                // 复位2: 拉低 → 释放 → 检测
                // ----------------------------------------------------------
                S_RST2_LOW: begin
                    state   <= S_RST2_WAIT;
                    timer   <= T_RST_WAIT;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;
                end

                S_RST2_WAIT: begin
                    state      <= S_RST2_CHECK;
                    timer      <= T_PRESENCE;
                    sensor_ok  <= (dq_in == 1'b0);
                end

                S_RST2_CHECK: begin
                    state     <= S_WR_CC2_LOW;
                    timer     <= T_WR0_LOW;  // 0xCC bit0 = 0
                    shift_reg <= CMD_SKIP_ROM;
                    bit_cnt   <= 4'd0;
                    dq_out    <= 1'b0;
                    dq_oe     <= 1'b1;
                end

                // ==========================================================
                // 写 Skip ROM 第2次 (0xCC)
                // ==========================================================
                S_WR_CC2_LOW: begin
                    state   <= S_WR_CC2_REL;
                    timer   <= T_SLOT;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;
                end

                S_WR_CC2_REL: begin
                    state   <= S_WR_CC2_NEXT;
                    timer   <= T_RECOV;
                end

                S_WR_CC2_NEXT: begin
                    if (bit_cnt < 4'd7) begin
                        state     <= S_WR_CC2_LOW;
                        timer     <= shift_reg[1] ? T_WR1_LOW : T_WR0_LOW;
                        bit_cnt   <= bit_cnt + 4'd1;
                        shift_reg <= shift_reg >> 1;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end else begin
                        state     <= S_WR_BE_LOW;
                        timer     <= T_WR0_LOW;  // 0xBE bit0 = 0
                        shift_reg <= CMD_READ_SCR;
                        bit_cnt   <= 4'd0;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end
                end

                // ==========================================================
                // 写 Read Scratchpad (0xBE = 10111110, LSB first)
                // ==========================================================
                S_WR_BE_LOW: begin
                    state   <= S_WR_BE_REL;
                    timer   <= T_SLOT;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;
                end

                S_WR_BE_REL: begin
                    state   <= S_WR_BE_NEXT;
                    timer   <= T_RECOV;
                end

                S_WR_BE_NEXT: begin
                    if (bit_cnt < 4'd7) begin
                        state     <= S_WR_BE_LOW;
                        timer     <= shift_reg[1] ? T_WR1_LOW : T_WR0_LOW;
                        bit_cnt   <= bit_cnt + 4'd1;
                        shift_reg <= shift_reg >> 1;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end else begin
                        // 0xBE 发送完毕 → 开始读温度数据
                        state     <= S_RD_LSB_LOW;
                        timer     <= T_RD_LOW;
                        shift_reg <= 8'd0;
                        bit_cnt   <= 4'd0;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end
                end

                // ==========================================================
                // 读温度 LSB (低字节, 8 bits, LSB first)
                // ==========================================================
                S_RD_LSB_LOW: begin
                    state   <= S_RD_LSB_SAMPLE;
                    timer   <= T_RD_SAMPLE;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;     // 释放
                end

                S_RD_LSB_SAMPLE: begin
                    state               <= S_RD_LSB_WAIT;
                    timer               <= T_SLOT;
                    shift_reg[bit_cnt]  <= dq_in;  // 采样
                    dq_oe               <= 1'b0;   // 保持释放
                end

                S_RD_LSB_WAIT: begin
                    state   <= S_RD_LSB_NEXT;
                    timer   <= T_RECOV;
                end

                S_RD_LSB_NEXT: begin
                    if (bit_cnt < 4'd7) begin
                        state     <= S_RD_LSB_LOW;
                        timer     <= T_RD_LOW;
                        bit_cnt   <= bit_cnt + 4'd1;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end else begin
                        // LSB 读完 → 保存并开始读 MSB
                        raw_temp[7:0] <= shift_reg;
                        state         <= S_RD_MSB_LOW;
                        timer         <= T_RD_LOW;
                        shift_reg     <= 8'd0;
                        bit_cnt       <= 4'd0;
                        dq_out        <= 1'b0;
                        dq_oe         <= 1'b1;
                    end
                end

                // ==========================================================
                // 读温度 MSB (高字节, 8 bits, LSB first)
                // ==========================================================
                S_RD_MSB_LOW: begin
                    state   <= S_RD_MSB_SAMPLE;
                    timer   <= T_RD_SAMPLE;
                    dq_out  <= 1'b1;
                    dq_oe   <= 1'b0;
                end

                S_RD_MSB_SAMPLE: begin
                    state               <= S_RD_MSB_WAIT;
                    timer               <= T_SLOT;
                    shift_reg[bit_cnt]  <= dq_in;
                    dq_oe               <= 1'b0;
                end

                S_RD_MSB_WAIT: begin
                    state   <= S_RD_MSB_NEXT;
                    timer   <= T_RECOV;
                end

                S_RD_MSB_NEXT: begin
                    if (bit_cnt < 4'd7) begin
                        state     <= S_RD_MSB_LOW;
                        timer     <= T_RD_LOW;
                        bit_cnt   <= bit_cnt + 4'd1;
                        dq_out    <= 1'b0;
                        dq_oe     <= 1'b1;
                    end else begin
                        // MSB 读完 → 保存并开始计算
                        raw_temp[15:8] <= shift_reg;
                        state          <= S_DONE;
                        timer          <= T_RESTART;
                    end
                end

                // ==========================================================
                // DONE: 计算温度 BCD 值, 然后重新开始循环
                // ==========================================================
                S_DONE: begin
                    // 温度计算:
                    //   temp_tenths = (raw_temp * 10 + 8) / 16
                    //   12位有符号值, 此处仅处理正温度 (bit[11]=0)
                    if (sensor_ok) begin
                        temp_tenths <= (({4'd0, raw_temp} * 12'd10 + 12'd8) >> 4);
                    end else begin
                        temp_tenths <= 12'd0;
                    end

                    // BCD 分解 (temp_tenths = 温度 × 10):
                    //   例: 25.3°C → temp_tenths = 253
                    //     disp_hundreds = 2  (整数十位)
                    //     disp_tens     = 5  (整数个位)
                    //     disp_ones     = 3  (小数十分位)
                    disp_hundreds <= temp_tenths / 12'd100;
                    disp_tens     <= (temp_tenths / 12'd10) % 12'd10;
                    disp_ones     <= temp_tenths % 12'd10;

                    // 重新开始下一轮温度读取
                    state   <= S_INIT;
                    timer   <= T_INIT;
                end

                default: begin
                    state <= S_INIT;
                    timer <= T_INIT;
                end

            endcase

        end else begin
            // 定时器递减 (每个 µs 减1)
            timer <= timer - 21'd1;
        end
    end
end

// ==========================================================================
// 6. 数码管动态扫描显示
// ==========================================================================

// ---- 扫描计数器 (50M / 2^14 ≈ 3052 Hz) ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        scan_cnt <= 14'd0;
    else
        scan_cnt <= scan_cnt + 14'd1;
end

assign active_digit = scan_cnt[13:11];    // 高3位: 0~5 循环

// ---- 位选译码 (低有效, 1位亮) ----
// 物理排列: SEL[5]=最右, SEL[0]=最左
always @(*) begin
    case (active_digit)
        3'd5: sel_reg = 6'b011111;   // SEL[5] 最右位 (十分位)
        3'd4: sel_reg = 6'b101111;   // SEL[4] 个位 (带小数点)
        3'd3: sel_reg = 6'b110111;   // SEL[3] 十位
        3'd2: sel_reg = 6'b111011;   // 空
        3'd1: sel_reg = 6'b111101;   // 空
        3'd0: sel_reg = 6'b111110;   // SEL[0] 最左位 (空)
        default: sel_reg = 6'b111111;
    endcase
end

// ---- 7段译码函数 (共阳极, 0=亮 1=灭) ----
// 段映射: DIG[7:0] = {DP, G, F, E, D, C, B, A}
function [7:0] seg_decode;
    input [3:0] num;
    begin
        case (num)
            4'd0:  seg_decode = 8'b1100_0000;   // 0: ABCDEF
            4'd1:  seg_decode = 8'b1111_1001;   // 1: BC
            4'd2:  seg_decode = 8'b1010_0100;   // 2: ABDEG
            4'd3:  seg_decode = 8'b1011_0000;   // 3: ABCDG
            4'd4:  seg_decode = 8'b1001_1001;   // 4: BCFG
            4'd5:  seg_decode = 8'b1001_0010;   // 5: ACDFG
            4'd6:  seg_decode = 8'b1000_0010;   // 6: ACDEFG
            4'd7:  seg_decode = 8'b1111_1000;   // 7: ABC
            4'd8:  seg_decode = 8'b1000_0000;   // 8: ABCDEFG
            4'd9:  seg_decode = 8'b1001_0000;   // 9: ABCDFG
            4'd14: seg_decode = 8'b1000_0110;   // E (错误指示)
            default: seg_decode = 8'b1111_1111;  // 全灭
        endcase
    end
endfunction

// ---- 当前位段码输出 ----
// 显示格式: [空] [空] [空]  [十位]  [个位.]  [十分位]
//           左3位(SEL0~2)   右3位(SEL3~5)
always @(*) begin
    case (active_digit)
        // ---- 位5 (最右, SEL[5]): 十分位 (小数位) ----
        3'd5: begin
            if (sensor_ok)
                dig_reg = seg_decode(disp_ones);
            else
                dig_reg = seg_decode(4'd14);    // 无传感器显示 'E'
        end

        // ---- 位4 (SEL[4]): 个位 (整数个位) + 小数点 ----
        3'd4: begin
            if (sensor_ok)
                dig_reg = seg_decode(disp_tens) & 8'b0111_1111;  // DP=0 亮
            else
                dig_reg = seg_decode(4'd14) & 8'b0111_1111;
        end

        // ---- 位3 (SEL[3]): 十位 (整数十位), 前导零消隐 ----
        3'd3: begin
            if (sensor_ok) begin
                if (disp_hundreds == 4'd0)
                    dig_reg = 8'b1111_1111;     // 消隐
                else
                    dig_reg = seg_decode(disp_hundreds);
            end else begin
                dig_reg = 8'b1111_1111;         // 无传感器时灭
            end
        end

        // ---- 位0~2: 左侧3位 (SEL[0]~SEL[2]) 不显示 ----
        default: begin
            dig_reg = 8'b1111_1111;
        end
    endcase
end

endmodule
