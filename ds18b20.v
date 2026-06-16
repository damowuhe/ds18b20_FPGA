// ==========================================================================
// ds18b20.v — DS18B20 数字温度传感器 + 6位共阳极数码管显示
//            温度精确到小数点后2位，显示在最右边4位数码管
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
    inout  wire        dq,           // DS18B20 1-Wire 数据线
    output wire        buzzer,       // 无源蜂鸣器 PWM 驱动 (J1)
    input  wire        key5          // 低电平按键 (KEY5, PIN_F8)
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

	// ---- 蜂鸣器报警阈值 (改这里的整数即可!) ----
	localparam [5:0] TEMP_HIGH_DEG = 6'd32;      // 高温报警: > 33°C → 响
	localparam [5:0] TEMP_LOW_DEG  = 6'd31;      // 低温报警: < 27°C → 响
	// 以下自动计算, 无需手动修改
	localparam [13:0] ALARM_HIGH = TEMP_HIGH_DEG * 8'd100;
	localparam [13:0] ALARM_LOW  = TEMP_LOW_DEG  * 8'd100;
	localparam [14:0] BUZZER_DIV  = 15'd20000;   // PWM 基准
	localparam [14:0] BUZZER_HALF = 15'd10000;   // 50% 占空比

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
reg  [13:0] temp_hundredths; // 温度×100

// ---- BCD 显示位 ----
reg  [3:0]  disp_hundreds;
reg  [3:0]  disp_tens;
reg  [3:0]  disp_ones;
reg  [3:0]  disp_tenths; // 百分位

// ---- 数码管扫描 ----
reg  [13:0] scan_cnt;
wire [2:0]  active_digit;
reg  [5:0]  sel_reg;
reg  [7:0]  dig_reg;

	// ---- 蜂鸣器 PWM ----
	reg  [16:0] buzzer_cnt;
	reg         buzzer_reg;
	wire        alarm_on;
	reg  [8:0]  melody_step;
	reg  [11:0] melody_timer;

	// ---- KEY5 消抖 & 静音切换 ----
	reg         key5_sync;
	reg         key5_stable;
	reg         key5_prev;
	reg  [13:0] key5_db_cnt;
	reg         buzzer_muted;

// ==========================================================================
// 3. 引脚连接
// ==========================================================================
assign dq    = dq_oe ? dq_out : 1'bz;
assign dq_in = dq;
assign sel   = sel_reg;
assign dig   = dig_reg;
	assign buzzer = buzzer_reg;
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
        temp_hundredths <= 14'd0;
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
                        temp_hundredths <= (({4'd0, raw_temp} * 18'd100 + 18'd8) >> 4);
                    end else begin
                        temp_hundredths <= 14'd0;
                    end

                    // BCD 分解 (temp_tenths = 温度 × 10):
                    //   例: 25.3°C → temp_tenths = 253
                    //     disp_hundreds = 2  (整数十位)
                    //     disp_tens     = 5  (整数个位)
                    //     disp_ones     = 3  (小数十分位)
                    disp_hundreds <= temp_hundredths / 14'd1000;
                    disp_tens     <= (temp_hundredths / 14'd100) % 14'd10;
                    disp_ones     <= (temp_hundredths / 14'd10) % 14'd10;
                    disp_tenths   <= temp_hundredths % 14'd10;

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

// ==========================================================================
// 6. 蜂鸣器报警 + 《晴天》旋律
// ==========================================================================

// ---- KEY5 消抖 & 切换静音 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key5_sync    <= 1'b1;
        key5_stable  <= 1'b1;
        key5_prev    <= 1'b1;
        key5_db_cnt  <= 14'd0;
        buzzer_muted <= 1'b0;
    end else if (us_tick) begin
        key5_sync <= key5;
        if (key5_sync != key5_stable) begin
            if (key5_db_cnt == 14'd10000) begin
                key5_stable <= key5_sync;
                key5_db_cnt <= 14'd0;
            end else begin
                key5_db_cnt <= key5_db_cnt + 14'd1;
            end
        end else begin
            key5_db_cnt <= 14'd0;
        end
        key5_prev <= key5_stable;
        if (key5_stable == 1'b0 && key5_prev == 1'b1)
            buzzer_muted <= ~buzzer_muted;
    end
end

assign alarm_on = !buzzer_muted && sensor_ok && ((temp_hundredths > ALARM_HIGH) || (temp_hundredths < ALARM_LOW));

// ---- 毫秒节拍 ----
reg  [9:0] ms_cnt;
wire       ms_tick;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ms_cnt <= 10'd0;
    else if (us_tick) begin
        if (ms_cnt == 10'd999)
            ms_cnt <= 10'd0;
        else
            ms_cnt <= ms_cnt + 10'd1;
    end
end

assign ms_tick = (ms_cnt == 10'd999) && us_tick;

// ---- 音符半周期查找 ----
reg [16:0] melody_note;
always @(*) begin
    case (melody_step)
        0: melody_note = 17'd42550;
        1: melody_note = 17'd42550;
        2: melody_note = 17'd63750;
        3: melody_note = 17'd63750;
        4: melody_note = 17'd56800;
        5: melody_note = 17'd50600;
        6: melody_note = 17'd0;
        7: melody_note = 17'd42550;
        8: melody_note = 17'd42550;
        9: melody_note = 17'd63750;
        10: melody_note = 17'd63750;
        11: melody_note = 17'd56800;
        12: melody_note = 17'd50600;
        13: melody_note = 17'd56800;
        14: melody_note = 17'd63750;
        15: melody_note = 17'd85125;
        16: melody_note = 17'd0;
        17: melody_note = 17'd42550;
        18: melody_note = 17'd42550;
        19: melody_note = 17'd63750;
        20: melody_note = 17'd63750;
        21: melody_note = 17'd56800;
        22: melody_note = 17'd50600;
        23: melody_note = 17'd0;
        24: melody_note = 17'd50600;
        25: melody_note = 17'd56800;
        26: melody_note = 17'd50600;
        27: melody_note = 17'd47775;
        28: melody_note = 17'd50600;
        29: melody_note = 17'd56800;
        30: melody_note = 17'd47775;
        31: melody_note = 17'd50600;
        32: melody_note = 17'd56800;
        33: melody_note = 17'd63750;
        34: melody_note = 17'd85125;
        35: melody_note = 17'd63750;
        36: melody_note = 17'd63750;
        37: melody_note = 17'd50600;
        38: melody_note = 17'd47775;
        39: melody_note = 17'd50600;
        40: melody_note = 17'd56800;
        41: melody_note = 17'd63750;
        42: melody_note = 17'd56800;
        43: melody_note = 17'd50600;
        44: melody_note = 17'd50600;
        45: melody_note = 17'd50600;
        46: melody_note = 17'd50600;
        47: melody_note = 17'd56800;
        48: melody_note = 17'd50600;
        49: melody_note = 17'd56800;
        50: melody_note = 17'd63750;
        51: melody_note = 17'd85125;
        52: melody_note = 17'd63750;
        53: melody_note = 17'd63750;
        54: melody_note = 17'd50600;
        55: melody_note = 17'd47775;
        56: melody_note = 17'd50600;
        57: melody_note = 17'd56800;
        58: melody_note = 17'd63750;
        59: melody_note = 17'd56800;
        60: melody_note = 17'd50600;
        61: melody_note = 17'd50600;
        62: melody_note = 17'd50600;
        63: melody_note = 17'd50600;
        64: melody_note = 17'd56800;
        65: melody_note = 17'd50600;
        66: melody_note = 17'd56800;
        67: melody_note = 17'd63750;
        68: melody_note = 17'd67550;
        69: melody_note = 17'd63750;
        70: melody_note = 17'd63750;
        71: melody_note = 17'd63750;
        72: melody_note = 17'd63750;
        73: melody_note = 17'd67550;
        74: melody_note = 17'd63750;
        75: melody_note = 17'd63750;
        76: melody_note = 17'd63750;
        77: melody_note = 17'd63750;
        78: melody_note = 17'd63750;
        79: melody_note = 17'd63750;
        80: melody_note = 17'd67550;
        81: melody_note = 17'd63750;
        82: melody_note = 17'd63750;
        83: melody_note = 17'd63750;
        84: melody_note = 17'd63750;
        85: melody_note = 17'd63750;
        86: melody_note = 17'd63750;
        87: melody_note = 17'd67550;
        88: melody_note = 17'd63750;
        89: melody_note = 17'd63750;
        90: melody_note = 17'd63750;
        91: melody_note = 17'd63750;
        92: melody_note = 17'd63750;
        93: melody_note = 17'd63750;
        94: melody_note = 17'd42550;
        95: melody_note = 17'd42550;
        96: melody_note = 17'd42550;
        97: melody_note = 17'd0;
        98: melody_note = 17'd42550;
        99: melody_note = 17'd42550;
        100: melody_note = 17'd42550;
        101: melody_note = 17'd42550;
        102: melody_note = 17'd42550;
        103: melody_note = 17'd42550;
        104: melody_note = 17'd42550;
        105: melody_note = 17'd42550;
        106: melody_note = 17'd42550;
        107: melody_note = 17'd42550;
        108: melody_note = 17'd42550;
        109: melody_note = 17'd47775;
        110: melody_note = 17'd50600;
        111: melody_note = 17'd50600;
        112: melody_note = 17'd50600;
        113: melody_note = 17'd0;
        114: melody_note = 17'd63750;
        115: melody_note = 17'd63750;
        116: melody_note = 17'd63750;
        117: melody_note = 17'd63750;
        118: melody_note = 17'd75825;
        119: melody_note = 17'd67550;
        120: melody_note = 17'd63750;
        121: melody_note = 17'd42550;
        122: melody_note = 17'd47775;
        123: melody_note = 17'd50600;
        124: melody_note = 17'd63750;
        125: melody_note = 17'd63750;
        126: melody_note = 17'd63750;
        127: melody_note = 17'd0;
        128: melody_note = 17'd63750;
        129: melody_note = 17'd63750;
        130: melody_note = 17'd63750;
        131: melody_note = 17'd63750;
        132: melody_note = 17'd50600;
        133: melody_note = 17'd63750;
        134: melody_note = 17'd75825;
        135: melody_note = 17'd67550;
        136: melody_note = 17'd63750;
        137: melody_note = 17'd42550;
        138: melody_note = 17'd47775;
        139: melody_note = 17'd50600;
        140: melody_note = 17'd63750;
        141: melody_note = 17'd56800;
        142: melody_note = 17'd56800;
        143: melody_note = 17'd0;
        144: melody_note = 17'd0;
        145: melody_note = 17'd50600;
        146: melody_note = 17'd56800;
        147: melody_note = 17'd47775;
        148: melody_note = 17'd50600;
        149: melody_note = 17'd50600;
        150: melody_note = 17'd63750;
        151: melody_note = 17'd42550;
        152: melody_note = 17'd33750;
        153: melody_note = 17'd31875;
        154: melody_note = 17'd33750;
        155: melody_note = 17'd42550;
        156: melody_note = 17'd63750;
        157: melody_note = 17'd63750;
        158: melody_note = 17'd63750;
        159: melody_note = 17'd37900;
        160: melody_note = 17'd37900;
        161: melody_note = 17'd0;
        162: melody_note = 17'd37900;
        163: melody_note = 17'd42550;
        164: melody_note = 17'd42550;
        165: melody_note = 17'd42550;
        166: melody_note = 17'd42550;
        167: melody_note = 17'd47775;
        168: melody_note = 17'd50600;
        169: melody_note = 17'd56800;
        170: melody_note = 17'd50600;
        171: melody_note = 17'd47775;
        172: melody_note = 17'd50600;
        173: melody_note = 17'd50600;
        174: melody_note = 17'd50600;
        175: melody_note = 17'd45075;
        176: melody_note = 17'd40175;
        177: melody_note = 17'd50600;
        178: melody_note = 17'd50600;
        179: melody_note = 17'd45075;
        180: melody_note = 17'd40175;
        181: melody_note = 17'd33750;
        182: melody_note = 17'd28375;
        183: melody_note = 17'd33750;
        184: melody_note = 17'd31875;
        185: melody_note = 17'd31875;
        186: melody_note = 17'd31875;
        187: melody_note = 17'd0;
        188: melody_note = 17'd31875;
        189: melody_note = 17'd31875;
        190: melody_note = 17'd42550;
        191: melody_note = 17'd42550;
        192: melody_note = 17'd37900;
        193: melody_note = 17'd42550;
        194: melody_note = 17'd47775;
        195: melody_note = 17'd56800;
        196: melody_note = 17'd50600;
        197: melody_note = 17'd47775;
        198: melody_note = 17'd42550;
        199: melody_note = 17'd37900;
        200: melody_note = 17'd63750;
        201: melody_note = 17'd37900;
        202: melody_note = 17'd33750;
        203: melody_note = 17'd33750;
        204: melody_note = 17'd50600;
        205: melody_note = 17'd56800;
        206: melody_note = 17'd47775;
        207: melody_note = 17'd50600;
        208: melody_note = 17'd50600;
        209: melody_note = 17'd63750;
        210: melody_note = 17'd42550;
        211: melody_note = 17'd33750;
        212: melody_note = 17'd31875;
        213: melody_note = 17'd33750;
        214: melody_note = 17'd42550;
        215: melody_note = 17'd63750;
        216: melody_note = 17'd63750;
        217: melody_note = 17'd63750;
        218: melody_note = 17'd37900;
        219: melody_note = 17'd37900;
        220: melody_note = 17'd0;
        221: melody_note = 17'd37900;
        222: melody_note = 17'd42550;
        223: melody_note = 17'd42550;
        224: melody_note = 17'd42550;
        225: melody_note = 17'd42550;
        226: melody_note = 17'd47775;
        227: melody_note = 17'd50600;
        228: melody_note = 17'd56800;
        229: melody_note = 17'd50600;
        230: melody_note = 17'd47775;
        231: melody_note = 17'd50600;
        232: melody_note = 17'd50600;
        233: melody_note = 17'd50600;
        234: melody_note = 17'd45075;
        235: melody_note = 17'd40175;
        236: melody_note = 17'd50600;
        237: melody_note = 17'd50600;
        238: melody_note = 17'd45075;
        239: melody_note = 17'd40175;
        240: melody_note = 17'd33750;
        241: melody_note = 17'd28375;
        242: melody_note = 17'd33750;
        243: melody_note = 17'd31875;
        244: melody_note = 17'd31875;
        245: melody_note = 17'd31875;
        246: melody_note = 17'd0;
        247: melody_note = 17'd31875;
        248: melody_note = 17'd31875;
        249: melody_note = 17'd42550;
        250: melody_note = 17'd42550;
        251: melody_note = 17'd37900;
        252: melody_note = 17'd42550;
        253: melody_note = 17'd47775;
        254: melody_note = 17'd75825;
        255: melody_note = 17'd67550;
        256: melody_note = 17'd63750;
        257: melody_note = 17'd56800;
        258: melody_note = 17'd50600;
        259: melody_note = 17'd56800;
        260: melody_note = 17'd56800;
        261: melody_note = 17'd50600;
        262: melody_note = 17'd63750;
        263: melody_note = 17'd63750;
        264: melody_note = 17'd0;
        default: melody_note = 17'd0;
    endcase
end

// ---- 音符时长查找 ----
reg [11:0] melody_dura;
always @(*) begin
    case (melody_step)
        0: melody_dura = 12'd500;
        1: melody_dura = 12'd500;
        2: melody_dura = 12'd1000;
        3: melody_dura = 12'd500;
        4: melody_dura = 12'd500;
        5: melody_dura = 12'd500;
        6: melody_dura = 12'd500;
        7: melody_dura = 12'd500;
        8: melody_dura = 12'd500;
        9: melody_dura = 12'd500;
        10: melody_dura = 12'd250;
        11: melody_dura = 12'd250;
        12: melody_dura = 12'd250;
        13: melody_dura = 12'd250;
        14: melody_dura = 12'd500;
        15: melody_dura = 12'd500;
        16: melody_dura = 12'd500;
        17: melody_dura = 12'd500;
        18: melody_dura = 12'd500;
        19: melody_dura = 12'd1000;
        20: melody_dura = 12'd500;
        21: melody_dura = 12'd500;
        22: melody_dura = 12'd500;
        23: melody_dura = 12'd1000;
        24: melody_dura = 12'd250;
        25: melody_dura = 12'd250;
        26: melody_dura = 12'd250;
        27: melody_dura = 12'd250;
        28: melody_dura = 12'd250;
        29: melody_dura = 12'd250;
        30: melody_dura = 12'd250;
        31: melody_dura = 12'd250;
        32: melody_dura = 12'd500;
        33: melody_dura = 12'd500;
        34: melody_dura = 12'd500;
        35: melody_dura = 12'd500;
        36: melody_dura = 12'd500;
        37: melody_dura = 12'd500;
        38: melody_dura = 12'd500;
        39: melody_dura = 12'd500;
        40: melody_dura = 12'd250;
        41: melody_dura = 12'd250;
        42: melody_dura = 12'd500;
        43: melody_dura = 12'd500;
        44: melody_dura = 12'd500;
        45: melody_dura = 12'd500;
        46: melody_dura = 12'd250;
        47: melody_dura = 12'd250;
        48: melody_dura = 12'd500;
        49: melody_dura = 12'd1000;
        50: melody_dura = 12'd500;
        51: melody_dura = 12'd500;
        52: melody_dura = 12'd500;
        53: melody_dura = 12'd500;
        54: melody_dura = 12'd500;
        55: melody_dura = 12'd500;
        56: melody_dura = 12'd500;
        57: melody_dura = 12'd250;
        58: melody_dura = 12'd250;
        59: melody_dura = 12'd500;
        60: melody_dura = 12'd500;
        61: melody_dura = 12'd500;
        62: melody_dura = 12'd500;
        63: melody_dura = 12'd250;
        64: melody_dura = 12'd250;
        65: melody_dura = 12'd500;
        66: melody_dura = 12'd750;
        67: melody_dura = 12'd250;
        68: melody_dura = 12'd250;
        69: melody_dura = 12'd250;
        70: melody_dura = 12'd250;
        71: melody_dura = 12'd250;
        72: melody_dura = 12'd250;
        73: melody_dura = 12'd500;
        74: melody_dura = 12'd250;
        75: melody_dura = 12'd250;
        76: melody_dura = 12'd250;
        77: melody_dura = 12'd250;
        78: melody_dura = 12'd250;
        79: melody_dura = 12'd250;
        80: melody_dura = 12'd500;
        81: melody_dura = 12'd250;
        82: melody_dura = 12'd250;
        83: melody_dura = 12'd250;
        84: melody_dura = 12'd250;
        85: melody_dura = 12'd250;
        86: melody_dura = 12'd250;
        87: melody_dura = 12'd500;
        88: melody_dura = 12'd250;
        89: melody_dura = 12'd250;
        90: melody_dura = 12'd250;
        91: melody_dura = 12'd250;
        92: melody_dura = 12'd250;
        93: melody_dura = 12'd250;
        94: melody_dura = 12'd500;
        95: melody_dura = 12'd250;
        96: melody_dura = 12'd250;
        97: melody_dura = 12'd250;
        98: melody_dura = 12'd250;
        99: melody_dura = 12'd250;
        100: melody_dura = 12'd500;
        101: melody_dura = 12'd250;
        102: melody_dura = 12'd250;
        103: melody_dura = 12'd250;
        104: melody_dura = 12'd250;
        105: melody_dura = 12'd250;
        106: melody_dura = 12'd250;
        107: melody_dura = 12'd250;
        108: melody_dura = 12'd250;
        109: melody_dura = 12'd250;
        110: melody_dura = 12'd2000;
        111: melody_dura = 12'd1000;
        112: melody_dura = 12'd250;
        113: melody_dura = 12'd250;
        114: melody_dura = 12'd250;
        115: melody_dura = 12'd250;
        116: melody_dura = 12'd500;
        117: melody_dura = 12'd500;
        118: melody_dura = 12'd500;
        119: melody_dura = 12'd500;
        120: melody_dura = 12'd500;
        121: melody_dura = 12'd500;
        122: melody_dura = 12'd500;
        123: melody_dura = 12'd500;
        124: melody_dura = 12'd1000;
        125: melody_dura = 12'd1000;
        126: melody_dura = 12'd250;
        127: melody_dura = 12'd250;
        128: melody_dura = 12'd250;
        129: melody_dura = 12'd250;
        130: melody_dura = 12'd500;
        131: melody_dura = 12'd500;
        132: melody_dura = 12'd500;
        133: melody_dura = 12'd500;
        134: melody_dura = 12'd500;
        135: melody_dura = 12'd500;
        136: melody_dura = 12'd500;
        137: melody_dura = 12'd500;
        138: melody_dura = 12'd500;
        139: melody_dura = 12'd500;
        140: melody_dura = 12'd2000;
        141: melody_dura = 12'd1000;
        142: melody_dura = 12'd1000;
        143: melody_dura = 12'd500;
        144: melody_dura = 12'd500;
        145: melody_dura = 12'd500;
        146: melody_dura = 12'd500;
        147: melody_dura = 12'd500;
        148: melody_dura = 12'd500;
        149: melody_dura = 12'd500;
        150: melody_dura = 12'd500;
        151: melody_dura = 12'd500;
        152: melody_dura = 12'd500;
        153: melody_dura = 12'd500;
        154: melody_dura = 12'd500;
        155: melody_dura = 12'd500;
        156: melody_dura = 12'd500;
        157: melody_dura = 12'd500;
        158: melody_dura = 12'd500;
        159: melody_dura = 12'd500;
        160: melody_dura = 12'd500;
        161: melody_dura = 12'd500;
        162: melody_dura = 12'd500;
        163: melody_dura = 12'd500;
        164: melody_dura = 12'd500;
        165: melody_dura = 12'd500;
        166: melody_dura = 12'd500;
        167: melody_dura = 12'd500;
        168: melody_dura = 12'd500;
        169: melody_dura = 12'd500;
        170: melody_dura = 12'd500;
        171: melody_dura = 12'd2000;
        172: melody_dura = 12'd500;
        173: melody_dura = 12'd500;
        174: melody_dura = 12'd500;
        175: melody_dura = 12'd500;
        176: melody_dura = 12'd500;
        177: melody_dura = 12'd500;
        178: melody_dura = 12'd500;
        179: melody_dura = 12'd500;
        180: melody_dura = 12'd500;
        181: melody_dura = 12'd500;
        182: melody_dura = 12'd500;
        183: melody_dura = 12'd500;
        184: melody_dura = 12'd1000;
        185: melody_dura = 12'd500;
        186: melody_dura = 12'd500;
        187: melody_dura = 12'd500;
        188: melody_dura = 12'd500;
        189: melody_dura = 12'd500;
        190: melody_dura = 12'd500;
        191: melody_dura = 12'd500;
        192: melody_dura = 12'd500;
        193: melody_dura = 12'd500;
        194: melody_dura = 12'd500;
        195: melody_dura = 12'd500;
        196: melody_dura = 12'd500;
        197: melody_dura = 12'd500;
        198: melody_dura = 12'd500;
        199: melody_dura = 12'd750;
        200: melody_dura = 12'd250;
        201: melody_dura = 12'd1000;
        202: melody_dura = 12'd500;
        203: melody_dura = 12'd500;
        204: melody_dura = 12'd500;
        205: melody_dura = 12'd500;
        206: melody_dura = 12'd500;
        207: melody_dura = 12'd500;
        208: melody_dura = 12'd500;
        209: melody_dura = 12'd500;
        210: melody_dura = 12'd500;
        211: melody_dura = 12'd500;
        212: melody_dura = 12'd500;
        213: melody_dura = 12'd500;
        214: melody_dura = 12'd500;
        215: melody_dura = 12'd500;
        216: melody_dura = 12'd500;
        217: melody_dura = 12'd500;
        218: melody_dura = 12'd500;
        219: melody_dura = 12'd500;
        220: melody_dura = 12'd500;
        221: melody_dura = 12'd500;
        222: melody_dura = 12'd500;
        223: melody_dura = 12'd500;
        224: melody_dura = 12'd500;
        225: melody_dura = 12'd500;
        226: melody_dura = 12'd500;
        227: melody_dura = 12'd500;
        228: melody_dura = 12'd500;
        229: melody_dura = 12'd500;
        230: melody_dura = 12'd2000;
        231: melody_dura = 12'd500;
        232: melody_dura = 12'd500;
        233: melody_dura = 12'd500;
        234: melody_dura = 12'd500;
        235: melody_dura = 12'd500;
        236: melody_dura = 12'd500;
        237: melody_dura = 12'd500;
        238: melody_dura = 12'd500;
        239: melody_dura = 12'd500;
        240: melody_dura = 12'd500;
        241: melody_dura = 12'd500;
        242: melody_dura = 12'd500;
        243: melody_dura = 12'd1000;
        244: melody_dura = 12'd500;
        245: melody_dura = 12'd500;
        246: melody_dura = 12'd500;
        247: melody_dura = 12'd500;
        248: melody_dura = 12'd500;
        249: melody_dura = 12'd500;
        250: melody_dura = 12'd500;
        251: melody_dura = 12'd500;
        252: melody_dura = 12'd500;
        253: melody_dura = 12'd500;
        254: melody_dura = 12'd500;
        255: melody_dura = 12'd500;
        256: melody_dura = 12'd500;
        257: melody_dura = 12'd500;
        258: melody_dura = 12'd1000;
        259: melody_dura = 12'd500;
        260: melody_dura = 12'd500;
        261: melody_dura = 12'd1000;
        262: melody_dura = 12'd3000;
        default: melody_dura = 12'd500;
    endcase
end

localparam MELODY_LEN = 265;

// 蜂鸣器旋律 PWM 播放
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        buzzer_cnt   <= 17'd0;
        buzzer_reg   <= 1'b0;
        melody_step  <= 9'd0;
        melody_timer <= 12'd0;
    end else if (alarm_on) begin
        if (ms_tick && melody_timer == melody_dura - 1) begin
            melody_timer <= 12'd0;
            buzzer_cnt   <= 17'd0;
            if (melody_step == MELODY_LEN - 1)
                melody_step <= 9'd0;
            else
                melody_step <= melody_step + 9'd1;
        end else begin
            if (ms_tick)
                melody_timer <= melody_timer + 12'd1;
            if (melody_note == 17'd0) begin
                buzzer_cnt <= 17'd0;
                buzzer_reg <= 1'b0;
            end else if (buzzer_cnt >= melody_note - 1) begin
                buzzer_cnt <= 17'd0;
                buzzer_reg <= ~buzzer_reg;
            end else begin
                buzzer_cnt <= buzzer_cnt + 17'd1;
            end
        end
    end else begin
        buzzer_cnt   <= 17'd0;
        buzzer_reg   <= 1'b0;
        melody_step  <= 9'd0;
        melody_timer <= 12'd0;
    end
end

// 7. 数码管动态扫描显示
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
        3'd5: sel_reg = 6'b011111;   // SEL[5] 最右 (百分位)
        3'd4: sel_reg = 6'b101111;   // SEL[4] 十分位
        3'd3: sel_reg = 6'b110111;   // SEL[3] 个位 (带小数点)
        3'd2: sel_reg = 6'b111011;   // SEL[2] 十位
        3'd1: sel_reg = 6'b111101;   // SEL[1] 空
        3'd0: sel_reg = 6'b111110;   // SEL[0] 空
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
        // ---- 位5 (SEL[5] 最右): 百分位 ----
        3'd5: begin
            if (sensor_ok)
                dig_reg = seg_decode(disp_tenths);
            else
                dig_reg = seg_decode(4'd14);    // 无传感器显示 'E'
        end

        // ---- 位4 (SEL[4]): 十分位 ----
        3'd4: begin
            if (sensor_ok)
                dig_reg = seg_decode(disp_ones);
            else
                dig_reg = 8'b1111_1111;
        end

        // ---- 位3 (SEL[3]): 个位 + 小数点 ----
        3'd3: begin
            if (sensor_ok)
                dig_reg = seg_decode(disp_tens) & 8'b0111_1111;  // DP=0 亮
            else
                dig_reg = seg_decode(4'd14) & 8'b0111_1111;
        end

        // ---- 位2 (SEL[2]): 十位, 前导零消隐 ----
        3'd2: begin
            if (sensor_ok) begin
                if (disp_hundreds == 4'd0)
                    dig_reg = 8'b1111_1111;     // 消隐
                else
                    dig_reg = seg_decode(disp_hundreds);
            end else begin
                dig_reg = 8'b1111_1111;
            end
        end

        // ---- 位0~1: 左侧2位不显示 ----
        default: begin
            dig_reg = 8'b1111_1111;
        end
    endcase
end

endmodule
