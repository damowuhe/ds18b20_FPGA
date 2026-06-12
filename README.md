# DS18B20 数字温度传感器 (FPGA)

基于 **Altera Cyclone IV E (EP4CE6F17C8N)** 的 DS18B20 数字温度传感器驱动，实时温度显示在 6 位共阳极数码管的最右边 3 位，精确到小数点后一位。

## 硬件平台

| 项目 | 规格 |
|------|------|
| FPGA | EP4CE6F17C8N (Cyclone IV E) |
| 开发板 | AWC_C4MB_V11 |
| 时钟 | 50 MHz 板载晶振 |
| 数码管 | CS3061BH/S 6位共阳极 7段数码管 |
| 温度传感器 | DS18B20 (1-Wire 协议) |
| 开发工具 | Quartus Prime 18.0 Standard Edition |
| 仿真工具 | ModelSim (Verilog) |

## 引脚分配

| 信号 | FPGA 引脚 | 说明 |
|------|----------|------|
| `clk` | PIN_E1 | 50 MHz 系统时钟 |
| `rst_n` | PIN_E15 | 低电平复位 (KEY1) |
| `dq` | **PIN_E6** | DS18B20 1-Wire 数据线 |
| `sel[5:0]` | B1, A2, B3, A3, B4, A4 | 数码管位选 (低有效) |
| `dig[7:0]` | A5, B8, A7, B6, B5, A6, A8, B7 | 数码管段选 (低有效) |

> **重要**: DS18B20 的 DQ 引脚 (PIN_E6) 需要外接 **4.7kΩ 上拉电阻到 3.3V**。传感器使用外部供电模式 (VDD 接 3.3V)。

## 硬件连接

```
         VCC (3.3V)
           │
          ┌┤ 4.7kΩ
          ││
          ││
    ┌─────┴┴─────┬──────────────┐
    │            │              │
   DQ (E6)     VDD           GND
    │            │              │
   FPGA       DS18B20      DS18B20
```

## 数码管显示

6 位数码管布局 (共阳极, SEL 低有效):

```
 [空]  [空]  [空]  [2]  [5.]  [3]
 SEL0  SEL1  SEL2  SEL3  SEL4  SEL5
 ← 左侧              → 右侧
```

- **SEL5 (最右)**: 十分位 (小数位)
- **SEL4**: 个位 + 小数点
- **SEL3**: 十位 (前导零消隐)
- **SEL0~SEL2 (左3位)**: 不显示

示例:
- `25.3°C` → `_ _ _ 2 5. 3`
- `5.7°C` → `_ _ _ _ 5. 7`
- 无传感器 → `_ _ _ _ _ E`

## 工作原理

### 1-Wire 协议时序

DS18B20 使用 Maxim/Dallas 1-Wire 单总线协议:

| 操作 | 低电平 | 释放/等待 | 说明 |
|------|--------|----------|------|
| 复位脉冲 | 500 µs | 70 µs | 检测传感器存在脉冲 |
| 写 "1" | 5 µs | 65 µs | 时隙总长 70 µs |
| 写 "0" | 60 µs | 10 µs | 时隙总长 70 µs |
| 读位 | 5 µs | 采样 @ 15 µs | 时隙总长 70 µs |

### 温度读取流程

```
复位 + 存在检测
    ↓
写 Skip ROM (0xCC)     ← 跳过地址匹配
    ↓
写 Convert T (0x44)    ← 启动温度转换
    ↓
等待 750 ms            ← 12位精度转换时间
    ↓
复位 + 存在检测
    ↓
写 Skip ROM (0xCC)
    ↓
写 Read Scratchpad (0xBE)  ← 读暂存器
    ↓
读温度 LSB (1字节)
    ↓
读温度 MSB (1字节)
    ↓
计算温度 (BCD) → 更新显示 → 循环
```

### 温度计算

DS18B20 12位温度寄存器格式 (有符号, LSB = 0.0625°C):

```
温度 (°C) = raw_temp / 16
显示值 (十分之一度) = (raw_temp × 10 + 8) / 16
```

示例: 25.0°C → raw = 0x0190 → `temp_tenths = 250` → 显示 `25.0`

## 项目文件

| 文件 | 说明 |
|------|------|
| `ds18b20.v` | 顶层模块 (DS18B20 驱动 + 数码管显示) |
| `ds18b20_tb.v` | 仿真测试平台 (含 1-Wire 从机行为模型) |
| `ds18b20.qpf` | Quartus Prime 项目文件 |
| `ds18b20.qsf` | 引脚分配 & 工程设置 |
| `release_nceo.tcl` | nCEO 引脚释放脚本 |

## 使用方法

### 编译下载

1. 用 **Quartus Prime 18.0** 打开 `ds18b20.qpf`
2. 执行 `Processing → Start Compilation`
3. 用 USB Blaster 连接开发板，执行 `Tools → Programmer`

### 仿真

1. 打开 ModelSim
2. 编译 `ds18b20.v` 和 `ds18b20_tb.v`
3. 运行仿真 (预设模拟温度 25.0°C)

或者在 Quartus 中使用 NativeLink 自动仿真。

## 状态机设计

FSM 共 29 个状态，使用单一 `always` 块实现，保证综合可靠性:

```
S_INIT → S_RST1_LOW → S_RST1_WAIT → S_RST1_CHECK
  → S_WR_CC_LOW → S_WR_CC_REL → S_WR_CC_NEXT   (写 0xCC)
  → S_WR_44_LOW → S_WR_44_REL → S_WR_44_NEXT   (写 0x44)
  → S_WAIT_CONV (750ms)
  → S_RST2_LOW → S_RST2_WAIT → S_RST2_CHECK
  → S_WR_CC2_LOW → S_WR_CC2_REL → S_WR_CC2_NEXT (写 0xCC)
  → S_WR_BE_LOW → S_WR_BE_REL → S_WR_BE_NEXT   (写 0xBE)
  → S_RD_LSB_LOW → S_RD_LSB_SAMPLE → S_RD_LSB_WAIT → S_RD_LSB_NEXT (读 LSB)
  → S_RD_MSB_LOW → S_RD_MSB_SAMPLE → S_RD_MSB_WAIT → S_RD_MSB_NEXT (读 MSB)
  → S_DONE → S_INIT (循环)
```

## 许可

MIT License
