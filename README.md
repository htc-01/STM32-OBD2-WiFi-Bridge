# 无线汽车OBD仪表主机端 (STM32 + ESP-01)

基于 **STM32F103C8T6** 与 **ESP-01 WiFi模块** 的无线汽车OBD2仪表数据采集主机。通过CAN总线连接车辆OBD2接口，实时读取发动机运行数据，并以兼容 `car111plus` 的帧格式通过串口/WiFi无线输出，可供上位机或手机App显示。

---

## ✨ 功能特性

- **多协议自动检测**：上电自动扫描并适配常见的ISO 15765-4 CAN协议
  - CAN 11-bit / 500 kbps
  - CAN 11-bit / 250 kbps
  - CAN 29-bit / 500 kbps
  - CAN 29-bit / 250 kbps
- **PID轮询采集**：循环请求以下6个核心OBD2 PID，间隔仅 **5ms**
  | PID | 参数 | 单位 |
  |-----|------|------|
  | 0x04 | 发动机计算负载 (LOAD) | % |
  | 0x05 | 冷却液温度 (TMP) | °C |
  | 0x0C | 发动机转速 (RPM) | rpm |
  | 0x0D | 车速 (SPD) | km/h |
  | 0x10 | 空气流量 (MAF) | g/s |
  | 0x11 | 节气门位置 (THR) | % |
- **无线透传**：通过ESP-01将数据帧无线发送至上位机（需配合ESP端固件）
- **DMA串口发送**：高效非阻塞printf重定向，降低CPU占用
- **帧同步防错位**：接收响应后根据实际PID同步状态机索引，避免超时错位累积
- **CAN错误自恢复**：实时监测TEC/REC计数器，异常时自动重启CAN外设

---

## 🛠️ 硬件架构

```
┌─────────────────┐      UART 460800       ┌──────────┐      WiFi      ┌─────────┐
│   汽车OBD2口    │◄─────CAN bus──────────►│ STM32F1  │◄───USART1─────►│ ESP-01  │◄────► 上位机/手机
│   (ECU)         │   11b/29b 500k/250k    │ STM32F103│   DMA发送      │ WiFi模组 │      (Dashboard App)
└─────────────────┘                        │ C8T6     │   中断接收     └─────────┘
                                           └──────────┘
                                           主频: 72 MHz (HSE 8MHz + PLL x9)
```

### 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| CAN_RX | PA11 | CAN总线接收 |
| CAN_TX | PA12 | CAN总线发送 |
| USART1_TX | PA9 | 连接ESP-01 RX |
| USART1_RX | PA10 | 连接ESP-01 TX |
| esp_3v3 | PA8 | ESP-01 电源开关控制 |
| esp_en | PB14 | ESP-01 CH_PD/EN 使能 |
| esp_gnd | PB15 | ESP-01 地（固定低电平） |
| SWDIO | PA13 | 调试 |
| SWCLK | PA14 | 调试 |

> ⚠️ **硬件提示**：ESP-01峰值电流可达300mA以上，建议为其独立供电或使用足够的去耦电容。PB15仅作逻辑地控制，不可承载大电流。

---

## 📁 项目结构

```
.
├── Core/
│   ├── Inc/
│   │   ├── main.h                  # 主头文件（引脚定义）
│   │   ├── stm32f1xx_hal_conf.h    # HAL库配置
│   │   └── stm32f1xx_it.h          # 中断处理头文件
│   └── Src/
│       ├── main.c                  # 主程序：OBD2协议检测 + PID轮询 + 数据输出
│       ├── stm32f1xx_hal_msp.c     # HAL MSP初始化
│       ├── stm32f1xx_it.c          # 中断服务程序
│       └── system_stm32f1xx.c      # 系统时钟配置
├── Drivers/
│   ├── CMSIS/                      # ARM CMSIS核心库
│   └── STM32F1xx_HAL_Driver/       # STM32F1 HAL驱动库
├── cmake/stm32cubemx/              # CMake子项目（CubeMX生成）
├── build/                          # 构建输出目录
├── startup_stm32f103xb.s           # 启动汇编文件
├── STM32F103XX_FLASH.ld            # 链接脚本
├── CMakeLists.txt                  # CMake主配置
├── CMakePresets.json               # CMake预设
├── car.ioc                         # STM32CubeMX工程文件
└── README.md                       # 本文件
```

---

## 🔌 通信协议

### 串口/WiFi输出帧格式（兼容 car111plus）

```
[LOAD=<负载> TMP=<温度> RPM=<转速> SPD=<车速> MAF=<空流> THR=<节气门>]
```

**示例**：
```
[LOAD=23 TMP=89 RPM=2150 SPD=65 MAF=12 THR=34]
```

- 帧头：`[`
- 帧尾：`]`
- 各字段以空格分隔
- 数据通过 `printf` → DMA USART1 → ESP-01 → WiFi 发送

### 串口接收格式

主机端同样支持接收上述格式的数据帧，可用于回显或双向同步（见 `ParseRxData()`）。

---

## 🚀 快速开始

### 环境要求

- **IDE**：CLion / VS Code + Cortex-Debug / 任何支持CMake的IDE
- **工具链**：`arm-none-eabi-gcc` 或 ARM Clang
- **构建工具**：CMake >= 3.22
- **烧录器**：ST-Link V2 / J-Link / DAP-Link

### 构建与烧录

```bash
# 克隆仓库
git clone <repo-url>
cd car

# 配置并构建（以GCC为例）
cmake --preset=default
cmake --build build/Debug

# 烧录（使用OpenOCD示例）
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program build/Debug/car.elf verify reset exit"
```

或直接在CLion中选择 `STM32_GCC` / `STM32_ARMClang` 预设进行构建。

### 首次上电流程

1. 将CAN_H/CAN_L连接至汽车OBD2接口（或CAN分析仪模拟ECU）
2. 上电后STM32自动执行ESP-01上电时序（拉低EN → 开启3.3V → 延时200ms → 拉高EN）
3. 主机发送协议探测帧，自动识别车辆CAN波特率及ID类型
4. 探测成功后进入PID轮询循环，数据开始通过WiFi输出

---

## ⚙️ 核心代码逻辑

### 1. OBD2协议自动检测 (`OBD2_DetectProtocol`)

上电后依次尝试4种主流协议，发送测试PID `0x0C`（发动机转速），若在500ms内收到有效响应ID（`0x7E8~0x7EF` 或 `0x18DAF100~0x18DAF1EF`），则锁定该协议。

### 2. PID状态机轮询

```c
OBD2_IDLE  ──发送PID──►  OBD2_WAITING
     ▲                        │
     └──收到响应/超时─────────┘
```

- 发送间隔：`5ms`
- 接收超时：`100ms`
- 响应PID与状态机索引实时同步，防止错位

### 3. ESP-01电源管理 (`ESP_PowerOn / PowerOff / HardReset`)

支持软件控制ESP-01的硬开关机与复位，便于远程重启WiFi模块恢复连接。

---

## 📝 配置与调参

所有关键参数集中在 `main.c` 顶部：

| 宏定义 | 默认值 | 说明 |
|--------|--------|------|
| `PID_SEND_INTERVAL` | 5 | PID发送间隔（ms） |
| `UART_TX_BUF_SIZE` | 512 | 串口DMA发送环形缓冲区大小 |
| `ESP_PWR_ACTIVE_LEVEL` | `GPIO_PIN_SET` | ESP电源开关有效电平 |
| `ESP_EN_ACTIVE_LEVEL` | `GPIO_PIN_SET` | ESP使能引脚有效电平 |

---

## 📋 支持的车辆

本项目基于 **ISO 15765-4 (CAN)** 标准，适用于2008年以后的大部分汽油车及部分柴油车（支持OBD2 CAN协议的车型）。

常见支持品牌：
- 大众/奥迪/斯柯达（VAG集团，通常500k 29bit）
- 丰田/本田/日产/马自达（通常500k 11bit）
- 福特/通用/宝马/奔驰等

> 若车辆仅支持K-Line（ISO 9141-2 / KWP 2000）或J1850协议，则需要额外硬件（如ELM327芯片）进行协议转换。

---

## 🤝 关联项目

- **显示端/上位机**：可配合 `car111plus` 兼容格式的串口仪表APP或网页Dashboard使用
- **ESP-01固件**：需要ESP端运行透传程序（AT固件或自定义UDP/TCP透传固件）

---

## 📜 许可证

本项目基于STM32CubeMX生成的HAL库代码，MCU相关驱动版权归 STMicroelectronics 所有。用户添加的应用层代码以 MIT 许可证开源（或根据你的选择替换）。

---

## 🙋 常见问题

**Q: 连接车辆后无数据输出？**
> A: 检查CAN线序（CAN_H/CAN_L是否接反）、终端电阻（120Ω）是否接入，以及车辆是否已上电（部分车型需点火到ON档）。

**Q: 协议检测失败？**
> A: 某些车型需要在点火后数秒才开启CAN总线；也可能是250k/500k以外的非标波特率，可修改 `CAN_SetBaudrate` 中的分频值适配。

**Q: ESP-01频繁断连？**
> A: 确保供电足够（建议独立LDO 3.3V/500mA），并检查天线附近的干扰源。

---

> 🚗 **项目状态**：个人DIY项目，持续迭代中。欢迎提交Issue或PR改进协议兼容性、增加新PID支持！
