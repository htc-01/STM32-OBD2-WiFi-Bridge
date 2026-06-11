/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CAN OBD2 Dashboard - Multi-Protocol Auto-Detect
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DBG_UART &huart1

// 串口DMA发送环形缓冲区
#define UART_TX_BUF_SIZE 512
static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart_tx_wr = 0;
static volatile uint16_t uart_tx_rd = 0;
static volatile uint8_t uart_tx_dma_busy = 0;
static volatile uint16_t uart_tx_dma_len = 0;

// DMA句柄
DMA_HandleTypeDef hdma_usart1_tx;

// PID发送间隔 (ms)
#define PID_SEND_INTERVAL 5

// OBD2 CAN 协议定义 (ELM327 编号)
typedef enum {
    PROTO_NONE = 0,
    PROTO_CAN_11B_500K = 6,
    PROTO_CAN_11B_250K = 8,
    PROTO_CAN_29B_500K = 7,
    PROTO_CAN_29B_250K = 9,
} OBD2_Protocol_t;

static OBD2_Protocol_t obd2_protocol = PROTO_NONE;

// PID 轮询列表
#define PID_COUNT 6
static const uint8_t obd2_pid_list[PID_COUNT] = {
    0x04, 0x05, 0x0C, 0x0D, 0x10, 0x11
};
static const char* pid_names[PID_COUNT] = {
    "LOAD", "TMP", "RPM", "SPD", "MAF", "THR"
};

// 解析后的数据
static volatile int16_t pid_values[PID_COUNT] = {-1, -1, -1, -1, -1, -1};
static volatile uint8_t  pid_valid[PID_COUNT] = {0, 0, 0, 0, 0, 0};
static volatile uint32_t pid_rx_count = 0;
static volatile uint32_t pid_tx_count = 0;
static volatile uint32_t pid_timeout_count = 0;
static volatile uint8_t  rx_ready = 0;

static CAN_RxHeaderTypeDef rx_header_buf;
static uint8_t rx_data_buf[8];

// 状态机
typedef enum { OBD2_IDLE, OBD2_WAITING } OBD2_State_t;
static volatile OBD2_State_t obd2_state = OBD2_IDLE;
static volatile uint32_t obd2_wait_tick = 0;
static volatile uint8_t obd2_current_pid_idx = 0;

/* 串口接收缓冲区（参考 car111plus 帧格式） */
#define UART_RX_BUF_SIZE 128
static uint8_t uart_rx_byte;
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint8_t uart_rx_wr_idx = 0;
static volatile uint8_t uart_rx_ready = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void ESP_PowerOn(void);
static void ESP_PowerOff(void);
static void ESP_HardReset(void);
static void MX_DMA_Init(void);
static void UART_TxBuf_StartDMA(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- 串口DMA发送 ---------- */
static void UART_TxBuf_StartDMA(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (uart_tx_dma_busy) {
        __set_PRIMASK(primask);
        return;
    }

    uint16_t rd = uart_tx_rd;
    uint16_t wr = uart_tx_wr;

    if (rd == wr) {
        __set_PRIMASK(primask);
        return;
    }

    uint16_t len = (wr > rd) ? (wr - rd) : (UART_TX_BUF_SIZE - rd);
    uart_tx_dma_busy = 1;
    uart_tx_dma_len = len;
    __set_PRIMASK(primask);

    if (HAL_UART_Transmit_DMA(&huart1, &uart_tx_buf[rd], len) != HAL_OK) {
        __disable_irq();
        uart_tx_dma_busy = 0;
        __enable_irq();
    }
}

int _write(int file, char *ptr, int len)
{
    (void)file;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    int written = 0;
    for (int i = 0; i < len; i++) {
        uint16_t next = (uart_tx_wr + 1) % UART_TX_BUF_SIZE;
        if (next == uart_tx_rd) break; // 缓冲区满
        uart_tx_buf[uart_tx_wr] = ptr[i];
        uart_tx_wr = next;
        written++;
    }

    __set_PRIMASK(primask);

    UART_TxBuf_StartDMA();
    return written;
}

int __io_putchar(int ch)
{
    char c = ch;
    _write(0, &c, 1);
    return ch;
}

/* DMA发送完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    __disable_irq();
    uart_tx_rd = (uart_tx_rd + uart_tx_dma_len) % UART_TX_BUF_SIZE;
    uart_tx_dma_busy = 0;
    __enable_irq();

    UART_TxBuf_StartDMA();
}

/* ESP-01 电源控制 -----------------------------------------------------------*/
// 警告：PB15(esp_gnd) 被直接用作 ESP-01 的功率地。GPIO 引脚不适合承载 ESP-01
// 的峰值电流(>300mA)，长期/大量使用可能导致 STM32 复位或损坏。强烈建议将
// ESP-01 的 GND 直接接到系统公共地（如排针 GND、面包板公共地），而非 GPIO。
//
// 以下宏定义电源和使能的有效电平，请根据实际电路修改：
//   GPIO_PIN_SET   = 高电平有效（高侧开关 / N-MOS+驱动）
//   GPIO_PIN_RESET = 低电平有效（PMOS 高侧开关 / 低使能 LDO）
#define ESP_PWR_ACTIVE_LEVEL    GPIO_PIN_SET    // esp_3v3 供电开关有效电平
#define ESP_EN_ACTIVE_LEVEL     GPIO_PIN_SET    // esp_en  CH_PD/EN 有效电平

static void ESP_PowerOn(void)
{
    GPIO_PinState en_off = (ESP_EN_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    // PB15(esp_gnd) 已作为功率地使用，必须始终保持低电平，严禁拉高！
    // CubeMX 初始化时已将其设为 RESET(低)，此处不再操作。

    // 1. EN先拉低，确保ESP处于复位状态
    HAL_GPIO_WritePin(esp_en_GPIO_Port, esp_en_Pin, en_off);

    // 2. 开启3.3V供电
    HAL_GPIO_WritePin(esp_3v3_GPIO_Port, esp_3v3_Pin, ESP_PWR_ACTIVE_LEVEL);

    // 3. 等待电源稳定（至少100ms，留200ms余量）
    HAL_Delay(200);

    // 4. 拉高EN释放复位，ESP开始从Flash启动
    HAL_GPIO_WritePin(esp_en_GPIO_Port, esp_en_Pin, ESP_EN_ACTIVE_LEVEL);

    // 5. 等待ESP-01启动完成（典型500ms~1s）
    HAL_Delay(1000);
    printf("[ESP] Power on sequence completed\r\n");
}

static void ESP_PowerOff(void)
{
    GPIO_PinState en_off = (ESP_EN_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    GPIO_PinState pwr_off = (ESP_PWR_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    // 1. 先拉低EN，让ESP进入复位
    HAL_GPIO_WritePin(esp_en_GPIO_Port, esp_en_Pin, en_off);
    HAL_Delay(50);

    // 2. 关闭3.3V供电
    HAL_GPIO_WritePin(esp_3v3_GPIO_Port, esp_3v3_Pin, pwr_off);

    // 注意：此处严禁操作 PB15(esp_gnd)。若拉高，ESP-01 将因 GND 被拉到 3.3V 而烧毁。

    printf("[ESP] Powered off\r\n");
}

static void ESP_HardReset(void)
{
    printf("[ESP] Hard resetting...\r\n");
    ESP_PowerOff();
    HAL_Delay(500);
    ESP_PowerOn();
}

// 检查响应 ID 是否有效
// 11-bit: 0x7E8 ~ 0x7EF
// 29-bit: 0x18DAF100 ~ 0x18DAF1EF
static int IsValidResponseID(uint32_t id, uint32_t ide)
{
    if (ide == CAN_ID_STD) {
        return (id >= 0x7E8 && id <= 0x7EF);
    } else {
        return (id >= 0x18DAF100 && id <= 0x18DAF1EF);
    }
}

// 发送原始 OBD2 请求帧
static int OBD2_SendRaw(uint32_t id, uint8_t is_ext, uint8_t pid)
{
    CAN_TxHeaderTypeDef txh;
    uint8_t txd[8];
    uint32_t mb;

    uint32_t tec = (CAN1->ESR >> 16) & 0xFF;
    uint32_t rec = (CAN1->ESR >> 24) & 0xFF;
    if (rec > 200 || tec > 100) {
        HAL_CAN_Stop(&hcan);
        HAL_CAN_Start(&hcan);
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0);
        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX1);
        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX2);
    }

    txh.StdId = is_ext ? 0 : (id & 0x7FF);
    txh.ExtId = is_ext ? id : 0;
    txh.IDE   = is_ext ? CAN_ID_EXT : CAN_ID_STD;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;
    txh.TransmitGlobalTime = DISABLE;

    txd[0] = 0x02;
    txd[1] = 0x01;
    txd[2] = pid;
    txd[3] = 0x00; txd[4] = 0x00; txd[5] = 0x00; txd[6] = 0x00; txd[7] = 0x00;

    return HAL_CAN_AddTxMessage(&hcan, &txh, txd, &mb);
}

// 根据当前协议发送请求（自动选择 11b/29b）
static int OBD2_SendRequest(uint8_t pid)
{
    uint32_t tx_id;
    uint8_t is_ext;

    switch (obd2_protocol) {
        case PROTO_CAN_29B_500K:
        case PROTO_CAN_29B_250K:
            tx_id = 0x18DB33F1;  // 29-bit 功能请求 ID
            is_ext = 1;
            break;
        default:
            tx_id = 0x7DF;       // 11-bit 功能请求 ID
            is_ext = 0;
            break;
    }
    return OBD2_SendRaw(tx_id, is_ext, pid);
}

// 解析 PID 数据
static void ParsePID(uint8_t pid, uint8_t *data, uint8_t dlc)
{
    int16_t value = -1;
    switch (pid) {
        case 0x04: // 计算机负载 (%)
            if (dlc >= 4) value = (int16_t)(data[3] * 100 / 255);
            break;
        case 0x05: // 冷却液温度 (°C)
            if (dlc >= 4) value = (int16_t)data[3] - 40;
            break;
        case 0x0C: // 发动机转速 (RPM)
            if (dlc >= 5) value = (int16_t)((data[3] * 256 + data[4]) / 4);
            break;
        case 0x0D: // 车速 (km/h)
            if (dlc >= 4) value = (int16_t)data[3];
            break;
        case 0x10: // 空气流量 (g/s)
            if (dlc >= 5) value = (int16_t)((data[3] * 256 + data[4]) / 100);
            break;
        case 0x11: // 节气门位置 (%)
            if (dlc >= 4) value = (int16_t)(data[3] * 100 / 255);
            break;
    }
    for (int i = 0; i < PID_COUNT; i++) {
        if (obd2_pid_list[i] == pid) {
            pid_values[i] = value;
            pid_valid[i] = 1;
            break;
        }
    }
}

// 配置 CAN 波特率并重启
static int CAN_SetBaudrate(uint32_t prescaler)
{
    HAL_CAN_Stop(&hcan);
    hcan.Init.Prescaler = prescaler;
    if (HAL_CAN_Init(&hcan) != HAL_OK) return -1;

    CAN_FilterTypeDef cf = {0};
    cf.FilterBank = 0;
    cf.FilterMode = CAN_FILTERMODE_IDMASK;
    cf.FilterScale = CAN_FILTERSCALE_32BIT;
    cf.FilterIdHigh = 0x0000; cf.FilterIdLow = 0x0000;
    cf.FilterMaskIdHigh = 0x0000; cf.FilterMaskIdLow = 0x0000;
    cf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    cf.FilterActivation = ENABLE;
    cf.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &cf) != HAL_OK) return -1;
    if (HAL_CAN_Start(&hcan) != HAL_OK) return -1;
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return -1;
    return 0;
}

// 上电协议自动检测
static OBD2_Protocol_t OBD2_DetectProtocol(void)
{
    const struct {
        OBD2_Protocol_t proto;
        uint32_t prescaler;
        uint8_t is_ext;
        uint32_t tx_id;
        const char* name;
    } tests[] = {
        {PROTO_CAN_11B_500K, 4, 0, 0x7DF,      "ISO15765-4 CAN 11b 500k"},   // 36M/(4*18)=500k
        {PROTO_CAN_11B_250K, 8, 0, 0x7DF,      "ISO15765-4 CAN 11b 250k"},   // 36M/(8*18)=250k
        {PROTO_CAN_29B_500K, 4, 1, 0x18DB33F1, "ISO15765-4 CAN 29b 500k"},   // 36M/(4*18)=500k
        {PROTO_CAN_29B_250K, 8, 1, 0x18DB33F1, "ISO15765-4 CAN 29b 250k"},   // 36M/(8*18)=250k
    };

    printf("[DETECT] Scanning OBD2 CAN protocols...\r\n");

    for (int i = 0; i < 4; i++) {
        printf("[DETECT] Trying %s... ", tests[i].name);

        if (CAN_SetBaudrate(tests[i].prescaler) != 0) {
            printf("INIT FAIL\r\n");
            continue;
        }
        HAL_Delay(50);

        // 发送 PID 0x0C(转速) 测试请求，该 PID 几乎所有 ECU 都支持
        if (OBD2_SendRaw(tests[i].tx_id, tests[i].is_ext, 0x0C) == HAL_OK) {
            uint32_t start = HAL_GetTick();
            int found = 0;
            while (HAL_GetTick() - start < 500) {
                if (rx_ready) {
                    rx_ready = 0;
                    uint32_t rid = (rx_header_buf.IDE == CAN_ID_EXT) ? rx_header_buf.ExtId : rx_header_buf.StdId;
                    if (IsValidResponseID(rid, rx_header_buf.IDE)) {
                        found = 1;
                        break;
                    }
                }
            }
            if (found) {
                printf("OK\r\n");
                printf("[DETECT] Protocol detected: %s\r\n", tests[i].name);
                // 清空检测阶段残留的 CAN 接收帧，避免污染正式轮询
                CAN_RxHeaderTypeDef hdr;
                uint8_t d[8];
                while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
                    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hdr, d);
                }
                rx_ready = 0;
                return tests[i].proto;
            }
        }
        printf("NO RESP\r\n");
        HAL_Delay(100);
    }

    printf("[DETECT] No CAN protocol found!\r\n");
    return PROTO_NONE;
}

// CAN 接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
        uint32_t rid = (hdr.IDE == CAN_ID_EXT) ? hdr.ExtId : hdr.StdId;
        // 检测阶段或运行阶段都接收有效响应
        if (IsValidResponseID(rid, hdr.IDE)) {
            rx_header_buf = hdr;
            memcpy((void*)rx_data_buf, data, 8);
            rx_ready = 1;
        }
    }
}

/* 串口接收中断回调（参考 car111plus 帧格式：[...]） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    if (uart_rx_byte == '[') {
        uart_rx_wr_idx = 0;
        uart_rx_buf[uart_rx_wr_idx++] = uart_rx_byte;
    } else if (uart_rx_wr_idx > 0 && uart_rx_wr_idx < UART_RX_BUF_SIZE) {
        uart_rx_buf[uart_rx_wr_idx++] = uart_rx_byte;
        if (uart_rx_byte == ']') {
            uart_rx_buf[uart_rx_wr_idx] = '\0';
            uart_rx_ready = 1;
            uart_rx_wr_idx = 0;
        }
    }

    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* 解析串口接收数据（car111plus 兼容格式） */
static void ParseRxData(void)
{
    uart_rx_ready = 0;

    int load = 0, tmp = 0, rpm = 0, spd = 0, maf = 0, thr = 0;
    int n = sscanf((char *)uart_rx_buf,
                   "[LOAD=%d TMP=%d RPM=%d SPD=%d MAF=%d THR=%d]",
                   &load, &tmp, &rpm, &spd, &maf, &thr);

    if (n == 6) {
        printf("[RX] Parsed: LOAD=%d TMP=%d RPM=%d SPD=%d MAF=%d THR=%d\r\n",
               load, tmp, rpm, spd, maf, thr);
        /* 这里可以添加将接收数据应用到本地变量的逻辑 */
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */
  /* 启动串口接收中断 */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  ESP_PowerOn();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ---------- OBD2 协议自动检测 ----------
    if (obd2_protocol == PROTO_NONE) {
        obd2_protocol = OBD2_DetectProtocol();
        if (obd2_protocol == PROTO_NONE) {
            printf("[OBD2] No protocol detected, retry in 2s...\r\n");
            HAL_Delay(2000);
        } else {
            printf("[OBD2] Protocol ready, start polling PIDs\r\n");
        }
        continue;
    }

    // ---------- 串口接收处理 ----------
    if (uart_rx_ready) {
        ParseRxData();
    }

    // ---------- PID 轮询状态机 ----------
    static uint32_t last_pid_send_tick = 0;
    if (obd2_state == OBD2_IDLE) {
        uint32_t now = HAL_GetTick();
        if (now - last_pid_send_tick >= PID_SEND_INTERVAL) {
            last_pid_send_tick = now;
            uint8_t pid = obd2_pid_list[obd2_current_pid_idx];
            if (OBD2_SendRequest(pid) == HAL_OK) {
                obd2_state = OBD2_WAITING;
                obd2_wait_tick = now;
                pid_tx_count++;
            }
        }
    }
    else if (obd2_state == OBD2_WAITING) {
        if (rx_ready) {
            rx_ready = 0;

            // 从 OBD2 响应帧中提取实际 PID（ISO 15765-4 单帧格式：data[1]=0x41, data[2]=PID）
            if (rx_header_buf.DLC >= 3 && rx_data_buf[1] == 0x41) {
                uint8_t resp_pid = rx_data_buf[2];
                ParsePID(resp_pid, (uint8_t*)rx_data_buf, rx_header_buf.DLC);
                pid_rx_count++;

                // 同步状态机索引到实际响应的 PID，防止超时/延迟导致的错位累积
                for (int i = 0; i < PID_COUNT; i++) {
                    if (obd2_pid_list[i] == resp_pid) {
                        obd2_current_pid_idx = i;
                        break;
                    }
                }
            }

            // 按 car111plus 兼容格式输出完整数据帧
            printf("[LOAD=%d TMP=%d RPM=%d SPD=%d MAF=%d THR=%d]\r\n",
                   (int)pid_values[0], (int)pid_values[1], (int)pid_values[2],
                   (int)pid_values[3], (int)pid_values[4], (int)pid_values[5]);

            // 轮询下一个 PID
            obd2_current_pid_idx = (obd2_current_pid_idx + 1) % PID_COUNT;
            obd2_state = OBD2_IDLE;
        }
        else if (HAL_GetTick() - obd2_wait_tick > 100) {  // 100ms 超时
            pid_timeout_count++;
            obd2_current_pid_idx = (obd2_current_pid_idx + 1) % PID_COUNT;
            obd2_state = OBD2_IDLE;
        }
    }
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_usart1_tx.Instance = DMA1_Channel4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
  HAL_DMA_Init(&hdma_usart1_tx);

  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;                // 500k @ 36MHz APB1: 36M/(4*18) = 500k
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;      // 位时间 = 1+15+2 = 18 TQ
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;  // 发送失败自动重试
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

  // 链接DMA到UART发送
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, esp_en_Pin|esp_gnd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(esp_3v3_GPIO_Port, esp_3v3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : esp_en_Pin */
  GPIO_InitStruct.Pin = esp_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(esp_en_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : esp_gnd_Pin */
  GPIO_InitStruct.Pin = esp_gnd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(esp_gnd_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : esp_3v3_Pin */
  GPIO_InitStruct.Pin = esp_3v3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(esp_3v3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
