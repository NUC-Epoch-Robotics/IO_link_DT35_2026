# IO_link_APM32F405RGT0

基于 `STM32F405 + LTC2874` 的 4 通道 IO-Link Master 工程，用于驱动 `SICK DT35-B15551` 激光测距传感器，并输出 4 路测距结果。

当前工程已经实现：

- 4 路 IO-Link 端口独立任务运行
- `LTC2874` 4 通道供电、Wakeup、COM2 配置
- `DT35` 从上电到 `OPERATE` 的完整建链流程
- 读取过程数据 `PD`
- 将过程数据换算为距离值并存入数组

## 1. 工程目标

本工程的目标是：

1. 在一块 `STM32F405` 开发板上实现 4 路 IO-Link Master。
2. 每一路都可以独立连接一只 `DT35-B15551`。
3. 通过 `LTC2874` 完成 IO-Link 物理层。
4. 在 FreeRTOS 下以 4 个任务分别管理 4 路 DT35。
5. 将每一路测得的距离存入：
   - `g_dt35_distance[4]`
   - `g_actu_distance_M[4]`

## 2. 工程目录

主要目录说明：

- [APP](APP)
  - 应用层任务、DT35 状态机、过程数据解析
- [BSP](BSP)
  - 板级驱动，主要是 `LTC2874`
- [Core](Core)
  - CubeMX 生成的启动、外设、中断、FreeRTOS 入口
- [IOLink_Stack](IOLink_Stack)
  - IO-Link 协议层、UART/DMA HAL 层、校验与事务实现
- [F405_ADC](F405_ADC)
  - 同板可运行的参考示例，用于对照 IO-Link 行为
- [SICK-DX35-Extended-20200224-IODD1.1.xml](SICK-DX35-Extended-20200224-IODD1.1.xml)
  - DT35 IODD 文件
- [2874fb.pdf](2874fb.pdf)
  - LTC2874 数据手册

## 3. 4 路资源映射

当前 4 路 IO-Link 资源映射如下：

| 端口 | UART | TXEN | LTC2874 Port |
|---|---|---|---|
| Port1 | `UART5` | `TXEN1` | Port1 |
| Port2 | `UART4` | `TXEN2` | Port2 |
| Port3 | `USART3` | `TXEN3` | Port3 |
| Port4 | `USART6` | `TXEN4` | Port4 |

对应实现位置：

- [iol_port_hal.c](IOLink_Stack/iol_port_hal.c)

所有串口统一配置为：

- 波特率默认 `38400`
- `9 bit`
- `Even`
- `1 stop bit`
- `RX DMA Circular`

## 4. 关键全局变量

### 4.1 测距结果

在 [app_dt35.c](APP/sensor_dt35/app_dt35.c) 中：

```c
volatile uint16_t g_dt35_distance[DT35_PORT_COUNT];
volatile float g_actu_distance_M[DT35_PORT_COUNT];
```

含义：

- `g_dt35_distance[i]`
  - 第 `i` 路 DT35 的原始有效距离值
- `g_actu_distance_M[i]`
  - 第 `i` 路换算后的实际距离，单位：米

索引对应关系：

- `[0]` -> Port1
- `[1]` -> Port2
- `[2]` -> Port3
- `[3]` -> Port4

### 4.2 通道使能

```c
volatile uint8_t g_dt35_port_enabled[DT35_PORT_COUNT];
```

用于控制每一路是否参与运行。

例如：

```c
g_dt35_port_enabled[0] = 1; // 开启 Port1
g_dt35_port_enabled[1] = 0; // 关闭 Port2
```

## 5. 软件分层

整个工程分为 4 层。

### 5.1 应用层

文件：

- [app_dt35.c](APP/sensor_dt35/app_dt35.c)
- [app_dt35.h](APP/sensor_dt35/app_dt35.h)

职责：

- 维护每一路 DT35 的状态机
- 管理 FreeRTOS 任务
- 在 `OPERATE` 状态读取过程数据
- 解析 `PD`
- 计算距离

### 5.2 IO-Link 事务层

文件：

- [iol_master.c](IOLink_Stack/iol_master.c)
- [iol_master.h](IOLink_Stack/iol_master.h)
- [iol_utils.c](IOLink_Stack/iol_utils.c)

职责：

- 组 IO-Link 主站报文
- 计算校验
- 发起一次事务
- 读取固定长度回复
- 解析 `Direct Parameter` / `Process Data` / `ISDU`

### 5.3 IO-Link HAL 层

文件：

- [iol_port_hal.c](IOLink_Stack/iol_port_hal.c)
- [iol_port_hal.h](IOLink_Stack/iol_port_hal.h)

职责：

- 串口选择
- TXEN 控制
- UART DMA 环形接收
- Wakeup 后串口恢复
- 波特率切换
- 固定长度取回包

### 5.4 PHY / 板级层

文件：

- [ltc2874.c](BSP/LTC2874/ltc2874.c)
- [ltc2874.h](BSP/LTC2874/ltc2874.h)

职责：

- SPI 读写 LTC2874 寄存器
- 4 路端口供电
- 4 路 Wakeup
- PHY 参数设置

## 6. FreeRTOS 任务结构

当前是 **4 个独立任务**，每一路一个任务：

- `DT35_1_App_Task` -> Port1
- `DT35_2_App_Task` -> Port2
- `DT35_3_App_Task` -> Port3
- `DT35_4_App_Task` -> Port4

文件：

- [app_dt35.c](APP/sensor_dt35/app_dt35.c)
- [freertos.c](Core/Src/freertos.c)

虽然是 4 个任务，但底层 IO-Link 栈仍通过一个 `mutex` 串行访问总线事务层：

```c
static SemaphoreHandle_t s_iol_bus_mutex = NULL;
```

这样做的目的是：

- 保留 4 路独立任务结构
- 避免共享事务状态互相踩踏

## 7. IO-Link 工作流程

### 7.1 每一路端口启动流程

在 [app_dt35.c](APP/sensor_dt35/app_dt35.c) 中，每一路状态机流程如下：

1. `PORT_STATE_INACTIVE`
   - 打开该路 `LTC2874` 供电

2. `PORT_STATE_WAIT_POWER_STABLE`
   - 等待 `100 ms`

3. `PORT_STATE_WAKEUP`
   - 调用 `IOL_HAL_WakeupPort()`
   - 通过 `LTC2874` 发 `Wakeup`
   - 重建串口 DMA 接收

4. `PORT_STATE_WAIT_WAKEUP_RECOVERY`
   - 等待 `5 ms`

5. `PORT_STATE_SET_COM2`
   - 将串口切到 `COM2 = 38400`

6. `PORT_STATE_WAIT_COM2_STABLE`
   - 等待 `10 ms`

7. `PORT_STATE_STARTUP_COM2`
   - 读 `Direct Parameter 0x02`
   - 作为设备在线确认

8. `PORT_STATE_ENTER_MASTER_IDENT`
   - 向 `0x00` 写入 `0x95`

9. `PORT_STATE_ENTER_PREOPERATE`
   - 向 `0x00` 写入 `0x9A`

10. `PORT_STATE_CONFIGURE_PROCESS_DATA`
   - 通过 ISDU 把 `ProcessDataSelect` 写成 `3`

11. `PORT_STATE_ENTER_OPERATE`
   - 向 `0x00` 写入 `0x99`

12. `PORT_STATE_OPERATE`
   - 周期读过程数据

13. `PORT_STATE_FAULT`
   - 出错后等待 `1000 ms` 再重试

### 7.2 为什么要写 `ISDU 83 = 3`

这是本工程一个非常关键的兼容处理。

文件位置：

- [app_dt35.c](APP/sensor_dt35/app_dt35.c)

在 `PREOPERATE` 后、`OPERATE` 前，会执行：

```c
IOL_Master_ISDUWrite(83, 0, false, &pd_select, 1);
```

其中：

```c
pd_select = 3;
```

原因：

- 同样是 `DT35-B15551`
- 不同年份/批次的默认 `ProcessDataSelect` 不一致
- 有些传感器默认就会输出当前主站期望的 `PD`
- 有些不会输出，导致 `PD` 一直超时

把 `ISDU 83 = 3` 写进去后，可以把不同批次 DT35 的过程数据格式统一成主站当前能解析的格式。

这是当前工程中让“不同年份 DT35 都能读到数据”的关键逻辑，**不要随意删除**。

## 8. 过程数据读取

### 8.1 读取方式

在 [iol_master.c](IOLink_Stack/iol_master.c) 中：

- 默认通过 `PROCESS + TYPE_2` 读 `PD`
- 若连续超时，会自动回退：
  - `TYPE_2`
  - `TYPE_1`
  - `TYPE_0`

相关函数：

- `IOL_Master_ReadProcessData()`
- `IOL_Master_ReadProcessDataEx()`

### 8.2 自动回退机制

每个端口维护：

- `pd_timeout_count`
- `pd_type_index`

如果该路 `PD` 连续超时：

1. 先尝试降级 `TYPE_2 -> TYPE_1`
2. 再尝试 `TYPE_1 -> TYPE_0`
3. 如果仍不行，再让该路重新从 `WAKEUP` 开始建链

这部分逻辑在：

- [app_dt35.c](APP/sensor_dt35/app_dt35.c)

## 9. 距离解析与换算

### 9.1 原始过程数据解析

在 [app_dt35.c](APP/sensor_dt35/app_dt35.c) 中：

```c
raw_word = (pd_buf[0] << 8) | pd_buf[1];
raw_distance = raw_word >> 2;
```

即：

- 两字节过程数据拼成 16 位
- 右移 2 位得到有效距离值

### 9.2 距离换算

经实测标定：

- `1 m` 对应原始值 `0x00FA`

当前工程采用：

```c
g_actu_distance_M[i] = g_dt35_distance[i] * 0.004f;
```

因此：

- `g_dt35_distance[i]` 是原始距离计数
- `g_actu_distance_M[i]` 是以米为单位的实际距离

## 10. 串口输出

当前工程还会通过 `USART1` 周期发送 4 路距离值。

文件：

- [app_dt35.c](APP/sensor_dt35/app_dt35.c)

关键函数：

- `uart_send_data()`
- `DT35_App_Task()`

发送内容：

- 命令字 `Distance = 0x0001`
- 数据内容为 `g_actu_distance_M[0..3]`

## 11. 接收端示例代码

参考示例位于：

- [接收端](/E:\learn_keil\IO_link_APM32F4-5RGT0\接收端)

- 通过：
```c
float val[4];
```
储存接收到的DT35数据
## 12. Core 文件夹中必须保留的手工修改

由于 `STM32CubeMX` 重新生成代码时可能覆盖 `Core` 文件，以下内容每次生成后都要复查。

### 12.1 [main.c](Core/Src/main.c)

必须确认：

- `USER CODE BEGIN Includes` 中有：

```c
#include "iol_port_hal.h"
```

- `USER CODE BEGIN 2` 中有：

```c
IOL_HAL_Init();
```

### 12.2 [usart.c](Core/Src/usart.c)

必须确认 4 路串口 NVIC 使能和关闭仍在 `USER CODE` 区里：

- `UART4_IRQn`
- `UART5_IRQn`
- `USART3_IRQn`
- `USART6_IRQn`

### 12.3 [stm32f4xx_it.h](Core/Inc/stm32f4xx_it.h)

必须确认 `USER CODE BEGIN EFP` 中仍有：

```c
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void USART3_IRQHandler(void);
void USART6_IRQHandler(void);
```

### 12.4 [stm32f4xx_it.c](Core/Src/stm32f4xx_it.c)

必须确认 `USER CODE` 中仍有：

```c
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
```

以及 4 个中断实现：

```c
void UART4_IRQHandler(void)
{
  IOL_HAL_UART_IRQHandler(&huart4);
  HAL_UART_IRQHandler(&huart4);
}

void UART5_IRQHandler(void)
{
  IOL_HAL_UART_IRQHandler(&huart5);
  HAL_UART_IRQHandler(&huart5);
}

void USART3_IRQHandler(void)
{
  IOL_HAL_UART_IRQHandler(&huart3);
  HAL_UART_IRQHandler(&huart3);
}

void USART6_IRQHandler(void)
{
  IOL_HAL_UART_IRQHandler(&huart6);
  HAL_UART_IRQHandler(&huart6);
}
```

## 13. 编译与下载

推荐流程：

1. 使用 `STM32CubeMX` 修改 `.ioc`
2. 重新生成代码
3. 先复查 `Core` 中的用户区修改
4. 在 `Keil MDK` 中 `Rebuild`
5. 下载到板子

如果出现：

- 串口任务不进回调
- 所有通道都无数据
- 原本正常的端口突然失效

首先优先检查：

- `Core/Src/usart.c`
- `Core/Inc/stm32f4xx_it.h`
- `Core/Src/stm32f4xx_it.c`

## 14. 当前已知行为

### 14.1 已确认可工作

- 4 路任务结构可以工作
- 4 路端口可以独立运行
- `DT35-B15551` 可正常建链和测距
- 不同年份/批次的 `DT35-B15551`，在写 `ISDU 83 = 3` 后都可输出距离

### 14.2 当前设计上的说明

- 4 个任务是独立的
- 但底层事务仍通过 `mutex` 串行访问协议栈
- 这是为了避免共享事务状态冲突

### 14.3 默认 PD 兼容策略

每一路默认按：

1. `TYPE_2`
2. `TYPE_1`
3. `TYPE_0`

依次尝试。

如果一条路始终失败，只影响该路重试，不直接影响其他端口。

## 15. 关键文件索引

### 应用层

- [APP/sensor_dt35/app_dt35.c](APP/sensor_dt35/app_dt35.c)
- [APP/sensor_dt35/app_dt35.h](APP/sensor_dt35/app_dt35.h)

### IO-Link 协议层

- [IOLink_Stack/iol_master.c](IOLink_Stack/iol_master.c)
- [IOLink_Stack/iol_master.h](IOLink_Stack/iol_master.h)
- [IOLink_Stack/iol_utils.c](IOLink_Stack/iol_utils.c)

### HAL / 端口层

- [IOLink_Stack/iol_port_hal.c](IOLink_Stack/iol_port_hal.c)
- [IOLink_Stack/iol_port_hal.h](IOLink_Stack/iol_port_hal.h)

### PHY 层

- [BSP/LTC2874/ltc2874.c](BSP/LTC2874/ltc2874.c)
- [BSP/LTC2874/ltc2874.h](BSP/LTC2874/ltc2874.h)

### Core

- [Core/Src/main.c](Core/Src/main.c)
- [Core/Src/usart.c](Core/Src/usart.c)
- [Core/Inc/stm32f4xx_it.h](Core/Inc/stm32f4xx_it.h)
- [Core/Src/stm32f4xx_it.c](Core/Src/stm32f4xx_it.c)
- [Core/Src/freertos.c](Core/Src/freertos.c)

---

如果后续要继续扩展，建议优先保持这三点不变：

1. `PREOPERATE` 后写 `ISDU 83 = 3`
2. `TXEN + UART DMA Circular` 这套收发方式
3. `Core` 里的用户区中断和串口 NVIC 修改
