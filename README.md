# yuntai

`yuntai` 是一套基于 `STM32F405xx + STM32CubeMX + FreeRTOS` 的云台控制工程。当前代码已经具备完整的板级初始化、任务调度、姿态解算、双轴电机级联 PID、USB/CAN/SPI 通信抽象和链路存活保护机制，适合作为云台控制与通信框架的主工程继续迭代。

这份 README 以当前仓库里的真实代码为准，不按“计划中的功能”描述。

## 1. 当前工程状态

当前版本更接近一套“云台联调/框架版”工程，已经跑通的主链路是：

```text
BMI088(SPI1) -> Mahony姿态解算 -> 欧拉角
                                   |-> USB CDC 回传给上位机/视觉

测试正弦目标 -> 云台角度外环 PID -> 速度内环 PID -> CAN2 -> GM6020
```

需要特别说明的是：

- 视觉 USB 接收协议已经实现了解包，`Rx_Data.Taget_Pitch` / `Rx_Data.Taget_Yaw` 可以被正确更新。
- 但当前控制任务 `Gimbal_Motor_Control_ALL_Test()` 仍在使用内置正弦测试目标，不是“视觉闭环控制版本”。
- DR16、VOFA、USART 通道等模块已经准备好接口，但还没有进入当前主业务流程。

如果你现在重新接手这个工程，可以把它理解成：

- 底层外设和通信框架已经搭好。
- 云台双轴控制任务已经能独立运行。
- 视觉输入链路已具备接入条件，但还差最后一步“把接收目标真正接进控制律”。

## 2. 启动流程

系统启动路径如下：

```text
main()
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO_Init()
  -> MX_DMA_Init()
  -> MX_CAN1_Init()
  -> MX_CAN2_Init()
  -> MX_SPI1_Init()
  -> MX_USART1_UART_Init()
  -> MX_USART6_UART_Init()
  -> MX_TIM1_Init()
  -> MX_USART3_UART_Init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
  -> StartDefaultTask()
       -> MX_USB_DEVICE_Init()
       -> Task_Init()
            -> Gimbal_Init()
       -> Task_loop()
            -> 创建 4 个业务任务
       -> defaultTask 自删除
```

这里的设计很清晰：

- `Core/Src/main.c` 只负责板级初始化和启动 RTOS。
- `Core/Src/freertos.c` 里的 `defaultTask` 只负责“拉起业务系统”。
- 真正的应用入口在 `user_file/4_Task/MyTask.c`。

## 3. 运行时任务结构

当前业务任务由 `Task_loop()` 创建，共 4 个：

| 任务 | 周期/触发方式 | 作用 |
| --- | --- | --- |
| `Gimbal_Motor_Control_ALL_Test` | `1 ms` 周期 | 读取电机反馈、Yaw 锁零、生成测试目标、执行双轴级联 PID、通过 CAN2 下发 GM6020 控制量 |
| `Gimbal_Euler` | `1 ms` 周期 | 读取 BMI088 陀螺仪/加速度计/温度，使用 DWT 或 HAL Tick 估计 dt，执行 Mahony 姿态解算并输出连续 Yaw |
| `Gimbal_Manifold_Control` | `10 ms` 周期 | 通过 USB CDC 将当前 `Pitch/Yaw` 回传给视觉或上位机 |
| `Gimbal_Task` | `1 ms` 周期 | 每 100 ms 做一次 CAN/SPI/USB 存活判断，并在在线状态切换时执行保护动作 |

几个关键点：

- `defaultTask` 不承担业务逻辑，建完任务后会 `vTaskDelete(NULL)`。
- IMU 数据就绪中断接口已经预留，但宏 `GIMBAL_IMU_DRDY_ENABLE` 当前为 `0`，所以现在走的是 1 kHz 轮询模式。
- Yaw 轴在进入闭环前必须先完成“上电锁零”，避免编码器初值导致误动作。

## 4. 目录结构

工程目录建议按“生成代码”和“手写业务代码”来理解。

```text
yuntai/
|-- Core/                      CubeMX 生成的主程序、外设初始化、FreeRTOS 入口
|-- USB_DEVICE/                CubeMX 生成的 USB CDC 设备层
|-- Drivers/                   STM32 HAL / CMSIS 官方驱动
|-- Middlewares/               FreeRTOS 与 USB Device 中间件
|-- user_file/                 手写应用代码
|   |-- 1_middlewares/
|   |   |-- Driver/            对 HAL 的二次封装：CAN / SPI / USB / USART
|   |   `-- Algorthm/          PID / Quaternion / DWT / Matrix
|   |-- 2_Device/              设备抽象：BMI088 / Motor / Manifold / DR16 / VOFA
|   |-- 3_Module/
|   |   `-- Gimbal/            云台模块主逻辑
|   `-- 4_Task/                任务创建与系统级任务入口
|-- cmake/                     工具链与 CubeMX CMake 拼装脚本
|-- CMakeLists.txt             顶层构建入口
|-- CMakePresets.json          Debug / Release 预设
|-- yuntai.ioc                 STM32CubeMX 工程文件
|-- startup_stm32f405xx.s      启动文件
`-- STM32F405XX_FLASH.ld       链接脚本
```

### 4.1 `Core/` 和 `USB_DEVICE/`

这两部分主要是 CubeMX 生成代码：

- `Core/Src/main.c`：主入口。
- `Core/Src/freertos.c`：RTOS 初始化和默认任务。
- `Core/Src/can.c` / `spi.c` / `usart.c` / `gpio.c`：外设初始化。
- `USB_DEVICE/App/usbd_cdc_if.c`：USB CDC 和自定义 `drv_usb` 的衔接点。

这部分原则上应尽量保持“可再生成”，自定义逻辑尽量放在 `USER CODE` 区域或 `user_file/` 中。

### 4.2 `user_file/1_middlewares`

这是整个工程最有价值的一层，承担“可复用基础设施”职责。

#### Driver

- `Driver/CAN/`：CAN 双缓冲接收、按标准 ID 提取、发送封装、链路存活检测。
- `Driver/SPI/`：SPI1 DMA 收发、BMI088 片选管理、事务同步、链路存活检测。
- `Driver/USB/`：USB CDC 双缓冲接收、发送忙状态管理、最小发送间隔、链路存活检测。
- `Driver/USART/`：基于 `ReceiveToIdle` 的串口双缓冲收发封装。

这层的特点是风格统一：都带有“管理对象 + 双缓冲/状态机 + Alive Guard”设计，后面继续扩功能时会很省心。

#### Algorthm

- `PID/`：通用 PID，支持输出限幅、积分限幅、目标限幅、死区、输出低通、斜率限制等。
- `Quaternion/`：Mahony 姿态更新和四元数转欧拉角。
- `DWT/`：优先用 DWT 周期计数器计算 dt，失败再回退到 HAL Tick。
- `Matrix/`：矩阵工具，目前不是主链路核心。

### 4.3 `user_file/2_Device`

这一层是“具体硬件/协议”的设备抽象。

- `BMI088/`：BMI088 初始化、陀螺仪/加速度计/温度读取、坐标系映射、姿态融合入口。
- `Motor/`：DJI 电机反馈解析、控制帧映射、单级/级联 PID 调用。
- `Manifold/`：USB 视觉协议收发，当前使用固定格式 `[Header][Pitch][Yaw][Tail]`。
- `DR16/`：遥控器数据解析与状态边沿判断，当前未接入云台主流程。
- `VOFA/`：VOFA 数据发送接口，当前未接入主流程。

### 4.4 `user_file/3_Module/Gimbal`

这是当前业务核心：

- 负责模块级初始化。
- 维护 Pitch/Yaw 目标、零点、连续角等状态。
- 承担姿态解算、控制任务、保护动作。
- 统一组织 BMI088、Motor、Manifold、PID 等子模块。

如果你要继续开发云台逻辑，这里是第一优先阅读文件。

### 4.5 `user_file/4_Task`

这里只有一层很薄的任务调度封装：

- `Task_Init()`：调用 `Gimbal_Init()`。
- `Task_loop()`：创建业务任务。

这层不做复杂业务，更像“应用启动器”。

## 5. 关键数据流

### 5.1 姿态链路

`Gimbal_Euler()` 每 1 ms 执行：

1. 通过 `BMI088_ReadGyro()` / `BMI088_ReadAccel()` / `BMI088_ReadTemp()` 采样 IMU。
2. 通过 `ALG_DWT_Timebase_GetDtS()` 计算本周期 dt。
3. 经过 `BMI088_Complementary_Filter()` 调用 Mahony 更新四元数。
4. 输出欧拉角后，对 Yaw 做“上电归零 + 连续角展开”。
5. 最终结果写入 `Gimbal_Euler_Angle_to_send`，供控制任务和 USB 回传任务使用。

### 5.2 控制链路

`Gimbal_Motor_Control_ALL_Test()` 每 1 ms 执行：

1. 从 CAN 缓冲提取 Pitch/Yaw 电机反馈。
2. 等待 Yaw 编码器就绪并完成锁零。
3. 生成 Pitch/Yaw 正弦测试目标。
4. 角度外环 PID 输出目标角速度。
5. 速度内环 PID 输出最终电机控制量。
6. 做限幅后通过 CAN2 发给 GM6020。

当前控制使用的反馈源：

- Pitch：使用 IMU 欧拉角 `-Gimbal_Euler_Angle_to_send.pitch`
- Yaw：使用连续化后的欧拉角 `Gimbal_Euler_Angle_to_send.yaw`

### 5.3 USB 视觉链路

当前 USB 协议在 `dvc_manifold.c` 中实现：

- 接收帧格式：`[Header][Pitch(float)][Yaw(float)][Tail]`
- 发送帧格式：`[Header][Pitch(float)][Yaw(float)][Tail]`

接收端支持：

- 一个回调中拼接多帧
- 单帧跨多个回调分片接收
- 帧头重同步
- Pitch/Yaw 数值合法性检查

但要再强调一次：当前收上来的 `Rx_Data.Taget_Pitch` / `Rx_Data.Taget_Yaw` 还没有参与闭环控制。

## 6. 当前硬件映射

按现有代码，大致可以整理为：

- MCU：`STM32F405xx`
- RTOS：`FreeRTOS`
- IMU：`BMI088`，走 `SPI1 + DMA`
- 云台电机：两路 `GM6020`，挂在 `CAN2`
  - Pitch 电机 ID：`4`
  - Yaw 电机 ID：`2`
- USB：`USB FS CDC`，用于视觉/上位机通信
- USART3：预留给 `DR16`
- USART1 / USART6：通用串口接口，当前主链路未使用
- EXTI：已配置 `ACCEL_INT` / `GYRO_INT` 中断脚，但默认未启用 IMU 中断驱动模式

需要注意：

- `CAN1` 已初始化，但当前云台主逻辑实际使用的是 `CAN2`。
- `BMI088_Init()` 若失败，会直接从 `Gimbal_Init()` 返回；而 `Task_loop()` 仍会继续创建任务，所以实际联调前要优先保证 IMU 上电正常。

## 7. 构建与烧录

### 7.1 构建依赖

建议准备以下环境：

- `CMake >= 3.22`
- `Ninja`
- `arm-none-eabi-gcc`
- `STM32CubeMX` 或 `STM32CubeIDE`，用于维护 `.ioc`

工程使用 `CMakePresets.json` 管理构建预设，默认是 `Ninja + gcc-arm-none-eabi`。

注意当前 `CMakePresets.json` 中写死了 Ninja 路径：

```json
"CMAKE_MAKE_PROGRAM": "C:/ST/STM32CubeCLT_1.21.0/Ninja/bin/ninja.exe"
```

如果你的本机安装路径不同，需要先改这里。

### 7.2 命令行构建

在工程根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成产物默认在：

```text
build/Debug/yuntai.elf
build/Debug/yuntai.map
```

### 7.3 烧录

可以使用以下任一方式：

- STM32CubeIDE 直接下载
- STM32CubeProgrammer / `STM32_Programmer_CLI`
- 其他 ST-Link / J-Link 工具链

命令行示例：

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/yuntai.elf -v -rst
```

## 8. 二次开发建议

如果你接下来要继续往下做，最常改的地方基本就是下面这些：

### 想改控制参数

优先看：

- `user_file/3_Module/Gimbal/Gimbal.c`
- `user_file/1_middlewares/Algorthm/PID/alg_pid.*`

包括：

- Pitch/Yaw 外环、内环 PID
- 电机输出限幅
- 角度死区
- 输出低通和斜率限制
- IMU dt 保护范围

### 想把视觉目标真正接进闭环

优先看：

- `user_file/2_Device/Manifold/dvc_manifold.c`
- `user_file/3_Module/Gimbal/Gimbal.c`

建议做法：

1. 先保留当前 USB 协议层不动。
2. 在 `Gimbal_Motor_Control_ALL_Test()` 或新的控制任务里，把 `Rx_Data.Taget_Pitch / Taget_Yaw` 接入目标生成逻辑。
3. 明确“视觉角度”是绝对角、相对角还是增量角，再决定是否要做坐标变换、零点映射和限幅。

### 想切回遥控器控制

优先看：

- `user_file/2_Device/DR16/dvc_dr16.*`
- `user_file/1_middlewares/Driver/USART/drv_usart.*`
- `Core/Src/usart.c`

当前 DR16 解析器已经完成，但还没在 `Gimbal_Init()` 和任务层中接起来。

### 想继续强化底层稳定性

优先看：

- `user_file/1_middlewares/Driver/CAN/drv_can.*`
- `user_file/1_middlewares/Driver/SPI/drv_spi.*`
- `user_file/1_middlewares/Driver/USB/drv_usb.*`

这三层已经有统一的链路在线检测设计，后续很适合继续补：

- 错误码统计
- 超时恢复策略
- 更细粒度的任务通知/队列机制

## 9. 当前版本的真实边界

为了避免后面再被文档误导，这里把目前代码的边界再单独列出来：

- 当前主控制任务是“测试版双轴闭环”，不是正式视觉自瞄闭环。
- USB 视觉协议已经能收发，但接收目标尚未用于控制。
- DR16 和 VOFA 模块存在，但尚未进入运行主线。
- IMU 中断触发模式代码已预留，但默认关闭。
- `Core/` 和 `USB_DEVICE/` 仍带有 CubeMX 再生成属性，改动时要注意保留用户代码边界。

## 10. 推荐阅读顺序

如果是重新接手这个工程，建议按这个顺序读：

1. `Core/Src/main.c`
2. `Core/Src/freertos.c`
3. `user_file/4_Task/MyTask.c`
4. `user_file/3_Module/Gimbal/Gimbal.c`
5. `user_file/2_Device/BMI088/*`
6. `user_file/2_Device/Motor/*`
7. `user_file/2_Device/Manifold/*`
8. `user_file/1_middlewares/Driver/*`
9. `user_file/1_middlewares/Algorthm/PID/*`

这样读下来，基本可以从“系统怎么起来”一路看到“数据怎么流到电机输出”。
