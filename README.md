# yuntai 云台工程 README

## 1. 工程简介

`yuntai` 是基于 STM32F405xx + FreeRTOS 的 RM 云台控制工程，核心链路为：

`BMI088 姿态解算 + 视觉增量输入 -> 云台目标角更新 -> 级联 PID -> CAN 下发 GM6020`

当前云台支持俯仰/偏航双轴控制，核心控制任务运行在 FreeRTOS 业务任务中。

## 2. 代码入口

- FreeRTOS 线程入口：`Core/Src/freertos.c` 的 `StartDefaultTask()`
- 任务封装：`user_file/4_Task/MyTask.c`
- 云台主逻辑：`user_file/3_Module/Gimbal/Gimbal.c`

执行顺序：

1. `Task_Init()`
2. `Gimbal_Init()`（USB/CAN/IMU/电机/PID 初始化）
3. `Task_loop()`
4. 创建任务并进入循环：
   - `Gimbal_Motor_Control_test()`
   - `Gimbal_Euler()`
   - `Gimbal_Manifold_Control()`
   - `Gimbal_Task()`

## 3. 构建环境要求

- CMake >= 3.22
- Ninja
- `arm-none-eabi-gcc` 工具链
- STM32CubeMX/CubeIDE（用于 `.ioc` 维护与代码生成）

说明：本工程预设在 `CMakePresets.json`，默认 `Ninja` + `cmake/gcc-arm-none-eabi.cmake`。

## 4. 编译命令

在 `E:\liantiao\yuntai` 下执行：

```powershell
cmake --preset Debug
cmake --build build/Debug
```

产物：

- `build/Debug/yuntai.elf`
- `build/Debug/yuntai.map`

## 5. 烧录方式

### 方式 A：CubeIDE / CubeProgrammer

1. 选择工程 `yuntai`
2. 使用 ST-Link 或 J-Link 连接
3. 烧录 `build/Debug/yuntai.elf`

### 方式 B：命令行（可选）

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/yuntai.elf -v -rst
```

## 6. 当前控制说明（当前版本）

云台控制：

- `Gimbal_Motor_Control_test()` 以 1ms 周期运行
- 视觉输入采用增量角形式：
  - `Rx_Data.Gimbal_Pitch_Angle_Increment`
  - `Rx_Data.Gimbal_Yaw_Angle_Increment`
- 目标角做软件限位保护（防止线束拉扯）
- 电机控制采用角度外环 + 速度内环级联 PID

姿态解算：

- `Gimbal_Euler()` 周期读取 BMI088
- 使用 Mahony 更新四元数并输出欧拉角
- `Gimbal_Manifold_Control()` 周期发送欧拉角到视觉端

## 7. 关键参数（调参入口）

文件：`user_file/3_Module/Gimbal/Gimbal.c`

- `GIMBAL_CTRL_PERIOD_TICK`：控制周期 tick（当前 1）
- `GIMBAL_CTRL_DT`：控制 `dt`
- `GIMBAL_PITCH_MIN_ANGLE` / `GIMBAL_PITCH_MAX_ANGLE`：俯仰限位
- `GIMBAL_YAW_MIN_ANGLE` / `GIMBAL_YAW_MAX_ANGLE`：偏航限位
- `GIMBAL_MOTOR_CMD_LIMIT`：电机输出限幅
- `GIMBAL_VISION_INC_LIMIT`：单次视觉增量限幅

电机 PID 参数：

- `Gimbal_Init()` 内 `Motor_Set_PID_Params()`（俯仰/偏航外环和内环）

## 8. USB CDC 接收链路（已替换）

文件：`USB_DEVICE/App/usbd_cdc_if.c`

当前 `CDC_Receive_FS()` 已改为：

- 在回调中统一转发到 `USB_Rx_Callback(Buf, *Len)`
- 由 `user_file/1_middlewares/Driver/USB/drv_usb.c` 完成双缓冲、回调分发与下一包接收准备

## 9. 已知注意事项

1. 本工程运行依赖 FreeRTOS。
   - 云台控制与姿态任务均使用 `vTaskDelayUntil()`。
2. 当前控制周期为 1ms。
   - 若总线或任务负载偏高，可评估降至 2ms。
3. `Gimbal_Init()` 中若 `BMI088_Init()` 失败会提前返回。
   - 此时电机初始化不会继续执行，需先保证 IMU 初始化成功。
4. 视觉接收协议解析仍是阶段化实现。
   - 当前 `dvc_manifold.c` 主要是帧头/帧尾基础校验，完整字段解析需按协议继续补齐。

## 10. 目录简表

- `Core/`：CubeMX 生成的 HAL / FreeRTOS 入口
- `USB_DEVICE/`：USB CDC 设备协议层
- `user_file/1_middlewares/`：驱动与算法
- `user_file/2_Device/`：设备层（电机、DR16、BMI088、VOFA、Manifold）
- `user_file/3_Module/Gimbal/`：云台控制模块
- `user_file/4_Task/`：任务封装入口
- `yuntai.ioc`：CubeMX 工程文件

---



