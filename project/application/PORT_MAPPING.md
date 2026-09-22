# application 层 C → C++ 移植规约（阶段 4）

> **执行状态：已完成**（7 个 `.cpp` + `ui/` + 两个补的底层模块，全量编译链接通过）。
> 实际施工中与本规约的偏差、以及上电前必须看的差异清单，统一记在
> **`../MIGRATION_NOTES.md` 的「阶段 4」一节**（例如：`gyro_b` 实测是 rad/s 不是 deg/s；
> 电机 PID 实际跑在 1kHz 任务合成的 tick 上；`bsp_can` 为 MI 电机补了扩展帧支持）。

> 目标：把 `application/` 的整车控制逻辑（`robot.c` `chassis.c` `gimbal.c` `shoot.c` `cmd/robot_cmd.c` `gimbal_algorithm.c`）
> **逐逻辑**改写成 C++，底层换成已迁入的 `project/bsp` + `project/modules`。
> 本仓 `Modules/` 与 `bsp/`（C）**不再参与编译**，也不从中移植（例外见 §5）。
> 原 C 文件保留在同目录作对照，但**不编译**（只编译 `application/**/*.cpp`）。

## 1. 硬性规则

1. **控制逻辑逐行对照**：状态机、判断条件、参数、限幅、时序（`osDelay` 周期）都不许"顺手优化"。看不懂或有疑问的地方，保留原逻辑并在注释里标 `// TODO(移植): ...`。
2. **禁堆**：不用 `new/malloc/std::`。对象一律静态实例或成员。
3. **禁异常/RTTI**（编译选项已 `-fno-exceptions -fno-rtti`），不要用 `try/catch`、`dynamic_cast`。
4. **命名**：保持原 C 的公共函数名（`ChassisInit` → `ChassisInit`，或按新框架风格 `Chassis_Init`——**但一旦定下，`robot.cpp` 必须按它调用**，见 §4 冻结清单）。
5. **消息中心全部去掉**（`SubRegister/SubGetMessage/PubRegister/PubPushMessage`）→ 改成**直接调用**：
   - 每类数据一个全局实例，定义在拥有它的模块 `.cpp` 里，`extern` 声明放在对应 `.h`：
     - `Chassis_Ctrl_Cmd_s chassis_cmd_recv;`（定义在 `robot_cmd.cpp`，`extern` 声明在 `robot_cmd.h`）
     - `Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;`（同上）
     - `Shoot_Ctrl_Cmd_s shoot_cmd_recv;`（同上）
     - `Chassis_Upload_Data_s chassis_feedback_data;`（定义在 `chassis.cpp`）
     - `Gimbal_Upload_Data_s gimbal_feedback_data;`（定义在 `gimbal.cpp`）
     - `Shoot_Upload_Data_s shoot_feedback_data;`（定义在 `shoot.cpp`）
   - 原来 `SubGetMessage(x, &dst)` 的地方删掉（cmd 写的就是上面那个实例）；原来 `PubPushMessage(x, &src)` 的地方删掉（数据已经是全局的）。
6. **单位**：对方框架与你的 C 版一致 —— 电机角度 **deg**、速度 **deg/s**、IMU 欧拉角/累计角 **deg**、`Gyro` **deg/s**；**MI 电机（pitch）角度是 rad**。
7. 每个 `.cpp` 顶部写一行 `@file` 注释说明"由 application/xxx.c 移植"。

## 2. 底层 API 速查（只列要用的）

```cpp
// ---- 姿态（替代 INS_Init/INS_Task/attitude_t）----
#include "ahrs.h"
AHRS g_ahrs;
AHRS::Config{ accel_range=BMI088::AccRange::G6, gyro_range=BMI088::GyroRange::Dps2000,
              board_yaw=0, board_pitch=0, board_roll=0, accel_lpf_coef=0.0085f,
              process_noise_quat=10.0f, process_noise_bias=0.001f, obs_noise_accel=1000000.0f,
              preheat_timeout_ms=30000 };
g_ahrs.init(cfg);   // 内部会初始化 BMI088 并预热/校准
g_ahrs.start();     // 内部建 CMSIS-RTOS2 任务做解算(替代 INS_Task 的 1kHz 循环)
g_ahrs.output_      // Output{ float q[4]; float euler[3]; // [0]=roll [1]=pitch [2]=yaw, deg
                    //         float yaw_total;         // deg, 累计
                    //         float gyro_b[3]; float accel_b[3]; float motion_accel_b[3]; float motion_accel_n[3]; }
// 字段映射: attitude_t{YawTotalAngle→output_.yaw_total, Pitch→euler[1], Roll→euler[0],
//                      Yaw→euler[2], Gyro[i]→gyro_b[i], Accel[i]→accel_b[i]}

// ---- 电机 ----
#include "dji_motor.h"
DJIMotor m;  // 静态实例
DJIMotor::Config{ CAN_HandleTypeDef* can_handle; uint8_t motor_id; DJIMotor_Type motor_type;
                  uint8_t direction;      // DJIM_DIRECTION_NORMAL / DJIM_DIRECTION_REVERT
                  float reduction_ratio;  // DJIM_REDUCTION_RATIO_M3508=19.0f / _M2006=36.0f / 6020 用 1.0f
                  PID::Config pid_angle; PID::Config pid_velocity;
                  uint8_t pos_freq_div; float initial_angle; };
m.init(cfg);
m.setEnable(1/0); m.setAngle(deg); m.setVelocity(deg_s); m.setCurrent(a);
m.setAngleCircular(deg); m.setAngleIncrement(deg);
// 只读状态: m.angle_(deg) m.velocity_(deg/s, 已低通) m.velocity_raw_ m.current_ m.temperature_ m.motor_valid_
DJIMotor::timbaseSelect(&htim5);   // 对方的机制: 把电机环挂到 1ms 时基中断上
                                   // ⚠本工程已改成"只记录句柄、不注册回调": 电机环改由
DJIMotor::taskUpdate();            // MOTOR 任务 5 分频以 200Hz 显式调用(与旧固件 motor_task.c 一致,
                                   // 见 MIGRATION_NOTES.md 「阶段 5.1」); pid_velocity.period 要填 5

// ---- PID（就是你 controller.c 的 C++ 版）----
#include "pid.h"
PID p; PID::Config{ kp, ki, kd, PID::Mode::Position|Delta, features, integral_limit, derivative_limit,
                    output_min, output_max, dead_zone, feedforward_gain, filter_alpha, kp_extra, kd_extra, period };
// features: FeatureIntegralLimit|FeatureDerivativeLimit|FeatureOutputLimit|FeatureDeadZone|FeatureFilter
//         | FeatureVariableGain|FeatureDerivativeOnMeasurement|FeatureFeedforward|FeatureTrapezoidIntegral
p.init(cfg); p.setSetpoint(x); p.update(feedback); p.output_; p.setIntegral(v); p.resetIntegral();

// ---- 其它 ----
#include "daemon.h"     // Daemon{ Config{ TIM::Config tim_config{&htim5}, uint32_t cycle, void(*cb)(void*), void* dev }, online_ }
#include "serial.h"     // Serial{ Config{ usart_handle, htim, rx_callback(uint16_t) }, send(), recv_buf_[256] }
#include "referee.h"    // Referee_Init(&huart6) / Referee_GetData() / Referee_Online() /
                        // Referee_UIDraw/UIChar/UIRefresh/UIDelete/SendInteractive
#include "remote.h"     // Remote_Init(&huart3) -> const dt7_rc_t*; Remote_Online(); remote_data; REMOTE_RC_*/
#include "minipc_comm.h"// Minipc_Init(&huart1) / Minipc_GetData() / Minipc_Online() / Minipc_Send(yaw,pitch,roll,speed,color)
#include "rgb_led.h"    // RGB{ initDefault(); set(r,g,b) }
#include "power.h"      // Power_Init(&hadc3); Power_GetBusVoltage()
#include "bsp_log.h"    // LOG_INFO(mod, who, fmt, ...) / LOG_ERR(...)   mod: LOG_MOD_SYS/INS/MOTOR/GIMB/CHAS/SHOT/COMM/SM
#include "bsp_dwt.h"    // DWT_GetTimeline_ms() 等；DWT_Init() 已在 robot.cpp 调过
#include "bsp_tim.h"    // TIM 类(中断回调) —— 原 bsp 定时器中断注册
#include "bsp_gpio.h" "bsp_spi.h" "bsp_pwm.h" "bsp_adc.h" "bsp_can.h"(class CAN) "bsp_usart.h"(class USART)
```

## 3. 任务划分（沿用本仓自己的，不用对方的 Chassis_Task/Gimbal_Task/Shoot_Task）

`application/robot_task.h` + `application/robot.cpp` 负责建任务：

| 任务 | 周期 | 内容 |
|------|------|------|
| INS | 1kHz | `g_ahrs.start()` 的内部任务负责解算；本任务只保留原来的"每 1ms 发一次视觉数据"（`Minipc_Send`），以及心跳 `iwdg_alive_ins` |
| MOTOR | 1kHz | 原来的 `MotorControlTask()` 的活已被 `DJIMotor` 的 htim5 回调接管；本任务保留心跳 `iwdg_alive_motor` 与超时打印（不要再遍历电机） |
| ROBOTTASK | 200Hz(`osDelay(5)`) | `RobotTask()`：`RobotCMDTask(); GimbalTask(); ShootTask(); ChassisTask();`（顺序保持原样） |
| DAEMON | 100Hz(`osDelay(10)`) | 原 `DaemonTask()` 的离线检查（改用各模块自己的 `Daemon`/`online_`）+ 超时打印；心跳 `iwdg_alive_robot`；**IWDG 先不做**（对方框架没有，见 §5） |
| UI | 挂起等待 | `UITask()`（裁判 UI 刷新）|

优先级/栈沿用原 `robot_task.h`（INS `osPriorityAboveNormal` 1024 字，其它 `osPriorityNormal`；DAEMON 128，UI 512，ROBOT 1024）。
对方框架用 CMSIS-RTOS2，本仓 `cmsis_os.h` 是 v1 兼容层：`osThreadDef/osThreadCreate` 仍可用，也可改用 `osThreadNew`。**栈单位注意**：`osThreadNew` 的 `osThreadAttr_t.stack_size` 是**字节**，`osThreadDef` 的第 5 个参数是**字**。

## 4. 各文件冻结清单（跨文件接口，不要自行改）

- `application/robot_def.h`（已写好，勿改结构）
- `application/chassis/chassis.{h,cpp}`：`void ChassisInit();` `void ChassisTask();`
- `application/shoot/shoot.{h,cpp}`：`void ShootInit();` `void ShootTask();`
- `application/gimbal/gimbal.{h,cpp}`：`void GimbalInit();` `void GimbalTask();` `DJIMotor* GetYawMotor();` `MIMotor* GetPitchMotor();`
- `application/cmd/robot_cmd.{h,cpp}`：`void RobotCMDInit();` `void RobotCMDTask();` + 定义三个 cmd 实例
- `application/gimbal_algorithm/gimbal_algorithm.{h,cpp}`：`void GimbalAlgorithmReset();`
  `float Cal_FollowControl_Set_Yaw(AHRS::Output, Gimbal_Ctrl_Cmd_s);` `float Cal_FollowControl_Set_Pitch(...);` `float Cal_FollowControl_Feedforward(...);`
- `application/robot.{h,cpp}`：`void Robot_Init();` `void Robot_Task();`（`extern "C"`，给 `Src/freertos.c` 用）

## 5. 对方框架里没有、需要补的（已另行安排，按冻结 API 调用）

- **MI 电机**（pitch 用的就是它）→ 新增 `project/modules/motor/mi_motor/`：
  ```cpp
  class MIMotor {
  public:
      struct Config { CAN_HandleTypeDef* can_handle; uint8_t motor_id; };
      void init(const Config& config);
      void setEnable(uint8_t en);
      void stop();
      void setMechPositionToZero();                      // 通信类型6: 设机械零位
      void locationControl(float location, float kp, float kd);  // 位置模式, rad
      float angle_ = 0.0f;       // rad
      float speed_ = 0.0f;       // rad/s
      float torque_ = 0.0f;      // N*m
      float temperature_ = 0.0f; // C
      uint8_t motor_id_ = 0;
      uint8_t valid_ = 0;
  };
  ```
- **超级电容** → 新增 `project/modules/super_cap/`：
  ```cpp
  class SuperCap {
  public:
      struct Config { CAN_HandleTypeDef* can_handle; uint32_t rx_id; uint32_t tx_id; };
      struct Msg { uint16_t vol; uint16_t current; uint16_t power; };
      void init(const Config& config);
      void send(uint8_t* data, uint16_t len);
      Msg cap_msg_ = {};
  };
  ```
- **蜂鸣器 / IWDG**：本次**不做**（原 `BuzzerInit/BuzzerTask/BSPIWDGFeed` 调用点删掉并写 `// TODO(移植)`）。
- **视觉协议**：原 `SendMinipcData`（`master_process` 的 seasky 协议）→ 改用 `Minipc_Send(...)`（对方的 20 字节帧）。
  **注意这是与视觉组的接口，协议变了，需要同步视觉端**——代码里写 `// TODO(协议)` 标出来。
- **`message_center`**：删掉，改直接调用（§1.5）。

## 6. 云台 yaw 的结构适配（唯一必须改控制结构的地方，务必按此做）

原 C 版把 **IMU 当反馈塞进电机控制器**：`angle_feedback_source = OTHER_FEED` +
`other_angle_feedback_ptr = &INS.YawTotalAngle`（角度环）、`other_speed_feedback_ptr = &INS.Gyro[2]`（速度环）。
对方 `DJIMotor` 没有外部反馈通路，因此改成**两个环都在应用层**：

```
误差 = gimbal_cmd_recv.yaw - g_ahrs.output_.yaw_total        (deg)
PID_angle_yaw(位置式, features 同原版: TrapezoidIntegral|IntegralLimit|DerivativeOnMeasurement|Feedforward,
              kp=80 ki=60 kd=6 dead_zone=0.1 integral_limit=250 output_max=330 feedforward_gain=120 period=5)
   → 输出 = 期望角速度 (deg/s)
PID_speed_yaw(位置式, kp=50 ki=60 integral_limit=6000 output_max=20000, period=1)  反馈 = g_ahrs.output_.gyro_b[2]
   → 输出 = 电流值
yaw_motor.setCurrent(输出 / 25000.0f);   // DJIMotor 的 6020 电流量程 = 25000 计数/1.0
```
- 0 电流态（`GIMBAL_ZERO_FORCE`）：`yaw_motor.setEnable(0)`，并清零两个 PID 的积分（原逻辑）。
- 回到控制态时：以当前 `yaw_total` 重置 `PID_angle_yaw` 的设定值，避免阶跃（原逻辑 `motor_init` 段）。
- **pitch（MI 电机）逻辑完全不变**：回零状态机、`locationControl(target, kp, kd)`、
  `setMechPositionToZero()`、软限位 `PITCH_MAX_ANGLE/PITCH_MIN_ANGLE` 都照搬，只是 `pitch_motor->measure.angle` → `pitch_motor.angle_`，
  原来从 `motor_controller.angle_PID.Kp/.Kd` 取的 kp/kd 改成 `gimbal.cpp` 里的常量（值不变：Kp=15, Kd=1.0）。

## 7. 验收

```bash
cmake --preset "GCC 13.3.0 arm-none-eabi (ucrt64)" && cmake --build build/Debug -j8
```
必须零 error；`Src/freertos.c` 里 `#define SENTRY_APP_LAYER 1` 打开后能链接出 `Sentry.elf`。
