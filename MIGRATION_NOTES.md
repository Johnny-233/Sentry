# 从 infantry_cpp 迁入说明

> 来源：`D:\MyTrain\robomaster\electronic_control\infantry_cpp`（远程 `unity-as/infantry_cpp`，分支当时为 `feat/cmd-chassis-cmd-decouple`）  
> 迁入内容：`project/` + `tools/`（**未**拷贝对方 `Core/` / `Drivers/` / `CMakeLists.txt` / `.claude/`）  
> 状态：**阶段 3 / 4 完成，阶段 5（首次烧录上电）、5.1（现场调试）也过了**——已完成烧录并复位运行，
> 运行期自检通过（无栈溢出、无 HardFault；遥控/视觉/裁判三路 RX DMA 改为 CIRCULAR 后可长期在线）。
> 当前镜像：FLASH 148012 B / 14.12%，RAM 101104 B / 77.14%（电机环回到 200Hz 的版本）。

## 阶段 5.1：现场调试（遥控 / 云台 / 底盘）+ 电机环回到 200Hz

上板后出现的问题、根因与处理：

| 现象 | 根因 | 处理 |
| --- | --- | --- |
| 上电约 1s 后遥控（以及视觉、裁判）全部失联，遥控控不了车 | 三路 RX DMA 都配成了 `DMA_NORMAL`：收满 256 字节（≈12 帧 VT13）后 stream 的 EN=0，而 HAL 仍以为 `RxState=BUSY_RX`，重新 arm 直接返回 `HAL_BUSY` | `Src/usart.c` 三路 RX 全改 `DMA_CIRCULAR`（TX 保持 `DMA_NORMAL`） |
| 底盘 / 摩擦轮电流抖动，甚至顶到 `MaxOut`(±14.648A) | 旧固件电机环是 **200Hz(5ms)**：`Modules/motor/motor_task.c` 里 `if(cnt%5==0) DJIMotorControl();`；而对方框架把电机环挂在 **1ms** 时基中断上（`DJIMotor::timbaseSelect` + `TIM::startIT`）。采样率×5 会让速度反馈里的高频噪声整段进 PID（框架的速度还带一阶低通，等效噪声带宽一起抬高） | **不重新整定 PID**，把电机环改回旧固件节奏：`timbaseSelect()` 只记录句柄不再注册回调，新增 `DJIMotor::taskUpdate()`，由 MOTOR 任务 5 分频以 200Hz 显式调用；`chassis.cpp` / `shoot.cpp` 里 `pid_velocity` 的 `period` 由 `1` 改 `5`(ms) |

### 5.1.1 又补齐两处与旧固件不一致的地方（这才是"抖动"的主因）

把旧 `application/*.c` 和 `Modules/motor/DJImotor/dji_motor.c` 逐行对回来，发现移植时丢了两样东西：

1. **速度反馈低通系数反了**：旧固件 `dji_motor.c:138`
   `speed_aps = (1-SPEED_SMOOTH_COEF)*speed_aps + SPEED_SMOOTH_COEF*new`，而 `SPEED_SMOOTH_COEF = 0.85`（弱滤波）。
   框架里 `DJIM_VELOCITY_LPF_ALPHA` 却是 `0.15`（强滤波）。同采样率下相位滞后大约 5~6 倍，
   `Kp=10` 的速度环装上去必然抖 → 已把系数改成 `0.85f` 与旧固件一致。
   电流反馈同理补上旧 `CURRENT_SMOOTH_COEF = 0.9`（`DJIM_CURRENT_LPF_ALPHA`）。

2. **丢了电流内环（串级）**：旧底盘/摩擦轮/拨盘的 `close_loop_type = SPEED_LOOP | CURRENT_LOOP`，
   旧 `dji_motor.c` 是**两级**：
   ```
   速度PID(量纲: 度/秒 → ESC计数, MaxOut 12000/15000/5000)
        └→ 电流PID(反馈=电调上报电流, Kp 0.5/0.7, MaxOut 15000/5000) → 发给电调
   ```
   而移植版是"速度环输出直接给电调"（只有一级）。旧电流环是纯 P 且反馈≈指令，
   稳态等效 `cmd = ref/(1+Kp)`，即前向增益被压低到 1/1.5 ~ 1/3 —— 移植版等效增益高了 2~3 倍。
   已在 `DJIMotor` 里补上可选电流内环（`Config.pid_current` + `Config.current_loop_enable`、
   `update()` 里速度环输出作为参考、`current_` 作为反馈），并给底盘/摩擦轮/拨盘按旧参数打开：
   底盘 `Kp=0.5, Ki=0, MaxOut=15000计数`；摩擦轮 `Kp=0.7, Ki=0.1, MaxOut=15000计数`；
   拨盘 `Kp=0.7, Ki=0.1, MaxOut=5000计数`。
   量纲注意：内环参考/反馈都在安培域，故 `pidPort(旧配置, 1.0f, 5)`，**只有 MaxOut 要换算成安培**
   （写法见 `pid_port.h` 里新增的说明和 `chassis.cpp` / `shoot.cpp` 的 `k*CurrentPid`）。

> 旧云台(gimbal.c)是 `ANGLE_LOOP | SPEED_LOOP`，**没有**电流环；且 yaw 的两个环我们现在跑在应用层
> （`gimbal.cpp` 的 `yaw_angle_pid`/`yaw_speed_pid` + `setCurrent`），所以云台不用开内环。

现场实测（`g_motor_loop_*` 计数器，见下）：

| 量 | 读数 | 结论 |
| --- | --- | --- |
| `g_motor_loop_dcyc` | 840653 cycle @168MHz = **5.004 ms** | 电机环 = 199.8Hz，正好是旧固件 `cnt%5` 的节奏 |
| `g_motor_loop_maxdcyc` | 858457 cycle = 5.11 ms | 没有长间隔（分频稳） |
| `g_motor_task_dcyc` / max | 181265 / 236493 cycle = 1.08 / 1.41 ms | 1kHz MOTOR 任务没被饿死 |
| `g_stack_overflow_cnt` / `g_fault_cfsr` | 0 / 0 | 无栈溢出、无 HardFault |

调试点：`DJIMotor::taskUpdate()` 与 MOTOR 任务里各留了一组 `volatile uint32_t g_motor_loop_cnt/
g_motor_loop_dcyc/g_motor_loop_maxdcyc`、`g_motor_task_dcyc/g_motor_task_maxdcyc`，只做计数，不影响控制。

改动文件：

- `project/modules/motor/dji_motor/dji_motor.{h,cpp}`：新增 `static void taskUpdate()`（跑一遍所有注册实例的 PID + 按 CAN 组各发一帧）；`timCallback()` 转调它；`timbaseSelect()` 改成空实现（原因写在函数头注释里）。
- `project/application/robot.cpp`：MOTOR 任务里 `USER_TIM_PeriodElapsedCallback(&htim5)` **仍是 1ms**（只管挂在 htim5 上的各 Daemon / Serial 整帧超时），电机环单独 `if(++motor_cnt>=5)` 调用；`MiMotorControl()` 仍 10 分频(100Hz)，与旧固件同节奏。
- `project/application/chassis/chassis.cpp`、`project/application/shoot/shoot.cpp`：`pidPort(..., 1)` → `pidPort(..., 5)`。
- （5.1.1）`project/modules/motor/dji_motor/dji_motor.{h,cpp}`：`DJIM_VELOCITY_LPF_ALPHA` 0.15→0.85、新增 `DJIM_CURRENT_LPF_ALPHA 0.9`；新增 `Config.pid_current` / `Config.current_loop_enable` / `PID pid_current_`，`update()` 里在速度环之后串上电流内环。
- （5.1.1）`project/application/chassis/chassis.cpp`、`shoot/shoot.cpp`：新增 `kChassisCurrentPid` / `kFrictionCurrentPid` / `kLoaderCurrentPid`（旧 `current_PID` 参数），并在电机配置里 `.pid_current = pidPort(k*CurrentPid, 1.0f, 5), .current_loop_enable = 1`。
- （5.1.1）`project/application/pid_port.h`：补一段"内环电流 PID 怎么换算"的说明（安培域 → `output_scale=1`，只有 MaxOut 要从计数换算）。

⚠️ **电机环周期必须和 `pid_velocity.period` 一致**：对方 PID 内部是 `integral += error * period`、`derivative = (error - last_error) / period`，`period` 单位是 **ms**。以后要改电机环频率，这两个数必须一起改（`pid_port.h` 的秒→毫秒换算也是按这个 period 推出来的）。

> 附带一条：位置环(`PidMode::Position`)的分频 `pos_freq_div` 注释写的是"更新频率 = 1000Hz / div"，现在电机环是 200Hz，实际变成 "200Hz / div"。本工程没有电机用位置环（底盘/摩擦轮/拨盘都是速度环，云台 yaw 用 `setCurrent` 电流环），所以暂时无影响。

## 阶段 5：首次烧录上电 + 修掉的两个崩溃

**烧录方式**（仓库 `.vscode/tasks.json` 里 "Flash (DAPLink 300kHz)" 同款）：

```bash
openocd -f openocd.cfg -c "adapter speed 300" \
        -c "program build/Debug/Sentry.elf verify reset exit"
# → Programming Started/Finished → Verified OK → Resetting Target
# 探针: CMSIS-DAP(SWD), device id 0x100f6413 = STM32F407IG 1024KB
```

首次上电**立刻 HardFault**，抓到现场后定位到两个问题（都已修）：

1. **`Src/freertos.c` 的 `defaultTask` 重复调用 `Robot_Task()`**（阶段 3 的遗留：那时应用层还是对方那套、
   `Cmd_Task` 按 1kHz 设计，所以挂在 defaultTask 上）。阶段 4 的应用层已经自己建了 ROBOT 任务（200Hz）
   也在调 `Robot_Task()` → 整套控制逻辑被**同时跑两遍**（cmd/chassis/gimbal/shoot 的全局指令被两个任务
   竞争写），而且 `defaultTask` 只有 1KB 栈 → 栈溢出踩坏紧邻的 FreeRTOS 堆/TCB。
   **修**：`defaultTask` 只保留 USB 初始化 + 空转，`Robot_Task()` 由 ROBOT 任务独占。
2. **DAEMON 任务栈 512B 溢出**（真正把内核踩坏的那个）：它第一拍时其它任务还没跑，`iwdg_alive_*` 全 0
   → 走"任务卡死"分支打 `LOG_ERR`（日志格式化 + snprintf + RTT），512B 装不下 → 踩坏旁边的 heap/TCB
   → 调度器里 HardFault。现象：`pxCurrentTCB` 被踩成 NULL，PendSV 的 `ldr r0,[r1]`（r1=NULL）从地址 0
   读到向量表第一项 `_estack = 0x20020000`，再 `ldmia` 从那儿取数 → **精确数据总线错误**
   （CFSR=0x8200、BFAR=0x20020000）；另一次表现为 `xTaskIncrementTick` 里 UsageFault UNALIGNED。
   **修**：DAEMON 512B→1KB、INS 1KB→2KB（它启动时也打日志），`configTOTAL_HEAP_SIZE` 20KB→24KB
   （原来只剩 368B，太紧）。

**顺手打开的栈溢出检测（保留在固件里）**：`configCHECK_FOR_STACK_OVERFLOW 2` + 在 `Src/freertos.c`
实现了强 `vApplicationStackOverflowHook`（把任务名拷到 `g_stack_overflow_task`、计数 `g_stack_overflow_cnt`
后停机）。就是它把"`daemontask`"揪出来的；以后再有人栈给小了会同样报出来。

**复位后的运行期自检**（OpenOCD 挂上去读变量）：

| 检查项 | 结果 |
|---|---|
| `g_vt_head`（200Hz ROBOT 任务每拍 +1） | 4 秒 +816 ≈ **204 Hz** ✓ 在跑 |
| `g_stack_overflow_cnt` | 0 ✓ |
| `g_fault_cfsr` / `g_fault_hfsr` | 0 / 0 ✓ 无故障 |
| `xFreeBytesRemaining` | 4776 B ✓ 堆有余量 |
| 当前任务 / CPU 模式 | IDLE / Thread ✓ 正常调度 |

**还没验证的（需要你在台架上继续）**：VT13 遥控是否真收到数据（`remote_data->key`/`mouse_x` 是否非 0）、
视觉链路 CRC16 是否对得上（RTT 看 `[vision]` 日志）、pitch 回零方向（`PITCH_HOMING_DIR`）、
IMU 预热要多久、以及全部标定值。

## 阶段 4：`application/` 控制逻辑 → C++（已完成）

方向（按你的决定）：**底层丢弃本仓 `Modules/` + `bsp/`，直接用对方 `project/` 的；
`application/` 的控制逻辑保留，重写成 C++ 适配对方接口**。移植规约与逐条映射见
**`project/application/PORT_MAPPING.md`**。

| 本仓 C（原件保留作对照，不编译） | 移植后 | 行数 |
|---|---|---|
| `application/robot.c` + `robot_task.h` | `project/application/robot.cpp`（5 个任务的划分不变）+ `robot.h` | 238 |
| `application/robot_def.h` | `application/robot_def.h`（同样的枚举/结构体，`attitude_t`→`AHRS::Output`） | — |
| `application/chassis/chassis.c` | `project/application/chassis/chassis.cpp` | 201 |
| `application/shoot/shoot.c` | `project/application/shoot/shoot.cpp` | 178 |
| `application/gimbal/gimbal.c` | `project/application/gimbal/gimbal.cpp` | 349 |
| `application/gimbal_algorithm/gimbal_algorithm.c` | `project/application/gimbal_algorithm/gimbal_algorithm.cpp` | 199 |
| `application/cmd/robot_cmd.c` | `project/application/cmd/robot_cmd.cpp` | 952 |
| （原 `Modules/referee/referee_task.c` 的 UI 布局） | `project/application/ui/ui.cpp` + `ui.h` | 402 |
| （原 `Modules/motor/MImotor/`、`Modules/super_cap/`） | `project/modules/motor/mi_motor/`、`project/modules/super_cap/` | 286 |

改法与已知差异（**上电前请逐条看**）：

1. **消息中心删除** → cmd 的三个指令实例与三个反馈实例改成全局变量直接读写（规约 §1.5）。
2. **云台 yaw 结构变了**（唯一必须改控制结构的地方）：对方 `DJIMotor` 没有"外部反馈"通路，
   原先把 `INS.YawTotalAngle`/`Gyro[2]` 塞进电机控制器的那两个环，搬到了应用层
   （角度环→速度环→`setCurrent(out/25000)`），参数一字未改。
3. **PID 量纲/单位换算**：旧 PID 输出是 CAN 电流**计数**、积分用**秒**；对方 PID 输出是**安培**、
   积分用**毫秒**。统一由 `application/pid_port.h` 的 `pidPort()` 换算，应用层保留旧数字原样。
4. **控制周期走自己的**：仍是 INS 1kHz / MOTOR 1kHz / ROBOT 200Hz / DAEMON 100Hz / UI。
   但对方框架的"1ms 时基"在本板**跑不起来**（`Src/tim.c` 的 htim5 是 PWM、`stm32f4xx_it.c` 没有
   `TIM5_IRQHandler`、NVIC 未使能、`main.c` 也没转调 `USER_TIM_PeriodElapsedCallback`），
   所以由 1kHz 的 MOTOR 任务每个周期调一次 `USER_TIM_PeriodElapsedCallback(&htim5)`——
   这一下同时驱动了电机环（DJIMotor 跑 PID + 发 CAN）和挂在 htim5 上的各 Daemon / Serial 超时。
5. **`bsp_can` 补了扩展帧支持**（对方框架原来只收标准帧）：新增 `CAN::Config.ext_flag` +
   32 位掩码滤波器 + `fifoCallback` 的 `CAN_ID_EXT` 分发（通信类型 2），否则 **MI 电机（pitch）
   的反馈收不到**。
6. **AHRS 初始化不能在关中断区间里**：对方 `AHRS::init()` 内部会 `preheat()`（用 `HAL_Delay`
   等 IMU 加热到 40°C，最坏 30s），HAL tick 来自 TIM14 中断 → `Robot_Init()` 里**去掉了**
   原 C 版的 `__disable_irq()` 包裹（原版能这么写是因为它的 `INS_Init` 在 INS 任务里跑）。
   代价：**开机要等 IMU 预热完成**（对方框架行为）。
7. **视觉/小电脑链路用回本仓原本的 seasky 协议**：`Modules/master_machine`（`master_process` +
   `seasky_protocol`）已移植成 C++ 放在 `project/modules/master_machine/`，**只把 bsp 换成对方的
   `Serial` + `Daemon`**；对方那套 20 字节帧的 `project/modules/minipc_comm/` **不再编译**
   （文件保留作对照，CMake 里用 `list(REMOVE_ITEM ...)` 排除）。
   收帧 `Minipc_Recv_s.Vision`（yaw/pitch/can_fire/**linear_velocity_x/y**/**gimbal_mode**）与
   发帧 `Minipc_Send_s` 的字段/偏移逐字节保持原样，所以 `Sentry_ChassisAC()` 的底盘自主平移/
   小陀螺触发又用回真实视觉数据（原先的 0 占位已撤掉）。发送两处与原版一致：
   INS 任务每 1ms `SendMinipcData(NULL)`、`RobotCMDTask()` 结尾 `EnemyJudge() +
   SendMinipcData(&minipc_send_data)`。
   > 发送数据**只有一份**（模块持有的那份，`Minipc_GetSendData()`）：1kHz（INS）与 200Hz（`RobotCMDTask`）
   > 两条路径共用，所以每帧都带正确的敌我颜色。原实现是 cmd 一份、模块一份，1kHz 那帧的 `detect_color`
   > 从来没被填过（一直发 0=红）——合成后这个老问题消失了。
   > 仍然为 0 的字段：`occupation`（占领状态）—— 裁判协议里没有"占领"字段（`robot_pos` 只有
   > x/y/angle，`event_data` 是位域），没有可靠来源。
   > 已接上的：`self_sentry_hp`/`self_hero_hp`/`self_infantry_hp`/`remain_time`/`remain_bullet`/
   > `match_progress`/`bullet_speed`（映射见 `project/application/cmd/robot_cmd.cpp` 的
   > `VisionSetMatchData()`），以及 `vx/vy` —— 按"用源代码的逻辑"：反用 `Sentry_ChassisAC()` 里
   > 那条"视觉速度 → 底盘指令"的换算（`4 * REDUCTION_RATIO_WHEEL * 360 / PERIMETER_WHEEL * 1000`），
   > 数据源是 `chassis_cmd_recv.vx/vy`（**指令速度**，不是轮子实测速度；视觉自己开车时会把它刚下发的
   > 指令回给它们）。要改成实测速度/场地系，只需改那一处。
8. **MI 电机（云台 pitch）也是用你的模块**：`project/modules/motor/mi_motor/` 按
   `Modules/motor/MImotor/mi_motor.c` 忠实还原（注册表 + 周期重发 + 全部公开功能），
   **只把 bsp 换成对方的 `CAN` 类**（走我新加的 `ext_flag` 扩展帧通路）。
   周期重发按原 `motor_task.c` 的节奏：1kHz 的 MOTOR 任务里 `if(cnt%10==0) MiMotorControl();`（100Hz）。
9. **遥控：已换成新遥控 VT13/VT03（图传链路，带键鼠）**，按你的要求：
   - `project/modules/remote/remote_config.h`：`REMOTE_DEVICE_VT13`（原来是 DT7）；
   - `Src/usart.c` 的 `huart3`：**921600 / 8B / NONE**（原来是 100000 / 9B / EVEN）。
     注意 `Remote::init()` 会校验串口参数，不匹配会直接拒绝初始化并打 `UART params mismatch`。
   - **旧的 DBUS/Dt7 逻辑完整保留、只是暂时用不上**：`remote.h` 的 DT7 分支、`dt7` 驱动、
     `robot_cmd.cpp` 里的 DT7 分支都还在。切回去 = 对调 `remote_config.h` 的两行注释 +
     把 `huart3` 改回 100000 / 9B / EVEN。
   - 摇杆逻辑与旧版相同；**开关/FN 映射已按你的要求定好**（`robot_cmd.cpp` 顶部那层"机型适配"宏）：
     | 输入 | 语义 | 落到原判据 |
     |------|------|-----------|
     | 右三档开关 `mode_sw` = **C(0)** | 零电流停机 | `RC_SW_VALUE_DOWN` → `AnythingStop()` |
     | = **N(1)** | 手动模式（底盘跟随） | `RC_SW_VALUE_MID` → `BasicSet()+RemoteControlSet()` |
     | = **S(2)** | 哨兵自动模式 | `RC_SW_VALUE_UP` → `BasicSet()+SentrySet()` |
     | **左 FN** 按住 | 小陀螺方向一（`chassis_rotate_buff = +1`） | 左"开关"= MID |
     | **右 FN** 按住 | 小陀螺方向二（`chassis_rotate_buff = -1`） | 左"开关"= UP |
     | 两个 FN 都不按 | 底盘跟随（`CHASSIS_FOLLOW_GIMBAL_YAW`） | 左"开关"= DOWN |
     左右 FN 的转向如果实测反了：把 `RC_SWITCH_LEFT()` 里的 `1u`/`2u` 对调（别动 `RC_SW_VALUE_*`，
     那会连带改掉右边三档开关 N/S 的含义）。
   - `ControlDataDeal()` 保留了一个兜底：右开关值不在三个已知档位时一律 `AnythingStop()`
     （否则三个分支都不进，指令会保持上一拍的值、车继续按旧指令跑）。
   - **键鼠路径已接上**（`MouseKeySet()` = `MouseControl()` + `KeyControl()`，原来从定义起就没有调用点）：
     手动档（N）里"摇杆"与"键鼠"**二选一、粘滞切换**——动鼠标/按鼠标键/按键盘 → 键鼠接管；
     动摇杆 → 切回遥控器；上电默认遥控器。判据与对方 `cmd.cpp` 的 `mode_is_remote` 同一套。
     （必须二选一：`KeyControl()` 每拍都会写 `chassis_cmd_recv.vx/vy` 和 `chassis_mode`，
     两套同拍跑会把摇杆和左/右 FN 小陀螺覆盖掉。）
     按键动作沿用旧语义（"直接用"，未改判定与数值）：
     | 输入 | 动作 |
     |------|------|
     | 鼠标 x/y | 云台增量：`yaw -= x/660*3`、`pitch += y/660/57` |
     | 鼠标左键 | 开火：`reverse_flag` 时 `LOAD_REVERSE`，否则连发 `LOAD_BURSTFIRE`；松开 `LOAD_STOP` |
     | 鼠标右键 | 自瞄：`autoaim_mode = FIND_Enermy` + `FoundEnermy()`；`can_fire==1` 时连发 |
     | W/S/A/D | 键盘底盘：`vx/vy = ±20000 × chassis_speed_buff` |
     | R | 按一次切底盘模式（跟随 ↔ 小陀螺，本地上升沿计数） |
     | Q | 按住 → `reverse_flag=1` + 拨盘 `LOAD_REVERSE` |
     | SHIFT | `chassis_speed_buff = 2` |
     另有按 `robot_level` 缩放的 `chassis_rotate_buff/chassis_speed_buff`（1~10 级）与
     `power_flag` 时的小陀螺 buff 覆盖，都在 `KeyControl()` 里，原样保留。
     未接线（空着）：`CTRL/E/F/G/Z/X/C/V/B` 八个键、鼠标中键、滚轮中键位移 `mouse_z`、扳机、PAUSE。
   - 完整输入清单与旧语义见 `project/application/cmd/robot_cmd.cpp` 顶部注释与
     `project/modules/remote/vt13/vt13.h`。
10. **没做的**：蜂鸣器（`BuzzerInit/BuzzerTask`）、IWDG（对方框架没有，卡死只打印不再硬件复位）；
    `GIMBAL_FREE_MODE` 分支照旧 C 不动作（该模式未被下发）。
10b. **视觉收帧严格校验已打开**（`project/modules/master_machine/seasky_protocol.cpp` 的
    `VISION_RX_STRICT_CHECK = 1`）：长度 + 整帧 CRC16（小端，覆盖本次收到的 `len` 字节）不过就丢帧，
    这样"任何以 `0x5A` 开头的字节流"不再能影响云台指向与开火许可（`CODE_REVIEW.md` 第 530 行那条）。
    ⚠️ 前提是**小电脑端发整帧小端 CRC16**；原代码注释记着"开启校验后现场收不到小电脑数据"，
    首次联调看 RTT：`[vision] CRC16 mismatch ... (len=...)`、`short frame`，首次失败还会打一条
    `CRC16 diag`（实际长度 + 帧尾两字节 + 我们算出的 CRC，便于判断是算法、字节序还是覆盖长度不对）。
    真收不到就优先让 PC 端对齐；实在不行把宏改回 0 先跑通。
    （附带发现：原解析 `Vision.checksum = (rx_buf[25]<<8)|rx_buf[26]` 是**大端**读，而校验按**小端**比；
    该字段当前没被使用，我按原样保留。）
11. **标定参数全部保留旧值**（你已确认）：`PITCH_MAX_ANGLE/PITCH_MIN_ANGLE`、`YAW_CHASSIS_ALIGN_ECD`、
    `PITCH_HORIZON_ECD`、各 PID、`REDUCTION_RATIO_LOADER`、MI 电机回零参数等，一律没动，
    上你车前按本车核对/重标。
12. `reduction_ratio` 全部填 `1.0f`：旧 `speed_aps = 6*rpm` 是**转子** deg/s（不除减速比），
    旧代码的速度参考值（遥控 ±19800 / 键盘 ±20000 / 摩擦轮 35000）也是转子量纲，
    对方 `velocity_` 默认除减速比 → 填 19/36 会差一个减速比。

## 0. 目录：只有 `project/` 一套（旧的两套已删）

| 目录 | 角色 | 进不进固件 |
|------|------|-----------|
| `project/bsp/` | C++ 底层：外设抽象（对方的类） | ✅ 编 |
| `project/modules/` | C++ 底层：可复用模块（对方的模块 + 本仓补的 `mi_motor` / `master_machine`(seasky) / `super_cap`） | ✅ 编 |
| `project/application/` | **本仓整车控制逻辑的 C++ 移植版**（`robot` / `chassis` / `gimbal` / `shoot` / `cmd` / `gimbal_algorithm` / `ui`） | ✅ 编（7 个 .cpp） |
| `project/modules/minipc_comm/` | 对方那套 20 字节帧的视觉协议，已弃用 | ⛔ 文件留着作参考，CMake 里 `list(REMOVE_ITEM)` 排除 |
| `tools/` | 标定/生成脚本（`accel_calibration`、`crc8_gen.py`） | ⛔ 不上板 |

**已删除的东西（都是"不用了的那一套"）**：

- 顶层 `bsp/`（1542 行 C）、`Modules/`（5239 行 C）、`application/` 里的原 C 控制逻辑与头文件
  —— 共 121 个文件，**都在 git 里**（本分支上一次提交的内容），要看旧实现直接：
  `git show HEAD:Modules/motor/DJImotor/dji_motor.c > /tmp/dji_motor.c`（或 `git checkout HEAD -- Modules` 临时取回）。
- `project/application/` 里对方那套整车逻辑（cmd/chassis/gimbal/shoot，1197 行 C++，未跟踪）
  —— 与 `D:\CubeMX Project\27_infantry\project\application` **逐字节相同**，需要时从那拷回来。

> 代码里 `@file` 注释写的"由 application/xxx.c 移植"是**出处记录**，那些 .c 已随上面一起删除，
> 用 `git show HEAD:application/chassis/chassis.c` 可以取回。

## 1. 分工（历史说明：谁编、谁不编）

> `.ioc` 提醒：仓库里的 `Infantry-Little.ioc` 是**别的板子的空壳**（STM32F407VE / LQFP100，只配了 USART1），
> 与 `Src/`+`Inc/`（F407IGHx 全外设）对不上，**不要拿它重新生成代码**，否则会覆盖现有配置。

## 1. 阶段 3 改了什么

| 改动 | 文件 | 说明 |
|------|------|------|
| 接入 CMake | `CMakeLists.txt` | 递归扫描 `project/bsp` + `project/modules`，其头文件目录进 include 路径（阶段 4 后改为直接编 `project/application/**/*.cpp`） |
| 旧 C 层退出编译 | `CMakeLists.txt` | `application/` `Modules/` `bsp/` 不再加入 target（见 §0、§5） |
| 补 CMSIS-RTOS2 | `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/` | 底层模块用 `osThreadNew/osDelay/osTimerNew`，本仓原来只有 CMSIS-RTOS **v1** |
| FreeRTOS 配置 | `Inc/FreeRTOSConfig.h` | 按 CMSIS-RTOS2 要求：`configMAX_PRIORITIES 56`、`configUSE_TIMERS 1`、`configUSE_PORT_OPTIMISED_TASK_SELECTION 0`，SysTick 交给 `cmsis_os2.c` |
| 内核源码替换 | `cmake/stm32cubemx/CMakeLists.txt` | `cmsis_os.c` → `cmsis_os2.c`；SEGGER RTT 改用 `project/bsp/bsp_log/` 自带那份（消重复符号） |
| 调度骨架 | `Src/main.c`、`Src/freertos.c` | `osKernelInitialize()` → `MX_FREERTOS_Init()` → `osKernelStart()`；`defaultTask` 1kHz 空转，应用入口用 `#define SENTRY_APP_LAYER` 开关（见 §2） |
| 消重名 | `project/modules/crc/crc.h` → `crc_rm.h` | 与 CubeMX 的 `Inc/crc.h` 重名，是全仓唯一与 `Inc/` 撞名的框架头 |
| C/C++ 边界 | `Src/stm32f4xx_it.c`、`Src/usbd_conf.c` | CubeMX 生成文件不再反向依赖应用层 C++ 符号 |
| 补 `hadc3` | `Src/adc.c`、`Inc/adc.h`、`Src/main.c` | 母线电压 ADC3 / PF10 = ADC3_IN8（`project/modules/power` 要用），本仓原来只有 ADC1（内部温度传感器） |

### 怎么编

```bash
cmake --preset "GCC 13.3.0 arm-none-eabi (ucrt64)"   # 或 VS Code CMake Tools 选同一个 preset
cmake --build build/Debug -j8
# 产物: build/Debug/Sentry.elf / .hex / .bin
```

> 注意 `cmake --preset Debug`（Ninja）与 `build/Debug` 现有的 MinGW Makefiles 缓存冲突（老问题），
> 用上面带编译器路径那个 preset 即可。

## 2. 下一步：把 `application/` 的控制逻辑移植成 C++

目标：**控制逻辑 1:1 保留本仓原样，接口换成 C++ 底层的，语言换成 C++**。

入口约定（已预留）：移植好的应用层用 `extern "C"` 提供 `Robot_Init()` / `Robot_Task()`
（可照 `project/application/robot.h` 的写法，但内容是**我们的**逻辑），
然后在 `Src/freertos.c` 里打开 `#define SENTRY_APP_LAYER 1`。现在这个开关是关的，所以固件里只有底层 + 空转的 1kHz 任务。

逐文件待移植（行数）：

| 本仓 C | 行数 | 移植后接到什么 C++ 底层 |
|--------|------|--------------------------|
| `application/robot.c` | 46 | `DWT_Init` / `BSP_LogInit` / `Remote_Init` / `Referee_Init` / `Minipc_Init` / `AHRS::init` |
| `application/chassis/chassis.c` | 195 | `DJIMotor`（4×M3508）+ `bsp_can`；运动学/速度环原来用 `Modules/algorithm/controller.c` |
| `application/gimbal/gimbal.c` | 284 | `DJIMotor`（GM6020）+ `AHRS` + `PID` |
| `application/shoot/shoot.c` | 190 | `DJIMotor`（2×摩擦轮 M3508 + 拨盘 M2006）+ `Daemon` |
| `application/cmd/robot_cmd.c` | 836 | `Remote` / `Referee` / `Minipc` / `RGB` / `Daemon` / `message_center` |
| `application/gimbal_algorithm/gimbal_algorithm.c` | 185 | 纯算法，可直接搬（用的矩阵/数学换成 `project/modules/utils/matrix.hpp`） |

移植时要先定的三件事（都是设计选择，不是纯搬运）：

1. **`message_center` 要不要一起搬**：`robot_cmd.c` 靠它做发布/订阅，`project/modules` 里没有对应物。
   要么把它也移植成 C++，要么在应用层改成直接调用（对方 `project/application/cmd.cpp` 是直接调用）。
2. **底盘/云台 core 用谁的**：对方把运动学+速度环+功率限幅放在
   `project/application/chassis/chassis_core/`（`chassis_motion` / `chassis_velocity`）和 `gimbal/gimbal_core/`，
   它们也在 `application/` 下、现在没编。要么当底层复用，要么按本仓 `chassis.c` 的逻辑自己写。
3. **控制周期从哪来**：`project/` 版把速度环放在 `htim5` 中断里（`DJIMotor::timbaseSelect(&htim5)`），
   本仓 C 版走 `Modules/motor/motor_task.c` + 任务调度。移植时对齐其中一种。

## 3. 底层模块写死的外设句柄（移植应用层时要传这些）

| 句柄 | 用途 | 本车(Sentry) | infantry_cpp 原车 |
|------|------|--------------|-------------------|
| `hcan1` | 底盘 M3508 ×4 | ✅ CAN1 | CAN2 |
| `hcan2` | 云台 GM6020 + 摩擦轮 + 拨盘 | ✅ CAN2 | 云台 CAN2 / 发射 CAN1 |
| `huart1` | **小电脑 / 视觉** | ✅ USART1 | 裁判 |
| `huart3` | 遥控 DBUS（100000 9B EVEN） | ✅ USART3 | 同 |
| `huart6` | **裁判系统** | ✅ USART6 | 小电脑 |
| `hspi1` | BMI088 | ✅ 同 | 同 |
| `hi2c3` | IST8310（可选） | ✅ 同 | 同 |
| `htim5` | 电机 / daemon / 速度环时基 | ✅ 同 | 同 |
| `htim10` | IMU 加热 PWM | ✅ 同 | 同 |
| `hadc3` | 母线电压（PF10 = ADC3_IN8） | ⚠️ 本仓原来没有 ADC3，本次补上；见 §5 | 同 |

依据是迁移前能跑的 C 固件：`application/cmd/robot_cmd.c`（`minipcInit(&huart1)` / `UITaskInit(&huart6,...)`）、
`application/chassis/chassis.c`（`&hcan1`）、`application/gimbal/gimbal.c`（`&hcan2`）、`application/shoot/shoot.c`（`&hcan2`）。
串口参数在 `Src/usart.c`；`project/modules/remote/remote_config.h` 默认 `REMOTE_DEVICE_DT7`，与 `huart3` 一致。
DMA：USART1_RX = DMA2_S5、USART6_RX = DMA2_S2、USART3_RX = DMA1_S1，收发口都带 TX DMA，够 `serial` 的 Circular+Idle + `Transmit_DMA` 用。

## 4. 绝对不要照搬对方的应用层参数

`project/application/config.h` 与各 PID 配置是**对方机械/电调标定**（`GIMBAL_YAW_ECD`、`GIMBAL_PITCH_ECD`、
`GIMBAL_PITCH_CURRENT_FF`、底盘/云台/射击 PID、功率系数、电机方向 `REVERT/NORMAL`）。
它们只在参考件里，**没有进固件**；本仓原本的标定在 `application/` 与旧 `Modules/` 里，移植时以本仓为准。

## 5. 已知缺口 / 本次的取舍

- **`hadc3` 是手写的**：本仓 CubeMX 只有 ADC1（内部温度传感器通道），旧 Sentry 固件也完全没用 ADC。
  为对齐底层 `power` 模块的接口，照 CubeMX 生成风格把 `MX_ADC3_Init()`（PF10 = ADC3_IN8、480 cycles、12bit）
  写进 `Src/adc.c`，`main.c` 里调用。**若 PF10 没接 100k/10k 分压就撤掉**：母线电压只用于调试观察，
  底盘功率限幅走裁判系统数据。
- **看门狗**：旧 `BSPIWDG` 随 `bsp/` 退出编译；新底层有 `daemon`（离线回调），但**目前没有 IWDG**。
  要的话把 `bsp/bsp_iwdg.c` 摘出来（注意它依赖旧 `bsp_log.h`），或在 C++ 底层里新写。
- **USB**：`StartDefaultTask` 仍保留 `MX_USB_DEVICE_Init()`（CDC 没接业务，仅保持原行为）。
- **CubeMX 生成区被改过**：`Src/adc.c`、`Src/freertos.c`、`Src/main.c`、`Src/stm32f4xx_it.c`、`Src/usbd_conf.c`；
  以后真要重新生成，先合并这些差异。`cmake/stm32cubemx/CMakeLists.txt` 也改过（CMSIS-RTOS2 + SEGGER RTT）。
- **旧 C 底层为什么不能和 `project/` 底层共存**：同名同功能（`bsp_dwt.c` 与 `project/bsp/bsp_dwt.cpp`
  都定义 `DWT_Init`/`DWT_GetTimeline_s`；`crc.h` 撞名），一起编会重复定义 + 头文件串味。
  需要旧实现对照时用 `Reborn` 分支：`git worktree add ../Sentry-legacy Reborn`。

## 6. 与本仓规范的差异（已知，暂不改代码）

| 对方 (infantry_cpp) | 本仓 skills |
|---------------------|-------------|
| C++，`init`，禁堆 | C 规范偏 `Register` |
| 底层已是 `.cpp` | 文档示例仍多为 `.c` |

## 7. 回滚

- 空壳基线：`b871449`（或 `git log` 首提交）
- 本次阶段 3 改动未提交；`git diff` 看全部改动，`git checkout -- <file>` 单文件回滚
- `project/application/` 已撤回我改过的 3 处句柄，与 infantry_cpp 逐字节一致，可直接 diff 对照
