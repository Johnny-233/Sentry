# Sentry 代码审查报告（分支 Reborn）

- 审查基线: `cd34374`（Reborn 分支刚创建时与 main 相同）
- 审查范围: 全部自研代码（application / Modules / bsp / Src 的 USER CODE 部分 / CMake），不含 ST HAL、FreeRTOS、USB 库与 build 产物
- 方法: 人工精读跨层调用链 + 逐条用 read/grep 核对行号（本文所有行号均为实际读到的行号）
- 状态: **四批全部完成** —— 第 1 批（应用层 + 跨层）、第 2 批（Modules 驱动/通信层）、第 3 批（BSP/RTOS/初始化/构建）、第 4 批（算法层）均已逐条复核并去重；末尾附「修复优先级建议」与总体结论。子代理判定的「致命」有 1 条经我实测推翻（见 B1），「高」有 1 条下调为中（见 A2）。

## 结论速览

| 级别 | 数量 | 摘要 |
|---|---|---|
| 致命/高 | 12 | ［应用层］视觉掉线后仍追瞄开火；视觉串口多任务并发发送；异常=静默停机（无看门狗+死循环）；电机控制实际按 1kHz 而非 200Hz。［Modules 层］裁判帧长未校验的指针跳转+ISR 递归（可任意地址读/HardFault）；视觉接收 CRC16 被注释→未校验数据直接进云台与开火；BMI088 陀螺初始化索引下溢导致死循环；裁判 UI `vsprintf` 写爆 30 字节缓冲 |
| 中 | 24 | ［应用层］超电容越界读 6 字节；CAN 发送缓冲越界写；20KB 堆 vs 14.5KB 任务栈且 malloc 全不判空；daemon 计数 ISR 竞态；角度/弧度单位混用；启动期全程关中断且初始化无超时；多处"以为有保护其实没接线"的死代码。［Modules 层］裁判 0x0104 越界写 1 字节；MI 电机 6 个通信 API 实际不生效；float→int16 无 NaN/硬限幅；CAN EXT 单实例路由与滤波器 bank 重叠；遥控器 ISR 解析与任务清理无保护 |
| 低 | 17 | 日志打印地址而非值；底盘未限幅/电容判据存疑；死模块与 `-w` 关闭全部警告；DM 电机丢弃取反限幅后的力矩；BMI088 标定静止判据用错变量（该模块当前未接入）等 |

---

## 高 / 致命

### H1. 视觉数据没有新鲜度校验 —— 小电脑掉线后仍会继续追瞄并持续开火
- 位置: `Modules/master_machine/master_process.c:32-38`、`application/cmd/robot_cmd.c:139-176`、`robot_cmd.c:414-423`、`robot_cmd.c:520-521`、`application/robot_def.h:119,177`
- 证据:
```c
// master_process.c:32  离线回调只重启串口, 从不清数据
static void VisionOfflineCallback(void *id)
{
    USARTServiceInit(minipc_usart_instance);
    LOGWARNING("[vision] vision offline, restart communication.");
}
// robot_cmd.c:145  只按"数据内容"判断视觉是否工作
if (minipc_recv_data->Vision.yaw != 0.0f || minipc_recv_data->Vision.pitch != 0.0f)
    DataLebel.vision_flag = 1;
// robot_cmd.c:414  ShootAC 只看 fire_flag
if (DataLebel.fire_flag == 1) shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
// robot_def.h:119,177  两个为"掉线保护"预留的字段, 全项目从未被读写
uint8_t Power_Out;   // 掉线保护标志
float last_deep;     // cached copy of Vision.can_fire (offline detection)
```
- 问题: `vision_flag` 的清零条件是"yaw 与 pitch 同时为 0 持续 1s"。若小电脑最后一帧 yaw/pitch 非 0（正常追踪时必然如此）后死机或拔线，`vision_flag` 永远为 1，于是：`Sentry_GimbalAC` 一直 `FoundEnermy()` 追一个幽灵目标、`can_fire` 保持 1 就持续连发、`Sentry_ChassisAC` 用陈旧的 `linear_velocity_x/y` 继续跑底盘。daemon 虽然能检测离线（reload_count=10→100ms），但回调只重启串口，不清数据、不置标志。
- 建议: 在 `VisionOfflineCallback` 中 `memset(&minipc_recv_data, 0, sizeof(...))` 或置 offline 标志；`robot_cmd` 只消费"online 且最近 100ms 内更新"的数据（把 `Power_Out`/`last_deep` 用起来），否则强制 `shoot_mode=SHOOT_OFF`、`vx=vy=0`。

### H2. 视觉串口发送被 1kHz 与 200Hz 两个任务并发调用，且违反 bsp 约定的就绪检查
- 位置: `application/robot_task.h:69`、`application/cmd/robot_cmd.c:720`、`Modules/master_machine/master_process.c:87-104`、`bsp/usart/bsp_usart.h:58`、`bsp/usart/bsp_usart.c:83-89`
- 证据:
```c
// robot_task.h:69  INS 任务 1kHz
SendMinipcData(NULL);
// robot_cmd.c:720  机器人任务 200Hz
SendMinipcData(&minipc_send_data);
// master_process.c:87-89,104  同一个 static 缓冲, DMA 发送
static uint8_t send_buff[Minipc_Send_sIZE];
USARTSend(minipc_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA);
// bsp_usart.c:85  位或写错, 恒为真 → 永远返回"未就绪"
if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX) return 0;
// bsp_usart.h:58  明确要求: 连续 DMA/IT 发送需配合 USARTIsReady()
```
- 问题: 发送缓冲/长度是函数内 static，两个任务可抢占交错 → 帧内容互相覆盖（视觉端收到半新半旧的包）；同时在 HAL 层争用同一 UART 的 `__HAL_LOCK`，正是 `master_process.c:27` 注释自己担心的"发送接收同时进行导致再也进不了接收中断"的场景。另外：`USARTIsReady()` 用 `|` 而非 `&`，即使调用方想按约定检查也永远得到"忙"；`USARTSend` 返回值全程未判；`master_process.c:104` 用 DMA，而 105-107 行注释写"此处使用 IT 发送"，代码与注释矛盾。
- 建议: 发送收敛到单一任务（1kHz INS 或 200Hz robot 二选一，前者已带最新姿态）；或加互斥量+双缓冲并在发送前检查 `USARTIsReady()`；修正 `gState & HAL_UART_STATE_BUSY_TX`；检查 `USARTSend`/`HAL_UART_Transmit_DMA` 返回值并统计丢帧。

### H3. 没有任何看门狗 + configASSERT 直接死循环 + 多处 while(1)：所有异常都是"静默停机"
- 位置: `Inc/FreeRTOSConfig.h:123`、`Inc/stm32f4xx_hal_conf.h:57`、`application/shoot/shoot.c:139-141`、`Modules/message_center/message_center.c:31-35,39-46`、`bsp/usart/bsp_usart.c:40-47,76-78`、`bsp/can/bsp_can.c:112`、`Modules/motor/DJImotor/dji_motor.c:70-79,99-108`、`Modules/imu/ins_task.c:89-90`
- 证据:
```c
// FreeRTOSConfig.h:123
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
// stm32f4xx_hal_conf.h:57   IWDG 模块未启用(注释掉)
/* #define HAL_IWDG_MODULE_ENABLED */
// shoot.c:139
default:
    while (1)
        ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
```
- 问题: 任一处触发即整机停摆且电机保持最后指令（`DJIMotorControl` 仍在 1kHz 发同一帧电流）、无复位、无日志；部分点位还在 `RobotInit` 的 `__disable_irq()` 窗口内（`ins_task.c:89` 的 `while (BMI088Init(...) != BMI088_NO_ERROR);`、`bsp_usart.c:40`），连串口日志都发不出去。
- 建议: 启用 IWDG 并由 robot 任务喂狗；把 `while(1)` 改成"进入错误状态 + 停止电机输出 + 蜂鸣/LED 报警"；`configASSERT` 中先记录/复位而非空转。

### H4. `motor_task.c` 的分频变量从不自增 —— 电机控制实际以 1kHz 运行
- 位置: `Modules/motor/motor_task.c:4-11`（全文件 12 行，无 `cnt++`）
- 证据:
```c
void MotorControlTask()
{
    static uint8_t cnt = 0; 
    if(cnt%5==0) //200hz
    DJIMotorControl();
    if(cnt%10==0) //100hz
    MiMotorControl();
}
```
- 问题: `cnt` 恒为 0，两个条件恒真，而 `StartMOTORTASK` 是 `osDelay(1)`（1kHz）。于是 DJI 电机 PID+CAN 发送、MI 电机控制都跑在 1kHz，与注释声明的 200Hz/100Hz 不符：CAN 帧率变成设计的 5 倍（每 ms 最多 6 帧，实测 1kHz 控制下总线负载显著上升，容易丢帧/错误帧），PID 的 `dt`（DWT 实测算）虽自适应，但积分/微分与滤波参数是按 200Hz 整定的。
- 建议: 补 `cnt++;`（或用 DWT 时间戳分频），并确认 1kHz 下 CAN 负载与 MI 电机协议允许。

---

## 中

### M1. `SuperCapSend` 越界读 6 字节：接口 memcpy 8 字节，实参只有 2 字节
- 位置: `Modules/super_cap/super_cap.c:84-88`、`application/chassis/chassis.c:26,120,193`
- 证据:
```c
// chassis.c:26,120,193
static uint16_t power_data;
power_data = chassis_cmd_recv.power_limit;      // float -> uint16_t 截断
SuperCapSend(cap, (uint8_t*)&power_data);       // 只有 2 字节有效
// super_cap.c:86
memcpy(instance->can_ins->tx_buff, data, 8);    // 读 8 字节
```
- 问题: 每次调用都越界读 6 字节相邻 .bss 并把它们当控制量发给电容板；同时 `chassis_power_limit`（float）→`uint16_t` 无条件截断。行为取决于电容板固件如何解释这些字节，属于"不可预期但一定存在"的缺陷。
- 建议: 给 `SuperCapSend` 增加长度参数（或改成定长结构体 + `static_assert(sizeof==8)`），调用方传 `sizeof(power_data)`；确认电容协议对该字段的单位与缩放。

### M2. `DJIMotorControl` 停机清零越界写 `tx_buff`
- 位置: `Modules/motor/DJImotor/dji_motor.c:304-305`、`bsp/can/bsp_can.h:74-77`
- 证据:
```c
if (motor->stop_flag == MOTOR_STOP)
    memset(sender_assignment[group].tx_buff + 2 * num, 0, 16u);
```
- 问题: `tx_buff` 只有 8 字节，`num∈[0,3]`，写入区间为 `[2*num, 2*num+15]` → 越界 8~14 字节，落到同结构体的 `rx_buff[8]/rx_id/rx_len`（`#pragma pack(1)` 布局）。目前这些字段对"只发不收"的 `sender_assignment` 无实际用途，所以暂时没炸；但会**清掉同组中先于它写入的电机指令**（例如某一路摩擦轮/底盘电机被 Stop 时，注册顺序在它之前的同组电机指令被清零），属于隐患级内存破坏。
- 建议: `memset(..., 0, 2)`（只清本电机的两字节），或先清零整帧再统一填充。

### M3. 堆压力：20KB 堆里要放 5 个任务栈(约 14.5KB)，且所有 malloc 都不判空
- 位置: `Inc/FreeRTOSConfig.h:66-67`、`application/robot_task.h:38-52`、`Middlewares/.../CMSIS_RTOS/cmsis_os.c:202-230`、`dji_motor.c:167`、`daemon.c:13`、`message_center.c:74,96,104`、`bsp_usart.c:49`、`super_cap.c:76`
- 证据:
```c
#define configTOTAL_HEAP_SIZE ((size_t)20000)
osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024); // 词
// cmsis_os.c 把 thread_def->stacksize 直接传给 xTaskCreate (单位: 字)
DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
memset(instance, 0, sizeof(DJIMotorInstance));   // 未判 NULL
```
- 问题: 5 个任务栈 = (1024+1024+128+1024+512) 词 ≈ 14.5KB（还要加 TCB），剩下不到 5.5KB 要容纳约 9 个 `CANInstance`、8 个 `DJIMotorInstance`、11 个 daemon、6 个 topic/订阅者队列、referee/UI 结构等 —— 已接近极限。FreeRTOSConfig.h 中既无 `configCHECK_FOR_STACK_OVERFLOW` 也无 `configUSE_MALLOC_FAILED_HOOK`，失败时 `memset(NULL,...)` 直接 HardFault，或 `xTaskCreate` 失败（`osThreadCreate` 返回值无人检查）导致某任务根本不存在，现象隐蔽。
- 建议: 提高 `configTOTAL_HEAP_SIZE`（或改用静态分配 `xTaskCreateStatic`）；所有 `malloc` 判空并进错误状态；打开栈溢出检查与 malloc 失败钩子。

### M4. daemon 计数器在中断与任务间竞态；reload_count=0 时一注册就离线
- 位置: `Modules/daemon/daemon.c:27-30`（喂狗，在中断中被调用：`dji_motor.c:131`、`master_process.c:53`、`remote_control.c:96`、`rm_referee.c:102`）、`daemon.c:44-45`（任务中递减）、`daemon.h:15,18`（`uint16_t`）、`daemon.c:17 vs 21`
- 问题: `temp_count` 是 16 位非原子变量，中断里 `=` 与任务里 `--` 构成读改写竞态（任务读到旧值后写回，可能吞掉一次喂狗或凭空递减），离线判定时间会抖动；另外 17 行把 0 修正为 100，21 行却用**未修正的** `config->reload_count` 赋给 `temp_count`，因此传 0 的模块会一注册就持续触发离线回调。
- 建议: 用临界区（或 `ATOMIC`/单写者方式）保护；`temp_count = instance->reload_count;`。

### M5. 单位混用：IMU 角度是"度"，pitch 指令是"弧度"，换算写反且该字段无人使用
- 位置: `Modules/algorithm/QuaternionEKF.c:201-203`、`Modules/imu/ins_task.c:168`、`Modules/motor/MImotor/mi_motor.c:353`（`@param location 控制位置 rad`）、`application/robot_def.h:19-20`、`application/cmd/robot_cmd.c:195,237`
- 证据:
```c
// QuaternionEKF.c:203  EKF 输出乘了 57.29 → 度
QEKF_INS.Pitch = asinf(...) * 57.295779513f;
// robot_cmd.c:195  (度-度)/57.39 : 既不是弧度也不是度
gimbal_cmd_send.real_pitch = ((...Pitch) - gimbal_fetch_data.init_location)/57.39;
// robot_cmd.c:237  视觉 pitch 误差(度) × 弧度增益
gimbal_cmd_send.pitch += 0.0005f * err_filtered;
```
- 问题: 同一条云台链路里 `attitude_t.Pitch` 是度、`gimbal_cmd_send.pitch` 与 `PITCH_MIN/MAX` 是弧度（`MI_motor_LocationControl` 要求 rad，`measure.angle` 也是 rad），视觉误差又是度；缩放系数全靠试凑。`real_pitch` 转换方向错误，而且全项目没有任何地方读取它（死字段）。前期"软件限位与机械限位不一致"的 pitch 问题，与这种单位混用是同一类风险来源。
- 建议: 统一命名（`*_deg` / `*_rad`）与转换宏（项目已有 `RAD_2_DEGREE`/`DEGREE_2_RAD` 却没用），视觉回传明确单位；删除或修正 `real_pitch`。

### M6. 启动期全程关中断 + 初始化无超时重试
- 位置: `application/robot.c:20-39`、`Modules/imu/ins_task.c:89-90`、`bsp/dwt/bsp_dwt.h:89`、`Src/stm32f4xx_hal_timebase_tim.c`
- 证据:
```c
__disable_irq();
BSPInit(); RobotCMDInit(); GimbalInit(); ShootInit(); ChassisInit();
OSTaskInit();
__enable_irq();
// ins_task.c:89
while (BMI088Init(&hspi1, 1) != BMI088_NO_ERROR) ;
```
- 问题: HAL 时基是 TIM14 中断（`stm32f4xx_hal_timebase_tim.c`），`bsp_dwt.h:89` 也明确警告"禁止在 __disable_irq() 与 __enable_irq() 之间使用 HAL_Delay"。当前初始化里的等待若依赖 HAL 超时/延时（或将来有人加一句 HAL_Delay），在该窗口内永远不会满足；而 `BMI088Init` 又是无上限重试 —— IMU 未插好/损坏时机器人在关中断状态下永久空转，看门狗与日志都无法救回。
- 建议: 把 `__disable_irq()` 缩小到真正需要原子性的几行；初始化重试加上次数/时间上限并进入可诊断的错误状态（蜂鸣/UI 提示）。

### M7. "以为有保护，其实没接线"的死代码
| 位置 | 现象 |
|---|---|
| `robot_cmd.c:300-310` | `AutoAimSet()` 全项目从未被调用；`DataLebel.aim_flag` 从未被赋值（referee 里那个 `aim_flag` 是另一个变量） |
| `robot_cmd.c:362-387` | `MidRoundPatrol()` 从未被调用 → 中场巡航功能不可达 |
| `robot_def.h:178` + `robot_cmd.c:480-490` | `Death_reInit` 只写不读 → "死亡重初始化"没有实现（`gimbal.c` 里没有任何相关分支） |
| `robot_cmd.c:620-627` | `chassis_cmd_send.chassis_speed_buff = 2` 但 `chassis.c` 从不读该字段 → Shift 加速无效（与自己的 TODO 注释一致）；`chassis.c:23` 的 static `chassis_rotate_buff` 同样是死变量 |
| `gimbal_algorithm.c:121-155` | `Cal_FollowControl_Set_Pitch` / `Cal_FollowControl_Feedforward` 从未被调用（pitch 实际走 `robot_cmd.c:214-244` 另一套滤波） |
- 建议: 要么接线并加测试，要么删除，避免维护者误判"已有该保护"。

---

## 低

- **L1** 同一类「把地址当数值打印」的格式化错误（本应被 `-Wall -Wformat` 抓到）：`application/robot_task.h:68,85,104,121` 的 `LOGERROR("... dt = [%f]", &ins_dt)` 应为 `ins_dt`；`bsp/can/bsp_can.c:122` 的 `LOGERROR("... tx [%d] or rx [%d] ...", &config->tx_id, &config->rx_id)` 应为 `config->tx_id, config->rx_id`。两处都在错误分支里，输出的诊断信息恰恰不可信。
- **L2** `application/chassis/chassis.c:161-178`: `LimitChassisOutput()` 注释写"根据裁判系统和电容剩余容量对输出进行限制"，实际没有任何限幅（`:33` 也自标"待进行限幅"）；`cap` 未判空即解引用；`power_flag` 判据 `13 < vol < 24` 语义存疑，而 `robot_cmd.c:587-594` 用它把陀螺速度提到 2 倍 —— 若方向理解反了，会在电容电量偏低时加速。
- **L3** `super_cap.c:65-72`: 只解析 `vol`，`current/power` 恒为 0；`SuperCap_Init_Config_s` 的 `recv_data_len/send_data_len` 被完全忽略（死配置）。
- **L4** `application/robot_task.h:20-24`: 任务句柄与 `OSTaskInit()` 定义在头文件里且非 static/inline（仅靠注释"只能被 robot.c 包含"约束），一旦第二个 .c 包含就重复定义。
- **L5** `CMakeLists.txt:41` `add_compile_options(-w)`: 关掉了**全部**编译警告 —— 上面 L1、隐式转换、未使用变量等问题本应由编译器提前暴露。建议 `-Wall -Wextra` 并分批清理。
- **L6** `Modules/ist8310/` 全项目无引用（被 CMake 编译进去的死模块）。
- **L7** `chassis.c:95-98`: 把长度量乘 `DEGREE_2_RAD` 后与 `wz` 相乘，量纲不一致（历史遗留，需确认 `wz` 单位是 deg/s 还是 rad/s）。

---

## 第二批：Modules 驱动 / 通信层（并行深审 + 我逐条复核）

> 本批由子代理深审，**标为高/致命与中危的每一条我都自己 read/grep 复核过**（含下面引用的原文）；与第一批重复的（`dji_motor.c:305`、`master_process` 并发发送、daemon、malloc 未判空、BMI088 无超时重试）已合并进 H2/H3/M2/M3/M4/M6，不再重复。标注「当前无调用方」的是已埋好但尚未接线的雷。

### 高

#### N1. 裁判系统帧长未校验就用于指针跳转与递归 —— ISR 内越界读任意地址 + 递归深度不受限
- 位置: `Modules/referee/rm_referee.c:29-40,92-95`（接收缓冲仅 255 字节: `:10,118`）
- 证据:
```c
memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);   // DataLength 未做任何范围校验
if (buff[SOF] == REFEREE_SOF) {                        // :32 首字节是 0xA5 即进入
    if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE) { ... }   // :35 CRC8 块(与 :92 平级)
    // :92 在 CRC8 块之外 —— CRC 失败也会执行
    if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
        JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
}
```
- 问题: `DataLength` 是帧头里的 16 位字段（最大 65535），直接参与地址计算 `buff+9+DataLength` → 对 255 字节缓冲区之外最多约 64KB 的任意地址读，落到不可访问区即 HardFault；若该字节恰为 0xA5 则递归，步长最小 9 字节、深度不限。该回调运行在串口中断里（`bsp/usart/bsp_usart.c:102-113`），用 MSP 栈，递归过深会踩坏栈下内存。触发条件很低：拔插瞬间、线缆噪声、DMA 把一帧拆成两次接收都会命中。另外 `:38` 算 CRC 长度用的是 `buff[DATA_LENGTH]`（低字节），与 `:92` 的 16 位口径不一致。
- 建议: ① 让 `bsp_usart` 把本次接收长度（`HAL_UARTEx_RxEventCallback` 的 `Size`，代码注释写着"暂时没用"）透传给 module_callback；② 解析前校验 `len >= 9` 且 `DataLength <= len - 9`（裁判数据段本就 <= 113 字节，可直接设上限）；③ 只在 CRC8/CRC16 通过后跳转；④ 递归改 while + 最大帧数上限。

#### N2. 视觉接收 CRC16 被整体注释 —— 未校验字节直接决定云台指向与开火
- 位置: `Modules/master_machine/seasky_protocol.c:42-53,97-112`、`master_process.c:50-55,106`
- 证据:
```c
if (rx_buf[0] == PROTOCOL_CMD_ID) {          // 只判 0x5A
    pro->header.sof = rx_buf[0];
    //pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];   // 长度解析被注释
    return 1; }
...
date_length = OFFSET_BYTE + pro.header.data_length;   // data_length 恒为 0
//if (CRC16_Check_Sum(rx_buf, date_length)) {          // :100 CRC16 校验被注释!
    memcpy(&recv_data->Vision.yaw,     &rx_buf[13], sizeof(float));
    memcpy(&recv_data->Vision.pitch,   &rx_buf[17], sizeof(float));
    memcpy(&recv_data->Vision.can_fire,&rx_buf[21], sizeof(int32_t));
```
- 问题: 解析函数签名（`master_process.h:129`）没有长度参数，`DecodeMinpc` 也拿不到本次长度，因此只要缓冲区首字节是 0x5A 就无条件采信 27 字节：短帧/半帧读到的是上次残留或 `memset` 后的 0；噪声命中概率约 1/256。而 `Vision.yaw/pitch/can_fire` 是直接控制量 —— `robot_cmd.c:208-210` 用 `Vision.yaw` 直接算云台目标角，`robot_cmd.c:168-175,414-423,520-521` 用 `can_fire` 决定是否连发。随机 4 字节还可能被当成 NaN/Inf 送进 PID 参考值（与 N7 联动）。
- 建议: 恢复 `CRC16_Check_Sum`（`:30-39` 已实现，取消注释即可）失败丢帧；把接收长度透传到解析函数并校验 `len >= 27`；应用侧对 `yaw/pitch` 做范围 + `isfinite` 检查，为 `can_fire` 增加"本帧有效"标志。

#### N3. BMI088 陀螺初始化索引下溢 → 越界读表 + 死循环（正在启动路径上）
- 位置: `Modules/imu/BMI088driver.c:14,310-327`；调用链 `robot.c:29 GimbalInit` → `gimbal.c:21 INS_Init` → `ins_task.c:89 while (BMI088Init(&hspi1,1) != BMI088_NO_ERROR);` → `BMI088driver.c:97`
- 证据:
```c
static uint8_t write_reg_num = 0;                       // :14 无符号
for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
    if (res != BMI088_Gyro_Init_Table[write_reg_num][1]) {
        write_reg_num--;                                    // :321 在 0 时下溢为 255
        error |= BMI088_Accel_Init_Table[write_reg_num][2]; // :323 越界读 6x3 表 + 用错表(陀螺查加速度计表)
    }
}
return BMI088_NO_ERROR;                                     // :327 无论 error 都返回成功
```
- 问题: 只要第 0 个寄存器写后回读不匹配（SPI 失联、虚焊、CS 异常、MISO 悬空读 0xFF 时必然不匹配），`write_reg_num` 变 255 → 以 255 索引 6 行表，越界读近 800 字节任意内存当错误码；随后 `write_reg_num++` 由 255 回绕到 0，**永远卡在第 0 个寄存器**，函数不返回。该调用发生在 `RobotInit()` 的 `__disable_irq()` 窗口内（`robot.c:25-37`），且无 IWDG → 静默卡死在启动，姿态恒为 0。`:327` 还恒返回成功，把 `error` 吞掉，掩盖真实故障。
- 建议: 索引 0 时直接返回错误码并用陀螺表；初始化加最大重试次数；`return error;`。

#### N4. 裁判 UI 的 `vsprintf` 写爆 30 字节缓冲
- 位置: `Modules/referee/referee_UI.c:327-331`，目标缓冲 `referee_protocol.h:328-332`
- 证据:
```c
typedef struct { Graph_Data_t Graph_Control; uint8_t show_Data[30]; } String_Data_t;  // 只有 30 字节
vsprintf((char *)graph->show_Data, fmt, ap);                       // 无长度上限
graph->Graph_Control.end_angle = strlen((const char *)graph->show_Data);
```
- 问题: `UICharDraw` 是公开的 printf 风格 API（`referee_UI.h:65-66`），`fmt` 由调用方决定。当前调用点是若干静态短串（`referee_task.c:83-87` 一带），暂未超；一旦某条状态文本超过 29 字符（例如改成带数值的 `"rpm=%d"`）就越界写相邻 static 数组（`UI_State_sta[12]`/`UI_State_dyn[6]`），覆盖图形数据乃至 `referee_id_t`、UI 序列号，使后续所有 UI 包（含清屏）发错目标；`strlen` 还会继续读未终止内存。
- 建议: 改 `vsnprintf(show_Data, sizeof(show_Data), fmt, ap)` 并 clamp 返回值；调用前定长清零。

### 中

#### N5. 裁判 0x0104 用错长度常量，越界写 1 字节
- 位置: `Modules/referee/rm_referee.c:60-62`、`referee_protocol.h:82-83,146`
- 证据: `case ID_referee_warning: memcpy(&referee_info.RefereeWarning, (buff + DATA_Offset), LEN_event_data);` —— `LEN_event_data = 4`（0x0101 用）、`LEN_referee_warning = 3`（0x0104 专用，定义了却没人用），而 `ext_referee_warning_t` 是 3 字节结构体。
- 问题: 多拷 1 字节，落在紧随其后的 `PowerHeatData.chassis_voltage` 最低字节（`rm_referee.h:33-34`，pack(1)）。危害有限（该字段每 100ms 被 0x0202 刷新），但确定是复制粘贴型越界。
- 建议: 改用 `LEN_referee_warning`（或统一 `sizeof(目标成员)`），拷贝前确认 `judge_length` 足够。

#### N6. MI 电机 6 个通信 API 写进接收实例、却发送另一个实例（或根本不发）
- 位置: `Modules/motor/MImotor/mi_motor.c:106-117`（GetID）、`:232-245`、`:255-268`、`:278-293`、`:304-316`（WritePram）、`:318-331`（MIMotorSetPid / MiMotorSetRef 的唯一实现路径）
- 证据:
```c
motor->motor_can_instace->tx_buff[i]=0;     // 写的是 CANRegister() 返回的"接收"实例
CANTransmit(&mi_sender_assignment[0], 5);   // 发的却是另一个静态发送实例(其 tx_buff 从未被填)
```
- 问题: 这 6 个公开 API 实际不产生有效报文（`MiMotorControl()` 周期发送的 `mi_sender_assignment[0]` 只由 `MI_motor_Control()` 填充）。**已确认这 6 个 API 全工程无调用方**，云台 pitch 走的是能正常工作的 `MI_motor_LocationControl → MI_motor_Control`（`:132-146` 同时写了发送实例的 `ExtId` 与 `tx_buff`，我复核过），属"已埋雷"。另 `:137,164,188,213` 用 `*((uint32_t*)&EXT_ID)` 违反严格别名规则。
- 建议: 统一到 `mi_sender_assignment[0]` 发送（同时设 `ExtId`），或显式区分收发实例；`EXT_ID` 用 `memcpy` 取值；头文件注明这些 API 需配合 `MiMotorControl()`。

#### N7. 电机输出 float→int 无 NaN/硬限幅保护
- 位置: `dji_motor.c:295`（`set = (int16_t)pid_ref;`）、`mi_motor.c:32-34`（`FloatToUint` 只夹端点）
- 问题: C11 6.3.1.4 规定浮点值无法由目标整型表示时该转换是未定义行为；DJI 电调电流上限约 ±16384/±30000，越界可能环绕成反向值。当前靠各环 `MaxOut <= 32767` 与 `f_Output_Limit`（`controller.c:84-94`）兜着；一旦某个闭环未启用或上游冒出 NaN（见 N2 未校验视觉数据、除零、EKF 数值问题）就会失控。`FloatToUint` 对 NaN 既不 >max 也不 <min，会带 NaN 做浮点转整数。
- 建议: 转换前 `if (!isfinite(pid_ref)) pid_ref = 0.0f;` + 按电机型号硬限幅 + `lrintf`；`FloatToUint` 开头加 `isfinite` 并断言 `1 <= bits <= 16`。

#### N8. CAN 扩展帧路由只支持一个实例；EXT/STD 滤波器 bank 重叠且 `SlaveStartFilterBank` 未初始化
- 位置: `bsp/can/bsp_can.c:16`（`static uint8_t ex_idx;`）、`:130-140`（注册时 `ex_idx=idx;`）、`:236-258`（EXT 分支只投给 `can_instance[ex_idx]`，不校验 `ext_id->motor_id`）、`:51-79`（`CANAddEXFilter`）
- 问题: ① 任何后注册的 EXT 设备都会把 `ex_idx` 指向自己 → **所有**扩展帧反馈都进最后一个实例；目前只有一台 MI 电机（云台 pitch）所以能用，接第二台即互相串数据。② `CANAddEXFilter` 中 `can_filter_conf` 是未初始化局部变量，`SlaveStartFilterBank` 只在 `hcan2` 分支赋值，`hcan1` 分支用的是栈垃圾值。③ CAN2 的 EXT 过滤器固定占 `FilterBank=14`，而 STD 过滤器也按 `can2_filter_idx++` 从 14 起分配（`:33-49`），两者抢同一 bank，目前靠 EXT mask 全 0 掩盖。
- 建议: EXT 按 `ext_id->motor_id` 查表路由；`CAN_FilterTypeDef` 声明处 `= {0}`；显式规划 bank 空间（STD 0-13 / EXT 独占 14）并用错误计数验证。

#### N9. 遥控器数据 ISR 解析 vs 任务清空，无临界区
- 位置: `Modules/remote/remote_control.c:12,55,94-106`
- 问题: `sbus_to_rc()` 在串口中断里逐字段改写 `rc_ctrl[TEMP]`（含 3x16 的 `key_count` 计数表）后整体 `memcpy` 到 `rc_ctrl[LAST]`；`RCLostCallback` 却在 DaemonTask 任务上下文对整个 `rc_ctrl` 做 `memset`，两者无互斥、`rc_ctrl` 也非 `volatile` → 掉线瞬间可能清掉正在解析的按键计数（丢上升沿 = 按键不响应）或产生半新半旧一帧。`:55` 还以 `*(uint16_t*)&rc_ctrl[TEMP].key[KEY_PRESS]` 绕开联合体成员做指针双关。
- 建议: ISR 写本地缓冲、任务侧取快照（双缓冲+序号）；离线只置标志，由任务统一清理并重启接收；改用 `key[KEY_PRESS].keys = ...`。

### 低（Modules 层）

- `Modules/motor/DMmotor/dmmotor.c:137-140`: 已算好的"取反 + 限幅"结果 `set` 被丢弃，实际下发的是未取反、未限幅的 `pid_ref` → `motor_reverse_flag` 对力矩完全无效；`pid_ref > DM_T_MAX` 时 12 位编码环绕成反向力矩。**已确认 `DMMotorInit` 全工程无调用方**。
- `Modules/BMI088/bmi088.c:334-335`: 标定静止判据比较的是从未被更新的 `_bmi088->gyro`（阻塞模式下 `BMI088Acquire` 只写 `data_store`），应为 `raw_data.gyro[j]` → "标定期间是否在动"的三轴判据失效。**该模块（`BMI088Register`）全工程无调用方**，实际使用的是 `Modules/imu/BMI088driver.c` 那条路径。
- `dji_motor.c:76-78,112-113` 与 `bsp_can.c:114-115,121-122`、`bsp_usart.c:40-47,76-78`: 错误分支清一色 `while(1) LOGERROR(...)` —— 无 IWDG 时就是"初始化期静默卡死"，且日志语句里还把 `&` 地址当 `%d` 打印（已并入 L1）。

---

## 第三批：BSP / RTOS / 初始化 / 构建（并行深审 + 我逐条复核）

> 同样只收录我复核过的条目；与既有条目重复的（`USARTIsReady` 位或、`-w` 关警告、无 IWDG、启动期关中断、malloc 不判空、EXT 单实例路由）已合并，不重复计价。**其中一条子代理判为"致命"的结论我实测推翻并降级（见 B1）**。

### 高

#### B1.（重要更正）`BuzzerTask()` 确实解引用空指针，但**不会** HardFault —— 后果是"蜂鸣器报警功能从未生效"
- 位置: `application/robot_task.h:100-101`（daemon 任务每 10ms 调用）→ `Modules/alarm/buzzer.c:6,8,25,50-53`
- 证据（我复核）:
```c
static PWMInstance *buzzer;                                  // :6
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0}; // :8 全 NULL
void BuzzerTask() { buzz = buzzer_list[i];
    if (buzz->alarm_level > ALARM_LEVEL_LOW) continue; ... }   // :52-53
```
  `BuzzerRegister()`（`:25`）**全工程只有定义、没有任何调用者**（grep 确认），所以 `buzzer_list[0..4]` 恒为 NULL。
- 为什么没炸: `alarm_level` 在 `BuzzzerInstance` 中的偏移是 8，读取的是 **0x00000008**；F407 从 Flash 启动时 0x00000000 段被别名到 0x08000000，该处放着中断向量表 —— 读到的是 `Reset_Handler` 地址（一个大数），远大于 `ALARM_LEVEL_LOW(4)`，于是 5 次循环全部 `continue`，函数直接返回。所以现象是"不崩、但蜂鸣器一声不响"。
- 仍要修的理由: 这是**依赖启动别名与结构体布局才侥幸不崩**的空指针解引用 —— 一旦改用 BootLoader/重映射、加 MPU、调整成员顺序或换成非别名的读法（写操作更会直接触发 BusFault），立刻变成 HardFault；而且报警通路形同虚设（`AlarmSetStatus` 也没有消费者），与 daemon 的离线告警设计意图完全脱节。
- 建议: `BuzzerTask()` 里加 `if (buzz == NULL) continue;`，并在 `BuzzerInit()` 中按等级注册实际需要的蜂鸣器实例（或把循环上界改为已注册数量）。

#### B2. CAN 电机离线保护形同虚设：丢失回调只打印日志，不停机也不冻结指令
- 位置: `Modules/motor/DJImotor/dji_motor.c:157-162`（注册于 `:189-194`，由 `robot_task.h:100` 的 `DaemonTask()` 驱动）
- 证据:
```c
static void DJIMotorLostCallback(void *motor_ptr) {
    ... LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", ...);   // 只打日志
}
```
- 问题: daemon 以 `reload_count=2`（20ms 无反馈）判离线，但回调既不置 `motor->stop_flag`、也不清零指令，`DJIMotorControl()` 继续按最后目标值发电流帧，PID 继续用陈旧的 `measure` 计算。全工程只有 `remote_control.c:134` 消费 `DaemonIsOnline()`，CAN 侧无人消费 —— 电机断线/总线故障时"离线保护"完全不起作用。
- 建议: 丢失回调里 `motor->stop_flag = MOTOR_STOP`（`dji_motor.c:304` 会下发零电流）并上报故障；恢复在线后要求显式重新使能。

#### B3. DMA2_Stream5（USART1 视觉接收）中断优先级 0，高于 FreeRTOS 系统调用上限，且该中断里正在跑协议解析
- 位置: `Src/dma.c:57,63,66`、`Src/usart.c:176`（`hdma_usart1_rx.Instance = DMA2_Stream5`）、`Inc/FreeRTOSConfig.h:100,111,118`、`bsp/usart/bsp_usart.c:102-113`、`master_process.c:50-55`
- 证据:
```c
HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);   // SPI1_RX  (Src/dma.c:57)
HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);   // SPI1_TX  (:63)
HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);   // USART1_RX(视觉) (:66)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5   // FreeRTOSConfig.h:111
```
- 问题: 分组为 4（`HAL_Init` 设定），优先级 0 是最高的，**不受 BASEPRI=5 屏蔽**：这条链（DMA2_Stream5 → `HAL_UARTEx_RxEventCallback` → `DecodeMinpc` → 视觉解析）里任何 `*FromISR` 调用或与任务共享的读写都得不到内核保护，`vPortValidateInterruptPriority()` 命中 `configASSERT` 就是关中断死循环（`FreeRTOSConfig.h:123`）。其余所有外设中断都是 5 或 15，唯独这 3 个 DMA 是 0 —— SPI1 那两个当前未启用 DMA（BMI088 走阻塞传输）所以是埋雷。
- 建议: 把这 3 个 DMA 中断优先级改成 5，与同外设的 `USART1_IRQn/SPI1_IRQn` 保持一致。

### 中

#### B4. CAN 标准帧过滤器结构体未初始化：16 位列表模式 4 个条目只填了 1 个
- 位置: `bsp/can/bsp_can.c:33-48`
- 证据: `CAN_FilterTypeDef can_filter_conf;`（`:35` 无初始化），只赋值了 `FilterMode/FilterScale/FilterFIFOAssignment/SlaveStartFilterBank/FilterIdLow/FilterBank/FilterActivation`，而 16 位 **列表** 模式下 HAL 会把 `FilterIdLow/FilterIdHigh/FilterMaskIdLow/FilterMaskIdHigh` 当作 4 个 ID 全部写进 bank → 每个 bank 放行 4 个 ID，其中 3 个是栈垃圾（同一构建下取值稳定）。`HAL_CAN_ConfigFilter` 的返回值也未检查。
- 后果: FIFO0/FIFO1 被灌入无关报文 → 每次中断遍历全部实例、高负载时 FIFO 深度 3 溢出且**丢帧是静默的**（未实现 `HAL_CAN_RxFifo0FullCallback`）。
- 建议: `= {0}`；改用 32 位 IDMASK 单 ID（`FilterIdHigh = rx_id << 5`、`FilterMaskIdHigh = 0x7FF << 5`）；检查 HAL 返回值。同样规则适用于 `CANAddEXFilter`（`:51-60`，目前"恰好"每个字段都赋值了）。

#### B5. CAN2 扩展帧过滤器把 yaw 用的 bank 14 覆盖成"掩码全 0 全通"（已并入 N8，此处补充必然性）
- 位置: `bsp/can/bsp_can.c:36,44-45,58-59,73-74`；注册顺序 `application/gimbal/gimbal.c:82-83`、`:63-67`
- 要点: `can2_filter_idx` 从 14 起自增 → yaw（标准帧）占用 CAN2 的 bank 14；**紧接着**注册的 pitch（`ext_flag=1`）在 `CANAddEXFilter` 里把同一个 bank 14 重写成 32 位 IDMASK 且 `FilterMaskId*(High/Low)=0` → CAN2 上的过滤器实际变成"全通"，标准帧过滤被清掉。每次上电必然发生，因 yaw 的 FIFO 也是 FIFO0 才被掩盖。
- 建议: 见 N8（统一 bank 分配链、EXT 用 IDMASK 只放行自己的 EXT_ID、越界返回错误）。

#### B6. `bsp_iic.c` 连续两次 malloc（内存泄漏）+ `IIC_SEQ_HOLDON` 与 `IIC_SEQ_RELEASE` 撞值
- 位置: `bsp/iic/bsp_iic.c:14-16`、`bsp/iic/bsp_iic.h:26-30`
- 证据:
```c
IICInstance *instance = (IICInstance *)malloc(sizeof(IICInstance));
instance = (IICInstance *)malloc(sizeof(IICInstance));   // 第一次的地址被覆盖 → 永久泄漏
memset(instance, 0, sizeof(IICInstance));                 // 且未判空
```
  另外 `IIC_SEQ_RELEASE` 是枚举首项（0），而 `IIC_SEQ_HOLDON = 0` 显式等于 0 → `seq_mode` 校验失去意义、"保持总线"永不生效。
- 触发: 任何 `IICRegister()`；当前 `Modules/ist8310` 全工程无引用，属潜在缺陷。
- 建议: 删除重复 malloc、判空；去掉 `IIC_SEQ_HOLDON = 0` 并加 `_Static_assert` 防止撞值。

#### B7. `bsp_spi.c` 忙标志只初始化第一个元素（SPI2 会死等）；DMA/IT 发送不释放片选
- 位置: `bsp/spi/bsp_spi.c:8,41-63,99-110`
- 证据: `uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT] = {1};` —— 只有 `[0]=1`（"未传输"），`[1]=0`（"正在传输"），因此对 SPI2 调用 `SPITransRecv()` 会在 `while (!SPIDeviceOnGoing[1]) {}` 里**永远空转**（该标志只在传输过程中才被写回 1）；且这些等待都在任务里忙等、无超时。另外 DMA/IT 发送分支只拉低片选、没有对应的 `HAL_SPI_TxCpltCallback` 释放片选。
- 触发: 使用 SPI2，或把 SPI 切到 DMA/IT 模式纯发送；当前 `Modules/BMI088/bmi088.c` 无调用者 → 潜在缺陷。
- 建议: `= {1, 1}`（或循环赋初值）；等待加超时 + `osDelay(1)`；补发送完成回调释放片选。

#### B8. 时钟树三方不一致：源码/ HAL 认为 12MHz，`.ioc` 写 8MHz，DWT 又硬编码 168
- 位置: `Src/main.c:166-169`（PLLM=6, PLLN=168, PLLP=2）、`Inc/stm32f4xx_hal_conf.h:98`（`HSE_VALUE 12000000U`）、`Infantry-Little.ioc:148,155`（`RCC.HSE_VALUE=8000000`、`RCC.PLLM=4`）、`bsp/bsp_init.h:18`（`DWT_Init(168)`）
- 我的判定（比子代理更保守）: **当前源码自洽** —— 12MHz/6×168/2 = 168MHz，DWT 的 168 也对得上，所以现在通信与时间轴是准的。真正的风险有两个：① 若板上晶振其实是 8MHz，则 HAL 按 12MHz 算出的 USART/CAN 分频全部偏 1.5 倍（115200→76800、1Mbps→667kbps），DWT 时间轴也快 1.5 倍（PID 的 dt、CAN 超时、任务 dt 统计全错）；② 若晶振是 12MHz 而某天用 CubeMX 按 `.ioc` 重新生成，PLLM 会被写回 4 → 12/4×168/2 = **252MHz，超过 F407 上限，直接跑不起来**。
- 建议: 实测 HSE（MCO 输出或读 `HAL_RCC_GetSysClockFreq()`）后统一三处；`DWT_Init(168)` 改为 `DWT_Init(SystemCoreClock / 1000000U)`。

#### B9. USART 接收回调不向模块传长度、只清 `Size` 字节，且解析全在中断里
- 位置: `bsp/usart/bsp_usart.c:102-118`、`bsp/usart/bsp_usart.h:11`
- 问题: 回调类型是 `void (*)()`，模块拿不到本次长度，于是 `JudgeReadData(...)`（`rm_referee.c:99-105`）、`get_protocol_info_vision(...)`（`master_process.c:54`）这些"按帧内长度或固定长度解析"的模块无法判断本包有多长，短包时会解析到上一包残留；`memset(recv_buff, 0, Size)` 只能缓解、且对更短的新包无效。这正是 N1/N2 两个高危的共同根因。
- 建议: 回调改成 `void (*)(USARTInstance *, uint16_t len)` 并把 `Size` 透传；或在回调里按 `recv_buff_size` 清满；重解析改由 `osSignalSet` 唤醒任务处理（`bsp/bsp_tools.h:5-7` 已有此建议）。

#### B10. `bsp_flash.c` 无地址校验、越界默认落到 SECTOR_11、写入循环多写一个 word
- 位置: `bsp/flash/bsp_flash.c:19-32,58-80,167-224`
- 问题: ① `ger_sector()`（`:167-224`）对任何越界地址都返回 `FLASH_SECTOR_11`，且 `flash_erase_address()` 丢弃 `HAL_FLASHEx_Erase` 的返回值 → 传错地址会**擦掉最后 128KB（通常是程序尾部）**且调用方无从得知；② `flash_write_single_address` 的循环条件是 `uw_address <= end_address`，而 `end_address` 是下一扇区起始地址 → 会多写一个 word 到相邻扇区；③ 擦写为阻塞操作（128KB 扇区擦除可达秒级）。当前全工程无调用者。
- 建议: 校验地址范围/对齐/不与代码段重叠；越界返回错误码；循环改 `<`；检查并上传 HAL 返回值；擦写放到低优先级任务。

#### B11. `usbd_cdc_if.c` 的 ISR/任务共享量非 volatile、`hcdc` 未判空
- 位置: `Src/usbd_cdc_if.c:34-36,233,287-302`
- 问题: `cdc_connected/rx_cbk/tx_cbk` 由 USB 中断写、任务读，无 `volatile`/临界区（编译器可缓存 → "主机已开串口但固件认为未连接"）；`hUsbDeviceFS.pClassData` 在枚举完成前为 NULL，`CDC_Transmit_FS` 直接解引用会 HardFault。当前 `USBInit/USBTransmit` 无调用者。
- 建议: 共享量加 `volatile` 或临界区；发送前判 `pClassData == NULL` 并检查返回值。

### 低

- `bsp/dwt/bsp_dwt.c:28-41,91-98`: 回绕计数用 `UINT32_MAX`（应 +1，每次回绕少算 1 个计数）；`CYCCNT` 被读两次、`bit_locker` 是"检查后设置"的非原子锁 → 极小概率把回绕多算一次，时间轴瞬跳约 25.6s，污染 `CANTransmit` 的超时判断（`bsp_can.c:166-174`）与所有 dt 统计。建议只读一次、用 `(uint64_t)UINT32_MAX + 1ULL`、必要时用 `__disable_irq()` 保护。
- `Src/stm32f4xx_it.c:105-116`: `HardFault_Handler` 里 `asm("bx lr")` 会从异常直接返回到触发故障的指令 → 故障无限重入，下面的 `while(1)` 永不执行，调试器看到 PC 乱跳、无任何诊断信息。建议把 CFSR/HFSR/BFAR 与栈帧写入 RTT 或 RAM 保留区后复位，至少保留 `while(1)`。
- 构建配置（`CMakeLists.txt:41`、`cmake/gcc-arm-none-eabi.cmake:30,50`、`CMakePresets.json`）：① `add_compile_options(-w)` 把工具链里已配好的 `-Wall -Wextra -Wpedantic` 全部压掉 —— 本次报告里的"位或写成恒真""过滤器结构体未初始化""%f 传指针""隐式窄化"本应被编译器直接抓出来；② `set(CMAKE_C_LINK_FALGS ...)` 变量名拼错（少一个 G），`-Wl,--no-warn-rwx-segments` 从未生效 —— 这正是我们构建日志里那句 `warning: Sentry.elf has a LOAD segment with RWX permissions` 的来源；③ `Debug` 与 `GCC 13.3.0 arm-none-eabi (ucrt64)` 两个 preset 共用 `build/Debug` 但 generator 分别是 Ninja / MinGW Makefiles → 切换 preset 会撞 CMake cache 冲突。

---

## 第四批：算法层（并行深审 + 我逐条复核）

> 与前三批重复的（PID 输出未限幅、malloc 不判空、gimbal_algorithm 死函数与单位混用、`robot_task.h` 的 `%f` 传指针、`-w` 关警告）已合并，不重复。子代理把第 2 条判为「高」，我复核后**下调为中**（触发前提是先出现 NaN/Inf），理由写在条目里。

### 高

#### A1. PID 前馈增益 `FF_Gain` 永远是 0 —— `PIDInit` 的 memcpy 字段错位
- 位置: `Modules/algorithm/controller.c:130-136`、`controller.h:55-97`（`PIDInstance`）与 `:100-117`（`PID_Init_Config_s`）；受影响的配置在 `application/gimbal/gimbal.c:39,48`
- 我的复核（按成员逐个算偏移）:
  - `PID_Init_Config_s`: Kp0 Ki4 Kd8 MaxOut12 DeadBand16 Improve20 IntegralLimit24 CoefA28 CoefB32 Output_LPF_RC36 Derivative_LPF_RC40 **FF_Gain44**，`sizeof == 48`。
  - `PIDInstance`: 前 11 个成员偏移与上面完全一致（所以 Kp/Ki/Kd/MaxOut/DeadBand/Improve/IntegralLimit/CoefA/CoefB/两个 RC 都能正确写入），但接下来是 **Measure@44**、Last_Measure48 …… Ref92、**FF_Gain@96**。
  - `memcpy(pid, config, sizeof(PID_Init_Config_s))` 只写 48 字节 = 偏移 0..47 → **`pid->FF_Gain`(96) 永远保持 memset 后的 0**，而 `config.FF_Gain` 被写进了 `pid->Measure`（下一周期就被真实测量值覆盖，所以没有额外危害）。
- 证据:
```c
// controller.c:133-135
memset(pid, 0, sizeof(PIDInstance));
memcpy(pid, config, sizeof(PID_Init_Config_s));   // 48 字节, 够不到 FF_Gain(偏移 96)
// controller.c:190
pid->Output += pid->FF_Gain*(pid->Measure - pid->Last_Measure);   // FF_Gain 恒为 0
```
- 后果: `gimbal.c:39` 的角度环 `.FF_Gain = 350.0`、`:48` 的速度环 `.FF_Gain = 500.0` **从未生效**，云台 yaw 的动态跟踪只能靠 Kp/Kd 硬顶，且没有任何编译/运行期提示（编译器本可报出结构体布局不一致，但工程开了 `-w`）。
- ⚠️ 修之前先想清楚: 现在的 Kp/Kd 是在「前馈失效」的前提下整定出来的，一旦把 FF 接通，云台会突然多出 350/500 的前馈量 → **必须重新整定后再上车**。
- 建议: `PIDInit` 改为逐字段赋值（或把 `PID_Init_Config_s.FF_Gain` 挪到与 `PIDInstance` 对应的位置），并加编译期契约 `_Static_assert(offsetof(PIDInstance, FF_Gain) == offsetof(PID_Init_Config_s, FF_Gain), "layout mismatch");`。

### 中

#### A2.（子代理判高，我下调为中）矩阵运算返回状态全部丢弃；逆矩阵失败时用「未求逆」的矩阵算 K
- 位置: `Modules/algorithm/QuaternionEKF.c:330-336,421-422`、`kalman_filter.c:326`、状态成员 `kalman_filter.h:86`
- 证据:
```c
kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // H·P'·Hᵀ
kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);                     // 期望 inv(S)
... Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);              // 用的是 temp_matrix1
```
- 我的复核: `MatStatus` 在全工程只有 1 处声明 + 若干赋值，**没有任何读取**（grep 确认）；CMSIS-DSP 文档（`matrix_functions.h`）写明矩阵奇异时返回 `ARM_MATH_SINGULAR` 且**不写 dst**，此时 `temp_matrix1` 里还是 `H·P'·Hᵀ` → `K = P'Hᵀ·(HP'Hᵀ)`，相当于「忘了求逆」，增益被放大约 S·(HP'Hᵀ) 倍。
- 为什么我下调为中: 触发前提是 S 已经奇异 —— 也就是 P 里已经出现 NaN/Inf（例如 A3 的无效量测、N2 的未校验视觉数据不直接进 EKF，但 `invSqrt(0)` 会）或 R 被写成 0；也就是说它是「已经出事之后的放大器」，而不是独立触发源。但一旦触发，后果确实是姿态飞掉（高）。
- 建议: 检查每次矩阵调用的返回值，非 `ARM_MATH_SUCCESS` 时本周期只做预测、跳过量测更新并置错误标志；给 S 加对角抖动 `R + 1e-6f·I`；在 `Kalman_Filter_Update` 出口检查 xhat/P 有限性。

#### A3. 卡尔曼的「量测有效性」机制在本工程完全失效（z 全零也当有效量测）
- 位置: `kalman_filter.c:268-274`（注释约定见 `:27-32`）、使用方 `QuaternionEKF.c:68,154-158`
- 我的复核: `UseAutoAdjustment` 全工程只有 3 处出现（注释 2 处 + 判断本身），**没有任何代码把它置 1**；`QuaternionEKF` 走的是 `else` 分支，即「原样采信 z」。于是当 BMI088 读取失败/全零时：`invSqrt(0)` 返回≈3e19 的**有限值而非 Inf**，`0×3e19=0`，z=(0,0,0) 被当成「重力方向为零向量」的有效观测（残差 = −h(x)），只靠卡方检验勉强拦截；`ins_task.c:130` 又丢弃 `BMI088_Read` 的返回值，也没有「IMU 离线→只预测」的降级路径。
- 顺带说明: 正因为该位从未置 1，子代理报的 `kalman_filter.c:458` 的 `H_data[-1]` 越界写**当前不可达**（那条路径只在 `H_K_R_Adjustment` 里），属潜伏缺陷；一旦按文件头示例启用自动调整就是确定性的堆越界写（`MeasurementMap` 被 memset 成 0，`0-1 = -1`）。
- 建议: 写 `MeasuredVector` 前判有效性（`|accel| ∈ [7.8,11.8]` 且 `accelInvNorm > 1e-3`），无效就不写；若启用自动调整，务必先修 `MeasurementMap` 的 1 起序号与越界；处理 `BMI088_Read` 返回值并定义「连续失败」的安全模式。

#### A4. `f_Integral_Limit` 用函数级 static 当临时变量，跨任务互相覆盖
- 位置: `Modules/algorithm/controller.c:40-43`（使用点 `:44-58`）
- 证据: `static float temp_Output, temp_Iout;` —— 所有 PID 实例共享这两个变量；温度环 PID 在 INS 任务（1kHz、AboveNormal）里跑（`ins_task.c:49`），云台/底盘 PID 在 motor 任务（1kHz、Normal）里跑（`dji_motor.c:266,280`），两者都会进入这个函数。
- 后果: 若「赋值 temp_Iout」与「读 temp_Iout」之间被抢占，本实例就会按**别人的积分值**决定是否 `ITerm=0`、把 `Iout` 钳到 ±IntegralLimit → 偶发积分丢失/输出抖动，堵转或大误差时更明显；该函数也彻底不可重入。
- 建议: 改成普通局部变量（两个 float，栈开销可忽略）。

#### A5.（对 M3 的重要更正）存在**两套 20KB 堆**：模块与 EKF 的 malloc 走 newlib 堆，不是 FreeRTOS 堆
- 位置: `Modules/algorithm/kalman_filter.h:31-38`、`user_lib.c:20-24`、`STM32F407IGHx_FLASH.ld:58`（`_Min_Heap_Size = 0x5000`）、`Inc/FreeRTOSConfig.h:67`（`configTOTAL_HEAP_SIZE 20000`）
- 我的复核（include 链）: `user_malloc` 的定义是 `#ifdef _CMSIS_OS_H → pvPortMalloc #else malloc`，而 `QuaternionEKF.c` 只包含 `QuaternionEKF.h → kalman_filter.h`，后者只包含 `stm32f407xx.h / arm_math.h / math.h / stdint.h / stdlib.h` —— **没有 cmsis_os.h**，所以这个 TU 里 `user_malloc` 就是 newlib `malloc`；`user_malloc` 的全部使用点都在 `kalman_filter.c`（同一个 TU 情况）。`user_lib.c:26-31` 的 `zmalloc` 更是写死 `malloc`。其余模块（`dji_motor.c:167`、`daemon.c:13`、`message_center.c:74/96/104`、`bsp_usart.c:49`、`super_cap.c:76`、`bsp_can.c`、`bsp_pwm.c`…）也都用裸 `malloc`。
- 修正后的账: FreeRTOS 堆(20KB) 主要装 5 个任务栈（(1024+1024+128+1024+512) 字 × 4B ≈ 14.5KB）+ TCB；**newlib 堆(0x5000=20KB) 装所有模块实例与 EKF 矩阵**。两者都不宽裕，且**全工程没有 `__malloc_lock`**（grep 无结果）→ ① 任一 NULL 都会走 `memset(NULL,…)` 直接 HardFault（`kalman_filter.c:155-180` 就有 10+ 次分配一个都没判）；② 只要有任务在运行期分配（`BuzzerInit()` 在 daemon 任务里调用 `PWMRegister` → malloc），就与其它任务的分配构成**无锁并发**，newlib 的堆结构可能被破坏。
- 建议: 统一到 `pvPortMalloc`（显式包含 `cmsis_os.h`）或提供 `__malloc_lock`；每处分配判空并上报；EKF 矩阵改成静态数组（6×6+3×3 合计 < 2KB），从根上去掉初始化期动态分配。

#### A6. 单位混用（与 M5 合并的新增证据）
- `gimbal_algorithm.c:48-49`: `error = cmd - current_angle`（度）而 `error_rate = -Gyro[]`（rad/s）；`:85` 的「静止」判据 `|error_rate| < 0.02f` 因此无法自洽 —— 若按 °/s 整定，实际门限松了 57 倍（≈1.15°/s），运动中也算「静止」，于是静态误差累积器持续注入补偿（上限 ±0.3°），表现为云台缓慢偏移、回中不准。
- `gimbal_algorithm.h:52-53` 的注释写「0.5rad(~29°)」，代码却把这个数当度用（配合 `robot_cmd.c:220` 的 0.3° 死区）——注释与量纲对不上。

### 低（算法层）

- `QuaternionEKF.c:480-489` 的 `invSqrt`: 用 `long*`/`float*` 类型双关（严格别名 UB，工程未加 `-fno-strict-aliasing`）；`x=0` 返回≈3e19 的有限值，把「传感器零输出」伪装成巨大的有效倒数；单步牛顿相对误差约 0.1~0.2%，而它同时用于四元数归一化、重力向量归一化与 gyro_norm → 系统性尺度偏差（当前被 `R=1e6` 掩盖）。建议直接用 `1.0f/sqrtf(x)`。
- `user_lib.c:166-173` 的 `Norm3d` 无零长度保护（`len=0` → 0/0 = NaN）；触发点 `ins_task.c:70-74` 的 `Norm3d(axis_rot)` 在「加速度与重力严格平行」时得到零向量。**我的补充**: 实际上被加速度计噪声救了（`acc_init` 几乎不可能与 (0,0,1) 严格平行），但 IMU 读失败导致 `acc_init` 全零时必定产生 NaN，并一路灌进 `IMU_QuaternionEKF_Init` 的初始四元数（全链路无 NaN 检测），最终到 `dji_motor.c:295` 的 `(int16_t)pid_ref` 就是 UB。建议 `Norm3d/Cross3d` 加 `len < 1e-6f` 保护，并对 `BMI088_Read` 的返回值做处理。
- `controller.h:24-27` 的 `#ifndef abs / #define abs(x) ((x > 0) ? x : -x)`：参数与整体括号都不全（`abs(a-b)` 会算错），且 `#ifndef` 对标准库函数恒成立 → 此头文件之后所有翻译单元的 `abs()` 都被这个语义不同的宏顶替（现有 6 处调用恰好只传单变量，暂未出错）。建议改名 `ABS_F` 并补全括号。
- `user_lib.c:199-206` 的 `AverageFilter`：`len==0` 时 `buf[-1]` 越界写 + `0/0` 返回 NaN（当前无调用者）。
- `application/gimbal_algorithm/gimbal_algorithm.h` 第 1-2 行直接 `#include <stdbool.h>`，**没有 include guard**（被 `robot_cmd.c` 与 `gimbal_algorithm.c` 同时包含）。
- `Modules/matrix/matrix.c` 实现正确（行主序索引、旋转矩阵、转置+平移反解都核对过）但**全工程无调用者**，属死代码；`MatrixCopy(A, n, m, B)` 的参数顺序与内部「n=列、m=行」的用法不一致，接入时容易误用。

### 算法层已核对无问题（避免误伤）

- PID 的位置式结构、P/I/D 顺序、梯形积分、变速积分、微分先行、微分/输出滤波公式与执行顺序均正确；`f_Output_Limit` 在 FF 之后执行，因此输出恒被钳在 ±MaxOut —— 这是 `dji_motor.c:295` 的 `(int16_t)` 转换当前安全的前提（MaxOut ≤ 20000 < 32767）。
- 变速积分**不会除零**（`CoefA==0` 时分支等价，除法不可达）；`f_PID_ErrorHandle` 的 `/fabsf(Ref)` 有 `|Ref|<1e-4` 提前返回保护。
- 各 PID 的 `dt` 来自各自的 DWT 计数器（无共享 dt），`DWT_Init(168)` 确实在 `BSPInit()` 里调用，`dt` 不会为 0。
- QuaternionEKF 的 F 矩阵（姿态部分 + 零偏 4×2 块）、H 矩阵、`h(x)`、欧拉角反解（含 YawRoundCount 的 ±180 回绕）按 `q̇ = 0.5·q⊗(ω−b)` 约定逐项推导**全部正确**；卡方检验链路维数（inv(S)·r → 1×1）与缓冲区复用无越界；自适应增益经复核**不会反号**；`acosf(fabsf(h))` 目前取不到 >1（依赖单步牛顿恒偏小这一性质，仍建议 clamp）；`Sqrt` 的牛顿迭代未找到死循环输入。

## 已检查且未发现问题的点（避免误报）

- **CAN ID 无冲突**: GM6020→`0x204+tx_id`、M3508/M2006→`0x200+tx_id`（`dji_motor.c:64,94`），实测 yaw(0x205)、摩擦轮(0x201/0x202)、拨盘(0x203)、底盘(0x201~0x204, hcan1) 互不重叠，且驱动内建冲突自检。
- **消息中心无死锁**: `PubPushMessage` 先话题锁后订阅者锁，`SubGetMessage` 只取订阅者锁，顺序一致；`QUEUE_SIZE=1` 下环形队列逻辑正确。
- **`INS_Init` 幂等**（`ins_task.c:82-85`），`GimbalInit` 与 INS 任务里各调一次是安全的。
- **`UITaskInit` 不阻塞**：等待 `robot_id != 0` 的循环在 `MyUIInit`（UI 任务内，`referee_task.c:50-55`）且带 `osDelay(100)`，并有 `init_flag` 保护，不会在关中断窗口里死等。
- **`rc_data[TEMP]` 即 `rc_data[0]`**（`remote_control.h:10` `#define TEMP 0`），不存在双缓冲误用。
- **裁判系统离线时的分支**：`robot_level` 走 default→倍率 1、`robot_id=0`→蓝方、`HP=0`→`Death_reInit`（未被使用），都不会产生危险动作。
- **`ShootRC`/`ChassisRC` 等遥控器路径的档位映射**与 `switch_is_down/mid/up` 使用一致。
- **BSP/RTOS 层已核对无问题**: 中断优先级分组（`HAL_Init` 设为 GROUP_4）、SysTick/PendSV/TIM14 = 15 且 tick 累加只在 TIM14 发生、除 DMA2_Stream0/3/5 外所有外设中断都是 5 或 15；串口缓冲上限 256 对 255/36/18 的注册长度都有余量；CAN 帧 `DLC<=8` 的 memcpy 不越界、`CANSetDLC` 有 0/>8 校验、`CANTransmit` 返回值判断方向正确；`EXT_ID_t{motor_id:8,data:16,mode:5}` 位域布局与 MI/DM 扩展帧一致；CAN 位时序 42MHz/(3x14) = 1Mbps；SPI1 Mode 3 与 BMI088 要求一致；I2C 的 `dev_address << 1` 符合 HAL 7 位地址约定；USART3 用 9B+EVEN 表示 DBUS 的 8E1；链接脚本 RAM/CCMRAM/FLASH 与 F407IG 相符；`-mcpu/-mfpu/-mfloat-abi` 与 FreeRTOS CM4F 端口匹配；`configSUPPORT_STATIC_ALLOCATION` 的 idle 任务用静态内存、不占 20KB 堆。
- **Modules 层已核对无问题**: DJI CAN ID 映射与分组（M3508/M2006 1-4 走 0x1FF、5-8 走 0x200，GM6020 1-4 走 0x1FF、5-8 走 0x2FF，反馈 ID 0x200+id / 0x204+id，`dji_motor.c:42-115` 与注释一致）；DJI 反馈解析字节序与多圈 4096 阈值判向；DM 反馈 16/12/12 位域解包；MI 协议标度（角度 ±4π、速度 ×30、力矩 ×12，`mi_motor.c:65-68`）；裁判系统各 `LEN_*` 与 pack(1) 结构体逐一核对（除 N5）；UI 帧长（`UIGraphRefresh` 最大 120 字节、`DataLength<=113`）未超 512 缓冲；CRC8/CRC16 查表与 Append/Verify 用法对称；message_center 队列与锁序（已在第一批确认）；SBUS 位段解包与 `RectifyRCjoystick` 的"5 个 int16 连续"前提成立；BMI088 寄存器常量与 SPI dummy 字节时序（`BMI088_accel_read_muli_reg` 多发一次地址正好充当 accel 需要的 dummy）；`(attitude_t *)&INS.Gyro` 的字段顺序契约当前成立（建议补 `_Static_assert`）；视觉发送帧长计算（39+2=41 <= 50）。

## 需要人工确认（缺上下文，无法判定对错）

1. 超电容协议：`vol` 的单位/缩放、`power_flag` 的期望语义、下发 2 字节的格式（当前实发 8 字节）。
2. 视觉协议中 `Vision.yaw/pitch` 的单位（度？）以及 `robot_cmd.c:237` 的 `0.0005` 增益是否与实车手感一致。
3. `Sentry_ChassisAC` 中 `*4.0f * REDUCTION_RATIO_WHEEL * 360 / PERIMETER_WHEEL * 1000` 的 `4.0f` 物理含义（`robot_cmd.c:391-392`）。
4. 底盘 `wz` 指令量纲：`ChassisRotateSet` 输出 `4000 * buff`（`robot_cmd.c:267`）与 `MecanumCalculate` 里的 `wz * LF_CENTER * DEGREE_2_RAD` 是否自洽。
5. `EnemyJudge()` 用 `robot_id > 7` 判蓝方（`robot_cmd.c:682-689`）是否与本年度规则/实际 ID 分配一致。
### 第 2 批新增待确认项

6. CAN 扩展设备路由：`bsp_can.c` 用单变量 `ex_idx` 记录"最后一个 EXT 实例"且不校验 `motor_id`（见 N8），接第二台 MI/DM 电机前必须先改；同时确认 CAN2 上 EXT 过滤器(bank14) 与 STD 过滤器起始 bank 的实际归属（建议上机看 CAN 错误计数/CAN 分析仪）。
7. MI 电机通信类型 0/17/18 的响应目前无人解析（`bsp_can.h:30-66` 定义了 `RxCAN_info_type_0_s/type_17_s` 却未使用），因此即使修好 N6 的发送路径，`MI_motor_GetID/ReadParam` 也收不到应答 —— 需确认是否要补解析。
8. `RefereeSend()` 每包 `osDelay(115)`（`rm_referee.c:135-139`，UI 任务约 8.7Hz）且发送用 DMA、接收也用 DMA（huart6）—— 需确认链路上是否真的出现"发送后收不到数据、靠 daemon 300ms 兜底重启"，若是应改 IT 发送并在 TxCplt 中重启接收。
9. 裁判系统"一包多帧"的实际行为：`JudgeReadData` 的递归假设单次接收可能粘连多帧，而 `bsp_usart.c:111-113` 每次 IDLE 回调后都会 `memset(recv_buff,0,Size)` 并重启接收 —— 需确认真实发包节奏与 255 字节缓冲、DMA 重启时序的关系（也决定 N1 的修法）。
10. `CalMiMotorTorque()`（`mi_motor.c:375-422`，当前无调用方）里 `mi_motor_instance[0]` 未判空、`pid_measure` 在非 `OTHER_FEED` 时无赋值分支却被 `PIDCalculate` 使用 —— 启用前需补齐（一并确认 `bmi088.c` 的温度二次换算问题，该模块当前未接入）。

### 第 3 批新增待确认项

11. **HSE 晶振实际频率**（决定 B8 里哪一处是错的）：代码是 `HSE_VALUE=12MHz + PLLM=6`，`.ioc` 是 `8MHz + PLLM=4`。需用 MCO/示波器实测或读 `HAL_RCC_GetSysClockFreq()` 后统一三处（含 `bsp_init.h:18` 的 `DWT_Init(168)` 硬编码）。
12. **DBUS/USART3 停止位**：当前是 1 位停止位（`Src/usart.c:80`），部分接收机手册要求 2 位 —— 现车通信正常说明接收机容忍，换接收机时需注意。
13. **栈深度实测**：`FreeRTOSConfig.h` 未开 `configCHECK_FOR_STACK_OVERFLOW`，而 daemon 任务只有 128 字（512B）却要跑 RTT 日志、UI 任务 512 字（2KB）要跑协议组包 —— 建议用 `uxTaskGetStackHighWaterMark()` 在实机确认。
14. **IWDG 策略**：`stm32f4xx_hal_conf.h:57` 的 `HAL_IWDG_MODULE_ENABLED` 被注释、`main.c` 未初始化 IWDG，全工程无看门狗。是否启用、由哪个任务喂狗、喂狗前要检查哪些任务的心跳，需要项目决策（与 H3 直接相关）。
15. **`bsp_flash.h` 的 SECTOR_12-23 常量**（`0x081E0000` 一档）超出 1MB 的 F407IG 实际扇区范围（只到 sector 11 / `0x08100000`）—— 是否有迁移计划需确认（当前无调用者，见 B10）。
16. **死代码清单对「是否有保护」判断的影响**：`bsp/bsp_tools.c` 的 `CreateCallbackTask`、`bsp/usb` 的 `USBInit`、`bsp/spi` + `Modules/BMI088/bmi088.c` 的新寄存器驱动、`Modules/ist8310`、`bsp/flash`、`Modules/alarm` 的 `BuzzerRegister` 全部无调用者（实际生效的是 `Modules/imu/BMI088driver.c` 那条路径）—— 审查时容易误判为「已有该功能」，建议清理或明确标注。

### 第 4 批新增待确认项

17. **CMSIS-DSP 是预编译库**（`Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a`，仓库内无源码），A2 里「矩阵奇异时不写 dst」是依据随附头文件文档推断的，具体阈值与行为以该 `.a` 版本为准；另外 `ARM_MATH_MATRIX_CHECK` 没有出现在任何编译选项里，若库里未开维数检查，仓库中手工改写 `numRows/numCols` 的地方（`kalman_filter.c:306-330,471-479`）一旦写错就是静默越界。
18. **yaw 串级的量纲**：`gimbal.c:50-52` 的角度环反馈是 `YawTotalAngle`（度），速度环反馈是 `Gyro[2]`（rad/s），而角度环 `MaxOut=330`（`gimbal.c:38`）—— 若按 °/s 理解则串级等效增益差 57 倍（会被 Kp 整定吸收，但换电机/换算法时会立刻失控），需作者确认。
19. **温度环量纲**：`RefTemp=40`(℃)、`Kp=1000`、`MaxOut=2000` 与 `IMUPWMSet(uint16)` 的 PWM 计数值的对应关系（`ins_task.c:38-51`）需确认。
20. `ins_task.c:128` 的 `if ((count % 1) == 0)` 恒为真（疑似本想降频），以及 `InitQuaternion` 未检查 `BMI088_Read` 返回值 —— 直接影响 A3/算法层低危两条的实际可达性。
21. **newlib malloc 的并发窗口**：`Src/syscalls.c` 未提供 `__malloc_lock`，而 `BuzzerInit()`（daemon 任务）会在运行期 malloc、其它任务也可能分配 —— 是否存在真实并发分配需结合启动时序确认（见 A5）。

---

## 修复优先级建议

原则: 先堵「会失控 / 会静默卡死」的，再修「数据面污染」的，最后做工程化清理；组内按「触发概率 × 后果」排序。

### P0 —— 立即修（每条都能单独造成失控或整机静默失效）

| # | 动作 | 位置 | 为什么必须先做 |
|---|---|---|---|
| 1 | 恢复视觉接收 CRC16 + 长度校验（`len >= 27`），失败丢帧 | `seasky_protocol.c:100`、`master_process.c:50-55` | 现在任何以 0x5A 开头的字节流都能决定云台指向与开火许可 |
| 2 | 裁判帧长做范围校验、递归改循环、只在 CRC 通过后跳转 | `rm_referee.c:92-95` | ISR 内可越界读任意地址 + 无限递归，命中即 HardFault |
| 3 | 蜂鸣器空指针判空 + 真正注册实例 | `buzzer.c:50-53`、`robot_task.h:100-101` | 现在靠 0x0 别名到 Flash 向量表侥幸不崩，且报警通路是断的 |
| 4 | BMI088 陀螺初始化索引下溢修掉 + 加最大重试次数 | `BMI088driver.c:319-327` | 位于 RobotInit 的关中断窗口，IMU 异常即启动卡死 |
| 5 | 启用 IWDG 并明确喂狗策略；把 `while(1)`/`configASSERT` 改成「停机输出 + 可诊断」 | `FreeRTOSConfig.h:123`、`stm32f4xx_hal_conf.h:57`、`shoot.c:139` 等 | 这是所有「静默失效」的兜底 |

### P1 —— 上车前修（安全与正确性）

| # | 动作 | 位置 |
|---|---|---|
| 6 | 视觉掉线时清数据/置无效标志，云台与射击只用「在线且新鲜」的数据（启用 `Power_Out`/`last_deep`） | `master_process.c:32-38`、`robot_cmd.c:139-176,414-423` |
| 7 | 电机离线回调置 `stop_flag` 并上报故障 | `dji_motor.c:157-162` |
| 8 | `motor_task.c` 补 `cnt++`（1kHz → 200Hz/100Hz），改完复核 PID 参数 | `motor_task.c:4-11` |
| 9 | 视觉串口发送收敛到单一任务（或加锁+双缓冲）；修 `USARTIsReady` 的按位或 | `robot_task.h:69`、`robot_cmd.c:720`、`bsp_usart.c:85` |
| 10 | 修 `SuperCapSend` 8 字节拷贝、`DJIMotorControl` 的 `memset(...,16u)`、裁判 0x0104 长度常量 | `super_cap.c:86`、`dji_motor.c:305`、`rm_referee.c:61` |
| 11 | 矩阵调用检查返回值 + 量测有效性判据 + 两套堆的锁与判空 | `QuaternionEKF.c:336`、`kalman_filter.c:155-180,268-274` |
| 12 | DMA2_Stream0/3/5 中断优先级 0 → 5 | `Src/dma.c:57,63,66` |
| 13 | CAN 过滤器结构体清零 + 改 32 位 IDMASK；EXT/STD 的 bank 分开；EXT 按 `motor_id` 路由 | `bsp_can.c:35-48,51-79,236-258` |
| 14 | PID `FF_Gain` 修好后**必须重新整定云台**（现有 Kp/Kd 是在前馈失效下整定的） | `controller.c:135`、`gimbal.c:39,48` |

### P2 —— 工程化与健壮性（可与新功能并行）

1. **构建侧**: 去掉 `-w`（工具链里本来就有 `-Wall -Wextra -Wpedantic`）、修 `CMAKE_C_LINK_FALGS` 拼写、给两个 preset 分配不同 binaryDir、打开 `configCHECK_FOR_STACK_OVERFLOW` 与 `configUSE_MALLOC_FAILED_HOOK`（`CMakeLists.txt:41`、`cmake/gcc-arm-none-eabi.cmake:30,50`）。**这一步做完，L1、A9、B4 等条目会被编译器直接报出来。**
2. 所有 `malloc` 判空并进入可诊断错误状态；用 `uxTaskGetStackHighWaterMark()` 实测 daemon(512B)/UI(2KB) 的栈水位。
3. `HardFault_Handler` 改成记录 CFSR/HFSR/BFAR + 栈帧后复位（现在 `asm("bx lr")` 让故障无限重入、无法诊断，`stm32f4xx_it.c:105-116`）。
4. 统一单位与命名（度/弧度、`error_deg`/`rate_deg_s`）；补 `gimbal_algorithm.h` 的 include guard。
5. 清理「看起来有保护其实是死代码」的部分（`AutoAimSet`、`MidRoundPatrol`、`Death_reInit`、`chassis_speed_buff`、`Cal_FollowControl_Set_Pitch/Feedforward`、`Modules/matrix`、`Modules/ist8310`、`bsp_flash`、`bsp/usb`、`bmi088.c`、DM 电机），并标注「未接线」以免下次审查误判。
6. 实测 HSE 并统一 `main.c` / `stm32f4xx_hal_conf.h` / `.ioc`；`DWT_Init(168)` 改 `DWT_Init(SystemCoreClock/1000000U)`。

---

## 总体结论

**做得好的地方**: 分层清晰（application 业务 / Modules 模块 / bsp 驱动 / message_center 解耦通信），注册式设计（motor、daemon、usart、can、message_center）让新增设备成本很低；DWT 计时、message_center 的锁序、CAN ID 映射与分组、EKF 的数学推导（F/H/h(x)/欧拉角）都经得起逐行核对 —— 算法层「数学写错」的问题一条都没有。

**系统性问题**（比单个 bug 更值得处理）:

1. **保护机制普遍停留在「写好了但没接线」** —— 电机离线保护、视觉离线保护（`Power_Out`/`last_deep`）、视觉 CRC 校验、卡尔曼量测有效性（`UseAutoAdjustment` 从未置位）、看门狗（IWDG 未启用）、蜂鸣器告警（`BuzzerRegister` 无调用者）全都不生效。赛场上表现为「明明写了保护还是失控」，极难现场定位。
2. **配置静默失效** —— PID `FF_Gain`、Shift 加速、死亡重初始化、中场巡航、6 个 MI 通信 API：代码在、参数在，但不产生任何作用，且无任何告警（`-w` 关掉了全部编译警告）。
3. **无看门狗 + 大量 `while(1)`/`configASSERT` 死循环**，把可恢复错误一律变成「整机静默停机、电机保持最后指令」。
4. **单位混用**（度 / 弧度 / rad/s）贯穿云台与算法层，是「软件限位与机械限位不一致」那类问题的温床。

**推进顺序建议**: 先做 P0 的 5 条（都是局部改动，预计半天内），再做 P2 第 1 条打开编译告警并修掉新报出的问题，最后动 P1 中会改变控制手感的第 8、14 条（必须在实车重新整定）。

---

## 已实施的修复（Reborn 分支，2026-09-10）

按 P0 清单完成 4 个提交（每项可单独 revert）：

| 提交 | 内容 | 对应发现 |
|---|---|---|
| `286389b` | 蜂鸣器 `BuzzerTask` 判空；`BuzzerRegister` 上限由 `>` 改 `>=`；malloc 判空 | B1（原判「致命」，我复核为「不崩但功能全废」） |
| `2df2906` | BMI088 陀螺初始化索引下溢 → 有界重试 + 用陀螺表取错误码 + `return error`；`INS_Init` 无上限重试改 3 次并打印；`InitQuaternion` NaN 防护；`Norm3d` 零模长保护 | N3、A3 低危（姿态 NaN）、M6 |
| `eefc663` | `usart_module_callback` 增加 `(instance, len)` 并透传 `Size`；视觉接收恢复 CRC16 + 长度 + NaN 校验（先入临时结构体）；裁判解析长度校验 + 去递归 + 有界循环；DBUS 长度校验；0x0104 长度常量修正 | N1、N2、N5、B9 |
| `83091aa` | shoot 未知模式不再 `while(1)`（报错 + 停机）；message_center 去死循环 + 话题名有界复制 + 发布长度保护；HardFault 记录 CFSR/HFSR/BFAR/MMFAR + 可选自动复位 | H3（部分）、M2 低危、L3/B13 |
| `0d8c3b0` | **启用独立看门狗**：`bsp/bsp_iwdg.{c,h}`（寄存器直接操作、LSI 使能带超时、调试暂停冻结）+ ins/motor/robot 三任务心跳、daemon 100Hz 判定喂狗 + 启动时打印上次复位原因 | H3（无看门狗） |

编译验证：`cmake --build build/Debug` 通过，产出 `build/Debug/Sentry.elf`。**以上均未上机验证。**

### 上机验证清单（按优先级）

1. **视觉链路（最重要）**：`VISION_RX_CRC_CHECK` 默认 1。若 RTT 出现 `[vision] CRC16 mismatch`，说明小电脑端与本工程 CRC 约定不一致 —— 先确认 PC 端协议，必要时临时把该宏置 0 只做长度校验，**不要把 0 带上场**。正常应无任何 `[vision]` 报错且自瞄跟踪正常。
2. **裁判系统**：UI 是否正常刷新、`[ref] illegal DataLength` 是否出现（正常 0 次）；建议长时间跑一次确认没有漏帧。
3. **IMU**：正常上电不应出现 `[ins] BMI088 init failed` / `UNAVAILABLE`；若出现说明 SPI/线序有问题（现在会报错并继续，姿态保持水平，不再卡死启动）。
4. **蜂鸣器**：`BuzzerRegister` 仍无调用者，所以蜂鸣器依旧不会响（空指针已消除）；要恢复报警需按等级注册实例后启用。
5. **HardFault 行为**：无调试器时故障自动 `NVIC_SystemReset()`（宏 `FAULT_AUTO_RESET` 在 `Src/stm32f4xx_it.c`，置 0 则停在 `while(1)`）。建议先用调试器跑一次确认正常路径不受影响。
6. **遥控器/发射**：确认 DBUS 正常（18 字节校验不应误伤）、拨盘各模式正常；未知模式现在只报错停机、不卡死。

### 尚未处理（下一步）

- ~~**IWDG**~~ 已实现（`0d8c3b0`）：超时典型 1.0s（reload 500，LSI 有容差，最坏约 0.68~1.9s），daemon 100Hz 在 ins/motor/robot 三任务心跳都推进后喂狗，调试器暂停时冻结计数。
  - 上机验证：① 正常跑动不应出现任何 `[iwdg]` 报错，尤其不能无故复位；② 可临时把某个任务的心跳去掉（或在该任务里 `osDelay(2000)`）验证 1s 后确实复位，并确认 RTT 打出 `last reset was caused by IWDG`；③ 断点调试时不应被复位。
  - 已知边界：初始化阶段（`RobotInit`，含 IMU 标定最坏 >1s）**不在**看门狗覆盖范围内，避免正常启动被误复位；若希望覆盖初始化，需要在初始化流程里插入喂狗点（后续可选）。
- 剩余 `while(1)`：`bsp_can` / `bsp_usart` / `dji_motor` / `bsp_iic` 的注册错误分支，改成「报错 + 返回 NULL」需要调用方一并判空，属 M3「返回值不判空」的一揽子改造。
- P1 其余项：视觉掉线清数据（H1）、电机离线回调停机（B2）、`motor_task` 的 `cnt++`（H4，改完要重新整定）、`SuperCapSend` 越界读（M1）、`DJIMotorControl` 越界写（M2）、PID `FF_Gain`（A1，修完必须重新整定）、DMA 中断优先级（B3）等。

---

## 云台跟踪调参记录（2026-09-10 实车）

| 提交 | 改动 | 实车结果 |
|---|---|---|
| `9d61bdc` | ① 修 `PIDInit`(A1) 让 `FF_Gain` 生效，yaw 角度环 FF=120、速度环 FF=0；② yaw 命令滤波 `SLOW 0.99→0.7`、门限 `10/0.8→4/0.2`；③ pitch 增量增益 `0.0005→0.0012`、误差滤波 `alpha 0.3/0.2/0.1→0.7/0.5/0.4` | **pitch 出现震荡**（yaw 未报异常） |
| `c34c58b` | ③ 完全回退到 `0.0005` / `alpha 0.3/0.2/0.1`；yaw 侧 ①② 保留 | 待确认 |

**结论与后续顺序（pitch 想更快时）**：
1. 「增量式环路的有效速度增益」与「误差估计滞后」是两个耦合自由度：同时提高增益、又减小滤波滞后，会让相位裕度不足 → 震荡。
2. 正确顺序是**先加阻尼、再提高增益**：先把 MI 电机在线 `kd` 从 1.0 提到 1.5~2.0（`application/gimbal/gimbal.c` 的 pitch `.angle_PID.Kd`，经 `MI_motor_LocationControl` 传给电机内置环），确认不震荡后再**小步**提高增量增益 `0.0005 → 0.0007 → 0.0010`，每步实车确认。
3. 建议配合阶跃采样（RAM 环形缓冲 + OpenOCD `dump_image`）观察震荡频率/幅值与跟随误差，避免纯靠手感调参。
4. 小电脑数据为 **100 Hz**（10ms/帧）：与控制环(200Hz)不同步，但 yaw 的陀螺速度前馈在 1kHz 电机环里计算，不受帧率限制；进一步提升上限取决于 PC 端端到端延迟，可考虑 ① 让 PC 提到 200Hz ② 误差变化率超前补偿（`T_lead` 默认 0，符号需实车 A/B）。

**pitch 软限位问题（2026-09-10 已解决，`fdafc00`）**：
1. 实测：IMU **不在 pitch 轴上**（关节转 29.6° 时 IMU pitch 只动 0.85°）→ 不能用重力参考做绝对限位；
2. 实测：MI 电机**每次上电后上报角度的绝对基准都不同**（用户反馈读数不一致）→ 固定绝对限位不可行；
3. 最终方案：**上电回零(homing)** —— 首次进入云台模式时以小 kp、0.2 rad/s 缓慢推进位置目标找机械限位，位置误差连续 0.2s 超阈值即判定顶住，在该处发「设置机械零位」（设零后静默 10 拍，因为 MI 的发送缓冲是共享的，见 N6），随后正常控制；
4. 软限位改为「回零坐标系」（零点 = 回零顶住的那个限位，向另一侧让 0.86/0.91 rad，两端留 2.7° 余量）：`PITCH_MAX_ANGLE -0.02`、`PITCH_MIN_ANGLE -0.86`。该定义对「回零顶到哪一端」是方向无关的，因此安全；
5. 实车结果：每次上电炮管先自行走到限位（当前实测是朝上）、随后摇杆控制正常，行程与手感在断电重启/烧录/换电池后完全一致。

