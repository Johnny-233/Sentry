# 对方项目小用心 / 小巧思（可迁清单）

> 从 `infantry_cpp` 的 `project/`、`tools/`、实现文档里摘出。  
> **学机制，别照搬标定参数。**  
> 优先级：P0 建议先落地到你车；P1 有空再迁；P2 了解即可。

## P0 — 性价比高，建议优先学/留

| # | 巧思 | 在哪 | 一句话 | 迁到你车时 |
|---|------|------|--------|------------|
| 1 | **板外 IMU 椭球校准** | `tools/accel_calibration/` | 静止多姿态采加速度 → Python 拟合 `offset`+`M` → 固件只填数 | 整车 AHRS 前做一次；换芯片重做 |
| 2 | **子系统单独 Init** | `application/config.h` 的 `*_INIT_DEBUG` | 只起云台/底盘/射击，避免联调一锅端 | 直接用；上电前确认电机没电或限流 |
| 3 | **daemon 离线看门狗** | `modules/daemon/` | 超时未喂狗 → 离线回调（停电机/清指令） | 遥控、裁判、电机都挂上 |
| 4 | **串口 DMA Circular + Idle** | `modules/serial/` + README | 环缓冲少拆建 DMA；`full_daemon` 处理半截帧 | CubeMX 串口用 DMA Circular + Idle |
| 5 | **电机速度低通，raw 另存** | `modules/motor/dji_motor/` | 环路用滤波速度，调试仍看原始值 | 速度环/功率预测用滤波值 |

## P1 — 控制与结构亮点

| # | 巧思 | 在哪 | 一句话 | 迁到你车时 |
|---|------|------|--------|------------|
| 6 | **底盘 motion / velocity 拆分** | `application/chassis/chassis_core/` | 运动学只管 ω*；速度环+功率限幅+发电流另一层 | 保持拆分，别又糊回一个大文件 |
| 7 | **功率限幅后的积分反算** | chassis README / `design-rationale` | PID 自带抗饱和看不见功率二次限流，用 `1/Ti≈ki/kp` 回灌积分 | 有电容/功率限制时必做 |
| 8 | **速度/力矩前馈接口** | `DJIMotor` + gimbal `config.h` | 设定值前馈 + 输出前馈；pitch 重力补偿进配置 | 零位/FF 按你车重标 |
| 9 | **cmd 与 motion 解耦** | 对方分支：cmd 只写 `chassis_cmd` | 指令中枢不直接拧运动学内部状态 | 跟对方现分支思路对齐即可 |
| 10 | **C++ 禁堆 / 禁异常 / init 三步** | `.claude/docs/cpp/`（源仓） | 静态实例 + `Config` + `init`，无 malloc | 与本仓 C `Register` 二选一，勿混风格 |

## P2 — 文档与工程习惯

| # | 巧思 | 在哪 | 一句话 |
|---|------|------|--------|
| 11 | impl **五件套** | 源仓 `.claude/docs/impl/<模块>/` | overview / data-flow / interface / pipeline / design-rationale |
| 12 | **硬件事实单文件** | 源仓 `conventions/hardware.md` | 电机表+外设表唯一真相源 |
| 13 | **tools 与固件分离** | `tools/` | 拟合/仿真不上板，避免固件里塞 numpy 逻辑 |
| 14 | **crc8_gen.py** | `tools/crc8_gen.py` | 协议表生成，少手算 |

## 明确先别搬 / 先别当真的

- 对方 **yaw/pitch 编码器零位、PID、功率系数** → 只当样例  
- 源仓 `.claude/skills` → 你已有 `skills/` + `AI_GUARDRAILS.md`，不要覆盖  
- 射击里 **裁判热量检查** → 对方自己还是 TODO，不是成品巧思  

## 建议学习顺序（动手实验）

1. 跑通 `tools/accel_calibration/README.md`（不必先整车编译）  
2. 读 `project/modules/serial/README.md` + `daemon/README.md`  
3. 读 `project/application/chassis/README.md`（功率 + 反算那两节）  
4. 打开 `config.h`，看清 `*_INIT_DEBUG` 怎么单独起模块  
5. 阶段 3 配好板后：先电机/云台单模块，再 AHRS，再整车  

## 和阶段 3 的关系

本文件不改 `.ioc`、不生成 `Core`。  
外设对齐与 `Robot_Init` 挂载见 `MIGRATION_NOTES.md`；**你再确认后才做阶段 3**。
