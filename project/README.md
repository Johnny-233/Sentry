# project/ —— 全部用户代码（只有这一套）

- `bsp/`          板级支持包：外设抽象（gpio / can / usart / pwm / spi / tim / adc / i2c / dwt / log）
- `modules/`      可复用模块：对方的 algorithm / motor(dji,servo) / ahrs / remote / referee / serial / daemon / rgb_led / power …
                 ＋ 本仓自己补的 `motor/mi_motor`（MI 电机）、`master_machine`（seasky 视觉协议）、`super_cap`
- `application/`  **本仓整车控制逻辑的 C++ 移植版**：`robot`（入口+任务）/ `chassis` / `gimbal` / `shoot` /
                 `cmd`（robot_cmd）/ `gimbal_algorithm` / `ui`（裁判 UI），另加 `robot_def.h`、`pid_port.h`

分工原则：**底层 bsp 与通用模块用对方迁入的 C++ 实现；具体外设模块（MI 电机、超电、视觉协议）和整车逻辑用本仓的**。

> 迁移说明见仓库根目录 `MIGRATION_NOTES.md`（含阶段 3/4 的全部差异、遗留 TODO、以及被删掉的旧 C 代码怎么从 git 取回）；
> 移植规约见 `application/PORT_MAPPING.md`。
> **未上电验证：标定参数仍是旧值，重标零位/核对 PID 之前不要当本车成品烧录跑。**
