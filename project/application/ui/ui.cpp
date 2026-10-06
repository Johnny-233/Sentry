/**
 * @file    ui.cpp
 * @brief   裁判系统 UI（司机端界面）绘制
 * @note    旧→新绘制原语的对应关系见 ui.h;
 *          本文件里所有数值 / 坐标 / 颜色 / 层号都与旧 referee_task.c 逐字相同, 未做任何"顺手优化"。
 */

#include "ui.h"
#include "referee.h"
#include "cmsis_os.h"

/* ============================================================================
 * 内部变量
 * ==========================================================================*/

/* UI 绘制与交互所需的机器人状态数据(定义在本文件, robot_cmd.cpp 直接填这个实例) */
Referee_Interactive_info_t ui_data;

/* 裁判系统反馈数据指针。Referee_Init(&huart6) 已在应用层初始化, 这里取同一个静态实例。 */
static referee_info_t *referee_recv_info = nullptr;

/* 旧的两步式静态绘图缓冲(UI_shoot_line / UI_State_Cir / UI_State_Rec / UI_State_sta / UI_State_dyn)
 * 在这里没有对应物:
 *   Referee_UIDraw()/Referee_UIChar() 内部自己组包并**立即发送**, 一次调用 = 旧的"XxxDraw +
 *   UIGraphRefresh(cnt=1)"; 而 Referee_UIRefresh()/Referee_UICharRefresh() 需要的
 *   interaction_figure_t 指针/45 字节缓冲是 referee.cpp 的私有类型, 应用层拿不到, 故不再使用。
 *   绘图所需的"数据"只剩下面这张坐标表, 照搬旧值:
 *   TODO(移植): 旧 UILineDraw(...)×5 + UIGraphRefresh(id, 5, ...) 现在变成 5 次 Referee_UIDraw()，
 *               即"5 图形打一包"(旧 UI_Data_ID_Draw5 / 新 UI_Draw5) 变成 5 个单图形包，选手端显示结果相同。
 *   TODO(移植): 旧 static Graph_Data_t UI_Energy[3]（电容能量条）在旧 referee_task.c 里声明后从未使用，未移植。 */
static uint32_t shoot_line_location[10] = {960, 400, 540, 440, 420, 400, 380, 360, 340, 320};

/* 新原语的图形名是 uint8_t*(内部只 memcpy 前 3 字节, 不写回), 这里统一去掉 const, 免得每个调用点写强转 */
static uint8_t *UI_Name(const char *name)
{
    return (uint8_t *)name;
}

/* ============================================================================
 * 内部函数声明
 * ==========================================================================*/

static void MyUIRefresh(Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

/* 判断各种ID, 选择客户端ID */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   (旧 #define Robot_Red 0 / #define Robot_Blue 1)
    referee_recv_info->id.robot_color = referee_recv_info->robot_status.robot_id > 7 ? 1 : 0;
    referee_recv_info->id.robot_id = referee_recv_info->robot_status.robot_id;
    referee_recv_info->id.client_id = 0x0100 + referee_recv_info->id.robot_id; // 计算客户端ID
    referee_recv_info->id.receiver_id = 0;
    // TODO(移植): 新框架在 Referee_ParseFrame() 收到 0x0201(robot_status) 时已经把
    //             id.robot_id/robot_color/client_id 算好了（公式与旧 DeterminRobotID 相同），
    //             这里照搬旧逻辑重算一遍，值完全一致，receiver_id 仍按旧代码清 0。
}

void MyUIInit()
{
    referee_recv_info = Referee_GetData();

    if (!referee_recv_info->init_flag)
        osThreadTerminate(osThreadGetId()); // TODO(移植): 旧为 vTaskDelete(NULL)。本仓用 cmsis_os v1 兼容层，没有 vTaskDelete；osThreadTerminate(osThreadGetId()) 结束当前 UI 任务，语义相同（旧代码此处没有 return，照搬）
    while (referee_recv_info->robot_status.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                  // 确定ui要发送到的目标客户端
    Referee_UIDelete(UI_Delete_All, 0); // 清空UI(旧 UI_Data_Del_ALL == 新 UI_Delete_All == 2)

    // 绘制发射基准线; 新原语 Referee_UIDraw 参数序: (图形名, 操作, 图形类型, 图层, 颜色, 线宽, 起点x, 起点y, 终点x, 终点y)
    Referee_UIDraw(UI_Name("sl0"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_White,
                   (uint32_t)2, (uint32_t)shoot_line_location[0], (uint32_t)540, (uint32_t)shoot_line_location[0], (uint32_t)320);
    Referee_UIDraw(UI_Name("sl1"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)930, (uint32_t)shoot_line_location[1], (uint32_t)990, (uint32_t)shoot_line_location[1]);
    Referee_UIDraw(UI_Name("sl2"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_White,
                   (uint32_t)2, (uint32_t)860, (uint32_t)shoot_line_location[2], (uint32_t)1060, (uint32_t)shoot_line_location[2]);
    Referee_UIDraw(UI_Name("sl3"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)900, (uint32_t)shoot_line_location[3], (uint32_t)1020, (uint32_t)shoot_line_location[3]);
    Referee_UIDraw(UI_Name("sl4"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)930, (uint32_t)shoot_line_location[4], (uint32_t)990, (uint32_t)shoot_line_location[4]);

    Referee_UIDraw(UI_Name("sl5"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)930, (uint32_t)shoot_line_location[5], (uint32_t)990, (uint32_t)shoot_line_location[5]);
    Referee_UIDraw(UI_Name("sl6"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)930, (uint32_t)shoot_line_location[6], (uint32_t)990, (uint32_t)shoot_line_location[6]);
    Referee_UIDraw(UI_Name("sl7"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)950, (uint32_t)shoot_line_location[7], (uint32_t)970, (uint32_t)shoot_line_location[7]);
    Referee_UIDraw(UI_Name("sl8"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)950, (uint32_t)shoot_line_location[8], (uint32_t)970, (uint32_t)shoot_line_location[8]);
    Referee_UIDraw(UI_Name("sl9"), UI_Graph_Add, UI_Graph_Line, 7, UI_Color_Yellow,
                   (uint32_t)2, (uint32_t)950, (uint32_t)shoot_line_location[9], (uint32_t)970, (uint32_t)shoot_line_location[9]);

    // 状态圈; 新原语圆的参数序: (线宽, 圆心x, 圆心y, 半径)
    Referee_UIDraw(UI_Name("sa0"), UI_Graph_Add, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)740, (uint32_t)10);
    Referee_UIDraw(UI_Name("sa1"), UI_Graph_Add, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)690, (uint32_t)10);
    Referee_UIDraw(UI_Name("sa2"), UI_Graph_Add, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)640, (uint32_t)10);
    Referee_UIDraw(UI_Name("sa3"), UI_Graph_Add, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)590, (uint32_t)10);
    Referee_UIDraw(UI_Name("sa4"), UI_Graph_Add, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)540, (uint32_t)10);

    // 绘制车辆状态标志指示; 新原语 Referee_UIChar 参数序: (图形名, 操作, 图层, 颜色, 字号, 线宽, x, y, 格式串), 内部组包后立即发送
    Referee_UIChar(UI_Name("ss0"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 150, 750, "chassis:");
    Referee_UIChar(UI_Name("ss1"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 150, 700, "gimbal:");
    Referee_UIChar(UI_Name("ss2"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 150, 650, "shoot:");
    Referee_UIChar(UI_Name("ss3"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 150, 600, "loader:");
    Referee_UIChar(UI_Name("ss4"), UI_Graph_Add, 6, UI_Color_White, 15, 2, 150, 550, "autoaim:");

    Referee_UIChar(UI_Name("ss5"), UI_Graph_Add, 8, UI_Color_Yellow, 7, 2, 1070, 440, "2m");
    Referee_UIChar(UI_Name("ss6"), UI_Graph_Add, 8, UI_Color_Yellow, 7, 2, 1070, 400, "3m");
    Referee_UIChar(UI_Name("ss7"), UI_Graph_Add, 8, UI_Color_Yellow, 7, 2, 1070, 320, "4m");

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    Referee_UIChar(UI_Name("sd0"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 270, 750, "off      ");
    Referee_UIChar(UI_Name("sd1"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 270, 700, "off      ");
    Referee_UIChar(UI_Name("sd2"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 270, 650, "off      ");
    Referee_UIChar(UI_Name("sd3"), UI_Graph_Add, 8, UI_Color_White, 15, 2, 270, 600, "off      ");
    Referee_UIChar(UI_Name("sd4"), UI_Graph_Add, 6, UI_Color_White, 15, 2, 270, 550, "off      ");

    Referee_UIDraw(UI_Name("sr0"), UI_Graph_Add, UI_Graph_Rectangle, 6, UI_Color_White, (uint32_t)3, (uint32_t)600, (uint32_t)300, (uint32_t)1320, (uint32_t)800);
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 60)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        // TODO(移植): 旧代码 case 0 缺 break，会直接掉进 case 1（最终生效的是 case 1 的值），照搬不改
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->loader_mode = LOAD_BURSTFIRE;
        _Interactive_data->autoaim_mode = AUTO_OFF;
    }
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->loader_mode = LOAD_1_BULLET;
        _Interactive_data->autoaim_mode = AUTO_ON;

        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->loader_mode = LOAD_STOP;
        _Interactive_data->autoaim_mode = FIND_Enermy;

        break;
    }
    case 3:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->loader_mode = LOAD_REVERSE;
        _Interactive_data->autoaim_mode = FIND_Enermy;

        break;
    }
    default:
        break;
    }
    // TODO(移植): 旧 referee_task.c 里该函数定义后从未被调用（测试用），照搬保留。
    //             编译选项带 -w，未使用静态函数不报错；要用时在 UITask() 里调用即可。
}

/* UI 刷新(模式切换后重画对应的字符/图形)。
 * 旧签名要传 referee_info_t*, 新原语自己取 client_id 组包, 故去掉第一个参数。 */
static void MyUIRefresh(Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
            case CHASSIS_ZERO_FORCE:
                Referee_UIChar(UI_Name("sd0"), UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 750, "off      ");
                Referee_UIDraw(UI_Name("sa0"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)740, (uint32_t)10);
                break;
            case CHASSIS_ROTATE:
                Referee_UIChar(UI_Name("sd0"), UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 750, "rotate   ");
                Referee_UIDraw(UI_Name("sa0"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Purplish_red, (uint32_t)5, (uint32_t)130, (uint32_t)740, (uint32_t)10);
                // 此处注意字数对齐问题，字数相同才能覆盖掉
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                Referee_UIChar(UI_Name("sd0"), UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 750, "follow   ");
                Referee_UIDraw(UI_Name("sa0"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Green, (uint32_t)5, (uint32_t)130, (uint32_t)740, (uint32_t)10);
                break;
        }
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
            case GIMBAL_ZERO_FORCE:
            {
                Referee_UIChar(UI_Name("sd1"), UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 700, "off      ");
                Referee_UIDraw(UI_Name("sa1"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)690, (uint32_t)10);

                break;
            }
            case GIMBAL_GYRO_MODE:
            {
                Referee_UIChar(UI_Name("sd1"), UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 700, "normal   ");
                Referee_UIDraw(UI_Name("sa1"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Green, (uint32_t)5, (uint32_t)130, (uint32_t)690, (uint32_t)10);

                break;
            }
        }
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        switch (_Interactive_data->shoot_mode)
        {
            case SHOOT_OFF:
            {
                Referee_UIChar(UI_Name("sd2"), UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 650, "off      ");
                Referee_UIDraw(UI_Name("sa2"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)640, (uint32_t)10);

                break;
            }
            case SHOOT_ON:
            {
                Referee_UIChar(UI_Name("sd2"), UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 650, "normal   ");
                Referee_UIDraw(UI_Name("sa2"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Green, (uint32_t)5, (uint32_t)130, (uint32_t)640, (uint32_t)10);

                break;
            }
        }
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    //loader
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1)
    {
        switch (_Interactive_data->loader_mode)
        {
            case LOAD_1_BULLET:
            {
                Referee_UIChar(UI_Name("sd3"), UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 600, "normal   ");
                Referee_UIDraw(UI_Name("sa3"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Green, (uint32_t)5, (uint32_t)130, (uint32_t)595, (uint32_t)10);
                break;
            }
            case LOAD_BURSTFIRE:
            {
                Referee_UIChar(UI_Name("sd3"), UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 600, "angry    ");
                Referee_UIDraw(UI_Name("sa3"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Purplish_red, (uint32_t)5, (uint32_t)130, (uint32_t)595, (uint32_t)10);
                break;
            }
            case LOAD_REVERSE:
            {
                Referee_UIChar(UI_Name("sd3"), UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 600, "reverse  ");
                Referee_UIDraw(UI_Name("sa3"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_Main, (uint32_t)5, (uint32_t)130, (uint32_t)595, (uint32_t)10);

                break;
            }
            case LOAD_STOP:
            {
                Referee_UIChar(UI_Name("sd3"), UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 600, "off      ");
                Referee_UIDraw(UI_Name("sa3"), UI_Graph_Change, UI_Graph_Circle, 9, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)595, (uint32_t)10);
                break;
            }
        }
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 0;
    }

    if (_Interactive_data->Referee_Interactive_Flag.aim_flag == 1)
    {
        switch (_Interactive_data->autoaim_mode)
        {
            case AUTO_OFF:
            {
                Referee_UIChar(UI_Name("sd4"), UI_Graph_Change, 6, UI_Color_White, 15, 2, 270, 550, "off      ");
                Referee_UIDraw(UI_Name("sa4"), UI_Graph_Change, UI_Graph_Circle, 6, UI_Color_White, (uint32_t)5, (uint32_t)130, (uint32_t)540, (uint32_t)10);
                Referee_UIDraw(UI_Name("sr0"), UI_Graph_Change, UI_Graph_Rectangle, 6, UI_Color_White, (uint32_t)3, (uint32_t)600, (uint32_t)300, (uint32_t)1320, (uint32_t)800);
                break;
            }
            case FIND_Enermy:
            {
                Referee_UIChar(UI_Name("sd4"), UI_Graph_Change, 6, UI_Color_Yellow, 15, 2, 270, 550, "find     ");
                Referee_UIDraw(UI_Name("sa4"), UI_Graph_Change, UI_Graph_Circle, 6, UI_Color_Yellow, (uint32_t)5, (uint32_t)130, (uint32_t)540, (uint32_t)10);
                Referee_UIDraw(UI_Name("sr0"), UI_Graph_Change, UI_Graph_Rectangle, 6, UI_Color_Yellow, (uint32_t)3, (uint32_t)600, (uint32_t)300, (uint32_t)1320, (uint32_t)800);
                break;
            }
            case AUTO_ON:
            {
                Referee_UIChar(UI_Name("sd4"), UI_Graph_Change, 6, UI_Color_Main, 15, 2, 270, 550, "on       ");
                Referee_UIDraw(UI_Name("sa4"), UI_Graph_Change, UI_Graph_Circle, 6, UI_Color_Main, (uint32_t)5, (uint32_t)130, (uint32_t)540, (uint32_t)10);
                Referee_UIDraw(UI_Name("sr0"), UI_Graph_Change, UI_Graph_Rectangle, 6, UI_Color_Main, (uint32_t)3, (uint32_t)600, (uint32_t)300, (uint32_t)1320, (uint32_t)800);
                break;
            }
        }
        _Interactive_data->Referee_Interactive_Flag.aim_flag = 0;
    }
}

void UITask()
{
    MyUIRefresh(&ui_data);
}

/* 模式切换检测: 模式变化时对相应 flag 置位 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->loader_mode != _Interactive_data->loader_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
    }

    if (_Interactive_data->autoaim_mode != _Interactive_data->autoaim_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.aim_flag = 1;
        _Interactive_data->autoaim_last_mode = _Interactive_data->autoaim_mode;
    }
}
