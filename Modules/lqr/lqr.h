/* LQR结构体 */
typedef struct
{
    float K[2];
    float Real_K[2];            
    float Q[2][2];   // 状态权重矩阵
    float R[1];      // 控制权重
    
    float MaxOut;    // 输出限幅
    
    float Ref;       // 参考值
    float Measure;   // 测量值
    float Err;       // 误差
    float Output;    // 输出
    
} LQRInstance;

// LQR初始化配置结构体
typedef struct 
{
    float K[2];
    float Q_pos;     // 位置权重
    float Q_vel;     // 速度权重
    float R;         // 控制权重
    float MaxOut;    // 输出限幅
} LQR_Init_Config_s;



// 系统状态向量 [θ_x, ω_x, θ_y, ω_y]
typedef struct 
{
    float theta_x;  // X轴角度 (rad)
    float omega_x;  // X轴角速度 (rad/s)
    float theta_y;  // Y轴角度 (rad)
    float omega_y;  // Y轴角速度 (rad/s)
} VectorRef;

typedef struct 
{
    float theta_x;  // X轴角度 (rad)
    float omega_x;  // X轴角速度 (rad/s)
    float theta_y;  // Y轴角度 (rad)
    float omega_y;  // Y轴角速度 (rad/s)
} VectorMeasure;

float LQRCalculate(LQRInstance *lqr, float ref_angle, float measure_angle, float measure_velocity);
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);
