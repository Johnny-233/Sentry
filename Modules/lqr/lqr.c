#include "lqr.h"
#include "memory.h"
#include <math.h>


// LQR初始化
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config)
{
    memset(lqr, 0, sizeof(LQRInstance));
    lqr->K[0] = config->K[0];
    lqr->K[1] = config->K[1];
    // 初始化Q矩阵
    lqr->Q[0][0] = config->Q_pos;  // 位置权重
    lqr->Q[1][1] = config->Q_vel;  // 速度权重
    // 初始化R矩阵
    lqr->R[0] = config->R;
    // 输出限幅
    lqr->MaxOut = config->MaxOut;
}


// LQR计算
float LQRCalculate(LQRInstance *lqr, float ref_angle, float measure_angle, float measure_velocity)
{
    // 更新参考值和测量值
    lqr->Ref = ref_angle;
    lqr->Measure = measure_angle;
    lqr->Real_K[0]=sqrt(lqr->Q[0][0]/lqr->R[0])*lqr->K[0];
    lqr->Real_K[1]=sqrt(lqr->Q[1][1]/lqr->R[0])*lqr->K[1];
    // 计算误差
    lqr->Err = lqr->Ref - lqr->Measure;
    
    // LQR: u = -K * x
    lqr->Output = -(lqr->Real_K[0] * lqr->Err + lqr->Real_K[1] * measure_velocity);
    
    // 输出限幅
    if (lqr->Output > lqr->MaxOut)
    {
        lqr->Output = lqr->MaxOut;
    }
    if (lqr->Output < -lqr->MaxOut)
    {
        lqr->Output = -lqr->MaxOut;
    }
    
    return lqr->Output;
}