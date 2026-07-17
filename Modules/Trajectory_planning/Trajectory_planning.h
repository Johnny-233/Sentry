#include <stdint.h>

float FvCubicPlanning(float start_angle ,float delta_angle , float t);
float FvLinePlanning(float start_angle ,float end_angle , float t);
float LinePlanning(float start_angle ,float end_angle , int time_length ,int time, uint8_t* finish_flag);
float CubicPlanning(float start_angle, float end_angle, int duration, int current_time, uint8_t* finish_flag);