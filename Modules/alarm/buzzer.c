#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit()
{
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config == NULL || config->alarm_level >= BUZZER_DEVICE_CNT) // 等级非法或超上限(合法 0~BUZZER_DEVICE_CNT-1, 原来是 > 会放过越界值)
    {
        LOGERROR("[buzzer] invalid alarm_level [%d], register failed", config == NULL ? -1 : (int)config->alarm_level);
        return NULL;
    }
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    if (buzzer_temp == NULL)
    {
        LOGERROR("[buzzer] malloc failed, register failed");
        return NULL;
    }
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    buzzer->alarm_state = state;
}

void BuzzerTask()
{
    BuzzzerInstance *buzz;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        if (buzz == NULL) // 该等级未注册实例: 直接跳过(修复空指针解引用)
            continue;
        if (buzz->alarm_level > ALARM_LEVEL_LOW)
        {
            continue;
        }
        if (buzz->alarm_state == ALARM_OFF)
        {
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave)
            {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1 / SiFreq);
                break;
            default:
                break;
            }
            break;
        }
    }
}
