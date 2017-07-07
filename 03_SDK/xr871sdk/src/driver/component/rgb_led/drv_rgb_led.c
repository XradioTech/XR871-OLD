#include <string.h>
#include <stdio.h>
#include "kernel/os/os.h"
#include "driver/component/rgb_led/drv_rgb_led.h"

#define RGB_LED_DBG 1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_RGB_LED_DBG(fmt, arg...)	\
			LOG(RGB_LED_DBG, "[RGB_LED] "fmt, ##arg)


HAL_Status board_pwm_cfg(uint32_t id, HAL_BoardReq req, void *arg);


void Rgb_LedIOInit(Rgb_Led_Info *led_Info)
{
	PWM_Init_Param pwmParam;
	pwmParam.boardCfg = board_pwm_cfg;
	pwmParam.ch = led_Info->R_Led;
	HAL_PWM_IO_Init(&pwmParam);
	pwmParam.ch = led_Info->G_Led;
	HAL_PWM_IO_Init(&pwmParam);
	pwmParam.ch = led_Info->B_Led;
	HAL_PWM_IO_Init(&pwmParam);
}

MaxBrightness Rgb_LedInit(uint32_t hz, PWM_CHID ch, RGB_LED_TYPE type)
{

	PWM_SrcClk srcClkSet;
	PWM_Output_Init PwmOutputSet;
	HAL_Status  sta;

	srcClkSet.chGroup= ch / 2;
	srcClkSet.srcClkDiv= PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk= PWM_CLK_HOSC;


	PwmOutputSet.ch= ch;
	if(type == RGB_HIGH_LEVEL_VALID)
		PwmOutputSet.polarity = PWM_HIGHLEVE;
	else
		PwmOutputSet.polarity = PWM_LOWLEVE;
	PwmOutputSet.hz = hz;
	PwmOutputSet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);

	sta = HAL_PWM_CycleModeInit(&PwmOutputSet);
	if (sta != HAL_OK) {
			COMPONENT_WARN("PWM init error %d\n", sta);
			return 0;
		}

	return HAL_PWM_GetEnterCycleValue(ch);

}


int SetLedBrightness(PWM_CHID ch, uint32_t brightness)
{
	return HAL_PWM_SetDutyRatio(ch, brightness);
}

static Rgb_Led_Info Rgb_Reg;
MaxBrightness Drv_Rgb_Led_Cfg(Rgb_Led_Info *led_info)
{
	Rgb_Reg = *led_info;
	Rgb_LedIOInit(led_info);
	Rgb_LedInit(led_info->LedFrequency, led_info->R_Led, led_info->Type);
	Rgb_LedInit(led_info->LedFrequency, led_info->G_Led, led_info->Type);
	uint32_t birghtness = Rgb_LedInit(led_info->LedFrequency, led_info->B_Led, led_info->Type);
	COMPONENT_TRACK("end\n");
	return birghtness;
}

void DRV_Rgb_Led_DeInit()
{
	PWM_Init_Param pwm_Param;
	pwm_Param.boardCfg = board_pwm_cfg;
	pwm_Param.ch = Rgb_Reg.R_Led;
	HAL_PWM_DeInit(&pwm_Param);
	pwm_Param.ch = Rgb_Reg.G_Led;
	HAL_PWM_DeInit(&pwm_Param);
	pwm_Param.ch = Rgb_Reg.B_Led;
	HAL_PWM_DeInit(&pwm_Param);
}

void Drv_Rgb_LedEnable()
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	HAL_PWM_OutModeEnableCh(led_info->R_Led);
	HAL_PWM_OutModeEnableCh(led_info->G_Led);
	HAL_PWM_OutModeEnableCh(led_info->B_Led);
}
void Drv_Rgb_LedDisable()
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	HAL_PWM_OutModeDisableCh(led_info->R_Led);
	HAL_PWM_OutModeDisableCh(led_info->G_Led);
	HAL_PWM_OutModeDisableCh(led_info->B_Led);
}

void Drv_Rgb_Led_Set(Rgb_Led_Value *set)
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	SetLedBrightness(led_info->R_Led, set->r_Value);
	SetLedBrightness(led_info->G_Led, set->g_Value);
	SetLedBrightness(led_info->B_Led, set->b_Value);
}


static Rgb_Led_Value rgb_colour[7] = { {1, 0, 0}, //red
								{0, 1, 0}, //green
								{0, 0, 1}, //blue
								{1, 1, 0}, //yellow
								{0, 1, 1}, //turquoise
								{1, 0, 1}, //azaleine
								{1, 1, 1}, //white
							};

void RGB_LedTest()
{
	Rgb_Led_Info led_info;
	led_info.B_Led = PWM_GROUP0_CH0;
	led_info.G_Led = PWM_GROUP0_CH1;
	led_info.R_Led = PWM_GROUP1_CH2;
	led_info.LedFrequency = 500;
	led_info.Type = RGB_HIGH_LEVEL_VALID;

	MaxBrightness maxBrightness = Drv_Rgb_Led_Cfg(&led_info);
	Drv_Rgb_LedEnable();
	uint32_t brightness = maxBrightness / 5;
	Rgb_Led_Value colour;
	int i = 0;
	for (i = 0; i < 7; i++) {
		colour.r_Value = brightness * rgb_colour[i].r_Value;
		colour.g_Value = brightness * rgb_colour[i].g_Value;
		colour.b_Value = brightness * rgb_colour[i].b_Value;
		Drv_Rgb_Led_Set(&colour);
		OS_MSleep(500);
	}

}

