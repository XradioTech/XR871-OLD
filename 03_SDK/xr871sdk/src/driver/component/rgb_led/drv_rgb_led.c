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


MaxBrightness Rgb_LedInit(uint32_t hz, PWM_CH_ID ch, RGB_LED_TYPE type)
{
	HAL_Status ret = HAL_ERROR;
	PWM_ClkParam clk_cfg;

	clk_cfg.clk = PWM_CLK_HOSC;
	clk_cfg.div =  PWM_SRC_CLK_DIV_1;

	ret = HAL_PWM_GroupClkCfg(ch / 2, &clk_cfg);
	if (ret != HAL_OK)
		COMPONENT_WARN("group clk cfg error\n");

	PWM_ChInitParam ch_cfg;
	ch_cfg.hz = hz;
	ch_cfg.mode = PWM_CYCLE_MODE;

	if (type == RGB_HIGH_LEVEL_VALID)
		ch_cfg.polarity = PWM_HIGHLEVE;
	else
		ch_cfg.polarity = PWM_LOWLEVE;

	int cycle = HAL_PWM_ChInit(ch, &ch_cfg);
	if (cycle == -1)
		COMPONENT_WARN("channel init error\n");

	COMPONENT_TRACK("end\n");

	return cycle;
}


int SetLedBrightness(PWM_CH_ID ch, uint32_t brightness)
{
	HAL_Status ret = HAL_PWM_ChSetDutyRatio(ch, brightness);
	if (ret != HAL_OK)
		return -1;

	return 0;
}

static Rgb_Led_Info Rgb_Reg;
MaxBrightness Drv_Rgb_Led_Cfg(Rgb_Led_Info *led_info)
{
	Rgb_Reg = *led_info;
	Rgb_LedInit(led_info->ledFrequency, led_info->r_Led, led_info->type);
	Rgb_LedInit(led_info->ledFrequency, led_info->g_Led, led_info->type);
	Rgb_LedInit(led_info->ledFrequency, led_info->b_Led, led_info->type);
	uint32_t birghtness = Rgb_LedInit(led_info->ledFrequency, led_info->b_Led, led_info->type);
	COMPONENT_TRACK("end\n");
	return birghtness;
}

void DRV_Rgb_Led_DeInit()
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	HAL_PWM_ChDeinit(led_info->r_Led);
	HAL_PWM_ChDeinit(led_info->g_Led);
	HAL_PWM_ChDeinit(led_info->b_Led);
}

void Drv_Rgb_LedEnable()
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	HAL_PWM_EnableCh(led_info->r_Led, PWM_CYCLE_MODE, 1);
	HAL_PWM_EnableCh(led_info->g_Led, PWM_CYCLE_MODE, 1);
	HAL_PWM_EnableCh(led_info->b_Led, PWM_CYCLE_MODE, 1);
}
void Drv_Rgb_LedDisable()
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	HAL_PWM_EnableCh(led_info->r_Led, PWM_CYCLE_MODE, 0);
	HAL_PWM_EnableCh(led_info->g_Led, PWM_CYCLE_MODE, 0);
	HAL_PWM_EnableCh(led_info->b_Led, PWM_CYCLE_MODE, 0);
}

void Drv_Rgb_Led_Set(Rgb_Led_Value *set)
{
	Rgb_Led_Info *led_info = &Rgb_Reg;
	SetLedBrightness(led_info->r_Led, set->r_Value);
	SetLedBrightness(led_info->g_Led, set->g_Value);
	SetLedBrightness(led_info->b_Led, set->b_Value);
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
	led_info.b_Led = PWM_GROUP0_CH0;
	led_info.g_Led = PWM_GROUP0_CH1;
	led_info.r_Led = PWM_GROUP1_CH2;
	led_info.ledFrequency = 500;
	led_info.type = RGB_HIGH_LEVEL_VALID;

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

