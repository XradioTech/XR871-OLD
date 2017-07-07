#ifndef _SSD1306_H_
#define _SSD1306_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "driver/component/component_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_spi.h"


#define SSD1306_SETCONTRAST 			0x81
#define SSD1306_DISPLAYALLON_RESUME 	0xA4
#define SSD1306_DISPLAYALLON 			0xA5
#define SSD1306_NORMALDISPLAY 			0xA6
#define SSD1306_INVERTDISPLAY 			0xA7
#define SSD1306_DISPLAYOFF 				0xAE
#define SSD1306_DISPLAYON 				0xAF

#define SSD1306_SETDISPLAYOFFSET 		0xD3
#define SSD1306_SETCOMPINS 				0xDA

#define SSD1306_SETVCOMDETECT 			0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 		0xD5
#define SSD1306_SETPRECHARGE 			0xD9
#define SSD1306_ENABLE_CHARGE_PUMP		0x8D

#define SSD1306_SETMULTIPLEX 			0xA8
#define SSD1306_SETSTARTLINE 			0x40

#define SSD1306_MEMORYMODE 				0x20
#define SSD1306_HV_COLUMN_ADDRESS		0x21
#define SSD1306_HV_PAGE_ADDRESS			0x22
#define SSD1306_PAM_PAGE_START			0xB0

#define SSD1306_COMSCANINC 				0xC0
#define SSD1306_COMSCANDEC 				0xC8

#define SSD1306_SEGREMAP 				0xA0

#define SSD1306_CHARGEPUMP 				0x8D

#define SSD1306_EXTERNALVCC 			0x1
#define SSD1306_SWITCHCAPVCC 			0x2

// Scrolling #defines
#define SSD1306_SCROLL_ACTIVATE 						0x2F
#define SSD1306_SCROLL_DEACTIVATE 						0x2E
#define SSD1306_SCROLL_SET_VERTICAL_SCROLL_AREA 		0xA3
#define SSD1306_SCROLL_HORIZONTAL_RIGHT 				0x26
#define SSD1306_SCROLL_HORIZONTAL_LEFT 					0x27
#define SSD1306_SCROLL_VERTICAL_AND_HORIZONTAL_RIGHT 	0x29
#define SSD1306_SCROLL_VERTICAL_AND_HORIZONTAL_LEFT		0x2A

typedef enum {
	SSD1306_CMD,
	SSD1306_DATA,
}SSD1306_WR_MODE;

typedef struct {
	HAL_Status (*SSD1306_Write)(uint8_t data, SSD1306_WR_MODE mode);
	SPI_Port SSD1306_SPI_ID;
	SPI_TCTRL_SS_Sel SSD1306_SPI_CS;
	uint32_t SSD1306_SPI_MCLK;
	GPIO_Port SSD1306_dsPort;
	GPIO_Pin SSD1306_dsPin;
	GPIO_Port SSD1306_reset_Port;
	GPIO_Pin SSD1306_reset_Pin;
}SSD1306_t;


HAL_Status SSD1306_SPI_Init(SSD1306_t *SSD1306config);
HAL_Status SSD1306_SPI_DeInit();
void SSD1306_Init();
void SSD1306_set_brightness(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif /* _SSD1306_H */

