

#include <stdio.h>
#include <string.h>
#include <stdlib.h>



#include "kernel/os/os.h"
#include "driver/chip/hal_i2c.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_csi.h"


#include "driver/component/csi_camera/camera_csi.h"

#include "driver/component/csi_camera/gc0308/drv_gc0308.h"
#include "gc0308_cfg.h"



#define GC0308_I2CID I2C1_ID
#define GC0308_IIC_CLK_FREQ	100000
#define GC0308_SCCB_ID 0X21  			//GC0308 ID

static OS_Semaphore_t gc0308_sem_wait;
static volatile uint32_t gc0308_image_buff_addr = 0;
static volatile uint32_t gc0308_image_data_count = 0;
static uint32_t gc0308_image_size = 0;

static Cam_PowerCtrlCfg gc0308_power;
static DMA_Channel GC0308_dma_ch_fifo_a = DMA_CHANNEL_INVALID;
static DMA_Channel GC0308_dma_ch_fifo_b = DMA_CHANNEL_INVALID;


void GC0308_config_window(unsigned int startx,unsigned int starty,unsigned int width, unsigned int height);


static void GC0308Sccb_Init()
{
    GPIO_InitParam param;
    I2C_InitParam initParam;

    param.driving = GPIO_DRIVING_LEVEL_1;
    param.mode = GPIOA_P17_F4_I2C1_SCL;
    param.pull = GPIO_PULL_UP;
    HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_17, &param);

    param.driving = GPIO_DRIVING_LEVEL_1;
    param.mode = GPIOA_P18_F4_I2C1_SDA;
    param.pull = GPIO_PULL_UP;
    HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_18, &param);

    initParam.addrMode = I2C_ADDR_MODE_7BIT;
    initParam.clockFreq = GC0308_IIC_CLK_FREQ;
    HAL_I2C_Init(GC0308_I2CID, &initParam);
}

static int GC0308Sccb_Write(uint8_t sub_addr, uint8_t data)
{
    return HAL_I2C_SCCB_Master_Transmit_IT(GC0308_I2CID, GC0308_SCCB_ID, sub_addr, &data);
}

static int GC0308Sccb_Read(uint8_t sub_addr, uint8_t *data)
{
    return HAL_I2C_SCCB_Master_Receive_IT(GC0308_I2CID, GC0308_SCCB_ID, sub_addr, data);
}

int Drv_GC0308_EnvironmentInit(void)
{
    OS_Status sta = OS_SemaphoreCreate(&gc0308_sem_wait, 0, 1);
    if (sta != OS_OK) {
        COMPONENT_WARN("gc0308 semaphore create error, %d\n", sta);
        return COMP_ERROR;
    }
    return COMP_OK;
}

static Component_Status GC0308_Init(void)
{
    uint8_t temp;
    uint16_t i = 0;


    temp = 0x5a;

    GC0308Sccb_Write(0XFE, 0x80);
    OS_MSleep(100);

    GC0308Sccb_Write(0XFE, 0x00);
    OS_MSleep(100);

    if (GC0308Sccb_Read(0x00, &temp) != 1) {
        printf("GC0308 sccb read error\n");
        return COMP_ERROR;
    } else {
        printf("GC0308 chip id = 0x%02x \n", temp);
    }

    OS_MSleep(1000);


    for (i = 0; i < sizeof(gc0308_init_reg_tbl) / sizeof(gc0308_init_reg_tbl[0]); i++) {
        if (!GC0308Sccb_Write(gc0308_init_reg_tbl[i][0], gc0308_init_reg_tbl[i][1])) {
            COMPONENT_WARN("GC0308 sccb read error\n");
            return COMP_ERROR;
        }
    }

    printf("GC0308 Init Done \r\n");

    return COMP_OK;
}

static void GC0308_Stop_Dma(void *arg)
{
    DMA_Channel *ch = (DMA_Channel *)arg;
    HAL_DMA_Stop(*ch);
}

static void GC0308_Dma_Reque(DMA_Channel *ch)
{
    *ch = HAL_DMA_Request();
    if (*ch == DMA_CHANNEL_INVALID) {
        COMPONENT_WARN("dma error\n");
        return;
    }

    DMA_ChannelInitParam param;
    param.cfg =  HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
                                            DMA_WAIT_CYCLE_1,
                                            DMA_BYTE_CNT_MODE_NORMAL,

                                            DMA_DATA_WIDTH_32BIT,
                                            DMA_BURST_LEN_4,
                                            DMA_ADDR_MODE_INC,
                                            DMA_PERIPH_SRAM,

                                            DMA_DATA_WIDTH_32BIT,
                                            DMA_BURST_LEN_4,
                                            DMA_ADDR_MODE_INC,
                                            DMA_PERIPH_SRAM);


    param.endArg = (void *)ch;
    param.endCallback = GC0308_Stop_Dma;
    param.irqType = DMA_IRQ_TYPE_END;
    HAL_DMA_Init(*ch, &param);
}

void read_fifo_a(DMA_Channel channel, uint32_t len)
{

    HAL_DMA_Start(channel, CSI_FIFO_A, (gc0308_image_buff_addr + gc0308_image_data_count), len);
    gc0308_image_data_count += len;

}

void read_fifo_b(DMA_Channel channel, uint32_t len)
{

    HAL_DMA_Start(channel, CSI_FIFO_B, (gc0308_image_buff_addr + gc0308_image_data_count), len);
    gc0308_image_data_count += len;

}

void GC0308_Irq(void *arg)
{
    uint32_t irq_sta = HAL_CSI_Interrupt_Sta();
    HAL_CSI_Interrupt_Clear();
    CSI_FIFO_Data_Len len = HAL_CSI_FIFO_Data_Len();

    if (irq_sta & CSI_FIFO_0_A_READY_IRQ)
        read_fifo_a(GC0308_dma_ch_fifo_a, len.FIFO_0_A_Data_Len);
    else if (irq_sta & CSI_FIFO_0_B_READY_IRQ)
        read_fifo_b(GC0308_dma_ch_fifo_b, len.FIFO_0_B_Data_Len);
    else if (irq_sta & CSI_FRAME_DONE_IRQ) {
        OS_Status sta = OS_SemaphoreRelease(&gc0308_sem_wait);
        if (sta != OS_OK) {
            COMPONENT_WARN("GC0308 semaphore release error, %d\n", sta);
        }

        gc0308_image_size = gc0308_image_data_count;
        gc0308_image_data_count = 0;
    }

    if (irq_sta & CSI_FIFO_0_OVERFLOW_IRQ)
        COMPONENT_WARN("fifo overflow!\n");
}

void GC0308_Csi_Init()
{
    CSI_Config csi_cfg;
    HAL_CSI_Moudle_Enalbe(CSI_DISABLE);
    csi_cfg.src_Clk.clk =  CCM_AHB_PERIPH_CLK_SRC_HFCLK;
    csi_cfg.src_Clk.divN = CCM_PERIPH_CLK_DIV_N_1;
    csi_cfg.src_Clk.divM = CCM_PERIPH_CLK_DIV_M_1;
    HAL_CSI_Config(&csi_cfg);

    CSI_Sync_Signal signal;
    //CSI_POSITIVE CSI_NEGATIVE
    signal.vsync = CSI_POSITIVE;
    signal.herf = CSI_POSITIVE;
    signal.p_Clk = CSI_POSITIVE;
    HAL_CSI_Sync_Signal_Polarity_Cfg(&signal);

    CSI_Picture_Size size;
    size.hor_len = 640;
    size.hor_start = 0;
    HAL_CSI_Set_Picture_Size(&size);

    HAL_CSI_Double_FIFO_Mode_Enable(CSI_ENABLE);
    HAL_CSI_Interrupt_Cfg(CSI_FRAME_DONE_IRQ, CSI_ENABLE);
    HAL_CSI_Interrupt_Cfg(CSI_FIFO_0_A_READY_IRQ, CSI_ENABLE);
    HAL_CSI_Interrupt_Cfg(CSI_FIFO_0_B_READY_IRQ, CSI_ENABLE);
    HAL_CSI_Interrupt_Cfg(CSI_FIFO_0_OVERFLOW_IRQ, CSI_ENABLE);


    CSI_Call_Back cb;
    cb.arg = NULL;
    cb.callBack = GC0308_Irq;
    HAL_CSI_Interrupt_Enable(&cb, CSI_ENABLE);
    HAL_CSI_Moudle_Enalbe(CSI_ENABLE);

    COMPONENT_TRACK("end\n");
}



/**
  * @brief Seclet the light mode.
  * @note This function is used to set the light mode for camera.
  *           The appropriate mode helps to improve the shooting effect.
  * @param light_mode: light mode.
  * @retval None
  */
void Drv_GC0308_Light_Mode(CAM_LIGHT_MODE light_mode)
{
    uint8_t reg13val = 0XE7, reg01val = 0, reg02val = 0;
    switch(light_mode) {
    case LIGHT_AUTO:
        reg13val = 0XE7;
        reg01val = 0;
        reg02val = 0;
        break;
    case LIGHT_SUNNY:
        reg13val = 0XE5;
        reg01val = 0X5A;
        reg02val = 0X5C;
        break;
    case LIGHT_COLUDY:
        reg13val = 0XE5;
        reg01val = 0X58;
        reg02val = 0X60;
        break;
    case LIGHT_OFFICE:
        reg13val = 0XE5;
        reg01val = 0X84;
        reg02val = 0X4c;
        break;
    case LIGHT_HOME:
        reg13val = 0XE5;
        reg01val = 0X96;
        reg02val = 0X40;
        break;
    }
    GC0308Sccb_Write(0X13, reg13val);
    GC0308Sccb_Write(0X01, reg01val);
    GC0308Sccb_Write(0X02, reg02val);
}

/**
  * @brief Set the color saturation for camera.
  * @param sat: The color saturation.
  * @retval None
  */
void Drv_GC0308_Color_Saturation(CAM_COLOR_SATURATION sat)
{
    uint8_t reg4f5054val = 0X80, reg52val = 0X22, reg53val = 0X5E;
    switch(sat) {
    case COLOR_SATURATION_0://-2
        reg4f5054val = 0X40;
        reg52val = 0X11;
        reg53val = 0X2F;
        break;
    case COLOR_SATURATION_1://-1
        reg4f5054val = 0X66;
        reg52val = 0X1B;
        reg53val = 0X4B;
        break;
    case COLOR_SATURATION_2:
        reg4f5054val = 0X80;
        reg52val = 0X22;
        reg53val = 0X5E;
        break;
    case COLOR_SATURATION_3:
        reg4f5054val = 0X99;
        reg52val = 0X28;
        reg53val = 0X71;
        break;
    case COLOR_SATURATION_4:
        reg4f5054val = 0XC0;
        reg52val = 0X33;
        reg53val = 0X8D;
        break;
    }

    GC0308Sccb_Write(0X4F, reg4f5054val);
    GC0308Sccb_Write(0X50, reg4f5054val);
    GC0308Sccb_Write(0X51, 0X00);
    GC0308Sccb_Write(0X52, reg52val);
    GC0308Sccb_Write(0X53, reg53val);
    GC0308Sccb_Write(0X54, reg4f5054val);

}

/**
  * @brief Set the sensitivity for camera.
  * @param brihgt: The brightness value.
  * @retval None
  */
void Drv_GC0308_Brightness(CAM_BRIGHTNESS bright)
{
    uint8_t reg55val = 0X00;
    switch(bright) {
    case BRIGHT_0:		//-2
        reg55val = 0XB0;
        break;
    case BRIGHT_1:
        reg55val = 0X98;
        break;
    case BRIGHT_2:
        reg55val = 0X00;
        break;
    case BRIGHT_3:
        reg55val = 0X18;
        break;
    case BRIGHT_4:
        reg55val = 0X30;
        break;
    }

    GC0308Sccb_Write(0X55,reg55val);
}

/**
  * @brief Set the contarst for camera.
  * @param contrast: The contrast value.
  * @retval None
  */
void Drv_GC0308_Contrast(CAM_CONTARST contrast)
{
    uint8_t reg56val = 0X40;
    switch(contrast) {
    case CONTARST_0:	//-2
        reg56val = 0X30;
        break;
    case CONTARST_1:
        reg56val = 0X38;
        break;
    case CONTARST_2:
        reg56val = 0X40;
        break;
    case CONTARST_3:
        reg56val = 0X50;
        break;
    case CONTARST_4:
        reg56val = 0X60;
        break;
    }
    GC0308Sccb_Write(0X56,reg56val);
}

/**
  * @brief Set the effects for camera.
  * @param eft: effects.
  * @retval None
  */
void Drv_GC0308_Special_Effects(CAM_SPECAIL_EFFECTS eft)
{
    uint8_t reg3aval = 0X04;
    uint8_t reg67val = 0XC0;
    uint8_t reg68val = 0X80;
    switch(eft) {
    case IMAGE_NOMAL:	//nomal
        reg3aval = 0X04;
        reg67val = 0XC0;
        reg68val = 0X80;
        break;
    case IMAGE_NEGATIVE:
        reg3aval = 0X24;
        reg67val = 0X80;
        reg68val = 0X80;
        break;
    case IMAGE_BLACK_WHITE:
        reg3aval = 0X14;
        reg67val = 0X80;
        reg68val = 0X80;
        break;
    case IMAGE_SLANT_RED:
        reg3aval = 0X14;
        reg67val = 0Xc0;
        reg68val = 0X80;
        break;
    case IMAGE_SLANT_GREEN:
        reg3aval = 0X14;
        reg67val = 0X40;
        reg68val = 0X40;
        break;
    case IMAGE_SLANT_BLUE:
        reg3aval = 0X14;
        reg67val = 0X80;
        reg68val = 0XC0;
        break;
    case IMAGE_VINTAGE:
        reg3aval = 0X14;
        reg67val = 0XA0;
        reg68val = 0X40;
        break;
    }

    GC0308Sccb_Write(0X3A, reg3aval);
    GC0308Sccb_Write(0X68, reg67val);
    GC0308Sccb_Write(0X67, reg68val);
}

/**
  * @brief Set the window for camera.
  * @param sx: Starting coordinates.
  * @param sy: Starting coordinates.
  * @param width: Window width.
  * @param height: Window height.
  * @retval None
  */
void Drv_GC0308_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
    uint16_t endx;
    uint16_t endy;
    uint8_t temp;

    endx = sx + width * 2;		//V*2
    endy = sy + height * 2;
    if(endy > 784)
        endy -= 784;

    GC0308Sccb_Read(0X03, &temp);
    temp &= 0XF0;
    temp |= ((endx & 0X03) << 2) | (sx & 0X03);
    GC0308Sccb_Write(0X03, temp);
    GC0308Sccb_Write(0X19, sx>>2);
    GC0308Sccb_Write(0X1A, endx>>2);
    GC0308Sccb_Read(0X32, &temp);
    temp &= 0XC0;
    temp |= ((endy & 0X07) << 3) | (sy&0X07);
    GC0308Sccb_Write(0X32, temp);
    GC0308Sccb_Write(0X17, sy >> 3);
    GC0308Sccb_Write(0X18, endy >> 3);
}

//(140,16,640,480) is good for VGA
//(272,16,320,240) is good for QVGA
/* config_GC0308_window */
void GC0308_config_window(unsigned int startx,unsigned int starty,unsigned int width, unsigned int height)
{
    unsigned int endx;
    unsigned int endy;// "v*2"±ØÐë
    unsigned char temp_reg1, temp_reg2;
    unsigned char temp=0;

    endx=(startx+width*2)%784;
    endy=(starty+height*2);// "v*2"±ØÐë

    GC0308Sccb_Read(0x32, &temp_reg2 );
    temp_reg2 &= 0xc0;

    GC0308Sccb_Read(0x03, &temp_reg1 );
    temp_reg1 &= 0xf0;

    // Horizontal
    temp = temp_reg2|((endx&0x7)<<3)|(startx&0x7);
    GC0308Sccb_Write(0x32, temp );
    temp = (startx&0x7F8)>>3;
    GC0308Sccb_Write(0x17, temp );
    temp = (endx&0x7F8)>>3;
    GC0308Sccb_Write(0x18, temp );

    // Vertical
    temp =temp_reg1|((endy&0x3)<<2)|(starty&0x3);
    GC0308Sccb_Write(0x03, temp );
    temp = starty>>2;
    GC0308Sccb_Write(0x19, temp );
    temp = endy>>2;
    GC0308Sccb_Write(0x1A, temp );
}

/**
  * @brief Set the buff to save the picture.
  * @note The buff size must be sufficient.
  * @retval None
  */
void Drv_GC0308_Set_SaveImage_Buff(uint32_t image_buff_addr)
{
    gc0308_image_data_count = 0;
    gc0308_image_buff_addr = image_buff_addr;
}

/**
  * @brief Init the io for ctrl the camera power.
  * @param cfg: The io info.
  * @retval None
  */
void Drv_GC0308_PowerInit(Cam_PowerCtrlCfg *cfg)
{
    gc0308_power = *cfg;
    GPIO_InitParam param;
    param.driving = GPIO_DRIVING_LEVEL_1;
    param.mode = GPIOx_Pn_F1_OUTPUT;
    param.pull = GPIO_PULL_NONE;

    HAL_GPIO_Init(cfg->Cam_Pwdn_Port, cfg->Cam_Pwdn_Pin, &param);
    HAL_GPIO_Init(cfg->Cam_Reset_Port, cfg->Cam_Reset_Pin, &param);
}

/**
  * @brief Ctrl the reset pin.
  * @retval None
  */
void Drv_GC0308_Reset_Pin_Ctrl(GPIO_PinState state)
{

    HAL_GPIO_WritePin(gc0308_power.Cam_Reset_Port,
                      gc0308_power.Cam_Reset_Pin, state);
}

/**
  * @brief Ctrl the pwdn pin.
  * @retval None
  */
void Drv_GC0308_Pwdn_Pin_Ctrl(GPIO_PinState state)
{
    HAL_GPIO_WritePin(gc0308_power.Cam_Pwdn_Port,
                      gc0308_power.Cam_Pwdn_Pin, state);

}

/**
  * @brief Init the GC0308 and csi interface.
  * @retval Component_Status : The driver status.
  */
Component_Status Drv_GC0308_Init()
{
    GC0308_Csi_Init();
    GC0308Sccb_Init();
    if (GC0308_Init() == -1) {
        COMPONENT_WARN("GC0308  Init error!!\n");
        return COMP_ERROR;
    }

    GC0308_Dma_Reque(&GC0308_dma_ch_fifo_a);
    if (GC0308_dma_ch_fifo_a == DMA_CHANNEL_INVALID)
        return COMP_ERROR;

    GC0308_Dma_Reque(&GC0308_dma_ch_fifo_b);
    if (GC0308_dma_ch_fifo_b == DMA_CHANNEL_INVALID)
        return COMP_ERROR;

    return COMP_OK;
    COMPONENT_TRACK("end\n");
}

/**
  * @brief Enable the capture.
  * @note: This function use to start capture picture
  * @param mode: The captue mode, still is capture
  *    one picture, video is capture images continuously.
  * @ctrl:enable or disable
  * @retval Component_Status : The driver status.
  */
Component_Status Drv_GC0308_Capture_Enable(CSI_CAPTURE_MODE mode, CSI_CTRL ctrl)
{
    HAL_CSI_Capture_Enable(mode, CSI_ENABLE);

    return COMP_OK;
}

/**
  * @brief Wait capture complete.
  * @note: This function is wait the picture capture done.
  * @retval capture picture size.
  */
uint32_t Drv_GC0308_Capture_Componemt(uint32_t timeout_ms)
{
    OS_SemaphoreWait(&gc0308_sem_wait, timeout_ms);

    uint32_t temp = gc0308_image_size;
    gc0308_image_size = 0;


    return temp;
}

/**
  * @brief Deinit the GC0308 and csi interface.
  * @retval Component_Status : The driver status.
  */
void Drv_GC0308_DeInit()
{
    HAL_CSI_DeInit();
    HAL_I2C_DeInit(GC0308_I2CID);
    HAL_DMA_Release(GC0308_dma_ch_fifo_a);
    HAL_DMA_Release(GC0308_dma_ch_fifo_b);
    OS_Status sta = OS_SemaphoreDelete(&gc0308_sem_wait);
    if (sta != OS_OK) {
        COMPONENT_WARN("GC0308 semaphore delete error, %d\n", sta);
    }
}





