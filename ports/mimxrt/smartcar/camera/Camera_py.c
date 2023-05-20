
/*
 * camera_py.c
 * Used to drive MT9V032 and integrate in micropython project
 * 
 * 
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mphal.h"
#include "py/gc.h"

#include "dma_manager.h"

#include "fsl_iomuxc.h"
#include "fsl_common.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_flexio.h"
#include "fsl_flexio_camera.h"

#include "fsl_flexio.c"
#include "fsl_flexio_camera.c"

#include "LCD_py.h"
#include "Camera_py.h"

extern const mp_obj_type_t camera_type;

// #ifdef  ENABLE_CAMERA_MT9V032
/***** Constants definition: Copy from SEEKFREE_MT9V032.h *****/

//配置摄像头参数
#define MT9V032_DMA_CH  DMA_CH0     //定义摄像头的DMA采集通道
#define MT9V032_W       188         //图像宽度  范围1-752       RT102X RT105X RT106X 采集时列宽度必须为4的倍数
#define MT9V032_H       120         //图像高度	范围1-480

/**********
//--------------------------------------------------------------------------------------------------
//引脚配置
//--------------------------------------------------------------------------------------------------
#define MT9V032_COF_UART        USART_3         //配置摄像头所使用到的串口     
#define MT9V032_COF_UART_TX     UART3_TX_C6
#define MT9V032_COF_UART_RX     UART3_RX_C7

#define MT9V032_VSYNC_PIN       B21             //场中断引脚
#define MT9V032_VSYNC_IRQN      GPIO1_Combined_16_31_IRQn   //中断号

#define MT9V032_DATA_PIN        FLEXIO1_IO0_B31 //定义D0数据引脚  假设D0定义为FLEXIO1_IO8_B23 那么D1所使用的引脚则为FLEXIO1_IO9_B22，依次类推
#define MT9V032_PCLK_PIN        FLEXIO1_IO8_B23 //定义像素时钟引脚
#define MT9V032_HREF_PIN        FLEXIO1_IO9_B22 //定义行信号引脚
**********/

typedef struct _iomux_table_t {
    uint32_t muxRegister;
    uint32_t muxMode;
    uint32_t inputRegister;
    uint32_t inputDaisy;
    uint32_t configRegister;
} iomux_table_t;

static const iomux_table_t MT9V032_iomux_table[10] = {
    { MT9V032_DATA0_IOMUXC },
    { MT9V032_DATA1_IOMUXC },
    { MT9V032_DATA2_IOMUXC },
    { MT9V032_DATA3_IOMUXC },
    { MT9V032_DATA4_IOMUXC },
    { MT9V032_DATA5_IOMUXC },
    { MT9V032_DATA6_IOMUXC },
    { MT9V032_DATA7_IOMUXC },

    { MT9V032_PCLK_IOMUXC },
    { MT9V032_HREF_IOMUXC },
};

//摄像头命令枚举
typedef enum
{
    CMD_INIT = 0,               //摄像头初始化命令
    CMD_AUTO_EXP,               //自动曝光命令
    CMD_EXP_TIME,               //曝光时间命令
    CMD_FPS,                    //摄像头帧率命令
    CMD_SET_COL,                //图像列命令
    CMD_SET_ROW,                //图像行命令
    CMD_LR_OFFSET,              //图像左右偏移命令
    CMD_UD_OFFSET,              //图像上下偏移命令
    CMD_GAIN,                   //图像偏移命令
    CMD_LAST,                   //非命令位，主要用来占位计数
    
    CMD_SET_EXP_TIME = 0xF0,    //单独设置曝光时间命令
    CMD_GET_STATUS,             //获取摄像头配置命令
    CMD_GET_VERSION,            //固件版本号命令
	
    CMD_SET_ADDR = 0XFE,        //寄存器地址命令
    CMD_SET_DATA                //寄存器数据命令
} CAMERA_CMD;
#define NUM_CONFIG  (CMD_LAST-1)

/***** Constants definition: Copy from SEEKFREE_MT9V032.h *****/
typedef struct _config {
    qstr        name;
    uint16_t    value;
    uint8_t     cmd;
} config_t;

typedef struct _camera_obj_t {
    mp_obj_base_t   base;
    mp_obj_base_t  *uart;
    config_t        configs[NUM_CONFIG];

    FLEXIO_CAMERA_Type      FlexioCameraDevice;
    flexio_camera_config_t  FlexioCameraConfig;
    edma_config_t           edma_config;
    edma_handle_t           g_EDMA_Handle;
    edma_transfer_config_t  transferConfig;

    const machine_pin_obj_t *vSyncPin;

    uint32_t                frame_counter;
    uint32_t                grab_counter;
    mp_obj_t               *user_callback;
    bool                    hard_callback;
    uint8_t                 dma_chn;

    uint8_t     *frame_buffer;
} camera_obj_t;
camera_obj_t camera_obj;

AT_NONCACHEABLE_SECTION_ALIGN(uint8_t capture_image[MT9V032_H][MT9V032_W],4);

extern const mp_obj_type_t camera_type;
extern const mp_obj_type_t camera_line_type;

typedef enum {
    CONFIG_INDEX_AUTO_EXP = 0,
    CONFIG_INDEX_EXP_TIME,
    CONFIG_INDEX_FPS,
    CONFIG_INDEX_SET_COL,
    CONFIG_INDEX_SET_ROW,
    CONFIG_INDEX_LR_OFFSET,
    CONFIG_INDEX_UD_OFFSET,
    CONFIG_INDEX_GAIN,
} CAMERA_CONFIG_INDEX;

//需要配置到摄像头的数据
static const config_t default_cfg[NUM_CONFIG] = {
    {   (MP_QSTR_AUTO_EXP),  0,     CMD_AUTO_EXP }, // AUTO_EXP     自动曝光设置    范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    {   (MP_QSTR_EXP_TIME),  450,   CMD_EXP_TIME }, // EXP_TIME     曝光时间        摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
    {   (MP_QSTR_FPS),       50,    CMD_FPS },      // FPS          图像帧率        摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
    {   (MP_QSTR_SET_COL),MT9V032_W,CMD_SET_COL },  // SET_COL      图像列数量      范围1-752     K60采集不允许超过188
    {   (MP_QSTR_SET_ROW),MT9V032_H,CMD_SET_ROW },  // SET_ROW      图像行数量      范围1-480
    {   (MP_QSTR_LR_OFFSET), 0,     CMD_LR_OFFSET },// LR_OFFSET    图像左右偏移量  正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {   (MP_QSTR_UD_OFFSET), 0,     CMD_UD_OFFSET },// UD_OFFSET    图像上下偏移量  正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {   (MP_QSTR_GAIN),      32,    CMD_GAIN },     // GAIN         图像增益        范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
};

static uint16_t mt9v032_set_cmd(camera_obj_t *self, uint8_t cmd, uint16_t value)
{
    int errcode;
    mp_stream_p_t *uart_op = (mp_stream_p_t*)MP_OBJ_TYPE_GET_SLOT(self->uart->type, protocol);
    char send_buf[4] = { 0xA5, cmd, value >> 8, value };
    uart_op->write(self->uart, &send_buf, sizeof(send_buf), &errcode);
    
    char reply[4];
    for (int idx = 0; idx < 20; idx++)  // Wait reply for 20*10ms
        uart_op->read(self->uart, reply, 4, NULL);
    return (reply[1] << 8) | reply[2];
}

static uint16_t mt9v032_config(camera_obj_t *self)
{
    int errcode;
    int idx;
    mp_stream_p_t *uart_op = (mp_stream_p_t*)MP_OBJ_TYPE_GET_SLOT(self->uart->type, protocol);
    for (idx = 0; idx < NUM_CONFIG; idx++) {
        uint16_t value = self->configs[idx].value;
        char send_buf[4] = { 0xA5, self->configs[idx].cmd, value >> 8, value };
        uart_op->write(self->uart, &send_buf, sizeof(send_buf), &errcode);
        mp_hal_delay_ms(1);
    }
    return mt9v032_set_cmd(self, CMD_INIT, 0);
}

/// \classmethod \constructor()
/// Create and return an camera object.
///
/// Construct a camera object with given UART object.
///
/// camera(uart)
STATIC mp_obj_t MT9V032_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    extern const mp_obj_type_t machine_uart_type;
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    mp_obj_base_t *uart = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[0]);
    if (!mp_obj_is_type(uart, &machine_uart_type))
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("param has to be a machine.UART obj"));

    camera_obj_t *self = &camera_obj;
    memset(self, 0, sizeof(camera_obj));
    self->base.type = &camera_type;
    self->frame_buffer = m_malloc(MT9V032_H * MT9V032_W);
    // camera_obj_t *self = mp_obj_malloc(camera_obj_t, &camera_type);

    self->uart = uart;
    memcpy(self->configs, default_cfg, sizeof(default_cfg));

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t MT9V032_set_reg(mp_obj_t self_in, mp_obj_t addr_in, mp_obj_t data_in)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t addr = MP_OBJ_SMALL_INT_VALUE(addr_in);
    uint16_t data = MP_OBJ_SMALL_INT_VALUE(data_in);
    int errcode;
    mp_stream_p_t *uart_op = (mp_stream_p_t*)MP_OBJ_TYPE_GET_SLOT(self->uart->type, protocol);
    char send_buf[4] = { 0xA5, CMD_SET_ADDR, addr >> 8, addr };
    uart_op->write(self->uart, &send_buf, sizeof(send_buf), &errcode);
    return MP_OBJ_NEW_SMALL_INT(mt9v032_set_cmd(self, CMD_SET_DATA, data));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(MT9V032_set_reg_obj, MT9V032_set_reg);

STATIC mp_obj_t MT9V032_init(mp_obj_t self_in)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mt9v032_config(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(MT9V032_init_obj, MT9V032_init);

#define DMA_MUX_SRC         kDmaRequestMuxFlexIO1Request6Request7

#define FLEXIO_PIN_CONF     (IOMUXC_SW_PAD_CTL_PAD_SPEED(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PKE(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PUS(0) \
                                | IOMUXC_SW_PAD_CTL_PAD_DSE(6))  // 100MHz, 100K PD, Keeper, R/6
#define FXIO_SHFT_COUNT     1u

#define VSYNC_PIN_CONF      (IOMUXC_SW_PAD_CTL_PAD_SPEED(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PKE(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PUS(0b01) \
                                | IOMUXC_SW_PAD_CTL_PAD_DSE(4))  // 100MHz, 47K PU, Keeper, R/4

static volatile uint8_t mt9v032_finish_flag = 0;    //一场图像采集完成标志位

void mt9v032_dma_cb(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	// mt9v032_finish_flag = 1;//一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
    camera_obj.frame_counter++;
    if (camera_obj.user_callback) {
        // do a hard call first:
        if (camera_obj.hard_callback) {
            // extern void fool_stack_check(void);
            // extern void nofool_stack_check(void);
            mp_sched_lock();
            // When executing code within a handler we must lock the GC to prevent
            // any memory allocations.  We must also catch any exceptions.
            gc_lock();
                // fool_stack_check();
            nlr_buf_t nlr;
            if(nlr_push(&nlr) == 0) {
                mp_call_function_1(camera_obj.user_callback, MP_OBJ_NEW_SMALL_INT(camera_obj.frame_counter));
                nlr_pop();
            }
            else {
                // Uncaught exception; disable the callback so it doesn't run again.
                camera_obj.user_callback = NULL;
                //__HAL_TIM_DISABLE_IT(&tim->tim, irq_mask);
                mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
            }
                // nofool_stack_check();
            gc_unlock();
            mp_sched_unlock();
        }
        else    // Soft callback
            mp_sched_schedule(camera_obj.user_callback, MP_OBJ_NEW_SMALL_INT(camera_obj.frame_counter));
    }
}

STATIC mp_obj_t vSync_isr_handler(mp_obj_t self_in)
{
    machine_pin_obj_t *pin = self_in;
    camera_obj_t *self = &camera_obj;

    if (GPIO_PinRead(pin->gpio, pin->pin))
        return mp_const_none;       // This is the rising. Wondering how it comes

    int dma_chn = self->g_EDMA_Handle.channel;

    // void dma_restart(uint8 *dest_addr)
    DMA0->TCD[dma_chn].DADDR = (uint32_t)(capture_image);
    // flexio_flag_clear();
    FLEXIO_CAMERA_ClearStatusFlags(&self->FlexioCameraDevice, kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
    DMA0->SERQ = DMA_SERQ_SERQ(dma_chn);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(vSync_isr_obj, vSync_isr_handler);

void enable_vsync_irq(camera_obj_t *self, bool enable)
{
    mp_obj_t dest[2];
    mp_load_method((mp_obj_t)self->vSyncPin, MP_QSTR_irq, dest);
    
    if (enable) {
        mp_obj_t args[4];
        args[0] = dest[1];
        args[1] = (mp_obj_t)(&vSync_isr_obj);
        args[2] = MP_OBJ_NEW_SMALL_INT(2);  // Falling
        args[3] = mp_const_true;            // Hard irq
        mp_call_function_n_kw(dest[0], 4, 0, args);
    }
    else {
        mp_call_function_1(dest[0], dest[1]);
        // mp_call_function_n_kw(dest[0], 1, 0, args);
    }
}

STATIC mp_obj_t MT9V032_start(mp_obj_t self_in)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->dma_chn = allocate_dma_channel();
    
    {   // dma_mux_init()
#ifdef BSP_USING_SPI_LCD
        extern int dmamux_get_dma_channel(uint32_t source);
        dma_chn = dmamux_get_dma_channel(DMA_MUX_SRC);
#else
        DMAMUX_Deinit(DMAMUX);
        DMAMUX_Init(DMAMUX);
        DMAMUX_SetSource(DMAMUX, self->dma_chn, DMA_MUX_SRC);
        DMAMUX_EnableChannel(DMAMUX, self->dma_chn);
#endif
    }
    {
        const iomux_table_t *io = MT9V032_iomux_table;
        for (int pin = 0; pin < sizeof(MT9V032_iomux_table) / sizeof(iomux_table_t); io++, pin++) {
            IOMUXC_SetPinMux(io->muxRegister, io->muxMode, io->inputRegister, io->inputDaisy, io->configRegister, 0U);
            IOMUXC_SetPinConfig(io->muxRegister, io->muxMode, io->inputRegister, io->inputDaisy, io->configRegister, FLEXIO_PIN_CONF);
        }
        self->vSyncPin = &MT9V032_VSYNC_PIN_NAME;
        IOMUXC_SetPinMux(MT9V032_VSYNC_IOMUXC, 0);
        IOMUXC_SetPinConfig(MT9V032_VSYNC_IOMUXC, VSYNC_PIN_CONF);
        gpio_pin_config_t pin_config = {kGPIO_DigitalInput, 1, kGPIO_IntRisingEdge};
        GPIO_PinInit(self->vSyncPin->gpio, self->vSyncPin->pin, &pin_config);

        self->FlexioCameraDevice.flexioBase = FLEXIO1;                  //设置基地址
        self->FlexioCameraDevice.datPinStartIdx = 0+16;                    //设置数据引脚起始位
        self->FlexioCameraDevice.pclkPinIdx = 8+16;                        //设置像素时钟引脚
        self->FlexioCameraDevice.hrefPinIdx = 9+16;                        //设置行信号引脚
        self->FlexioCameraDevice.shifterStartIdx = 8-FXIO_SHFT_COUNT;   //设置环移器起始编号
        self->FlexioCameraDevice.shifterCount = FXIO_SHFT_COUNT;        //设置缓冲区数量
        self->FlexioCameraDevice.timerIdx = 7;                          //设置定时器编号

        CLOCK_EnableClock(kCLOCK_Flexio1);
        FLEXIO_Reset(FLEXIO1);
        FLEXIO_CAMERA_GetDefaultConfig(&self->FlexioCameraConfig);
        FLEXIO_CAMERA_Init(&self->FlexioCameraDevice, &self->FlexioCameraConfig);
        FLEXIO_CAMERA_ClearStatusFlags(&self->FlexioCameraDevice, kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
        FLEXIO_CAMERA_Enable(&self->FlexioCameraDevice, true);
    }
    {   // flexio_dma_init(&image[0][0], MT9V032_W*MT9V032_H, mt9v032_dma)
        edma_modulo_t s_addr_modulo;
#ifndef BSP_USING_SPI_LCD
        EDMA_GetDefaultConfig(&self->edma_config);
        EDMA_Deinit(DMA0);
        EDMA_Init(DMA0, &self->edma_config);
        DMA0->CERR = DMA_CERR_CERR_MASK | DMA_CERR_CAEI_MASK;
#endif        
        EDMA_CreateHandle(&self->g_EDMA_Handle, DMA0, self->dma_chn);
        EDMA_SetCallback(&self->g_EDMA_Handle, mt9v032_dma_cb, NULL);
        EDMA_PrepareTransfer(&self->transferConfig, 
                                (void *)FLEXIO_CAMERA_GetRxBufferAddress(&self->FlexioCameraDevice), 
                                4,
                                (void *)(capture_image), 
                                4,
                                4*FXIO_SHFT_COUNT,
                                self->configs[CONFIG_INDEX_SET_COL].value * self->configs[CONFIG_INDEX_SET_ROW].value,// MT9V032_W * MT9V032_H,
                                kEDMA_MemoryToMemory);

        EDMA_SubmitTransfer(&self->g_EDMA_Handle, &self->transferConfig);

        switch(4*FXIO_SHFT_COUNT)
        {
            case 4:     s_addr_modulo = kEDMA_Modulo4bytes;break;
            case 8:     s_addr_modulo = kEDMA_Modulo8bytes;break;
            case 16:    s_addr_modulo = kEDMA_Modulo16bytes;break;
            case 32:    s_addr_modulo = kEDMA_Modulo32bytes;break;
            default:assert(0);  //参数有误
        }

        EDMA_SetModulo(DMA0, self->dma_chn, s_addr_modulo, kEDMA_ModuloDisable);
        EDMA_StartTransfer(&self->g_EDMA_Handle);
    }
    {
        // flexio_enable_rxdma()
        FLEXIO_CAMERA_EnableRxDMA(&self->FlexioCameraDevice, true);
        NVIC_SetPriority(DMA0_DMA16_IRQn, 1);            //设置DMA中断优先级 范围0-15 越小优先级越高

        // 设置场中断
        enable_vsync_irq(self, true);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(MT9V032_start_obj, MT9V032_start);

STATIC mp_obj_t MT9V032_stop(mp_obj_t self_in)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    EDMA_StopTransfer(&self->g_EDMA_Handle);
    // rt_pin_irq_enable(self->vSyncPin->pin, PIN_IRQ_DISABLE);
    enable_vsync_irq(self, false);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(MT9V032_stop_obj, MT9V032_stop);

#define GRAY8BIT_RGB(v)     (((v<<10)&0xF800) | ((v<<5)&0x07E0) | ((v>>1)&0x1F))
// Lookup table for gray pixel to RGB565
static const uint16_t Gray_8bit_RGB[64] = {
    GRAY8BIT_RGB(0),	GRAY8BIT_RGB(1),	GRAY8BIT_RGB(2),	GRAY8BIT_RGB(3),	GRAY8BIT_RGB(4),	GRAY8BIT_RGB(5),	GRAY8BIT_RGB(6),	GRAY8BIT_RGB(7),
    GRAY8BIT_RGB(8),	GRAY8BIT_RGB(9),	GRAY8BIT_RGB(10),	GRAY8BIT_RGB(11),	GRAY8BIT_RGB(12),	GRAY8BIT_RGB(13),	GRAY8BIT_RGB(14),	GRAY8BIT_RGB(15),
    GRAY8BIT_RGB(16),	GRAY8BIT_RGB(17),	GRAY8BIT_RGB(18),	GRAY8BIT_RGB(19),	GRAY8BIT_RGB(20),	GRAY8BIT_RGB(21),	GRAY8BIT_RGB(22),	GRAY8BIT_RGB(23),
    GRAY8BIT_RGB(24),	GRAY8BIT_RGB(25),	GRAY8BIT_RGB(26),	GRAY8BIT_RGB(27),	GRAY8BIT_RGB(28),	GRAY8BIT_RGB(29),	GRAY8BIT_RGB(30),	GRAY8BIT_RGB(31),
    GRAY8BIT_RGB(32),	GRAY8BIT_RGB(33),	GRAY8BIT_RGB(34),	GRAY8BIT_RGB(35),	GRAY8BIT_RGB(36),	GRAY8BIT_RGB(37),	GRAY8BIT_RGB(38),	GRAY8BIT_RGB(39),
    GRAY8BIT_RGB(40),	GRAY8BIT_RGB(41),	GRAY8BIT_RGB(42),	GRAY8BIT_RGB(43),	GRAY8BIT_RGB(44),	GRAY8BIT_RGB(45),	GRAY8BIT_RGB(46),	GRAY8BIT_RGB(47),
    GRAY8BIT_RGB(48),	GRAY8BIT_RGB(49),	GRAY8BIT_RGB(50),	GRAY8BIT_RGB(51),	GRAY8BIT_RGB(52),	GRAY8BIT_RGB(53),	GRAY8BIT_RGB(54),	GRAY8BIT_RGB(55),
    GRAY8BIT_RGB(56),	GRAY8BIT_RGB(57),	GRAY8BIT_RGB(58),	GRAY8BIT_RGB(59),	GRAY8BIT_RGB(60),	GRAY8BIT_RGB(61),	GRAY8BIT_RGB(62),	GRAY8BIT_RGB(63),
};

/// \classmethod \grab(lcd)
/// Capture the current data to an internel buffer
/// Return the frame counter
STATIC mp_obj_t MT9V032_grab(size_t n_args, const mp_obj_t *args)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (self->grab_counter == self->frame_counter)
        return mp_const_none;
    self->grab_counter = self->frame_counter;

    int width = self->configs[CONFIG_INDEX_SET_COL].value;    // MT9V032_W;
    int height = self->configs[CONFIG_INDEX_SET_ROW].value;   // MT9V032_H;
    uint8_t *img_buf = self->frame_buffer;
    memcpy(img_buf, capture_image, width * height); // Longest take 16.4us

    if (n_args >= 2) {
        if (!mp_obj_is_type(args[1], &display_lcd_type))
            mp_raise_TypeError(MP_ERROR_TEXT("param has to be LCD object"));
        display_lcd_obj_t *lcd_obj = (display_lcd_obj_t*)MP_OBJ_TO_PTR(args[1]);
        display_lcd_p_t *lcd_op = (display_lcd_p_t*)MP_OBJ_TYPE_GET_SLOT(lcd_obj->base.type, protocol);
        uint16_t gray_line[MT9V032_W*2];

        uint16_t lcd_w = lcd_obj->lcd_drv->lcd.Width;
        uint16_t lcd_h = lcd_obj->lcd_drv->lcd.Height;
        // Process normal and enlarge display request
        if ((n_args == 3) && mp_obj_is_true(args[2])) {
            // When the camera height > screen height, display lower portion
            // Otherwise, display on top of screen. Leave lower screen not change.
            lcd_h /= 2;
            if (height > lcd_h) {
                int skip_line = (height - lcd_h) / 2;
                img_buf += skip_line * width;       // Skip few top lines
                height = lcd_h;
            }

            // When the camera width > screen width, display middle portion
            // Otherwise, display on the left of screen. Leave screen right not change
            lcd_w /= 2;
            int width_diff = 0;
            int left = 0;
            if (width > lcd_w) {
                width_diff = width - lcd_w;
                img_buf += width_diff / 2;
                width = lcd_w;
            }
            else
                left = (lcd_w - width) / 2;

            for (int hy = 0; hy < height; hy++) {
                uint16_t *line = gray_line;
                for (int wx = 0; wx < width; wx++) {
                    uint16_t pixel = Gray_8bit_RGB[*img_buf++ >> 2];
                    *line++ = pixel;
                    *line++ = pixel;
                }
                mp_buffer_info_t buf = { .buf = gray_line, .len = width*4, };
                Rect_t rect = { left, hy*2, width*2, 1};
                lcd_op->blit(lcd_obj, &rect, &buf);
                rect.y0++;
                lcd_op->blit(lcd_obj, &rect, &buf);
                img_buf += width_diff;
            }
        }
        else {
            // When the camera height > screen height, display lower portion
            // Otherwise, display on top of screen. Leave lower screen not change.
            if (height > lcd_h) {
                int skip_line = (height - lcd_h) / 2;
                img_buf += skip_line * width;       // Skip few top lines
                height = lcd_h;
            }

            // When the camera width > screen width, display middle portion
            // Otherwise, display on the left of screen. Leave screen right not change
            int width_diff = 0;
            int left = 0;
            if (width > lcd_w) {
                width_diff = width - lcd_w;
                img_buf += width_diff / 2;
                width = lcd_w;
            }
            else
                left = (lcd_w - width) / 2;

            for (int hy = 0; hy < height; hy++) {
                uint16_t *line = gray_line;
                for (int wx = 0; wx < width; wx++) {
                    uint16_t pixel = Gray_8bit_RGB[*img_buf++ >> 2];
                    *line++ = pixel;
                }
                mp_buffer_info_t buf = { .buf = gray_line, .len = width*2, };
                Rect_t rect = { left, hy, width, 1};
                lcd_op->blit(lcd_obj, &rect, &buf);
                img_buf += width_diff;
            }
        }
    }

    return MP_OBJ_NEW_SMALL_INT(self->frame_counter);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(MT9V032_grab_obj, 1, 3, MT9V032_grab);

/// \classmethod callback(func)
STATIC mp_obj_t MT9V032_callback(size_t n_args, const mp_obj_t *args)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args >= 2)
        self->user_callback = MP_OBJ_TO_PTR(args[1]);
    else
        self->user_callback = NULL;
    self->hard_callback = false;
    if (n_args == 3)
        self->hard_callback = mp_obj_is_true(args[2]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(MT9V032_callback_obj, 1, 3, MT9V032_callback);

/// \classmethod \image(num)
/// Return the full image with bytearray
STATIC mp_obj_t MT9V032_image(mp_obj_t self_in)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int width  = self->configs[CONFIG_INDEX_SET_COL].value;    // MT9V032_W;
    int height = self->configs[CONFIG_INDEX_SET_ROW].value;   // MT9V032_H;
    return mp_obj_new_bytearray_by_ref(width * height, self->frame_buffer);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(MT9V032_image_obj, MT9V032_image);

/// \classmethod \line(num)
/// Return a line with bytearray
STATIC mp_obj_t MT9V032_line(mp_obj_t self_in, mp_obj_t lineno_in)
{
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int lineno = mp_obj_get_int(lineno_in);
    if (lineno < 0 || lineno >= self->configs[CONFIG_INDEX_SET_ROW].value)
        mp_raise_ValueError(MP_ERROR_TEXT("index out of range."));
    int width = self->configs[CONFIG_INDEX_SET_COL].value;
    return mp_obj_new_bytearray_by_ref(width, ((uint8_t*)self->frame_buffer) + width * lineno);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(MT9V032_line_obj, MT9V032_line);

STATIC mp_obj_t lines_iterator_new(mp_obj_t camera_in, mp_obj_iter_buf_t *iter_buf);
/// \classmethod \lines()
/// Return an iterator for each lines
STATIC mp_obj_t MT9V032_lines(mp_obj_t self_in)
{
    mp_obj_iter_buf_t *iter_buf = m_new_obj(mp_obj_iter_buf_t);
    return lines_iterator_new(self_in, iter_buf);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(MT9V032_lines_obj, MT9V032_lines);

STATIC void MT9V032_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
	camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (dest[0] == MP_OBJ_NULL) {	// Load
        int idx;
        for (idx = 0; idx < NUM_CONFIG; idx++) {
            if (attr == self->configs[idx].name) {
                dest[0] = MP_OBJ_NEW_SMALL_INT(self->configs[idx].value);
                return;
            }
        }
        if (attr == MP_QSTR_Version) {
            dest[0] = MP_OBJ_NEW_SMALL_INT(mt9v032_set_cmd(self, CMD_GET_STATUS, CMD_GET_VERSION));
            return;
        }
        else {
            dest[0] = mp_obj_dict_get((mp_obj_t)MP_OBJ_TYPE_GET_SLOT(&camera_type, locals_dict), MP_ROM_QSTR(attr));
            dest[1] = self_in;
        }
    }
    else {
        if (dest[1] != MP_OBJ_NULL) {   // Store
            for (int idx = 0; idx < NUM_CONFIG; idx++) {
                if (attr == self->configs[idx].name) {
                    self->configs[idx].value = mp_obj_get_int(dest[1]);
                    if (idx == (CMD_EXP_TIME-1))
                        mt9v032_set_cmd(self, CMD_SET_EXP_TIME, self->configs[idx].value);

                    dest[0] = MP_OBJ_NULL; // indicate success
                    return;
                }
            }
        }
    }
}

STATIC void MT9V032_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    mp_printf(print, "Camera parameters:\r\n");
    for (int idx = 0; idx < NUM_CONFIG; idx++) {
        mp_printf(print, "\t%q = %d;", self->configs[idx].name, self->configs[idx].value);
    }
}

STATIC const mp_rom_map_elem_t MT9V032_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),    MP_ROM_PTR(&MT9V032_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_reg), MP_ROM_PTR(&MT9V032_set_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_grab),    MP_ROM_PTR(&MT9V032_grab_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),   MP_ROM_PTR(&MT9V032_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),    MP_ROM_PTR(&MT9V032_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_callback),MP_ROM_PTR(&MT9V032_callback_obj) },

    { MP_ROM_QSTR(MP_QSTR_image),   MP_ROM_PTR(&MT9V032_image_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),    MP_ROM_PTR(&MT9V032_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_lines),   MP_ROM_PTR(&MT9V032_lines_obj) },
};

STATIC MP_DEFINE_CONST_DICT(MT9V032_locals_dict, MT9V032_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    camera_type,
    MP_QSTR_MT9V032,
    MP_TYPE_FLAG_NONE,
    make_new, MT9V032_make_new,
    print, MT9V032_print,
    attr, MT9V032_attr,
    locals_dict, &MT9V032_locals_dict
    );


/******************************************************************************/
// lines iterator

typedef struct _mp_obj_lines_it_t {
    mp_obj_base_t base;
    uint8_t *image_buf;
    uint16_t width, height;
    uint16_t line_no;
} mp_obj_lines_it_t;

STATIC mp_obj_t lines_it_iternext(mp_obj_t self_in)
{
    mp_obj_lines_it_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->line_no < self->height)
        return mp_obj_new_bytearray_by_ref(self->width, self->image_buf + self->width * self->line_no++);
        // return mp_obj_new_bytearray();
    else
        return MP_OBJ_STOP_ITERATION;
}

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    mp_type_lines_it,
    MP_QSTR_iterator,
    MP_TYPE_FLAG_ITER_IS_ITERNEXT,
    iter, lines_it_iternext
    );

STATIC mp_obj_t lines_iterator_new(mp_obj_t camera_in, mp_obj_iter_buf_t *iter_buf) {
    assert(sizeof(mp_obj_lines_it_t) <= sizeof(mp_obj_iter_buf_t));
    camera_obj_t *camera = MP_OBJ_TO_PTR(camera_in);
    mp_obj_lines_it_t *o = (mp_obj_lines_it_t *)iter_buf;
    o->base.type = &mp_type_lines_it;
    o->image_buf = (uint8_t *)camera->frame_buffer;
    o->width = camera->configs[CONFIG_INDEX_SET_COL].value;    // MT9V032_W;
    o->height = camera->configs[CONFIG_INDEX_SET_ROW].value;   // MT9V032_H;
    o->line_no = 0;
    return MP_OBJ_FROM_PTR(o);
}

// #endif  // ENABLE_CAMERA_MT9V032